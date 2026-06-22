#include "MotorWorker.h"

#include <QtGlobal>
#include <QDateTime>

MotorWorker::MotorWorker(QObject *parent)
    : QObject(parent)
{
    velocityRampTimer = new QTimer(this);
    velocityRampTimer->setInterval(VELOCITY_RAMP_TIMER_MS);
    velocityRampTimer->setSingleShot(false);

    connect(velocityRampTimer,
            &QTimer::timeout,
            this,
            &MotorWorker::processVelocityRamp);
}

MotorWorker::~MotorWorker()
{
    if (velocityRampTimer != nullptr)
    {
        velocityRampTimer->stop();
    }

    if (tmc != nullptr)
    {
        tmc->closePort();
    }
}

void MotorWorker::ensureInterface()
{
    if (tmc != nullptr)
    {
        return;
    }

    tmc = new Tmc6460QtInterface(this);
    connect(tmc, &Tmc6460QtInterface::logMessage, this, &MotorWorker::logMessage);
    connect(tmc, &Tmc6460QtInterface::errorChanged, this, &MotorWorker::errorChanged);
}

void MotorWorker::connectAndInitialize(const QString &portName, int baudRate)
{
    ensureInterface();

    resetVelocityRampState();
    stopStallMonitor(true);

    if (!tmc->openPort(portName.trimmed(), baudRate))
    {
        emit connected(false, 0, tmc->lastError());
        return;
    }

    if (!tmc->initializeMotor())
    {
        emit connected(false, 0, tmc->lastError());
        return;
    }

    quint32 chipId = 0;
    if (!tmc->readChipId(&chipId))
    {
        emit connected(false, 0, tmc->lastError());
        return;
    }

    emit connected(true, chipId, QStringLiteral("Connected and initialized"));
}

void MotorWorker::readStatus()
{
    ensureInterface();

    if (!tmc->isOpen())
    {
        return;
    }

    Tmc6460QtInterface::RunStatus status;
    if (tmc->readRunStatus(&status))
    {
        emit errorChanged(QString());
        emit statusReady(status);

        emit logMessage(QStringLiteral("MONITOR: velRaw=%1 pos=%2 torqueFlux=%3 torqueRaw=%4 status=%5")
                        .arg(status.velocityActualRaw)
                        .arg(status.positionActual)
                        .arg(hex32(status.torqueFluxActual))
                        .arg(status.torqueActualRaw)
                        .arg(hex32(status.chipStatusFlags)));

#if TMC6460_ENABLE_SIMPLE_STALL_DETECTION
        checkSimpleStall(status);
#endif
    }
    else
    {
        emit errorChanged(tmc->lastError());
    }
}

void MotorWorker::applyTorque(int value)
{
    ensureInterface();

    resetVelocityRampState();
    stopStallMonitor(true);

    const int limitedTorque = qBound<int>(-MAX_ALLOWED_TORQUE_RAW, value, MAX_ALLOWED_TORQUE_RAW);

    if (limitedTorque != value)
    {
        emit logMessage(QStringLiteral("SAFETY: Torque command %1 clipped to %2")
                        .arg(value)
                        .arg(limitedTorque));
    }

    const bool ok = tmc->isOpen() && tmc->setTorqueTarget(limitedTorque);
    emit commandDone(QStringLiteral("Torque target %1").arg(limitedTorque), ok);
}

void MotorWorker::applyVelocityRaw(qint32 rawVelocity)
{
    ensureInterface();

    const qint32 limitedVelocity = qBound<qint32>(-MAX_ALLOWED_VELOCITY_RAW,
                                                  rawVelocity,
                                                  MAX_ALLOWED_VELOCITY_RAW);

    if (limitedVelocity != rawVelocity)
    {
        emit logMessage(QStringLiteral("SAFETY: Velocity command %1 clipped to %2")
                        .arg(rawVelocity)
                        .arg(limitedVelocity));
    }

    rawVelocity = limitedVelocity;

    if (!tmc->isOpen())
    {
        emit commandDone(QStringLiteral("Apply velocity target raw=%1").arg(rawVelocity), false);
        emit errorChanged(QStringLiteral("Cannot apply velocity: serial port is not open"));
        return;
    }

    stopStallMonitor(true);

#if TMC6460_ENABLE_VELOCITY_RAMP
    targetVelocityRaw = rawVelocity;

    if (!velocityRampTimer->isActive())
    {
        velocityRampTimer->start();
    }
#else
    resetVelocityRampState();

    const bool ok = writeVelocityRaw(rawVelocity);
    if (ok)
    {
        currentCommandedVelocityRaw = rawVelocity;
        startStallMonitor(rawVelocity);
    }

    emit commandDone(QStringLiteral("Velocity target raw=%1 direct").arg(rawVelocity), ok);
    if (!ok)
    {
        emit errorChanged(QStringLiteral("Velocity direct write failed"));
    }
#endif
}

void MotorWorker::processVelocityRamp()
{
    ensureInterface();

    if (!tmc->isOpen())
    {
        velocityRampTimer->stop();
        return;
    }

    if (currentCommandedVelocityRaw == targetVelocityRaw)
    {
        velocityRampTimer->stop();
        startStallMonitor(currentCommandedVelocityRaw);
        emit commandDone(QStringLiteral("Velocity reached raw=%1").arg(currentCommandedVelocityRaw), true);
        return;
    }

    qint32 nextRaw = currentCommandedVelocityRaw;

    if (currentCommandedVelocityRaw < targetVelocityRaw)
    {
        nextRaw += VELOCITY_RAMP_STEP_RAW;
        if (nextRaw > targetVelocityRaw)
        {
            nextRaw = targetVelocityRaw;
        }
    }
    else
    {
        nextRaw -= VELOCITY_RAMP_STEP_RAW;
        if (nextRaw < targetVelocityRaw)
        {
            nextRaw = targetVelocityRaw;
        }
    }

    if (!writeVelocityRaw(nextRaw))
    {
        velocityRampTimer->stop();
        stopStallMonitor(false);
        emit commandDone(QStringLiteral("Velocity ramp failed at raw=%1").arg(nextRaw), false);
        emit errorChanged(QStringLiteral("Velocity ramp write failed"));
        return;
    }

    currentCommandedVelocityRaw = nextRaw;

    if (currentCommandedVelocityRaw == targetVelocityRaw)
    {
        velocityRampTimer->stop();
        startStallMonitor(currentCommandedVelocityRaw);
        emit commandDone(QStringLiteral("Velocity reached raw=%1").arg(currentCommandedVelocityRaw), true);
    }
}

bool MotorWorker::writeVelocityRaw(qint32 rawVelocity)
{
    const bool ok = tmc->setVelocityTarget(rawVelocity);

    if (!ok)
    {
        emit logMessage(QStringLiteral("Velocity write failed at raw=%1")
                        .arg(rawVelocity));
    }

    return ok;
}

void MotorWorker::emergencyStop()
{
    ensureInterface();

    resetVelocityRampState();
    stopStallMonitor(true);

    const bool ok = tmc->isOpen() && tmc->emergencyStop();
    emit commandDone(QStringLiteral("E-STOP"), ok);
}

void MotorWorker::shutdown()
{
    resetVelocityRampState();
    stopStallMonitor(true);

    if (tmc != nullptr)
    {
        tmc->closePort();
    }
}

void MotorWorker::resetVelocityRampState()
{
    if (velocityRampTimer != nullptr)
    {
        velocityRampTimer->stop();
    }

    currentCommandedVelocityRaw = 0;
    targetVelocityRaw = 0;
}

void MotorWorker::startStallMonitor(qint32 rawVelocity)
{
#if TMC6460_ENABLE_SIMPLE_STALL_DETECTION
    if (qAbs(static_cast<qint64>(rawVelocity)) < STALL_MIN_TARGET_RAW)
    {
        stopStallMonitor(true);
        return;
    }

    stallMonitorActive = true;
    stallLatched = false;
    stallTargetVelocityRaw = rawVelocity;
    stallTimer.restart();

    emit stallStateChanged(false,
                           QStringLiteral("Monitoring, target raw=%1, stall threshold 60% after 5 s")
                               .arg(rawVelocity));
    emit logMessage(QStringLiteral("STALL_MONITOR: started targetRaw=%1 minRaw=%2")
                    .arg(stallTargetVelocityRaw)
                    .arg(qRound(qAbs(static_cast<double>(stallTargetVelocityRaw)) * STALL_MIN_VELOCITY_RATIO)));
#else
    Q_UNUSED(rawVelocity)
#endif
}

void MotorWorker::stopStallMonitor(bool clearGuiStatus)
{
    stallMonitorActive = false;
    stallLatched = false;
    stallTargetVelocityRaw = 0;

    if (clearGuiStatus)
    {
        emit stallStateChanged(false, QStringLiteral("Not active"));
    }
}

void MotorWorker::checkSimpleStall(const Tmc6460QtInterface::RunStatus &status)
{
    if (!stallMonitorActive || stallLatched)
    {
        return;
    }

    if (!stallTimer.isValid() || stallTimer.elapsed() < STALL_SETTLE_TIME_MS)
    {
        return;
    }

    const qint64 actualAbs = qAbs(static_cast<qint64>(status.velocityActualRaw));
    const qint64 minimumAbs = qRound(qAbs(static_cast<double>(stallTargetVelocityRaw)) * STALL_MIN_VELOCITY_RATIO);

    if (actualAbs >= minimumAbs)
    {
        return;
    }

    stallLatched = true;
    stallMonitorActive = false;
    resetVelocityRampState();

    const QString msg = QStringLiteral("STALL DETECTED: actual velocity %1 raw is below 60% of target %2 raw. Position=%3 TorqueFlux=%4")
                            .arg(status.velocityActualRaw)
                            .arg(stallTargetVelocityRaw)
                            .arg(status.positionActual)
                            .arg(hex32(status.torqueFluxActual));

    emit logMessage(msg);

    bool ok = false;
    if (tmc != nullptr && tmc->isOpen())
    {
        ok = tmc->holdAfterStall();
    }

    emit stallStateChanged(true, ok ? QStringLiteral("STALL DETECTED - motor hold applied")
                                    : QStringLiteral("STALL DETECTED - hold command failed"));

    if (!ok)
    {
        emit errorChanged(QStringLiteral("Stall detected but hold command failed"));
    }
}

QString MotorWorker::hex32(quint32 value)
{
    return QStringLiteral("0x%1").arg(value, 8, 16, QLatin1Char('0')).toUpper();
}
