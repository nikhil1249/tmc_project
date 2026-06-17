#include "MotorWorker.h"

#include <QtGlobal>

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
    }
    else
    {
        emit errorChanged(tmc->lastError());
    }
}

void MotorWorker::applyTorque(int value)
{
    ensureInterface();
    const bool ok = tmc->isOpen() && tmc->setTorqueTarget(value);
    emit commandDone(QStringLiteral("Apply torque %1").arg(value), ok);
}

void MotorWorker::applyVelocityRpm(int rpm)
{
    ensureInterface();

    if (!tmc->isOpen())
    {
        emit commandDone(QStringLiteral("Apply velocity target %1 rpm").arg(rpm), false);
        emit errorChanged(QStringLiteral("Cannot apply velocity: serial port is not open"));
        return;
    }

    targetVelocityRpm = rpm;

    if (!velocityRampTimer->isActive())
    {
        velocityRampTimer->start();
    }
}

void MotorWorker::processVelocityRamp()
{
    ensureInterface();

    if (!tmc->isOpen())
    {
        velocityRampTimer->stop();
        return;
    }

    if (currentCommandedVelocityRpm == targetVelocityRpm)
    {
        velocityRampTimer->stop();
        emit commandDone(QStringLiteral("Velocity reached %1 rpm").arg(currentCommandedVelocityRpm), true);
        return;
    }

    int nextRpm = currentCommandedVelocityRpm;

    if (currentCommandedVelocityRpm < targetVelocityRpm)
    {
        nextRpm += VELOCITY_RAMP_STEP_RPM;
        if (nextRpm > targetVelocityRpm)
        {
            nextRpm = targetVelocityRpm;
        }
    }
    else
    {
        nextRpm -= VELOCITY_RAMP_STEP_RPM;
        if (nextRpm < targetVelocityRpm)
        {
            nextRpm = targetVelocityRpm;
        }
    }

    if (!writeVelocityRpm(nextRpm))
    {
        velocityRampTimer->stop();
        emit commandDone(QStringLiteral("Velocity ramp failed at %1 rpm").arg(nextRpm), false);
        emit errorChanged(QStringLiteral("Velocity ramp write failed"));
        return;
    }

    currentCommandedVelocityRpm = nextRpm;

    if (currentCommandedVelocityRpm == targetVelocityRpm)
    {
        velocityRampTimer->stop();
        emit commandDone(QStringLiteral("Velocity reached %1 rpm").arg(currentCommandedVelocityRpm), true);
    }
}

bool MotorWorker::writeVelocityRpm(int rpm)
{
    const qint32 rawVelocity = Tmc6460QtInterface::rpmToVelocityRaw(rpm);
    const bool ok = tmc->setVelocityTarget(rawVelocity);

    if (!ok)
    {
        emit logMessage(QStringLiteral("Velocity write failed at %1 rpm, raw %2")
                        .arg(rpm)
                        .arg(rawVelocity));
    }

    return ok;
}

void MotorWorker::emergencyStop()
{
    ensureInterface();

    resetVelocityRampState();

    const bool ok = tmc->isOpen() && tmc->emergencyStop();
    emit commandDone(QStringLiteral("E-STOP"), ok);
}

void MotorWorker::shutdown()
{
    resetVelocityRampState();

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

    currentCommandedVelocityRpm = 0;
    targetVelocityRpm = 0;
}
