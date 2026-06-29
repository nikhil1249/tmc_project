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
        if (tmc->isOpen())
        {
            tmc->shutdownMotorSafe();
        }
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
    stopCalibrationEndMonitor(true);
    resetAutoCalibrationState();

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

#if TMC6460_ENABLE_AUTO_TORQUE_LEARN_ON_INIT
    emit logMessage(QStringLiteral("AUTO_TORQUE_LEARN_ON_INIT: enabled, starting automatic end-to-end torque learn at raw velocity=%1")
                    .arg(DEFAULT_AUTO_TORQUE_LEARN_VELOCITY_RAW));
    QTimer::singleShot(200, this, [this]() {
        startAutoEndToEndCalibration(DEFAULT_AUTO_TORQUE_LEARN_VELOCITY_RAW);
    });
#endif
}

void MotorWorker::readStatus()
{
    ensureInterface();

    if (!tmc->isOpen() || tmc->isBusy())
    {
        return;
    }

    Tmc6460QtInterface::RunStatus status;
    if (tmc->readRunStatus(&status))
    {
        emit errorChanged(QString());
        emit statusReady(status);

        // emit logMessage(QStringLiteral("MONITOR: velRaw=%1 pos=%2 torqueFlux=%3 torqueRaw=%4 status=%5")
        //                 .arg(status.velocityActualRaw)
        //                 .arg(status.positionActual)
        //                 .arg(hex32(status.torqueFluxActual))
        //                 .arg(status.torqueActualRaw)
        //                 .arg(hex32(status.chipStatusFlags)));

#if TMC6460_ENABLE_ACTUATOR_STOP_DETECTION
        checkCalibrationEndStop(status);
#endif
    }
    else
    {
        emit errorChanged(tmc->lastError());
    }
}

void MotorWorker::applyTorque(int value)
{
    Q_UNUSED(value)

#if !TMC6460_ENABLE_TORQUE_MODE
    emit logMessage(QStringLiteral("TORQUE_MODE_DISABLED: torque commands are disabled. This build uses velocity mode only."));
    emit commandDone(QStringLiteral("Torque mode disabled"), false);
    return;
#else
    ensureInterface();

    resetVelocityRampState();
    stopCalibrationEndMonitor(true);
    resetAutoCalibrationState();

    const int limitedTorque = qBound<int>(-MAX_ALLOWED_TORQUE_RAW, value, MAX_ALLOWED_TORQUE_RAW);

    if (limitedTorque != value)
    {
        emit logMessage(QStringLiteral("SAFETY: Torque command %1 clipped to %2")
                        .arg(value)
                        .arg(limitedTorque));
    }

    const bool ok = tmc->isOpen() && tmc->setTorqueTarget(limitedTorque);
    emit commandDone(QStringLiteral("Torque target %1").arg(limitedTorque), ok);
#endif
}



void MotorWorker::applyVelocityLimitRaw(qint32 limitRaw)
{
    ensureInterface();

    const qint32 absLimit = qAbs(limitRaw);
    const qint32 limitedVelocityLimit = qBound<qint32>(1, absLimit, MAX_ALLOWED_VELOCITY_RAW);

    if (limitedVelocityLimit != absLimit)
    {
        emit logMessage(QStringLiteral("SAFETY: Velocity limit %1 clipped to %2")
                        .arg(absLimit)
                        .arg(limitedVelocityLimit));
    }

    if (!tmc->isOpen())
    {
        emit commandDone(QStringLiteral("Velocity limit raw=%1").arg(limitedVelocityLimit), false);
        emit errorChanged(QStringLiteral("Cannot apply velocity limit: serial port is not open"));
        return;
    }

    const bool ok = tmc->setVelocityLimitRaw(limitedVelocityLimit);
    emit commandDone(QStringLiteral("Velocity limit raw=%1").arg(limitedVelocityLimit), ok);

    if (!ok)
    {
        emit errorChanged(QStringLiteral("Velocity limit write failed"));
    }
}

void MotorWorker::applyVelocityTorqueLimitRaw(int torqueLimitRaw)
{
    ensureInterface();

    const int limitedTorqueLimit = qBound<int>(MIN_VELOCITY_TORQUE_LIMIT_RAW, torqueLimitRaw, MAX_ALLOWED_TORQUE_RAW);

    if (limitedTorqueLimit != torqueLimitRaw)
    {
        emit logMessage(QStringLiteral("SAFETY: Velocity torque/flux limit %1 clipped to %2")
                        .arg(torqueLimitRaw)
                        .arg(limitedTorqueLimit));
    }

    if (!tmc->isOpen())
    {
        emit commandDone(QStringLiteral("Velocity torque/flux limit %1").arg(limitedTorqueLimit), false);
        emit errorChanged(QStringLiteral("Cannot apply velocity torque/flux limit: serial port is not open"));
        return;
    }

    const bool ok = tmc->setVelocityTorqueFluxLimit(limitedTorqueLimit);

    emit commandDone(QStringLiteral("Velocity torque/flux limit %1").arg(limitedTorqueLimit), ok);

    if (!ok)
    {
        emit errorChanged(QStringLiteral("Velocity torque/flux limit write failed"));
    }
}


void MotorWorker::readRunStatusSnapshot(const QString &tag)
{
    ensureInterface();

    if (tmc == nullptr || !tmc->isOpen())
    {
        emit logMessage(QStringLiteral("RUN_STATUS[%1]: READ FAILED: serial port is not open")
                        .arg(tag));
        return;
    }

    if (tmc->isBusy())
    {
        emit logMessage(QStringLiteral("RUN_STATUS[%1]: SKIPPED: interface busy")
                        .arg(tag));
        return;
    }

    Tmc6460QtInterface::RunStatus s;
    if (!tmc->readRunStatus(&s))
    {
        const QString err = s.errorText.isEmpty() ? tmc->lastError() : s.errorText;
        emit logMessage(QStringLiteral("RUN_STATUS[%1]: READ FAILED: %2")
                        .arg(tag)
                        .arg(err));
        return;
    }

    emit statusReady(s);

    emit logMessage(QStringLiteral("RUN_STATUS[%1]: POS=0x%2 (%3), VEL_TARGET=%4, VEL_ACTUAL=%5, TORQUE_TARGET=%6, FLUX_TARGET=%7, TORQUE_ACTUAL=%8, FLUX_ACTUAL=%9, RAW_TF_TARGET=0x%10, RAW_TF_ACTUAL=0x%11, STATUS=0x%12, EVENTS=0x%13, MOTION=0x%14, GDRV=0x%15")
                    .arg(tag)
                    .arg(s.positionActual, 8, 16, QLatin1Char('0')).toUpper()
                    .arg(s.positionActualSigned)
                    .arg(s.velocityTargetRaw)
                    .arg(s.velocityActualRaw)
                    .arg(s.torqueTargetRaw)
                    .arg(s.fluxTargetRaw)
                    .arg(s.torqueActualRaw)
                    .arg(s.fluxActualRaw)
                    .arg(s.rawTorqueFluxTarget, 8, 16, QLatin1Char('0')).toUpper()
                    .arg(s.torqueFluxActual, 8, 16, QLatin1Char('0')).toUpper()
                    .arg(s.chipStatusFlags, 8, 16, QLatin1Char('0')).toUpper()
                    .arg(s.chipEvents, 8, 16, QLatin1Char('0')).toUpper()
                    .arg(s.motorMotion, 8, 16, QLatin1Char('0')).toUpper()
                    .arg(s.gdrv, 8, 16, QLatin1Char('0')).toUpper());
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

    /*
     * If we are already holding at one hard end, do not allow the same
     * direction command to push again. This prevents the second CW/CCW click
     * from moving the remaining small angle or overloading the stop.
     * The opposite direction is allowed and clears the hold latch.
     */
    if (rawVelocity != 0 && holdingAtEnd)
    {
        const int requestedDirection = (rawVelocity > 0) ? 1 : -1;
        if (requestedDirection == holdingDirection)
        {
            emit logMessage(QStringLiteral("IGNORED: Already holding at this end. Use opposite direction to move away. direction=%1")
                            .arg(holdingDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE")));
            emit commandDone(QStringLiteral("Velocity target raw=%1 ignored: already at this end").arg(rawVelocity), true);
            return;
        }

        holdingAtEnd = false;
        holdingDirection = 0;
    }

    if (rawVelocity == 0)
    {
        holdingAtEnd = false;
        holdingDirection = 0;
    }

#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (rawVelocity != 0 && !autoCalibrationActive)
    {
        startAutoEndToEndCalibration(rawVelocity);
        return;
    }

    if (rawVelocity == 0)
    {
        resetAutoCalibrationState();
    }
#endif

    stopCalibrationEndMonitor(true);

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
        startCalibrationEndMonitor(rawVelocity);
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
        startCalibrationEndMonitor(currentCommandedVelocityRaw);
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
        stopCalibrationEndMonitor(false);
        emit commandDone(QStringLiteral("Velocity ramp failed at raw=%1").arg(nextRaw), false);
        emit errorChanged(QStringLiteral("Velocity ramp write failed"));
        return;
    }

    currentCommandedVelocityRaw = nextRaw;

    if (currentCommandedVelocityRaw == targetVelocityRaw)
    {
        velocityRampTimer->stop();
        startCalibrationEndMonitor(currentCommandedVelocityRaw);
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


void MotorWorker::applyPositionToEnd(int directionSign)
{
    Q_UNUSED(directionSign)
    emit commandDone(QStringLiteral("Position mode disabled"), false);
    emit errorChanged(QStringLiteral("Position mode is disabled. This build uses velocity mode only."));
}

void MotorWorker::emergencyStop()
{
    ensureInterface();

    resetVelocityRampState();
    stopCalibrationEndMonitor(true);
    resetAutoCalibrationState();

    /*
     * Load-safe E-Stop:
     * Do not clear the hold state and do not disable the gate driver.
     * The actuator must keep producing holding torque against the load.
     */
    emit logMessage(QStringLiteral("SAFE_ESTOP: keeping driver enabled and applying zero-velocity hold. Do not power off until load is mechanically safe."));

    const bool ok = tmc->isOpen() &&
                    tmc->safeLoadStopHold(learnedHoldTorqueFluxLimitRaw);

    if (ok)
    {
        holdingAtEnd = true;
        if (holdingDirection == 0)
            holdingDirection = (calibrationCommandDirection != 0) ? calibrationCommandDirection : 0;
        emit stallStateChanged(true, QStringLiteral("SAFE E-STOP HOLD"));
    }
    else
    {
        emit errorChanged(QStringLiteral("SAFE E-STOP HOLD failed. Mechanically support the load before disabling power."));
    }

    emit commandDone(QStringLiteral("SAFE E-STOP HOLD"), ok);
}

void MotorWorker::shutdown()
{
    resetVelocityRampState();
    stopCalibrationEndMonitor(true);
    resetAutoCalibrationState();

    if (tmc != nullptr)
    {
        if (tmc->isOpen())
        {
            emit logMessage(QStringLiteral("ACTION: GUI closing, safe motor shutdown requested"));
            const bool ok = tmc->shutdownMotorSafe();
            emit commandDone(QStringLiteral("GUI close shutdown"), ok);
        }

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

void MotorWorker::startCalibrationEndMonitor(qint32 rawVelocity)
{
#if TMC6460_ENABLE_ACTUATOR_STOP_DETECTION
    if (qAbs(static_cast<qint64>(rawVelocity)) < CAL_MIN_TARGET_RAW)
    {
        stopCalibrationEndMonitor(true);
        return;
    }

    calibrationMonitorActive = true;
    calibrationStopLatched = false;
    calibrationStartPositionValid = false;
    calibrationDirectionLearned = false;
    calibrationTargetVelocityRaw = rawVelocity;
    calibrationCommandDirection = (rawVelocity > 0) ? 1 : -1;
    calibrationPositionDirection = 0;
    calibrationStartPositionRaw = 0;
    calibrationPreviousPositionRaw = 0;
    calibrationProgressCounts = 0;
    calibrationBlockedSamples = 0;
    resetFinalEndCreepState();
    if (runtimeMoveMode == RuntimeMoveMode::None)
    {
        runtimeMoveMode = RuntimeMoveMode::VelocityToEnd;
    }
    calibrationTimer.restart();

    emit stallStateChanged(false, runtimeMoveMode == RuntimeMoveMode::PositionToEnd
                           ? QStringLiteral("Position run to end monitoring")
                           : QStringLiteral("Velocity run to end monitoring"));
    emit logMessage(QStringLiteral("ACTUATOR_RUN_START: mode=VELOCITY_ONLY, command=%1, targetRaw=%2, detection=NO_CALIBRATED_COUNTS, torqueThresholdRaw=%3")
                    .arg(calibrationCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                    .arg(calibrationTargetVelocityRaw)
                    .arg(TMC6460_HARD_END_TORQUE_RAW_THRESHOLD));
    emit logMessage(QStringLiteral("ACTUATOR_RULE: run until POSITION_ACTUAL stops changing, then final creep, then VELOCITY_TARGET=0 hold"));
#else
    Q_UNUSED(rawVelocity)
#endif
}

void MotorWorker::stopCalibrationEndMonitor(bool clearGuiStatus)
{
    calibrationMonitorActive = false;
    calibrationStopLatched = false;
    calibrationStartPositionValid = false;
    calibrationDirectionLearned = false;
    calibrationTargetVelocityRaw = 0;
    calibrationCommandDirection = 0;
    calibrationPositionDirection = 0;
    calibrationStartPositionRaw = 0;
    calibrationPreviousPositionRaw = 0;
    calibrationProgressCounts = 0;
    calibrationBlockedSamples = 0;
    resetFinalEndCreepState();
    runtimeMoveMode = RuntimeMoveMode::None;

    if (clearGuiStatus)
    {
        emit stallStateChanged(false, QStringLiteral("Not active"));
    }
}

qint64 MotorWorker::signedPositionDelta32(quint32 current, quint32 start)
{
    // Position register is 32-bit and may wrap. Casting the unsigned difference
    // to signed 32-bit gives the shortest signed delta across wrap.
    const quint32 delta = current - start;
    return static_cast<qint32>(delta);
}



qint16 MotorWorker::clampHoldTorqueLimit(qint32 raw) const
{
    return static_cast<qint16>(qBound<qint32>(MIN_VELOCITY_TORQUE_LIMIT_RAW,
                                             qAbs(raw),
                                             MAX_VELOCITY_TORQUE_LIMIT_RAW));
}

qint16 MotorWorker::estimateHoldTorqueLimitFromStatus(const Tmc6460QtInterface::RunStatus &status,
                                                      qint16 movingMaxAbsRaw) const
{
    const qint32 measuredAbs = qMax<qint32>(qAbs(status.torqueActualRaw), qAbs(movingMaxAbsRaw));
    return clampHoldTorqueLimit(measuredAbs + AUTO_TORQUE_HOLD_MARGIN_RAW);
}

void MotorWorker::updateAutoTorqueLearnDuringSample(const Tmc6460QtInterface::RunStatus &status)
{
#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (!autoCalibrationActive)
        return;

    const qint16 torqueAbs = static_cast<qint16>(qAbs(status.torqueActualRaw));

    if (autoCalibrationState == AutoCalibrationState::FirstLeg)
    {
        if (torqueAbs > autoFirstMaxMovingTorqueAbsRaw)
            autoFirstMaxMovingTorqueAbsRaw = torqueAbs;
    }
    else if (autoCalibrationState == AutoCalibrationState::SecondLeg)
    {
        if (torqueAbs > autoSecondMaxMovingTorqueAbsRaw)
            autoSecondMaxMovingTorqueAbsRaw = torqueAbs;
    }
#else
    Q_UNUSED(status)
#endif
}

void MotorWorker::resetAutoCalibrationState()
{
    autoCalibrationActive = false;
    autoCalibrationState = AutoCalibrationState::Idle;
    autoCalibrationFirstVelocityRaw = 0;
    autoCalibrationSecondVelocityRaw = 0;
    autoCalibrationFirstCommandDirection = 0;
    autoCalibrationFirstStartPosition = 0;
    autoCalibrationFirstEndPosition = 0;
    autoCalibrationSecondStartPosition = 0;
    autoCalibrationSecondEndPosition = 0;
    autoCalibrationFirstTravelCounts = 0;
    autoCalibrationSecondTravelCounts = 0;

    autoFirstStartTorqueAbsRaw = 0;
    autoSecondStartTorqueAbsRaw = 0;
    autoFirstMaxMovingTorqueAbsRaw = 0;
    autoSecondMaxMovingTorqueAbsRaw = 0;
    autoFirstEndTorqueAbsRaw = 0;
    autoSecondEndTorqueAbsRaw = 0;
    autoFirstHoldTorqueRequiredRaw = DEFAULT_VELOCITY_TORQUE_LIMIT_RAW;
    autoSecondHoldTorqueRequiredRaw = DEFAULT_VELOCITY_TORQUE_LIMIT_RAW;
}

void MotorWorker::startAutoEndToEndCalibration(qint32 firstVelocityRaw)
{
#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (tmc == nullptr || !tmc->isOpen())
    {
        emit commandDone(QStringLiteral("AUTO_CAL_START"), false);
        emit errorChanged(QStringLiteral("Cannot start auto calibration: serial port is not open"));
        return;
    }

    const qint32 limitedFirstVelocity = qBound<qint32>(-MAX_ALLOWED_VELOCITY_RAW,
                                                       firstVelocityRaw,
                                                       MAX_ALLOWED_VELOCITY_RAW);

    if (qAbs(limitedFirstVelocity) < CAL_MIN_TARGET_RAW)
    {
        emit commandDone(QStringLiteral("AUTO_CAL_START"), false);
        emit errorChanged(QStringLiteral("Auto calibration velocity is too small"));
        return;
    }

    resetVelocityRampState();
    stopCalibrationEndMonitor(true);
    resetAutoCalibrationState();

    autoCalibrationActive = true;
    autoCalibrationState = AutoCalibrationState::FirstLeg;
    autoCalibrationFirstVelocityRaw = limitedFirstVelocity;
    autoCalibrationSecondVelocityRaw = -limitedFirstVelocity;
    autoCalibrationFirstCommandDirection = (limitedFirstVelocity > 0) ? 1 : -1;

    emit stallStateChanged(false, QStringLiteral("AUTO CAL: running first end"));
    emit logMessage(QStringLiteral("AUTO_TORQUE_LEARN_BEGIN: firstCommand=%1, firstVelocityRaw=%2, secondVelocityRaw=%3, speedRaw=4000000, positionMode=DISABLED, torqueMode=DISABLED")
                    .arg(autoCalibrationFirstCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                    .arg(autoCalibrationFirstVelocityRaw)
                    .arg(autoCalibrationSecondVelocityRaw));

    Tmc6460QtInterface::RunStatus before;
    if (tmc->readRunStatus(&before))
    {
        autoCalibrationFirstStartPosition = before.positionActual;
        autoFirstStartTorqueAbsRaw = static_cast<qint16>(qAbs(before.torqueActualRaw));
        autoFirstMaxMovingTorqueAbsRaw = autoFirstStartTorqueAbsRaw;
        emit logMessage(QStringLiteral("AUTO_TORQUE_FIRST_START_SNAPSHOT: phase=RELEASE_OR_DOWN_CANDIDATE, POS=%1 (%2), VEL_ACTUAL=%3, TORQUE=%4, FLUX=%5, STATUS=%6, EVENTS=%7")
                        .arg(hex32(before.positionActual))
                        .arg(before.positionActualSigned)
                        .arg(before.velocityActualRaw)
                        .arg(before.torqueActualRaw)
                        .arg(before.fluxActualRaw)
                        .arg(hex32(before.chipStatusFlags))
                        .arg(hex32(before.chipEvents)));
    }

    const bool ok = writeVelocityRaw(autoCalibrationFirstVelocityRaw);
    if (!ok)
    {
        emit commandDone(QStringLiteral("AUTO_CAL first leg velocity raw=%1").arg(autoCalibrationFirstVelocityRaw), false);
        emit errorChanged(QStringLiteral("Auto calibration first leg velocity write failed"));
        resetAutoCalibrationState();
        return;
    }

    currentCommandedVelocityRaw = autoCalibrationFirstVelocityRaw;
    startCalibrationEndMonitor(autoCalibrationFirstVelocityRaw);
    emit commandDone(QStringLiteral("AUTO_CAL first leg started raw=%1").arg(autoCalibrationFirstVelocityRaw), true);
#else
    Q_UNUSED(firstVelocityRaw)
#endif
}

void MotorWorker::startAutoSecondLeg()
{
#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (!autoCalibrationActive || autoCalibrationState != AutoCalibrationState::WaitingBeforeReverse)
    {
        return;
    }

    if (tmc == nullptr || !tmc->isOpen())
    {
        emit errorChanged(QStringLiteral("Cannot start auto calibration second leg: serial port is not open"));
        resetAutoCalibrationState();
        return;
    }

    autoCalibrationState = AutoCalibrationState::SecondLeg;
    resetVelocityRampState();
    stopCalibrationEndMonitor(false);

    emit stallStateChanged(false, QStringLiteral("AUTO CAL: returning to other end"));
    emit logMessage(QStringLiteral("AUTO_CAL_SECOND_LEG_START: command=%1, velocityRaw=%2")
                    .arg(autoCalibrationSecondVelocityRaw > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                    .arg(autoCalibrationSecondVelocityRaw));

    Tmc6460QtInterface::RunStatus before;
    if (tmc->readRunStatus(&before))
    {
        autoCalibrationSecondStartPosition = before.positionActual;
        autoSecondStartTorqueAbsRaw = static_cast<qint16>(qAbs(before.torqueActualRaw));
        autoSecondMaxMovingTorqueAbsRaw = autoSecondStartTorqueAbsRaw;
        emit logMessage(QStringLiteral("AUTO_TORQUE_SECOND_START_SNAPSHOT: phase=LIFT_OR_RETURN_CANDIDATE, POS=%1 (%2), VEL_ACTUAL=%3, TORQUE=%4, FLUX=%5, STATUS=%6, EVENTS=%7")
                        .arg(hex32(before.positionActual))
                        .arg(before.positionActualSigned)
                        .arg(before.velocityActualRaw)
                        .arg(before.torqueActualRaw)
                        .arg(before.fluxActualRaw)
                        .arg(hex32(before.chipStatusFlags))
                        .arg(hex32(before.chipEvents)));
    }

    const bool ok = writeVelocityRaw(autoCalibrationSecondVelocityRaw);
    if (!ok)
    {
        emit commandDone(QStringLiteral("AUTO_CAL second leg velocity raw=%1").arg(autoCalibrationSecondVelocityRaw), false);
        emit errorChanged(QStringLiteral("Auto calibration second leg velocity write failed"));
        resetAutoCalibrationState();
        return;
    }

    currentCommandedVelocityRaw = autoCalibrationSecondVelocityRaw;
    startCalibrationEndMonitor(autoCalibrationSecondVelocityRaw);
    emit commandDone(QStringLiteral("AUTO_CAL second leg started raw=%1").arg(autoCalibrationSecondVelocityRaw), true);
#endif
}

void MotorWorker::finishAutoEndToEndCalibration()
{
#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    const qint64 endToEndCountsFromEndPositions = qAbs(signedPositionDelta32(autoCalibrationSecondEndPosition,
                                                                             autoCalibrationFirstEndPosition));

    learnedHoldTorqueFluxLimitRaw = clampHoldTorqueLimit(qMax<qint32>(autoFirstHoldTorqueRequiredRaw,
                                                                          autoSecondHoldTorqueRequiredRaw));

    if (tmc != nullptr && tmc->isOpen())
    {
        tmc->setVelocityTorqueFluxLimit(learnedHoldTorqueFluxLimitRaw);
        tmc->safeLoadStopHold(learnedHoldTorqueFluxLimitRaw);
    }

    emit logMessage(QStringLiteral("AUTO_TORQUE_LEARN_COMPLETE: firstCommand=%1, firstStart=%2, firstEnd=%3, secondStart=%4, secondEnd=%5, firstTravel=%6, secondTravel=%7, endToEndCounts=%8")
                    .arg(autoCalibrationFirstCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                    .arg(hex32(autoCalibrationFirstStartPosition))
                    .arg(hex32(autoCalibrationFirstEndPosition))
                    .arg(hex32(autoCalibrationSecondStartPosition))
                    .arg(hex32(autoCalibrationSecondEndPosition))
                    .arg(autoCalibrationFirstTravelCounts)
                    .arg(autoCalibrationSecondTravelCounts)
                    .arg(endToEndCountsFromEndPositions));

    emit logMessage(QStringLiteral("AUTO_CAL_VALUES_TO_SEND: CW_OR_FIRST_END=%1, CCW_OR_SECOND_END=%2, END_TO_END_COUNTS=%3, FIRST_TRAVEL_COUNTS=%4, SECOND_TRAVEL_COUNTS=%5")
                    .arg(hex32(autoCalibrationFirstEndPosition))
                    .arg(hex32(autoCalibrationSecondEndPosition))
                    .arg(endToEndCountsFromEndPositions)
                    .arg(autoCalibrationFirstTravelCounts)
                    .arg(autoCalibrationSecondTravelCounts));

    emit logMessage(QStringLiteral("AUTO_TORQUE_LEARN_VALUES: releaseOrFirstHold=%1, liftOrSecondHold=%2, learnedHoldTorqueFluxLimit=%3, margin=%4")
                    .arg(autoFirstHoldTorqueRequiredRaw)
                    .arg(autoSecondHoldTorqueRequiredRaw)
                    .arg(learnedHoldTorqueFluxLimitRaw)
                    .arg(AUTO_TORQUE_HOLD_MARGIN_RAW));

    emit stallStateChanged(true, QStringLiteral("AUTO CAL COMPLETE"));
    resetAutoCalibrationState();
#endif
}


void MotorWorker::checkCalibrationEndStop(const Tmc6460QtInterface::RunStatus &status)
{
    if (!calibrationMonitorActive || calibrationStopLatched)
    {
        return;
    }

    updateAutoTorqueLearnDuringSample(status);

    if (!calibrationStartPositionValid)
    {
        calibrationStartPositionRaw = status.positionActual;
        calibrationPreviousPositionRaw = status.positionActual;
        calibrationStartPositionValid = true;
        calibrationTimer.restart();

        emit logMessage(QStringLiteral("ACTUATOR_START_POSITION: mode=VELOCITY_ONLY, POS=%1 (%2), commandRaw=%3")
                        .arg(hex32(status.positionActual))
                        .arg(status.positionActualSigned)
                        .arg(calibrationTargetVelocityRaw));
        return;
    }

    const qint64 deltaFromStart = signedPositionDelta32(status.positionActual, calibrationStartPositionRaw);
    const qint64 deltaFromPrevious = signedPositionDelta32(status.positionActual, calibrationPreviousPositionRaw);
    const qint64 absDeltaFromPrevious = qAbs(deltaFromPrevious);

    if (!calibrationDirectionLearned && qAbs(deltaFromStart) >= TMC6460_HARD_END_MIN_TRAVEL_COUNTS)
    {
        calibrationDirectionLearned = true;
        calibrationPositionDirection = (deltaFromStart >= 0) ? 1 : -1;

        emit logMessage(QStringLiteral("ACTUATOR_DIRECTION_LEARNED: command=%1, positionDirection=%2, start=%3, current=%4, delta=%5")
                        .arg(calibrationCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                        .arg(calibrationPositionDirection > 0 ? QStringLiteral("INCREASING") : QStringLiteral("DECREASING"))
                        .arg(hex32(calibrationStartPositionRaw))
                        .arg(hex32(status.positionActual))
                        .arg(deltaFromStart));
    }

    calibrationProgressCounts = qAbs(deltaFromStart);

    const qint64 targetAbs = qAbs(static_cast<qint64>(calibrationTargetVelocityRaw));
    const qint64 actualAbs = qAbs(static_cast<qint64>(status.velocityActualRaw));
    const qint64 lowVelocityLimit = qMax<qint64>(TMC6460_HARD_END_LOW_VELOCITY_RAW,
                                                (targetAbs * TMC6460_HARD_END_LOW_VELOCITY_RATIO_PERCENT) / 100LL);

    const bool startupSettled = calibrationTimer.isValid() &&
                                calibrationTimer.elapsed() >= TMC6460_HARD_END_MIN_RUN_TIME_MS;
    const bool actuatorHasMoved = calibrationProgressCounts >= TMC6460_HARD_END_MIN_TRAVEL_COUNTS;
    const bool positionStopped = absDeltaFromPrevious <= TMC6460_HARD_END_NO_PROGRESS_COUNTS;
    const bool velocityLow = actualAbs <= lowVelocityLimit;
    const bool torqueLoaded = qAbs(status.torqueActualRaw) >= TMC6460_HARD_END_TORQUE_RAW_THRESHOLD;
    const bool possibleHardEnd = startupSettled && actuatorHasMoved && positionStopped && (velocityLow || torqueLoaded);

#if TMC6460_CALIBRATION_LOG_EVERY_SAMPLE
    emit logMessage(QStringLiteral("ACTUATOR_SAMPLE: mode=VELOCITY_ONLY, tMs=%1, POS=%2 (%3), travelFromStart=%4, deltaSample=%5, velCmd=%6, velActual=%7, torque=%8, flux=%9, posStopped=%10, velocityLow=%11, torqueLoaded=%12, blockedSamples=%13")
                    .arg(calibrationTimer.isValid() ? calibrationTimer.elapsed() : 0)
                    .arg(hex32(status.positionActual))
                    .arg(status.positionActualSigned)
                    .arg(calibrationProgressCounts)
                    .arg(deltaFromPrevious)
                    .arg(calibrationTargetVelocityRaw)
                    .arg(status.velocityActualRaw)
                    .arg(status.torqueActualRaw)
                    .arg(status.fluxActualRaw)
                    .arg(positionStopped ? 1 : 0)
                    .arg(velocityLow ? 1 : 0)
                    .arg(torqueLoaded ? 1 : 0)
                    .arg(calibrationBlockedSamples));
#endif

    if (calibrationTimer.isValid() && calibrationTimer.elapsed() >= TMC6460_HARD_END_SAFETY_TIMEOUT_MS)
    {
        handleCalibrationEndStop(QStringLiteral("SAFETY_TIMEOUT"), status, calibrationProgressCounts);
        return;
    }

    if (!startupSettled || !actuatorHasMoved)
    {
        calibrationPreviousPositionRaw = status.positionActual;
        calibrationBlockedSamples = 0;
        return;
    }

#if TMC6460_ENABLE_FINAL_END_CREEP
    /*
     * This build does not use calibrated stroke count. The fast run enters
     * final creep only when the position is already not progressing.
     * If CW was stopping early because of count-based logic, this removes that path.
     */
    if (!finalEndCreepActive)
    {
        if (possibleHardEnd)
        {
            ++calibrationBlockedSamples;

            if (calibrationBlockedSamples >= TMC6460_HARD_END_BLOCKED_SAMPLES)
            {
                finalEndCreepActive = true;
                finalEndCreepTimer.restart();
                finalEndCreepStableSamples = 0;
                finalEndCreepLastPositionRaw = status.positionActual;

                const int creepDirection = (calibrationTargetVelocityRaw >= 0) ? 1 : -1;
                finalEndCreepVelocityRaw = creepDirection * TMC6460_FINAL_END_CREEP_VELOCITY_RAW;

                emit logMessage(QStringLiteral("FINAL_CREEP_START: command=%1, travelFromStart=%2, POS=%3 (%4), creepVelocity=%5, velActual=%6, torque=%7, flux=%8")
                                .arg(calibrationCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                                .arg(calibrationProgressCounts)
                                .arg(hex32(status.positionActual))
                                .arg(status.positionActualSigned)
                                .arg(finalEndCreepVelocityRaw)
                                .arg(status.velocityActualRaw)
                                .arg(status.torqueActualRaw)
                                .arg(status.fluxActualRaw));

                if (tmc != nullptr && tmc->isOpen())
                {
                    tmc->setVelocityTarget(finalEndCreepVelocityRaw);
                }
            }
        }
        else
        {
            calibrationBlockedSamples = 0;
        }

        calibrationPreviousPositionRaw = status.positionActual;
        return;
    }
    else
    {
        const qint64 creepStep = qAbs(signedPositionDelta32(status.positionActual, finalEndCreepLastPositionRaw));

        if (creepStep <= TMC6460_HARD_END_NO_PROGRESS_COUNTS)
        {
            ++finalEndCreepStableSamples;
        }
        else
        {
            finalEndCreepStableSamples = 0;
        }

        finalEndCreepLastPositionRaw = status.positionActual;

        const bool creepStable = finalEndCreepStableSamples >= TMC6460_FINAL_END_CREEP_STABLE_SAMPLES;
        const bool creepTimeout = finalEndCreepTimer.isValid() &&
                                  finalEndCreepTimer.elapsed() >= TMC6460_FINAL_END_CREEP_TIMEOUT_MS;

        emit logMessage(QStringLiteral("FINAL_CREEP_SAMPLE: command=%1, tMs=%2, POS=%3 (%4), travelFromStart=%5, creepStep=%6, stable=%7/%8, velActual=%9, torque=%10, flux=%11")
                        .arg(calibrationCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                        .arg(finalEndCreepTimer.isValid() ? finalEndCreepTimer.elapsed() : 0)
                        .arg(hex32(status.positionActual))
                        .arg(status.positionActualSigned)
                        .arg(calibrationProgressCounts)
                        .arg(creepStep)
                        .arg(finalEndCreepStableSamples)
                        .arg(TMC6460_FINAL_END_CREEP_STABLE_SAMPLES)
                        .arg(status.velocityActualRaw)
                        .arg(status.torqueActualRaw)
                        .arg(status.fluxActualRaw));

        if (creepStable)
        {
            handleCalibrationEndStop(QStringLiteral("HARD_MECHANICAL_END"), status, calibrationProgressCounts);
            return;
        }

        if (creepTimeout)
        {
            /*
             * Timeout during creep means it was still able to move for the creep
             * duration. Do not call this an early normal stop. It is still a hard
             * end/stall condition for safety, but the log tells you to increase
             * creep timeout/torque if you see extra movement after this.
             */
            handleCalibrationEndStop(QStringLiteral("HARD_END_CREEP_TIMEOUT"), status, calibrationProgressCounts);
            return;
        }

        calibrationPreviousPositionRaw = status.positionActual;
        return;
    }
#else
    if (possibleHardEnd)
    {
        ++calibrationBlockedSamples;
        if (calibrationBlockedSamples >= TMC6460_HARD_END_BLOCKED_SAMPLES)
        {
            handleCalibrationEndStop(QStringLiteral("HARD_MECHANICAL_END"), status, calibrationProgressCounts);
            return;
        }
    }
    else
    {
        calibrationBlockedSamples = 0;
    }
#endif

    calibrationPreviousPositionRaw = status.positionActual;
}


void MotorWorker::resetFinalEndCreepState()
{
    finalEndCreepActive = false;
    finalEndCreepTimer.invalidate();
    finalEndCreepVelocityRaw = 0;
    finalEndCreepLastPositionRaw = 0;
    finalEndCreepStableSamples = 0;
}

void MotorWorker::handleCalibrationEndStop(const QString &reason,
                                           const Tmc6460QtInterface::RunStatus &status,
                                           qint64 progressCounts)
{
    calibrationStopLatched = true;
    calibrationMonitorActive = false;
    resetVelocityRampState();

    const int endedCommandDirection = calibrationCommandDirection;
    const quint32 detectedEndPosition = status.positionActual;

#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::FirstLeg)
    {
        autoFirstEndTorqueAbsRaw = static_cast<qint16>(qAbs(status.torqueActualRaw));
        autoFirstHoldTorqueRequiredRaw = estimateHoldTorqueLimitFromStatus(status, autoFirstMaxMovingTorqueAbsRaw);
    }
    else if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::SecondLeg)
    {
        autoSecondEndTorqueAbsRaw = static_cast<qint16>(qAbs(status.torqueActualRaw));
        autoSecondHoldTorqueRequiredRaw = estimateHoldTorqueLimitFromStatus(status, autoSecondMaxMovingTorqueAbsRaw);
    }
#endif

    resetFinalEndCreepState();
    holdingAtEnd = true;
    holdingDirection = (endedCommandDirection >= 0) ? 1 : -1;

    emit logMessage(QStringLiteral("CAL_END_%1: command=%2, startPos=%3, endPos=%4 (%5), travelCounts=%6, velActual=%7, torqueRaw=%8, fluxRaw=%9, status=%10, events=%11")
                    .arg(reason)
                    .arg(endedCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                    .arg(hex32(calibrationStartPositionRaw))
                    .arg(hex32(detectedEndPosition))
                    .arg(status.positionActualSigned)
                    .arg(progressCounts)
                    .arg(status.velocityActualRaw)
                    .arg(status.torqueActualRaw)
                    .arg(status.fluxActualRaw)
                    .arg(hex32(status.chipStatusFlags))
                    .arg(hex32(status.chipEvents)));

#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::FirstLeg)
    {
        autoCalibrationFirstEndPosition = detectedEndPosition;
        autoCalibrationFirstTravelCounts = progressCounts;
    }
    else if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::SecondLeg)
    {
        autoCalibrationSecondEndPosition = detectedEndPosition;
        autoCalibrationSecondTravelCounts = progressCounts;
    }
#endif

    bool ok = false;
    if (tmc != nullptr && tmc->isOpen())
    {
#if TMC6460_USE_VELOCITY_ZERO_HOLD_AFTER_END
        ok = tmc->safeLoadStopHold(learnedHoldTorqueFluxLimitRaw);
#else
        ok = tmc->safeLoadStopHold(learnedHoldTorqueFluxLimitRaw);
#endif
    }

    emit stallStateChanged(true,
                           ok ? QStringLiteral("END/STALL FOUND - velocity zero hold applied")
                              : QStringLiteral("END/STALL FOUND - hold command failed"));

    // Take a final snapshot after the stop sequence so you can send both the
    // detected end position and the settled end position.
    quint32 settledPosition = detectedEndPosition;
    bool settledValid = false;

    if (tmc != nullptr && tmc->isOpen() && !tmc->isBusy())
    {
        Tmc6460QtInterface::RunStatus settled;
        if (tmc->readRunStatus(&settled))
        {
            settledValid = true;
            settledPosition = settled.positionActual;

            emit logMessage(QStringLiteral("CAL_SETTLED_END: command=%1, POS=%2 (%3), VEL_TARGET=%4, VEL_ACTUAL=%5, TORQUE_ACTUAL=%6, FLUX_ACTUAL=%7, STATUS=%8, EVENTS=%9")
                            .arg(endedCommandDirection > 0 ? QStringLiteral("CW_POSITIVE") : QStringLiteral("CCW_NEGATIVE"))
                            .arg(hex32(settled.positionActual))
                            .arg(settled.positionActualSigned)
                            .arg(settled.velocityTargetRaw)
                            .arg(settled.velocityActualRaw)
                            .arg(settled.torqueActualRaw)
                            .arg(settled.fluxActualRaw)
                            .arg(hex32(settled.chipStatusFlags))
                            .arg(hex32(settled.chipEvents)));

#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
            if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::FirstLeg)
            {
                autoFirstHoldTorqueRequiredRaw = estimateHoldTorqueLimitFromStatus(settled, autoFirstMaxMovingTorqueAbsRaw);
                emit logMessage(QStringLiteral("AUTO_TORQUE_FIRST_HOLD_ESTIMATE: phase=RELEASE_OR_DOWN_CANDIDATE, startTorqueAbs=%1, maxMovingTorqueAbs=%2, endTorqueAbs=%3, settledTorqueAbs=%4, requiredHoldTorqueFluxLimit=%5")
                                .arg(autoFirstStartTorqueAbsRaw)
                                .arg(autoFirstMaxMovingTorqueAbsRaw)
                                .arg(autoFirstEndTorqueAbsRaw)
                                .arg(qAbs(settled.torqueActualRaw))
                                .arg(autoFirstHoldTorqueRequiredRaw));
            }
            else if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::SecondLeg)
            {
                autoSecondHoldTorqueRequiredRaw = estimateHoldTorqueLimitFromStatus(settled, autoSecondMaxMovingTorqueAbsRaw);
                emit logMessage(QStringLiteral("AUTO_TORQUE_SECOND_HOLD_ESTIMATE: phase=LIFT_OR_RETURN_CANDIDATE, startTorqueAbs=%1, maxMovingTorqueAbs=%2, endTorqueAbs=%3, settledTorqueAbs=%4, requiredHoldTorqueFluxLimit=%5")
                                .arg(autoSecondStartTorqueAbsRaw)
                                .arg(autoSecondMaxMovingTorqueAbsRaw)
                                .arg(autoSecondEndTorqueAbsRaw)
                                .arg(qAbs(settled.torqueActualRaw))
                                .arg(autoSecondHoldTorqueRequiredRaw));
            }
#endif
        }
    }

#if TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
    if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::FirstLeg)
    {
        if (settledValid)
        {
            autoCalibrationFirstEndPosition = settledPosition;
        }

        autoCalibrationState = AutoCalibrationState::WaitingBeforeReverse;
        emit stallStateChanged(false, QStringLiteral("AUTO CAL: first end found, reversing soon"));
        emit logMessage(QStringLiteral("AUTO_CAL_FIRST_LEG_DONE: firstEnd=%1, firstTravelCounts=%2, reverseDelayMs=%3")
                        .arg(hex32(autoCalibrationFirstEndPosition))
                        .arg(autoCalibrationFirstTravelCounts)
                        .arg(CAL_AUTO_REVERSE_DELAY_MS));

        QTimer::singleShot(CAL_AUTO_REVERSE_DELAY_MS, this, &MotorWorker::startAutoSecondLeg);
        return;
    }

    if (autoCalibrationActive && autoCalibrationState == AutoCalibrationState::SecondLeg)
    {
        if (settledValid)
        {
            autoCalibrationSecondEndPosition = settledPosition;
        }

        autoCalibrationState = AutoCalibrationState::Done;
        emit logMessage(QStringLiteral("AUTO_CAL_SECOND_LEG_DONE: secondEnd=%1, secondTravelCounts=%2")
                        .arg(hex32(autoCalibrationSecondEndPosition))
                        .arg(autoCalibrationSecondTravelCounts));
        finishAutoEndToEndCalibration();
        return;
    }
#endif
}

QString MotorWorker::hex32(quint32 value)
{
    return QStringLiteral("0x%1").arg(value, 8, 16, QLatin1Char('0')).toUpper();
}
