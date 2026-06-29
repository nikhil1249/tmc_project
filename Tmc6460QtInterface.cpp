#include "Tmc6460QtInterface.h"

#include <QByteArray>
#include <QElapsedTimer>
#include <QThread>
#include <QCoreApplication>
#include <QDebug>
#include <QtGlobal>

Tmc6460QtInterface::Tmc6460QtInterface(QObject *parent)
    : QObject(parent)
{
}

Tmc6460QtInterface::~Tmc6460QtInterface()
{
    closePort();
}

bool Tmc6460QtInterface::openPort(const QString &portName, qint32 baudRate)
{
    closePort();

    serialPort.setPortName(portName);
    serialPort.setBaudRate(baudRate);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort.open(QIODevice::ReadWrite))
    {
        setError(QString("Failed to open %1: %2").arg(portName, serialPort.errorString()));
        return false;
    }

    serialPort.setDataTerminalReady(true);
    serialPort.setRequestToSend(false);

    log(QString("Opened %1 at %2 baud").arg(portName).arg(baudRate));

    QThread::msleep(STARTUP_DELAY_MS);
    clearRx();

    return true;
}

void Tmc6460QtInterface::closePort()
{
    if (serialPort.isOpen())
    {
        serialPort.close();
    }
}

bool Tmc6460QtInterface::isOpen() const
{
    return serialPort.isOpen();
}

bool Tmc6460QtInterface::isBusy() const
{
    return busBusy;
}

QString Tmc6460QtInterface::lastError() const
{
    return errorText;
}

bool Tmc6460QtInterface::initializeMotor()
{
    setBusy(true);

    quint32 chipId = 0;
    if (!readChipId(&chipId))
    {
        setBusy(false);
        return false;
    }

    if (chipId != EXPECTED_CHIP_ID)
    {
        setError(QString("Unexpected CHIP_ID %1").arg(hex32(chipId)));
        setBusy(false);
        return false;
    }

    log("UART communication OK");
    log("========== TMC6460 CONFIG WRITE START ==========");

    const QVector<RegisterValue> table = configurationTable();
    for (const RegisterValue &entry : table)
    {
        if (!writeRegisterChecked(entry.address, entry.value, entry.name, true))
        {
            setBusy(false);
            return false;
        }
        QThread::msleep(WRITE_SETTLE_MS);
    }

    log("=========== TMC6460 CONFIG WRITE END ===========");

    if (!setDriverEnable(false))
    {
        setBusy(false);
        return false;
    }

    velocityModePrepared = false;
    torqueModePrepared = false;
    lastVelocityCommand = 0;
    estopActive = false;

    log("Configuration completed");
    readConfiguredRegisters(nullptr);

    setBusy(false);
    return true;
}

bool Tmc6460QtInterface::readConfiguredRegisters(QVector<RegisterValue> *valuesOut)
{
    log("========== TMC6460 CONFIG READBACK START ==========");

    QVector<RegisterValue> values;
    const QVector<RegisterValue> table = readbackTable();

    for (RegisterValue entry : table)
    {
        quint32 value = 0;
        if (!readRegister(entry.address, &value))
        {
            log(QString("READ  %1 [%2] = FAILED")
                    .arg(QString::fromLatin1(entry.name).leftJustified(32, QLatin1Char(' ')))
                    .arg(hex16(entry.address)));
            continue;
        }

        entry.value = value;
        values.append(entry);
        log(QString("READ  %1 [%2] = %3")
                .arg(QString::fromLatin1(entry.name).leftJustified(32, QLatin1Char(' ')))
                .arg(hex16(entry.address))
                .arg(hex32(value)));
    }

    if (valuesOut != nullptr)
    {
        *valuesOut = values;
    }

    log("=========== TMC6460 CONFIG READBACK END ===========");
    return true;
}

bool Tmc6460QtInterface::readChipId(quint32 *chipId)
{
    if (chipId == nullptr)
    {
        setError("readChipId called with null pointer");
        return false;
    }

    if (!readRegister(REG_CHIP_ID, chipId))
    {
        setError("Read CHIP_ID failed");
        return false;
    }

    log(QString("CHIP_ID = %1").arg(hex32(*chipId)));
    return true;
}

bool Tmc6460QtInterface::readChipStatusFlags(quint32 *statusFlags)
{
    if (statusFlags == nullptr)
    {
        setError("readChipStatusFlags called with null pointer");
        return false;
    }

    return readRegister(REG_CHIP_STATUS_FLAGS, statusFlags);
}

bool Tmc6460QtInterface::readRunStatus(RunStatus *status)
{
    if (status == nullptr)
    {
        setError("readRunStatus called with null pointer");
        return false;
    }

    RunStatus result;

    if (!isOpen())
    {
        result.errorText = QStringLiteral("Serial port is not open");
        *status = result;
        setError(result.errorText);
        return false;
    }

    quint32 value = 0;

    if (!readRegister(REG_CHIP_STATUS_FLAGS, &value))
    {
        result.errorText = QStringLiteral("Read CHIP_STATUS_FLAGS failed");
        *status = result;
        return false;
    }
    result.chipStatusFlags = value;

    if (!readRegister(REG_CHIP_EVENTS, &value))
    {
        result.errorText = QStringLiteral("Read CHIP_EVENTS failed");
        *status = result;
        return false;
    }
    result.chipEvents = value;

    if (!readRegister(REG_MCC_CONFIG_MOTOR_MOTION, &value))
    {
        result.errorText = QStringLiteral("Read MCC_CONFIG_MOTOR_MOTION failed");
        *status = result;
        return false;
    }
    result.motorMotion = value;

    if (!readRegister(REG_MCC_CONFIG_GDRV, &value))
    {
        result.errorText = QStringLiteral("Read MCC_CONFIG_GDRV failed");
        *status = result;
        return false;
    }
    result.gdrv = value;

    if (!readRegister(REG_FOC_PID_VELOCITY_TARGET, &value))
    {
        result.errorText = QStringLiteral("Read VELOCITY_TARGET failed");
        *status = result;
        return false;
    }
    result.rawVelocityTarget = value;
    result.velocityTargetRaw = static_cast<qint32>(value);

    if (!readRegister(REG_FOC_PID_VELOCITY_ACTUAL, &value))
    {
        result.errorText = QStringLiteral("Read VELOCITY_ACTUAL failed");
        *status = result;
        return false;
    }
    result.rawVelocityActual = value;
    result.velocityActualRaw = static_cast<qint32>(value);

    if (!readRegister(REG_FOC_PID_POSITION_ACTUAL, &value))
    {
        result.errorText = QStringLiteral("Read POSITION_ACTUAL failed");
        *status = result;
        return false;
    }
    result.positionActual = value;
    result.positionActualSigned = static_cast<qint32>(value);

    if (!readRegister(REG_FOC_PID_TORQUE_FLUX_TARGET, &value))
    {
        result.errorText = QStringLiteral("Read TORQUE_FLUX_TARGET failed");
        *status = result;
        return false;
    }
    result.rawTorqueFluxTarget = value;
    result.fluxTargetRaw = lowSigned16(value);
    result.torqueTargetRaw = highSigned16(value);

    if (!readRegister(REG_FOC_PID_TORQUE_FLUX_ACTUAL, &value))
    {
        result.errorText = QStringLiteral("Read TORQUE_FLUX_ACTUAL failed");
        *status = result;
        return false;
    }
    result.torqueFluxActual = value;
    result.fluxActualRaw = lowSigned16(value);
    result.torqueActualRaw = highSigned16(value);
    result.torqueCurrentMilliAmp = torqueRawToMilliAmp(result.torqueActualRaw);

    // Phase ADC values are useful while tuning current/torque limits.
    // They are allowed to fail without invalidating the whole run-status snapshot.
    if (readRegister(REG_MCC_ADC_IW_IU, &value))
    {
        result.phaseCurrentUraw = value & 0xFFFFUL;
        result.phaseCurrentWraw = (value >> 16) & 0xFFFFUL;
    }

    if (readRegister(REG_MCC_ADC_IV, &value))
    {
        result.phaseCurrentVraw = value & 0xFFFFUL;
    }

    result.valid = true;
    *status = result;
    return true;
}

bool Tmc6460QtInterface::setVelocityTarget(qint32 targetVelocity)
{
    setBusy(true);

    targetVelocity = qBound<qint32>(-MAX_ALLOWED_VELOCITY_RAW,
                                    targetVelocity,
                                    MAX_ALLOWED_VELOCITY_RAW);

    bool ok = false;

    // Loaded-actuator safety:
    // Velocity 0 means active safe hold, not idle/freewheel.
    // Do not call applyIdleStopSequence() here because that path can remove
    // active holding torque. Keep velocity mode active, keep GDRV enabled,
    // keep torque/flux limit non-zero, and command VELOCITY_TARGET = 0.
    if (targetVelocity == 0)
    {
        ok = safeLoadStopHold(currentVelocityTorqueFluxLimitRaw);
        log("ACTION: Velocity target raw=0 applied using SAFE_LOAD_STOP_HOLD");
        setBusy(false);
        return ok;
    }

    // Always refresh velocity limit before a non-zero command.
    // This value follows the GUI Min/Max velocity range through setVelocityLimitRaw().
    ok = writeRegisterChecked(REG_FOC_PID_VELOCITY_LIMIT,
                              static_cast<quint32>(currentVelocityLimitRaw),
                              "FOC_PID_VELOCITY_LIMIT RUN",
                              false) &&
         prepareVelocityModeForRun() &&
         writeVelocityTargetImmediate(targetVelocity, "FOC_PID_VELOCITY_TARGET");

    if (ok)
    {
        log(QString("ACTION: Velocity target raw=%1 written directly").arg(targetVelocity));
        debugReadRunRegistersOnce();
    }

    setBusy(false);
    return ok;
}


bool Tmc6460QtInterface::prepareVelocityModeForRun()
{
    if (estopActive || !driverEnabled)
    {
        if (!setDriverEnable(false))
        {
            return false;
        }

        QThread::msleep(DRIVER_TOGGLE_DELAY_MS);

        if (!setDriverEnable(true))
        {
            return false;
        }

        if (!waitForChipEvent(DRIVER_READY_EVENT_MASK, DRIVER_READY_TIMEOUT_MS))
        {
            log("WARNING: GDRV_ON_EVENT not seen after driver enable");
        }

        QThread::msleep(30);
        estopActive = false;
        velocityModePrepared = false;
        torqueModePrepared = false;
        positionModePrepared = false;
    }

    if (!velocityModePrepared)
    {
        // TMCL wizard-derived velocity mode value.
        // Do not clear or write FOC_PID_TORQUE_FLUX_TARGET here.
        // In this tuned setup, velocity mode runs by switching mode and
        // then writing only FOC_PID_VELOCITY_TARGET [0x0150].
        if (!writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                                  MOTOR_MOTION_VELOCITY_VALUE,
                                  "MCC_CONFIG_MOTOR_MOTION VELOCITY",
                                  false))
        {
            return false;
        }

        velocityModePrepared = true;
        torqueModePrepared = false;
        positionModePrepared = false;
        log(QString("MODE: Velocity, MCC_CONFIG_MOTOR_MOTION=%1")
                .arg(hex32(MOTOR_MOTION_VELOCITY_VALUE)));
    }

    return true;
}

bool Tmc6460QtInterface::prepareTorqueModeForRun()
{
    if (estopActive || !driverEnabled)
    {
        if (!setDriverEnable(false))
        {
            return false;
        }

        QThread::msleep(DRIVER_TOGGLE_DELAY_MS);

        if (!setDriverEnable(true))
        {
            return false;
        }

        if (!waitForChipEvent(DRIVER_READY_EVENT_MASK, DRIVER_READY_TIMEOUT_MS))
        {
            log("WARNING: GDRV_ON_EVENT not seen after driver enable");
        }

        QThread::msleep(30);
        estopActive = false;
        velocityModePrepared = false;
        torqueModePrepared = false;
        positionModePrepared = false;
    }

    if (!torqueModePrepared)
    {
        // TMCL wizard-derived torque mode value.
        // Do not clear FOC_PID_TORQUE_FLUX_TARGET here. Torque mode runs by
        // switching mode and then writing torque in upper 16 bits of [0x014E].
        if (!writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                                  MOTOR_MOTION_TORQUE_VALUE,
                                  "MCC_CONFIG_MOTOR_MOTION TORQUE",
                                  false))
        {
            return false;
        }

        torqueModePrepared = true;
        velocityModePrepared = false;
        positionModePrepared = false;
        lastVelocityCommand = 0;
        log(QString("MODE: Torque, MCC_CONFIG_MOTOR_MOTION=%1")
                .arg(hex32(MOTOR_MOTION_TORQUE_VALUE)));
    }

    return true;
}


bool Tmc6460QtInterface::preparePositionModeForRun()
{
    if (estopActive || !driverEnabled)
    {
        if (!setDriverEnable(false))
        {
            return false;
        }

        QThread::msleep(DRIVER_TOGGLE_DELAY_MS);

        if (!setDriverEnable(true))
        {
            return false;
        }

        if (!waitForChipEvent(DRIVER_READY_EVENT_MASK, DRIVER_READY_TIMEOUT_MS))
        {
            log("WARNING: GDRV_ON_EVENT not seen after driver enable");
        }

        QThread::msleep(30);
        estopActive = false;
        velocityModePrepared = false;
        torqueModePrepared = false;
        positionModePrepared = false;
    }

    if (!positionModePrepared)
    {
        // Basic position-loop setup. Velocity limit and torque/flux limit are already
        // controlled from the GUI. Position P/I can be tuned later after this test.
        // POSITION_COEFF: upper 16 bits = P, lower 16 bits = I.
        writeRegisterChecked(REG_FOC_PID_POSITION_COEFF,
                             0x00500000UL,
                             "FOC_PID_POSITION_COEFF POSITION_TEST",
                             false);
        writeRegisterChecked(REG_FOC_PID_POSITION_TOLERANCE,
                             0x00001000UL,
                             "FOC_PID_POSITION_TOLERANCE POSITION_TEST",
                             false);
        writeRegisterChecked(REG_FOC_PID_POSITION_TOLERANCE_DELAY,
                             0x00000008UL,
                             "FOC_PID_POSITION_TOLERANCE_DELAY POSITION_TEST",
                             false);

        if (!writeRegisterChecked(REG_FOC_PID_VELOCITY_LIMIT,
                                  static_cast<quint32>(currentVelocityLimitRaw),
                                  "FOC_PID_VELOCITY_LIMIT POSITION_RUN",
                                  false))
        {
            return false;
        }

        if (!writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                                  MOTOR_MOTION_POSITION_VALUE,
                                  "MCC_CONFIG_MOTOR_MOTION POSITION",
                                  false))
        {
            return false;
        }

        positionModePrepared = true;
        velocityModePrepared = false;
        torqueModePrepared = false;
        lastVelocityCommand = 0;
        log(QString("MODE: Position, MCC_CONFIG_MOTOR_MOTION=%1, velocityLimit=%2")
                .arg(hex32(MOTOR_MOTION_POSITION_VALUE))
                .arg(currentVelocityLimitRaw));
    }

    return true;
}

bool Tmc6460QtInterface::writeVelocityTargetImmediate(qint32 targetVelocity, const char *name)
{
    const quint32 rawValue = static_cast<quint32>(targetVelocity);
    if (!writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET, rawValue, name))
    {
        return false;
    }

    lastVelocityCommand = targetVelocity;
    return true;
}

bool Tmc6460QtInterface::applyVelocityWithSafeReverseRamp(qint32 targetVelocity)
{
    if (!prepareVelocityModeForRun())
    {
        return false;
    }

    // Use the last command as the base. If actual velocity is available and larger,
    // use it because it better represents motor inertia during reversal.
    qint32 startVelocity = lastVelocityCommand;
    quint32 actualRaw = 0;
    if (readRegister(REG_FOC_PID_VELOCITY_ACTUAL, &actualRaw))
    {
        const qint32 actualVelocity = static_cast<qint32>(actualRaw);
        if (qAbs(actualVelocity) > qAbs(startVelocity))
        {
            startVelocity = actualVelocity;
        }
    }

    const bool reverseRequested =
        (signOf(startVelocity) != 0) &&
        (signOf(targetVelocity) != 0) &&
        (signOf(startVelocity) != signOf(targetVelocity)) &&
        (qAbs(startVelocity) > SAFE_REVERSE_DEADBAND);

    if (!reverseRequested)
    {
        // Normal speed change or soft stop. For target zero, ramp down instead of hard zero.
        if (targetVelocity == 0 && qAbs(startVelocity) > SAFE_REVERSE_DEADBAND)
        {
            log(QString("Soft stop ramp: %1 -> 0").arg(startVelocity));
            qint32 value = startVelocity;
            while (value != 0)
            {
                value = (value > 0)
                    ? qMax<qint32>(0, value - SAFE_REVERSE_RAMP_STEP)
                    : qMin<qint32>(0, value + SAFE_REVERSE_RAMP_STEP);

                if (!writeVelocityTargetImmediate(value, "FOC_PID_VELOCITY_TARGET SOFT_STOP"))
                {
                    return false;
                }

                QThread::msleep(SAFE_REVERSE_RAMP_DELAY_MS);
            }
            return true;
        }

        return writeVelocityTargetImmediate(targetVelocity, "FOC_PID_VELOCITY_TARGET");
    }

    log(QString("Safe reverse ramp: %1 -> 0 -> %2").arg(startVelocity).arg(targetVelocity));

    // Step 1: decelerate to zero in the current direction.
    qint32 value = startVelocity;
    while (value != 0)
    {
        value = (value > 0)
            ? qMax<qint32>(0, value - SAFE_REVERSE_RAMP_STEP)
            : qMin<qint32>(0, value + SAFE_REVERSE_RAMP_STEP);

        if (!writeVelocityTargetImmediate(value, "FOC_PID_VELOCITY_TARGET RAMP_TO_ZERO"))
        {
            return false;
        }

        QThread::msleep(SAFE_REVERSE_RAMP_DELAY_MS);
    }

    // Step 2: let the mechanical system unload briefly at zero speed.
    QThread::msleep(SAFE_REVERSE_ZERO_DWELL_MS);

    // Step 3: ramp into the requested opposite direction.
    value = 0;
    while (value != targetVelocity)
    {
        value = (targetVelocity > 0)
            ? qMin<qint32>(targetVelocity, value + SAFE_REVERSE_RAMP_STEP)
            : qMax<qint32>(targetVelocity, value - SAFE_REVERSE_RAMP_STEP);

        if (!writeVelocityTargetImmediate(value, "FOC_PID_VELOCITY_TARGET RAMP_TO_TARGET"))
        {
            return false;
        }

        QThread::msleep(SAFE_REVERSE_RAMP_DELAY_MS);
    }

    return true;
}

void Tmc6460QtInterface::debugReadRunRegistersOnce()
{
    quint32 value = 0;

    if (readRegister(REG_MCC_CONFIG_GDRV, &value))
        log(QString("DEBUG GDRV [0x0101] = %1").arg(hex32(value)));

    if (readRegister(REG_MCC_CONFIG_MOTOR_MOTION, &value))
        log(QString("DEBUG MOTOR_MOTION [0x0100] = %1").arg(hex32(value)));

    if (readRegister(REG_FOC_PID_VELOCITY_TARGET, &value))
        log(QString("DEBUG VELOCITY_TARGET [0x0150] = %1").arg(hex32(value)));

    if (readRegister(REG_FOC_PID_VELOCITY_ACTUAL, &value))
        log(QString("DEBUG VELOCITY_ACTUAL [%1] = %2").arg(hex16(REG_FOC_PID_VELOCITY_ACTUAL)).arg(hex32(value)));

    if (readRegister(REG_FOC_PID_POSITION_ACTUAL, &value))
        log(QString("DEBUG POSITION_ACTUAL [%1] = %2").arg(hex16(REG_FOC_PID_POSITION_ACTUAL)).arg(hex32(value)));

    if (readRegister(REG_FOC_PID_TORQUE_FLUX_ACTUAL, &value))
        log(QString("DEBUG TORQUE_FLUX_ACTUAL [%1] = %2").arg(hex16(REG_FOC_PID_TORQUE_FLUX_ACTUAL)).arg(hex32(value)));

    if (readRegister(REG_CHIP_STATUS_FLAGS, &value))
        log(QString("DEBUG CHIP_STATUS_FLAGS [%1] = %2").arg(hex16(REG_CHIP_STATUS_FLAGS)).arg(hex32(value)));

    if (readRegister(REG_CHIP_EVENTS, &value))
        log(QString("DEBUG CHIP_EVENTS [%1] = %2").arg(hex16(REG_CHIP_EVENTS)).arg(hex32(value)));
}

quint32 Tmc6460QtInterface::makeTorqueFluxTarget(qint16 torqueRaw, qint16 fluxRaw)
{
    return (static_cast<quint32>(static_cast<quint16>(torqueRaw)) << 16) |
           static_cast<quint16>(fluxRaw);
}

int Tmc6460QtInterface::signOf(qint32 value)
{
    if (value > 0)
    {
        return 1;
    }
    if (value < 0)
    {
        return -1;
    }
    return 0;
}


quint32 Tmc6460QtInterface::makeTorqueFluxLimit(qint16 torqueLimitRaw, qint16 fluxLimitRaw)
{
    return (static_cast<quint32>(static_cast<quint16>(torqueLimitRaw)) << 16) |
           static_cast<quint16>(fluxLimitRaw);
}


bool Tmc6460QtInterface::setVelocityLimitRaw(qint32 limitRaw)
{
    setBusy(true);

    qint32 requestedLimit = qAbs(limitRaw);
    requestedLimit = qBound<qint32>(1, requestedLimit, MAX_ALLOWED_VELOCITY_RAW);

    const bool ok = writeRegisterChecked(REG_FOC_PID_VELOCITY_LIMIT,
                                         static_cast<quint32>(requestedLimit),
                                         "FOC_PID_VELOCITY_LIMIT GUI_RANGE",
                                         false);

    if (ok)
    {
        currentVelocityLimitRaw = requestedLimit;
        log(QString("ACTION: Velocity limit register updated raw=%1, register=%2")
                .arg(currentVelocityLimitRaw)
                .arg(hex32(static_cast<quint32>(currentVelocityLimitRaw))));
    }

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::setVelocityTorqueLimit(qint32 torqueLimitRaw)
{
    return setVelocityTorqueFluxLimit(torqueLimitRaw);
}

bool Tmc6460QtInterface::setVelocityTorqueFluxLimit(qint32 torqueLimitRaw)
{
    setBusy(true);

    const qint16 limitedTorque = static_cast<qint16>(qBound<qint32>(MIN_VELOCITY_TORQUE_LIMIT_RAW,
                                                                    torqueLimitRaw,
                                                                    MAX_VELOCITY_TORQUE_LIMIT_RAW));

    const quint32 limitRegisterValue = makeTorqueFluxLimit(limitedTorque, limitedTorque);

    // Velocity mode torque adjustment: only update the torque/flux limits.
    // Do not change motor mode and do not write FOC_PID_TORQUE_FLUX_TARGET.
    const bool ok = writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_LIMITS,
                                         limitRegisterValue,
                                         "FOC_PID_TORQUE_FLUX_LIMITS TORQUE_FLUX_LIMIT",
                                         false);

    if (ok)
    {
        currentVelocityTorqueFluxLimitRaw = limitedTorque;
        log(QString("ACTION: Velocity-mode torque/flux limit: torque limit=%1, flux limit=%2, register=%3")
                .arg(limitedTorque)
                .arg(limitedTorque)
                .arg(hex32(limitRegisterValue)));
    }

    setBusy(false);
    return ok;
}


bool Tmc6460QtInterface::setPositionTarget(qint32 targetPosition)
{
    setBusy(true);

    const bool ok = preparePositionModeForRun() &&
                    writeRegisterChecked(REG_FOC_PID_POSITION_TARGET,
                                         static_cast<quint32>(targetPosition),
                                         "FOC_PID_POSITION_TARGET",
                                         false);

    if (ok)
    {
        log(QString("ACTION: Position target=%1, register=%2")
                .arg(targetPosition)
                .arg(hex32(static_cast<quint32>(targetPosition))));
        debugReadRunRegistersOnce();
    }

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::setPositionTargetRelative(qint32 deltaCounts)
{
    RunStatus s;
    if (!readRunStatus(&s))
    {
        log(QString("ERROR: Cannot read current position for relative position move: %1")
                .arg(s.errorText));
        return false;
    }

    const qint32 target = static_cast<qint32>(s.positionActual + static_cast<quint32>(deltaCounts));
    log(QString("ACTION: Relative position move current=%1 (%2), delta=%3, target=%4")
            .arg(hex32(s.positionActual))
            .arg(s.positionActualSigned)
            .arg(deltaCounts)
            .arg(hex32(static_cast<quint32>(target))));

    return setPositionTarget(target);
}

bool Tmc6460QtInterface::holdPositionAtActual(const char *reason)
{
    setBusy(true);

    RunStatus s;
    bool ok = readRunStatus(&s);
    if (ok)
    {
        // Keep position loop active and command the currently measured position.
        // This gives a controlled hold with torque/flux limited by the GUI limit register.
        ok = preparePositionModeForRun() &&
             writeRegisterChecked(REG_FOC_PID_POSITION_TARGET,
                                  s.positionActual,
                                  "FOC_PID_POSITION_TARGET HOLD_ACTUAL",
                                  false) &&
             writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET,
                                  0x00000000UL,
                                  "FOC_PID_VELOCITY_TARGET ZERO_FOR_POSITION_HOLD",
                                  false);
    }

    if (ok)
    {
        log(QString("HOLD: %1 position hold applied at POS=%2")
                .arg(QString::fromLatin1(reason))
                .arg(hex32(s.positionActual)));
    }
    else
    {
        log(QString("ERROR: %1 position hold failed").arg(QString::fromLatin1(reason)));
    }

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::holdVelocityZeroAtActualEnd(const char *reason)
{
    setBusy(true);

    RunStatus s;
    const bool statusOk = readRunStatus(&s);
    if (statusOk)
    {
        log(QString("HOLD_CAPTURE: %1 detected position POS=%2 (%3), velActual=%4, torque=%5, flux=%6")
                .arg(QString::fromLatin1(reason))
                .arg(hex32(s.positionActual))
                .arg(s.positionActualSigned)
                .arg(s.velocityActualRaw)
                .arg(s.torqueActualRaw)
                .arg(s.fluxActualRaw));
    }
    else
    {
        log(QString("WARNING: %1 could not read position before velocity-zero hold: %2")
                .arg(QString::fromLatin1(reason))
                .arg(s.errorText));
    }

    bool ok = true;

    if (!driverEnabled)
    {
        ok &= setDriverEnable(true);
        QThread::msleep(30);
    }

    /*
     * Velocity-mode hold method:
     * Do not use FOC_PID_POSITION_TARGET.
     * Keep the FOC velocity loop active and command zero velocity.
     * The available holding force is limited by FOC_PID_TORQUE_FLUX_LIMITS,
     * which is set from the Velocity Torque/Flux Limit GUI field.
     *
     * This is not a true position servo. It is a zero-speed hold.
     * It is the best method when position target must not be used.
     */
    ok &= prepareVelocityModeForRun();
    ok &= writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET,
                               0x00000000UL,
                               "FOC_PID_VELOCITY_TARGET ZERO_VELOCITY_HOLD",
                               false);

    lastVelocityCommand = 0;

    if (ok)
    {
        velocityModePrepared = true;
        torqueModePrepared = false;
        positionModePrepared = false;

        log(QString("HOLD: %1 velocity-zero hold applied, no position target used")
                .arg(QString::fromLatin1(reason)));
    }
    else
    {
        log(QString("ERROR: %1 velocity-zero hold failed")
                .arg(QString::fromLatin1(reason)));
    }

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::setTorqueTarget(qint32 targetTorque)
{
    setBusy(true);

    const qint16 torqueRaw = static_cast<qint16>(qBound<qint32>(-MAX_ALLOWED_TORQUE_RAW,
                                                               targetTorque,
                                                               MAX_ALLOWED_TORQUE_RAW));
    if (torqueRaw == 0)
    {
        const bool ok = safeLoadStopHold(currentVelocityTorqueFluxLimitRaw);
        log("ACTION: Torque target raw=0 redirected to SAFE_LOAD_STOP_HOLD");
        setBusy(false);
        return ok;
    }

    const quint32 targetRegisterValue = makeTorqueFluxTarget(torqueRaw, DEFAULT_VELOCITY_TORQUE_LIMIT_RAW);

    // Keep flux target non-zero; zero-torque commands are redirected to safe hold above.
    const bool ok = prepareTorqueModeForRun() &&
                    writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_TARGET,
                                         targetRegisterValue,
                                         "FOC_PID_TORQUE_FLUX_TARGET",
                                         false);

    if (ok)
    {
        log(QString("ACTION: Torque target=%1, register=%2")
                .arg(torqueRaw)
                .arg(hex32(targetRegisterValue)));
    }

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::holdAfterStall()
{
    setBusy(true);

    const bool ok = applyIdleStopSequence("STALL_STOP");

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::applyIdleStopSequence(const char *reason)
{
    /*
     * Loaded-actuator safety:
     * The previous idle-stop sequence switched to torque/idle and wrote a zero
     * torque/flux target. That is blocked in this build because it can remove
     * holding torque and allow the load to back-drive the actuator.
     * Any legacy caller is redirected to the same safe hold used by E-STOP.
     */
    log(QString("BLOCKED: applyIdleStopSequence(%1) redirected to SAFE_LOAD_STOP_HOLD")
            .arg(QString::fromLatin1(reason)));

    return safeLoadStopHold(currentVelocityTorqueFluxLimitRaw);
}

bool Tmc6460QtInterface::shutdownMotorSafe()
{
    /*
     * Load-safe GUI close / shutdown.
     * Do not disable the gate driver here. With a suspended load, disabling
     * the driver removes holding torque and can back-drive/break the actuator.
     *
     * This leaves the controller in velocity mode with VELOCITY_TARGET = 0
     * and a non-zero torque/flux limit so the load remains held while power
     * is still available.
     */
    log("SHUTDOWN: GUI close requested. Applying load-safe zero-velocity hold; driver remains enabled.");
    return safeLoadStopHold(currentVelocityTorqueFluxLimitRaw);
}

bool Tmc6460QtInterface::safeLoadStopHold(qint32 holdTorqueFluxLimitRaw)
{
    setBusy(true);

    /*
     * SAFE LOAD STOP / E-STOP HOLD
     *
     * For a loaded actuator, never disable GDRV here. Disabling the driver or
     * switching to PWM_ON/IDLE removes holding torque and allows the load to
     * back-drive the gearbox.
     *
     * This function keeps the gate driver enabled, keeps velocity mode active,
     * refreshes the torque/flux limit, and commands zero velocity.
     */
    qint32 requestedLimit = qAbs(holdTorqueFluxLimitRaw);

    if (requestedLimit <= 0)
        requestedLimit = qAbs(currentVelocityTorqueFluxLimitRaw);

    if (requestedLimit <= 0)
        requestedLimit = DEFAULT_VELOCITY_TORQUE_LIMIT_RAW;

    requestedLimit = qBound<qint32>(MIN_VELOCITY_TORQUE_LIMIT_RAW,
                                    requestedLimit,
                                    MAX_VELOCITY_TORQUE_LIMIT_RAW);

    const qint16 limitRaw = static_cast<qint16>(requestedLimit);
    const quint32 limitRegisterValue = makeTorqueFluxLimit(limitRaw, limitRaw);

    bool ok = true;

    log(QString("SAFE_ESTOP_HOLD: keeping driver enabled, velocity zero hold, torque/flux limit=%1")
            .arg(limitRaw));

    if (!driverEnabled)
    {
        ok &= setDriverEnable(true);
        QThread::msleep(30);
    }

    ok &= writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_LIMITS,
                               limitRegisterValue,
                               "FOC_PID_TORQUE_FLUX_LIMITS SAFE_ESTOP_HOLD",
                               false);

    ok &= writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                               MOTOR_MOTION_VELOCITY_VALUE,
                               "MCC_CONFIG_MOTOR_MOTION VELOCITY_SAFE_ESTOP_HOLD",
                               false);

    ok &= writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET,
                               0x00000000UL,
                               "FOC_PID_VELOCITY_TARGET SAFE_ESTOP_HOLD",
                               false);

    if (ok)
    {
        lastVelocityCommand = 0;
        currentVelocityTorqueFluxLimitRaw = limitRaw;
        velocityModePrepared = true;
        torqueModePrepared = false;
        positionModePrepared = false;
        estopActive = false;
        driverEnabled = true;
        log("SAFE_ESTOP_HOLD: applied. Load is held by zero-speed velocity loop. Do not power off until load is mechanically safe.");
    }
    else
    {
        log("ERROR: SAFE_ESTOP_HOLD failed. Mechanically support the load before disabling power.");
    }

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::emergencyStop()
{
    /*
     * IMPORTANT: GUI E-Stop is now load-safe.
     * It intentionally does NOT disable the gate driver.
     */
    return safeLoadStopHold(currentVelocityTorqueFluxLimitRaw);
}

bool Tmc6460QtInterface::hardDisableDriverForUnloadedTestOnly()
{
    setBusy(true);

    log("HARD_DISABLE_UNLOADED_ONLY: this removes holding torque. Use only when load is mechanically supported or removed.");

    bool ok = true;
    ok &= writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET, 0, "FOC_PID_VELOCITY_TARGET HARD_DISABLE", false);
    QThread::msleep(50);
    ok &= setDriverEnable(false);

    if (ok)
    {
        lastVelocityCommand = 0;
        velocityModePrepared = false;
        torqueModePrepared = false;
        positionModePrepared = false;
        driverEnabled = false;
        estopActive = true;
    }

    setBusy(false);
    return ok;
}

void Tmc6460QtInterface::setBusy(bool busy)
{
    busBusy = busy;
}

void Tmc6460QtInterface::setError(const QString &message)
{
    errorText = message;
    qWarning().noquote() << message;
    emit errorChanged(message);
}

void Tmc6460QtInterface::log(const QString &message)
{
    qDebug().noquote() << message;
    emit logMessage(message);
}

void Tmc6460QtInterface::clearRx()
{
    if (!serialPort.isOpen())
    {
        return;
    }

    serialPort.clear(QSerialPort::Input);

    QElapsedTimer timer;
    timer.start();

    while (timer.elapsed() < 2)
    {
        if (serialPort.waitForReadyRead(1))
        {
            serialPort.readAll();
            timer.restart();
        }
    }
}

bool Tmc6460QtInterface::writeBytes(const QByteArray &bytes)
{
    if (!serialPort.isOpen())
    {
        setError("Serial port is not open");
        return false;
    }

    const qint64 written = serialPort.write(bytes);
    if (written != bytes.size())
    {
        setError(QString("UART transmit failed: wrote %1 of %2 bytes")
                     .arg(written)
                     .arg(bytes.size()));
        return false;
    }

    if (!serialPort.waitForBytesWritten(UART_TIMEOUT_MS))
    {
        setError(QString("UART transmit timeout: %1").arg(serialPort.errorString()));
        return false;
    }

    return true;
}

bool Tmc6460QtInterface::readExact(QByteArray *data, int length, int timeoutMs)
{
    if (data == nullptr)
    {
        setError("readExact called with null data");
        return false;
    }

    data->clear();

    QElapsedTimer timer;
    timer.start();

    while (data->size() < length)
    {
        const int remaining = timeoutMs - static_cast<int>(timer.elapsed());
        if (remaining <= 0)
        {
            setError(QString("UART timeout: received %1 of %2 bytes")
                         .arg(data->size())
                         .arg(length));
            return false;
        }

        if (serialPort.waitForReadyRead(qMin(remaining, 20)))
        {
            data->append(serialPort.read(length - data->size()));
        }

    }

    return true;
}

bool Tmc6460QtInterface::readRegister(quint16 address, quint32 *value)
{
    if (value == nullptr)
    {
        setError("readRegister called with null value");
        return false;
    }

    clearRx();

    const quint8 header = static_cast<quint8>(UART_READ_BASE |
                                              (((address >> 8) & 0x03) << 4));

    QByteArray request;
    request.append(static_cast<char>(header));
    request.append(static_cast<char>(address & 0xFF));

    if (!writeBytes(request))
    {
        return false;
    }

    QByteArray response;
    if (!readExact(&response, 6, UART_TIMEOUT_MS))
    {
        log(QString("Read %1 failed").arg(hex16(address)));
        clearRx();
        return false;
    }

    const quint8 response0 = static_cast<quint8>(response[0]);
    const quint8 response1 = static_cast<quint8>(response[1]);

    if ((response0 != header) || (response1 != static_cast<quint8>(address & 0xFF)))
    {
        setError(QString("Unexpected response for %1: got %2 %3")
                     .arg(hex16(address))
                     .arg(hex8(response0))
                     .arg(hex8(response1)));
        clearRx();
        return false;
    }

    *value = (static_cast<quint32>(static_cast<quint8>(response[2])) << 24) |
             (static_cast<quint32>(static_cast<quint8>(response[3])) << 16) |
             (static_cast<quint32>(static_cast<quint8>(response[4])) << 8)  |
             (static_cast<quint32>(static_cast<quint8>(response[5])));

    QThread::usleep(INTER_TRANSACTION_US);
    return true;
}

bool Tmc6460QtInterface::writeRegister(quint16 address, quint32 value)
{
    clearRx();

    const quint8 header = static_cast<quint8>(UART_WRITE_BASE |
                                              (((address >> 8) & 0x03) << 4));

    QByteArray request;
    request.append(static_cast<char>(header));
    request.append(static_cast<char>(address & 0xFF));
    request.append(static_cast<char>((value >> 24) & 0xFF));
    request.append(static_cast<char>((value >> 16) & 0xFF));
    request.append(static_cast<char>((value >> 8) & 0xFF));
    request.append(static_cast<char>(value & 0xFF));

    if (!writeBytes(request))
    {
        clearRx();
        return false;
    }

    QThread::usleep(INTER_TRANSACTION_US);
    clearRx();
    return true;
}

bool Tmc6460QtInterface::writeRegisterChecked(quint16 address,
                                              quint32 value,
                                              const char *name,
                                              bool logWrite)
{
    if (!writeRegister(address, value))
    {
        setError(QString("Write %1 failed").arg(QString::fromLatin1(name)));
        return false;
    }

    if (logWrite)
    {
        log(QString("WRITE %1 [%2] = %3")
                .arg(QString::fromLatin1(name).leftJustified(32, QLatin1Char(' ')))
                .arg(hex16(address))
                .arg(hex32(value)));
    }

    return true;
}

bool Tmc6460QtInterface::setDriverEnable(bool enable)
{
    const bool ok = writeRegisterChecked(REG_MCC_CONFIG_GDRV,
                                         enable ? GDRV_ON_VALUE : GDRV_OFF_VALUE,
                                         enable ? "MCC_CONFIG_GDRV ON" : "MCC_CONFIG_GDRV OFF");
    if (ok)
    {
        driverEnabled = enable;
        if (!enable)
        {
            velocityModePrepared = false;
            torqueModePrepared = false;
            positionModePrepared = false;
        }
    }
    return ok;
}

void Tmc6460QtInterface::quickCheckDriverEvent()
{
    QElapsedTimer timer;
    timer.start();

    while (timer.elapsed() < DRIVER_READY_FAST_CHECK_MS)
    {
        quint32 chipEvents = 0;
        if (readRegister(REG_CHIP_EVENTS, &chipEvents))
        {
            if ((chipEvents & DRIVER_READY_EVENT_MASK) != 0U)
            {
                log(QString("CHIP_EVENTS = %1").arg(hex32(chipEvents)));
                return;
            }
        }
        QThread::msleep(2);
    }

    log("WARNING: DRV ready event not seen in fast check. Continuing immediately.");
}

bool Tmc6460QtInterface::waitForChipEvent(quint32 eventMask, int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();

    while (timer.elapsed() < timeoutMs)
    {
        quint32 chipEvents = 0;
        if (readRegister(REG_CHIP_EVENTS, &chipEvents) && ((chipEvents & eventMask) != 0U))
        {
            log(QString("CHIP_EVENTS = %1").arg(hex32(chipEvents)));
            return true;
        }

        QThread::msleep(2);
    }

    return false;
}

qint16 Tmc6460QtInterface::lowSigned16(quint32 value)
{
    return static_cast<qint16>(value & 0xFFFFU);
}

qint16 Tmc6460QtInterface::highSigned16(quint32 value)
{
    return static_cast<qint16>((value >> 16) & 0xFFFFU);
}

int Tmc6460QtInterface::torqueRawToMilliAmp(qint16 torqueRaw)
{
    return qRound(static_cast<double>(torqueRaw) * TORQUE_RAW_TO_MILLIAMP);
}

QString Tmc6460QtInterface::hex8(quint8 value)
{
    return QString("0x%1").arg(value, 2, 16, QLatin1Char('0')).toUpper();
}

QString Tmc6460QtInterface::hex16(quint16 value)
{
    return QString("0x%1").arg(value, 4, 16, QLatin1Char('0')).toUpper();
}

QString Tmc6460QtInterface::hex32(quint32 value)
{
    return QString("0x%1").arg(value, 8, 16, QLatin1Char('0')).toUpper();
}

QVector<Tmc6460QtInterface::RegisterValue> Tmc6460QtInterface::configurationTable()
{
    return {
        {REG_CHIP_IO_MATRIX,               0x00000000UL, "CHIP_IO_MATRIX"},
        {REG_CHIP_IO_CONFIG,               0x70055038UL, "CHIP_IO_CONFIG"},
        {REG_MCC_ADC_CSA_GAIN,             0x00000005UL, "MCC_ADC_CSA_GAIN"},
        {REG_MCC_CONFIG_MOTOR_MOTION,      MOTOR_MOTION_IDLE_VALUE, "MCC_CONFIG_MOTOR_MOTION_IDLE"},
        {REG_MCC_CONFIG_GDRV,              GDRV_OFF_VALUE, "MCC_CONFIG_GDRV_OFF"},
        {REG_MCC_CONFIG_PWM,               0x00000017UL, "MCC_CONFIG_PWM"},
        {REG_FOC_PID_CONFIG,               0x00008558UL, "FOC_PID_CONFIG"},
        {REG_FOC_PID_FLUX_COEFF,           0x01E54C51UL, "FOC_PID_FLUX_COEFF"},
        {REG_FOC_PID_TORQUE_COEFF,         0x01E54C51UL, "FOC_PID_TORQUE_COEFF"},
        {REG_FOC_PID_VELOCITY_COEFF,       0x00960005UL, "FOC_PID_VELOCITY_COEFF"},
        {REG_FOC_PID_UQ_UD_LIMITS,         0x00004E20UL, "FOC_PID_UQ_UD_LIMITS"},
        {REG_FOC_PID_TORQUE_FLUX_LIMITS,   0x0BB80BB8UL, "FOC_PID_TORQUE_FLUX_LIMITS"},
        {REG_FEEDBACK_CONF_CH_A,           0x052AAAABUL, "FEEDBACK_CONF_CH_A"},
        {REG_FEEDBACK_CONF_CH_B,           0x06000000UL, "FEEDBACK_CONF_CH_B"},
        {REG_FEEDBACK_VELOCITY_FRQ_CONF,   0x001B4F00UL, "FEEDBACK_VELOCITY_FRQ_CONF"},
        {REG_FEEDBACK_VELOCITY_PER_CONF,   0xFFF00001UL, "FEEDBACK_VELOCITY_PER_CONF"},
        {REG_FEEDBACK_VELOCITY_PER_FILTER, 0x00000003UL, "FEEDBACK_VELOCITY_PER_FILTER"},
        {REG_FEEDBACK_OUTPUT_CONF,         0x00400001UL, "FEEDBACK_OUTPUT_CONF"},
        {REG_FOC_PID_VELOCITY_LIMIT,       0x00989680UL, "FOC_PID_VELOCITY_LIMIT"},
        {REG_FOC_PID_VELOCITY_TARGET,      0x00000000UL, "FOC_PID_VELOCITY_TARGET"},
        {REG_HALL_MAP_CONFIG,              0x00000001UL, "HALL_MAP_CONFIG"}
    };
}

QVector<Tmc6460QtInterface::RegisterValue> Tmc6460QtInterface::readbackTable()
{
    QVector<RegisterValue> table = configurationTable();
    table.append({REG_CHIP_STATUS_FLAGS, 0, "CHIP_STATUS_FLAGS"});
    table.append({REG_CHIP_EVENTS, 0, "CHIP_EVENTS"});
    table.append({REG_FOC_PID_VELOCITY_ACTUAL, 0, "FOC_PID_VELOCITY_ACTUAL"});
    table.append({REG_FOC_PID_POSITION_ACTUAL, 0, "FOC_PID_POSITION_ACTUAL"});
    table.append({REG_FOC_PID_TORQUE_FLUX_ACTUAL, 0, "FOC_PID_TORQUE_FLUX_ACTUAL"});
    table.append({REG_MCC_ADC_IW_IU, 0, "MCC_ADC_IW_IU"});
    table.append({REG_MCC_ADC_IV, 0, "MCC_ADC_IV"});
    return table;
}
