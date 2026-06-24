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
    bool ok = true;

    // Minimal live feedback read. Keep this short because status polling runs periodically.
    // Velocity actual comes directly from the official Trinamic address TMC6460_FOC_PID_VELOCITY_ACTUAL [0x0154].
    ok &= readRegister(REG_CHIP_STATUS_FLAGS, &result.chipStatusFlags);

    quint32 velocityActualReg = 0;
    if (readRegister(REG_FOC_PID_VELOCITY_ACTUAL, &velocityActualReg))
    {
        result.velocityActualRaw = static_cast<qint32>(velocityActualReg);

        // Temporary debug print requested for tuning. Status polling is 1 second,
        // so this prints PID velocity actual once per second.
        qDebug().noquote() << QString("DEBUG PID_VELOCITY_ACTUAL [0x0154] = %1, raw=%2")
                                  .arg(hex32(velocityActualReg))
                                  .arg(result.velocityActualRaw);
    }
    else
    {
        ok = false;
    }

    ok &= readRegister(REG_FOC_PID_POSITION_ACTUAL, &result.positionActual);
    ok &= readRegister(REG_FOC_PID_TORQUE_FLUX_ACTUAL, &result.torqueFluxActual);

    result.fluxActualRaw = lowSigned16(result.torqueFluxActual);
    result.torqueActualRaw = highSigned16(result.torqueFluxActual);
    result.torqueCurrentMilliAmp = torqueRawToMilliAmp(result.torqueActualRaw);

    result.valid = ok;
    *status = result;
    return ok;
}

bool Tmc6460QtInterface::setVelocityTarget(qint32 targetVelocity)
{
    setBusy(true);

    targetVelocity = qBound<qint32>(-MAX_ALLOWED_VELOCITY_RAW,
                                    targetVelocity,
                                    MAX_ALLOWED_VELOCITY_RAW);

    bool ok = false;

    // Important: velocity 0 must not remain in velocity loop.
    // On this actuator it can decay slowly and then vibrate. Use the same
    // Python/TMCL idle sequence that was stable during testing.
    if (targetVelocity == 0)
    {
        ok = applyIdleStopSequence("VELOCITY_ZERO_STOP");
        log("ACTION: Velocity target raw=0 applied using idle stop sequence");
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
        lastVelocityCommand = 0;
        log(QString("MODE: Torque, MCC_CONFIG_MOTOR_MOTION=%1")
                .arg(hex32(MOTOR_MOTION_TORQUE_VALUE)));
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
    setBusy(true);

    const qint16 limitedTorque = static_cast<qint16>(qBound<qint32>(MIN_VELOCITY_TORQUE_LIMIT_RAW,
                                                                    torqueLimitRaw,
                                                                    MAX_VELOCITY_TORQUE_LIMIT_RAW));

    const quint32 limitRegisterValue = makeTorqueFluxLimit(limitedTorque, limitedTorque);

    // Velocity mode torque adjustment: only update the torque/flux limits.
    // Do not change motor mode and do not write FOC_PID_TORQUE_FLUX_TARGET.
    const bool ok = writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_LIMITS,
                                         limitRegisterValue,
                                         "FOC_PID_TORQUE_FLUX_LIMITS VELOCITY_LIMIT",
                                         false);

    if (ok)
    {
        log(QString("ACTION: Velocity-mode torque limit=%1, flux limit=%2, register=%3")
                .arg(limitedTorque)
                .arg(limitedTorque)
                .arg(hex32(limitRegisterValue)));
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
    const quint32 targetRegisterValue = makeTorqueFluxTarget(torqueRaw, 0);

    // Torque command uses only upper 16 bits. Lower 16 bits/flux is kept zero.
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
    bool ok = true;

    // Keep gate driver enabled for a controlled stop unless it was already off.
    if (!driverEnabled)
    {
        ok &= setDriverEnable(true);
        QThread::msleep(30);
    }

    // 1) Remove velocity demand first.
    ok &= writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET,
                               0x00000000UL,
                               "FOC_PID_VELOCITY_TARGET ZERO_STOP",
                               false);

    // 2) Switch to torque mode.
    ok &= writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                               MOTOR_MOTION_TORQUE_VALUE,
                               "MCC_CONFIG_MOTOR_MOTION TORQUE_STOP",
                               false);

    // 3) Set torque and flux target to zero.
    ok &= writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_TARGET,
                               0x00000000UL,
                               "FOC_PID_TORQUE_FLUX_TARGET ZERO_STOP",
                               false);

    QThread::msleep(500);

    // 4) Put controller into the known Python/TMCL idle/PWM_ON state.
    ok &= writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                               MOTOR_MOTION_IDLE_VALUE,
                               "MCC_CONFIG_MOTOR_MOTION PWM_ON_IDLE_STOP",
                               false);

    QThread::msleep(500);

    lastVelocityCommand = 0;
    velocityModePrepared = false;
    torqueModePrepared = false;
    estopActive = false;

    log(QString("STOP: %1 sequence applied: VELOCITY_TARGET=0, TORQUE : TORQUE_FLUX_TARGET=0, PWM_ON/IDLE")
            .arg(QString::fromLatin1(reason)));

    return ok;
}


bool Tmc6460QtInterface::shutdownMotorSafe()
{
    setBusy(true);

    log("SHUTDOWN: Safe motor shutdown started");

    bool ok = true;

    // Stop all active command targets first.
    ok &= writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET,
                               0x00000000UL,
                               "FOC_PID_VELOCITY_TARGET ZERO",
                               false);

    ok &= writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_TARGET,
                               0x00000000UL,
                               "FOC_PID_TORQUE_FLUX_TARGET ZERO",
                               false);

    // Remove torque/flux limit so the controller cannot keep holding current
    // after the GUI is closed. During next initialization this is restored to
    // the Python/default value 0x0BB80BB8.
    ok &= writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_LIMITS,
                               0x00000000UL,
                               "FOC_PID_TORQUE_FLUX_LIMITS ZERO",
                               false);

    // Put motion controller back to the Python idle/PWM_ON state.
    ok &= writeRegisterChecked(REG_MCC_CONFIG_MOTOR_MOTION,
                               MOTOR_MOTION_IDLE_VALUE,
                               "MCC_CONFIG_MOTOR_MOTION IDLE",
                               false);

    QThread::msleep(100);

    // Finally disable the gate driver.
    ok &= writeRegisterChecked(REG_MCC_CONFIG_GDRV,
                               GDRV_OFF_VALUE,
                               "MCC_CONFIG_GDRV OFF",
                               false);

    lastVelocityCommand = 0;
    velocityModePrepared = false;
    torqueModePrepared = false;
    driverEnabled = false;
    estopActive = true;

    log(ok ? "SHUTDOWN: Safe motor shutdown completed"
           : "SHUTDOWN: Safe motor shutdown completed with errors");

    setBusy(false);
    return ok;
}

bool Tmc6460QtInterface::emergencyStop()
{
    setBusy(true);

    log("E-STOP command");

    bool ok = true;
    ok &= writeRegisterChecked(REG_FOC_PID_VELOCITY_TARGET, 0, "FOC_PID_VELOCITY_TARGET STOP");
    lastVelocityCommand = 0;
    ok &= writeRegisterChecked(REG_FOC_PID_TORQUE_FLUX_TARGET, 0, "FOC_PID_TORQUE_FLUX_TARGET STOP");

    // Keep the safety behavior: disable the gate driver.
    // Next non-zero torque/velocity command will automatically re-arm it.
    ok &= setDriverEnable(false);
    estopActive = true;
    velocityModePrepared = false;
    torqueModePrepared = false;

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
        {REG_FOC_PID_TORQUE_FLUX_TARGET,   0x00000000UL, "FOC_PID_TORQUE_FLUX_TARGET"},
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
