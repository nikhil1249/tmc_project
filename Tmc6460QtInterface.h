#ifndef TMC6460QTINTERFACE_H
#define TMC6460QTINTERFACE_H

#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QVector>
#include <QMetaType>
#include <QtGlobal>
#include <cstdint>

class Tmc6460QtInterface : public QObject
{
    Q_OBJECT

public:
    struct RegisterValue
    {
        quint16 address;
        quint32 value;
        const char *name;
    };

    struct RunStatus
    {
        quint32 chipStatusFlags = 0;
        quint32 chipEvents = 0;
        quint32 motorMotion = 0;
        quint32 gdrv = 0;
        quint32 velocityTarget = 0;
        quint32 torqueFluxTarget = 0;
        qint32 velocityActualRaw = 0;
        int velocityActualRpm = 0;
        quint32 positionActual = 0;
        quint32 torqueFluxActual = 0;
        quint32 phaseCurrentUraw = 0;
        quint32 phaseCurrentVraw = 0;
        quint32 phaseCurrentWraw = 0;
        qint16 torqueActualRaw = 0;
        qint16 fluxActualRaw = 0;
        int torqueCurrentMilliAmp = 0;
        bool valid = false;
    };

    explicit Tmc6460QtInterface(QObject *parent = nullptr);
    ~Tmc6460QtInterface() override;

    bool openPort(const QString &portName, qint32 baudRate = 115200);
    void closePort();
    bool isOpen() const;
    bool isBusy() const;
    QString lastError() const;

    bool initializeMotor();
    bool readConfiguredRegisters(QVector<RegisterValue> *valuesOut = nullptr);

    bool readChipId(quint32 *chipId);
    bool readChipStatusFlags(quint32 *statusFlags);
    bool readRunStatus(RunStatus *status);

    bool setVelocityTarget(qint32 targetVelocity);
    bool setTorqueTarget(qint32 targetTorque);
    bool emergencyStop();

    static qint16 lowSigned16(quint32 value);
    static qint16 highSigned16(quint32 value);
    static int torqueRawToMilliAmp(qint16 torqueRaw);

    // Excel-derived velocity calibration:
    // rpm = 0.0033 * raw - 0.142
    // raw = (rpm + 0.142) / 0.0033
    static constexpr double VELOCITY_RAW_TO_RPM_GAIN = 0.0033;
    static constexpr double VELOCITY_RAW_TO_RPM_OFFSET = -0.142;

    static int velocityRawToRpm(qint32 raw)
    {
        const double rpm =
            (VELOCITY_RAW_TO_RPM_GAIN * static_cast<double>(raw)) +
            VELOCITY_RAW_TO_RPM_OFFSET;

        return static_cast<int>(qRound(rpm));
    }

    static qint32 rpmToVelocityRaw(int rpm)
    {
        const double raw = (static_cast<double>(rpm) - VELOCITY_RAW_TO_RPM_OFFSET)
                           / VELOCITY_RAW_TO_RPM_GAIN;
        return static_cast<qint32>(qRound(raw));
    }

signals:
    void logMessage(const QString &message);
    void errorChanged(const QString &message);

private:
    static constexpr quint8 UART_READ_BASE  = 0x42;
    static constexpr quint8 UART_WRITE_BASE = 0x4A;

    static constexpr int UART_TIMEOUT_MS = 250;
    static constexpr int STARTUP_DELAY_MS = 2500;
    static constexpr int INTER_TRANSACTION_US = 1000;
    static constexpr int WRITE_SETTLE_MS = 2;
    static constexpr int DRIVER_TOGGLE_DELAY_MS = 10;
    static constexpr int DRIVER_READY_TIMEOUT_MS = 2000;
    static constexpr int DRIVER_READY_FAST_CHECK_MS = 60;

    // Soft reverse ramp settings. These prevent mechanical/electrical jerk when the
    // user commands opposite direction while the motor is already spinning.
    // Values are in the same internal unit used for FOC_PID_VELOCITY_TARGET.
    static constexpr qint32 SAFE_REVERSE_DEADBAND = 1000;
    static constexpr qint32 SAFE_REVERSE_RAMP_STEP = 10000;
    static constexpr int SAFE_REVERSE_RAMP_DELAY_MS = 20;
    static constexpr int SAFE_REVERSE_ZERO_DWELL_MS = 120;

    static constexpr quint32 DRIVER_READY_EVENT_MASK = (1UL << 31);

    static constexpr quint32 EXPECTED_CHIP_ID = 0x36343630UL;

    static constexpr quint16 REG_CHIP_ID                      = 0x0000;
    static constexpr quint16 REG_CHIP_STATUS_FLAGS            = 0x0004;
    static constexpr quint16 REG_CHIP_EVENTS                  = 0x0005;
    static constexpr quint16 REG_CHIP_IO_CONFIG               = 0x0008;
    static constexpr quint16 REG_MCC_ADC_IW_IU                = 0x00C1;
    static constexpr quint16 REG_MCC_ADC_IV                   = 0x00C2;
    static constexpr quint16 REG_MCC_ADC_CSA_GAIN             = 0x00C3;
    static constexpr quint16 REG_MCC_CONFIG_MOTOR_MOTION      = 0x0100;
    static constexpr quint16 REG_MCC_CONFIG_GDRV              = 0x0101;
    static constexpr quint16 REG_MCC_CONFIG_PWM               = 0x0102;
    static constexpr quint16 REG_FOC_PID_CONFIG               = 0x0140;
    static constexpr quint16 REG_FOC_PID_FLUX_COEFF           = 0x0142;
    static constexpr quint16 REG_FOC_PID_TORQUE_COEFF         = 0x0143;
    static constexpr quint16 REG_FOC_PID_VELOCITY_COEFF       = 0x0145;
    static constexpr quint16 REG_FOC_PID_UQ_UD_LIMITS         = 0x0149;
    static constexpr quint16 REG_FOC_PID_TORQUE_FLUX_LIMITS   = 0x014A;
    static constexpr quint16 REG_FOC_PID_VELOCITY_LIMIT       = 0x014B;
    static constexpr quint16 REG_FOC_PID_TORQUE_FLUX_TARGET   = 0x014E;
    static constexpr quint16 REG_FOC_PID_VELOCITY_TARGET      = 0x0150;
    static constexpr quint16 REG_FOC_PID_VELOCITY_ACTUAL      = 0x0151;
    static constexpr quint16 REG_FOC_PID_POSITION_ACTUAL      = 0x0152;
    static constexpr quint16 REG_FOC_PID_TORQUE_FLUX_ACTUAL   = 0x0153;
    static constexpr quint16 REG_FEEDBACK_CONF_CH_A           = 0x0240;
    static constexpr quint16 REG_FEEDBACK_CONF_CH_B           = 0x0241;
    static constexpr quint16 REG_FEEDBACK_VELOCITY_FRQ_CONF   = 0x0244;
    static constexpr quint16 REG_FEEDBACK_VELOCITY_PER_CONF   = 0x0245;
    static constexpr quint16 REG_FEEDBACK_VELOCITY_PER_FILTER = 0x0246;
    static constexpr quint16 REG_FEEDBACK_OUTPUT_CONF         = 0x0250;
    static constexpr quint16 REG_HALL_MAP_CONFIG              = 0x0300;

    static constexpr quint32 GDRV_OFF_VALUE = 0x80003431UL;
    static constexpr quint32 GDRV_ON_VALUE  = 0x80013431UL;

    static constexpr quint32 MOTOR_MOTION_VELOCITY_VALUE = 0x0000E586UL;

    // GUI current conversion. TMC6460 FOC torque actual is a signed raw current-axis value.
    // Keep this as a project calibration constant because the exact mA/raw depends on
    // board shunt, CSA gain and motor configuration. For the present 24 V / ~1 A test
    // actuator, 5000 raw is treated as approximately 1000 mA.
    static constexpr double TORQUE_RAW_TO_MILLIAMP = 0.2;

    QSerialPort serialPort;
    QString errorText;
    bool busBusy = false;
    bool driverEnabled = false;
    bool estopActive = false;
    bool velocityModePrepared = false;
    qint32 lastVelocityCommand = 0;

    void setBusy(bool busy);
    void setError(const QString &message);
    void log(const QString &message);

    void clearRx();
    bool writeBytes(const QByteArray &bytes);
    bool readExact(QByteArray *data, int length, int timeoutMs);

    bool readRegister(quint16 address, quint32 *value);
    bool writeRegister(quint16 address, quint32 value);
    bool writeRegisterChecked(quint16 address, quint32 value, const char *name, bool logWrite);

    bool setDriverEnable(bool enable);
    void quickCheckDriverEvent();
    bool waitForChipEvent(quint32 eventMask, int timeoutMs);

    bool prepareVelocityModeForRun();
    bool writeVelocityTargetImmediate(qint32 targetVelocity, const char *name);
    bool applyVelocityWithSafeReverseRamp(qint32 targetVelocity);
    static int signOf(qint32 value);

    static QString hex8(quint8 value);
    static QString hex16(quint16 value);
    static QString hex32(quint32 value);

    static QVector<RegisterValue> configurationTable();
    static QVector<RegisterValue> readbackTable();
};

Q_DECLARE_METATYPE(Tmc6460QtInterface::RunStatus)

#endif
