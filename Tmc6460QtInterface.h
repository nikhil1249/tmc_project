#ifndef TMC6460QTINTERFACE_H
#define TMC6460QTINTERFACE_H

#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QVector>
#include <QMetaType>
#include <QtGlobal>
#include <cstdint>

#include "TMC6460_HW_Abstraction.h"

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
    bool setVelocityLimitRaw(qint32 limitRaw);
    bool setTorqueTarget(qint32 targetTorque);
    bool setVelocityTorqueLimit(qint32 torqueLimitRaw);
    bool prepareVelocityModeForRun();
    bool prepareTorqueModeForRun();
    bool emergencyStop();
    bool shutdownMotorSafe();
    bool holdAfterStall();
    bool applyIdleStopSequence(const char *reason);
    void debugReadRunRegistersOnce();

    static qint16 lowSigned16(quint32 value);
    static qint16 highSigned16(quint32 value);
    static int torqueRawToMilliAmp(qint16 torqueRaw);

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

    static constexpr quint32 DRIVER_READY_EVENT_MASK = TMC6460_CHIP_EVENTS_GDRV_ON_EVENT_MASK;

    static constexpr quint32 EXPECTED_CHIP_ID = 0x36343630UL;

    static constexpr quint16 REG_CHIP_ID                      = 0x0000;
    static constexpr quint16 REG_CHIP_STATUS_FLAGS            = TMC6460_CHIP_STATUS_FLAGS;
    static constexpr quint16 REG_CHIP_EVENTS                  = TMC6460_CHIP_EVENTS;
    static constexpr quint16 REG_CHIP_IO_MATRIX               = TMC6460_CHIP_IO_MATRIX;
    static constexpr quint16 REG_CHIP_IO_CONFIG               = TMC6460_CHIP_IO_CONFIG;
    static constexpr quint16 REG_MCC_ADC_IW_IU                = TMC6460_MCC_ADC_IW_IU;
    static constexpr quint16 REG_MCC_ADC_IV                   = TMC6460_MCC_ADC_IV;
    static constexpr quint16 REG_MCC_ADC_CSA_GAIN             = TMC6460_MCC_ADC_CSA_GAIN;
    static constexpr quint16 REG_MCC_CONFIG_MOTOR_MOTION      = TMC6460_MCC_CONFIG_MOTOR_MOTION;
    static constexpr quint16 REG_MCC_CONFIG_GDRV              = TMC6460_MCC_CONFIG_GDRV;
    static constexpr quint16 REG_MCC_CONFIG_PWM               = TMC6460_MCC_CONFIG_PWM;
    static constexpr quint16 REG_FOC_PID_CONFIG               = TMC6460_FOC_PID_CONFIG;
    static constexpr quint16 REG_FOC_PID_FLUX_COEFF           = TMC6460_FOC_PID_FLUX_COEFF;
    static constexpr quint16 REG_FOC_PID_TORQUE_COEFF         = TMC6460_FOC_PID_TORQUE_COEFF;
    static constexpr quint16 REG_FOC_PID_VELOCITY_COEFF       = TMC6460_FOC_PID_VELOCITY_COEFF;
    static constexpr quint16 REG_FOC_PID_UQ_UD_LIMITS         = TMC6460_FOC_PID_UQ_UD_LIMITS;
    static constexpr quint16 REG_FOC_PID_TORQUE_FLUX_LIMITS   = TMC6460_FOC_PID_TORQUE_FLUX_LIMITS;
    static constexpr quint16 REG_FOC_PID_VELOCITY_LIMIT       = TMC6460_FOC_PID_VELOCITY_LIMIT;
    static constexpr quint16 REG_FOC_PID_TORQUE_FLUX_TARGET   = TMC6460_FOC_PID_TORQUE_FLUX_TARGET;
    static constexpr quint16 REG_FOC_PID_VELOCITY_TARGET      = TMC6460_FOC_PID_VELOCITY_TARGET;
    static constexpr quint16 REG_FOC_PID_VELOCITY_ACTUAL      = TMC6460_FOC_PID_VELOCITY_ACTUAL;
    static constexpr quint16 REG_FOC_PID_POSITION_ACTUAL      = TMC6460_FOC_PID_POSITION_ACTUAL;
    static constexpr quint16 REG_FOC_PID_TORQUE_FLUX_ACTUAL   = TMC6460_FOC_PID_TORQUE_FLUX_ACTUAL;
    static constexpr quint16 REG_FEEDBACK_CONF_CH_A           = TMC6460_FEEDBACK_CONF_CH_A;
    static constexpr quint16 REG_FEEDBACK_CONF_CH_B           = TMC6460_FEEDBACK_CONF_CH_B;
    static constexpr quint16 REG_FEEDBACK_VELOCITY_FRQ_CONF   = TMC6460_FEEDBACK_VELOCITY_FRQ_CONF;
    static constexpr quint16 REG_FEEDBACK_VELOCITY_PER_CONF   = TMC6460_FEEDBACK_VELOCITY_PER_CONF;
    static constexpr quint16 REG_FEEDBACK_VELOCITY_PER_FILTER = TMC6460_FEEDBACK_VELOCITY_PER_FILTER;
    static constexpr quint16 REG_FEEDBACK_OUTPUT_CONF         = TMC6460_FEEDBACK_OUTPUT_CONF;
    static constexpr quint16 REG_HALL_MAP_CONFIG              = TMC6460_HALL_MAP_CONFIG;

    static constexpr quint32 GDRV_OFF_VALUE = 0x80003431UL;
    static constexpr quint32 GDRV_ON_VALUE  = 0x80013431UL;

    // Values derived from the working Python/TMCL configuration.
    // IDLE/PWM_ON is written during initialization. Velocity/Torque are written only
    // when the corresponding GUI command is requested.
    static constexpr quint32 MOTOR_MOTION_IDLE_VALUE     = 0x00002386UL;
    static constexpr quint32 MOTOR_MOTION_VELOCITY_VALUE = 0x00002786UL;
    static constexpr quint32 MOTOR_MOTION_TORQUE_VALUE   = 0x00002586UL;

    // Python script limits. Keep GUI/software commands inside these limits.
    static constexpr qint32 PYTHON_CONST_VELOCITY_RAW = 4000000;
    static constexpr qint32 MAX_ALLOWED_VELOCITY_RAW  = 10000000;
    static constexpr qint32 MAX_ALLOWED_TORQUE_RAW    = 3000;
    static constexpr qint32 MIN_VELOCITY_TORQUE_LIMIT_RAW = 0;
    static constexpr qint32 MAX_VELOCITY_TORQUE_LIMIT_RAW = 3000;
    static constexpr qint16 DEFAULT_FLUX_LIMIT_RAW = 3000;

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
    bool torqueModePrepared = false;
    qint32 lastVelocityCommand = 0;
    qint32 currentVelocityLimitRaw = MAX_ALLOWED_VELOCITY_RAW;

    void setBusy(bool busy);
    void setError(const QString &message);
    void log(const QString &message);

    void clearRx();
    bool writeBytes(const QByteArray &bytes);
    bool readExact(QByteArray *data, int length, int timeoutMs);

    bool readRegister(quint16 address, quint32 *value);
    bool writeRegister(quint16 address, quint32 value);
    bool writeRegisterChecked(quint16 address, quint32 value, const char *name, bool logWrite = false);

    bool setDriverEnable(bool enable);
    void quickCheckDriverEvent();
    bool waitForChipEvent(quint32 eventMask, int timeoutMs);

    bool writeVelocityTargetImmediate(qint32 targetVelocity, const char *name);
    bool applyVelocityWithSafeReverseRamp(qint32 targetVelocity);
    static int signOf(qint32 value);
    static quint32 makeTorqueFluxTarget(qint16 torqueRaw, qint16 fluxRaw = 0);
    static quint32 makeTorqueFluxLimit(qint16 torqueLimitRaw, qint16 fluxLimitRaw);

    static QString hex8(quint8 value);
    static QString hex16(quint16 value);
    static QString hex32(quint32 value);

    static QVector<RegisterValue> configurationTable();
    static QVector<RegisterValue> readbackTable();
};

Q_DECLARE_METATYPE(Tmc6460QtInterface::RunStatus)

#endif
