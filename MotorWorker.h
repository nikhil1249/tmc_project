#ifndef MOTORWORKER_H
#define MOTORWORKER_H

#include "Tmc6460QtInterface.h"

#include <QObject>
#include <QString>
#include <QTimer>
#include <QElapsedTimer>

// Set to 1 to enable software RAW velocity ramping. Set to 0 to write final velocity directly.
#ifndef TMC6460_ENABLE_VELOCITY_RAMP
#define TMC6460_ENABLE_VELOCITY_RAMP 0
#endif

// Actuator end calibration / stop detection.
// 1 = CW/CCW button automatically runs one end, stops, then reverses to the other end.
// 0 = CW/CCW button behaves as normal single-direction run-to-end monitor.
#ifndef TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION
#define TMC6460_ENABLE_AUTO_END_TO_END_CALIBRATION 0
#endif

// 1 = mechanical-end detection is active while running. Keep enabled during calibration.
#ifndef TMC6460_ENABLE_ACTUATOR_STOP_DETECTION
#define TMC6460_ENABLE_ACTUATOR_STOP_DETECTION 1
#endif

// 1 = every calibration sample is logged. Disable after tuning if logs become too large.
#ifndef TMC6460_CALIBRATION_LOG_EVERY_SAMPLE
#define TMC6460_CALIBRATION_LOG_EVERY_SAMPLE 1
#endif

/*
 * End/stall hold method.
 * 1 = velocity-mode zero-speed hold only. This does NOT write position target.
 * 0 = use position target at actual position after end/stall. Keep 1 for your current test.
 */
#ifndef TMC6460_USE_VELOCITY_ZERO_HOLD_AFTER_END
#define TMC6460_USE_VELOCITY_ZERO_HOLD_AFTER_END 1
#endif

/*
 * Load-safe E-Stop.
 * This must be non-zero. It is used when the GUI E-Stop is clicked so the
 * driver remains enabled and the 10 kg load cannot back-drive the actuator.
 * Start with 800. Increase only if the load still moves. Reduce if heating is high.
 */
#ifndef TMC6460_SAFE_ESTOP_HOLD_TORQUE_FLUX_LIMIT_RAW
#define TMC6460_SAFE_ESTOP_HOLD_TORQUE_FLUX_LIMIT_RAW   800
#endif

class MotorWorker : public QObject
{
    Q_OBJECT

public:
    explicit MotorWorker(QObject *parent = nullptr);
    ~MotorWorker() override;

public slots:
    void connectAndInitialize(const QString &portName, int baudRate);
    void readStatus();
    void applyTorque(int value);
    void applyVelocityTorqueLimitRaw(int torqueLimitRaw);
    void applyVelocityLimitRaw(qint32 limitRaw);
    void applyVelocityRaw(qint32 rawVelocity);
    void applyPositionToEnd(int directionSign);
    void readRunStatusSnapshot(const QString &tag);
    void emergencyStop();
    void shutdown();

signals:
    void connected(bool ok, quint32 chipId, const QString &message);
    void statusReady(const Tmc6460QtInterface::RunStatus &status);
    void commandDone(const QString &action, bool ok);
    void stallStateChanged(bool stalled, const QString &message);
    void logMessage(const QString &message);
    void errorChanged(const QString &message);

private slots:
    void processVelocityRamp();

private:
    Tmc6460QtInterface *tmc = nullptr;
    QTimer *velocityRampTimer = nullptr;

    qint32 currentCommandedVelocityRaw = 0;
    qint32 targetVelocityRaw = 0;

    // Raw velocity ramp. Python test uses 4,000,000 raw; this ramps in raw units.
    static constexpr int VELOCITY_RAMP_TIMER_MS = 50;
    static constexpr qint32 VELOCITY_RAMP_STEP_RAW = 100000;
    static constexpr qint32 MAX_ALLOWED_VELOCITY_RAW = 10000000;
    static constexpr qint32 MAX_ALLOWED_TORQUE_RAW = 3000;

    /*
     * Calibration/end-to-end stop detection.
     *
     * Previous fixed 0-90 degree counts are intentionally not used.
     * On CW/CCW command, the motor runs in velocity mode until the feedback
     * position stops changing for multiple samples. This allows you to hit one
     * mechanical end, log that actual POSITION_ACTUAL, then run the opposite
     * direction and log the other end.
     *
     * Detection condition after startup settle:
     * - actuator has moved at least CAL_MIN_TOTAL_PROGRESS_COUNTS from start
     * - position increment is almost zero for CAL_BLOCKED_DEBOUNCE_SAMPLES
     * - actual velocity is low OR torque is loaded
     *
     * This avoids the old 2-3 second stop caused by theoretical counts.
     */
    static constexpr qint32 CAL_MIN_TARGET_RAW = 100000;
    static constexpr int CAL_SETTLE_TIME_MS = 1500;
    static constexpr int CAL_STATUS_SAMPLE_MS = 250;
    static constexpr int CAL_BLOCKED_DEBOUNCE_SAMPLES = 8;     // 8 * 250 ms = 2 seconds
    static constexpr qint64 CAL_MIN_TOTAL_PROGRESS_COUNTS = 50000;
    static constexpr qint64 CAL_MAX_POSITION_DELTA_PER_SAMPLE = 500;
    static constexpr qint32 CAL_BLOCKED_MIN_ACTUAL_RAW = 50000;
    static constexpr double CAL_BLOCKED_VELOCITY_RATIO = 0.08;
    static constexpr qint16 CAL_BLOCKED_MIN_TORQUE_RAW = 80;
    static constexpr int CAL_AUTO_REVERSE_DELAY_MS = 1000;
    static constexpr int CAL_SAFETY_TIMEOUT_MS = 30000;

    /*
     * Velocity-only hard-end/stall detection.
     *
     * No calibrated end-to-end count is used.
     * No absolute start/end position is used.
     *
     * The motor runs in velocity mode until real feedback shows that
     * POSITION_ACTUAL has stopped changing for a stable debounce time.
     * Then the code performs a slow final creep in the same direction and
     * stops only when the position is still stable during creep.
     */
#ifndef TMC6460_HARD_END_MIN_RUN_TIME_MS
#define TMC6460_HARD_END_MIN_RUN_TIME_MS                1200
#endif
#ifndef TMC6460_HARD_END_MIN_TRAVEL_COUNTS
#define TMC6460_HARD_END_MIN_TRAVEL_COUNTS              50000LL
#endif
#ifndef TMC6460_HARD_END_NO_PROGRESS_COUNTS
#define TMC6460_HARD_END_NO_PROGRESS_COUNTS             1500LL
#endif
#ifndef TMC6460_HARD_END_LOW_VELOCITY_RAW
#define TMC6460_HARD_END_LOW_VELOCITY_RAW               70000
#endif
#ifndef TMC6460_HARD_END_LOW_VELOCITY_RATIO_PERCENT
#define TMC6460_HARD_END_LOW_VELOCITY_RATIO_PERCENT     8
#endif
#ifndef TMC6460_HARD_END_TORQUE_RAW_THRESHOLD
#define TMC6460_HARD_END_TORQUE_RAW_THRESHOLD           80
#endif
#ifndef TMC6460_HARD_END_BLOCKED_SAMPLES
#define TMC6460_HARD_END_BLOCKED_SAMPLES                8
#endif
#ifndef TMC6460_HARD_END_SAFETY_TIMEOUT_MS
#define TMC6460_HARD_END_SAFETY_TIMEOUT_MS              30000
#endif

/*
 * Final hard-end creep.
 * This is the fix for the case where CW/CCW stops before the last small angle.
 * When possible stall/end is detected, it continues slowly in the same direction
 * and only holds after POSITION_ACTUAL remains stable.
 */
#ifndef TMC6460_ENABLE_FINAL_END_CREEP
#define TMC6460_ENABLE_FINAL_END_CREEP                  1
#endif
#ifndef TMC6460_FINAL_END_CREEP_VELOCITY_RAW
#define TMC6460_FINAL_END_CREEP_VELOCITY_RAW            500000
#endif
#ifndef TMC6460_FINAL_END_CREEP_TIMEOUT_MS
#define TMC6460_FINAL_END_CREEP_TIMEOUT_MS              4500
#endif
#ifndef TMC6460_FINAL_END_CREEP_STABLE_SAMPLES
#define TMC6460_FINAL_END_CREEP_STABLE_SAMPLES          10
#endif
    enum class RuntimeMoveMode
    {
        None,
        VelocityToEnd,
        PositionToEnd
    };

    enum class AutoCalibrationState
    {
        Idle,
        FirstLeg,
        WaitingBeforeReverse,
        SecondLeg,
        Done
    };

    bool calibrationMonitorActive = false;
    bool calibrationStopLatched = false;
    bool calibrationStartPositionValid = false;
    bool calibrationDirectionLearned = false;
    qint32 calibrationTargetVelocityRaw = 0;
    int calibrationCommandDirection = 0;
    int calibrationPositionDirection = 0;
    quint32 calibrationStartPositionRaw = 0;
    quint32 calibrationPreviousPositionRaw = 0;
    qint64 calibrationProgressCounts = 0;
    int calibrationBlockedSamples = 0;
    RuntimeMoveMode runtimeMoveMode = RuntimeMoveMode::None;
    QElapsedTimer calibrationTimer;

    bool finalEndCreepActive = false;
    QElapsedTimer finalEndCreepTimer;
    qint32 finalEndCreepVelocityRaw = 0;
    quint32 finalEndCreepLastPositionRaw = 0;
    int finalEndCreepStableSamples = 0;

    bool holdingAtEnd = false;
    int holdingDirection = 0;

    bool autoCalibrationActive = false;
    AutoCalibrationState autoCalibrationState = AutoCalibrationState::Idle;
    qint32 autoCalibrationFirstVelocityRaw = 0;
    qint32 autoCalibrationSecondVelocityRaw = 0;
    int autoCalibrationFirstCommandDirection = 0;
    quint32 autoCalibrationFirstStartPosition = 0;
    quint32 autoCalibrationFirstEndPosition = 0;
    quint32 autoCalibrationSecondStartPosition = 0;
    quint32 autoCalibrationSecondEndPosition = 0;
    qint64 autoCalibrationFirstTravelCounts = 0;
    qint64 autoCalibrationSecondTravelCounts = 0;

    void ensureInterface();
    bool writeVelocityRaw(qint32 rawVelocity);
    void resetVelocityRampState();
    void startCalibrationEndMonitor(qint32 rawVelocity);
    void stopCalibrationEndMonitor(bool clearGuiStatus);
    void startAutoEndToEndCalibration(qint32 firstVelocityRaw);
    void startAutoSecondLeg();
    void finishAutoEndToEndCalibration();
    void resetAutoCalibrationState();
    void checkCalibrationEndStop(const Tmc6460QtInterface::RunStatus &status);
    void handleCalibrationEndStop(const QString &reason,
                                  const Tmc6460QtInterface::RunStatus &status,
                                  qint64 progressCounts);
    void resetFinalEndCreepState();
    static qint64 signedPositionDelta32(quint32 current, quint32 start);
    static QString hex32(quint32 value);
};

#endif
