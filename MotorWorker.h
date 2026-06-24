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

// Simple stall detection copied from the working Python test flow:
// after command settle time, stall = abs(PID_VELOCITY_ACTUAL) < 60% of target.
#ifndef TMC6460_ENABLE_SIMPLE_STALL_DETECTION
#define TMC6460_ENABLE_SIMPLE_STALL_DETECTION 1
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
    static constexpr qint32 MAX_ALLOWED_TORQUE_RAW = 10000;

    // Python test.py uses a 5 s settle time and 60% velocity threshold.
    static constexpr int STALL_SETTLE_TIME_MS = 1000;
    static constexpr double STALL_MIN_VELOCITY_RATIO = 0.60;
    static constexpr qint32 STALL_MIN_TARGET_RAW = 100000;

    bool stallMonitorActive = false;
    bool stallLatched = false;
    qint32 stallTargetVelocityRaw = 0;
    QElapsedTimer stallTimer;

    void ensureInterface();
    bool writeVelocityRaw(qint32 rawVelocity);
    void resetVelocityRampState();
    void startStallMonitor(qint32 rawVelocity);
    void stopStallMonitor(bool clearGuiStatus);
    void checkSimpleStall(const Tmc6460QtInterface::RunStatus &status);
    static QString hex32(quint32 value);
};

#endif
