#ifndef MOTORWORKER_H
#define MOTORWORKER_H

#include "Tmc6460QtInterface.h"

#include <QObject>
#include <QString>
#include <QTimer>

// Set to 1 to enable software RPM ramping. Set to 0 to write final velocity directly.
#ifndef TMC6460_ENABLE_VELOCITY_RAMP
#define TMC6460_ENABLE_VELOCITY_RAMP 1
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

    // Velocity command from GUI is RPM. The worker ramps RPM internally and
    // converts each ramp step to the TMC6460 raw velocity register value.
    void applyVelocityRpm(int rpm);

    void emergencyStop();
    void shutdown();

signals:
    void connected(bool ok, quint32 chipId, const QString &message);
    void statusReady(const Tmc6460QtInterface::RunStatus &status);
    void commandDone(const QString &action, bool ok);
    void logMessage(const QString &message);
    void errorChanged(const QString &message);

private slots:
    void processVelocityRamp();

private:
    Tmc6460QtInterface *tmc = nullptr;
    QTimer *velocityRampTimer = nullptr;

    int currentCommandedVelocityRpm = 0;
    int targetVelocityRpm = 0;

    // Start safe. If motor is smooth, increase step to 50 or 100 rpm.
    static constexpr int VELOCITY_RAMP_TIMER_MS = 50;
    static constexpr int VELOCITY_RAMP_STEP_RPM = 25;

    void ensureInterface();
    bool writeVelocityRpm(int rpm);
    void resetVelocityRampState();
};

#endif
