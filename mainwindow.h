#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "Tmc6460QtInterface.h"

#include <QFile>
#include <QHash>
#include <QMainWindow>
#include <QThread>
#include <QTimer>

class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTextEdit;
class QLineEdit;
class QWidget;
class QGridLayout;
class MotorWorker;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

signals:
    void workerConnectAndInitialize(const QString &portName, int baudRate);
    void workerReadStatus();
    void workerApplyTorque(int value);
    void workerApplyVelocityTorqueLimit(int torqueLimitRaw);
    void workerApplyVelocityLimitRaw(qint32 limitRaw);
    void workerApplyVelocityRaw(qint32 rawVelocity);
    void workerEmergencyStop();
    void workerShutdown();

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void connectAndInitialize();

    void onTorqueSliderValueChanged(int value);
    void onTorqueSliderReleased();
    void onTorqueDirectValueChanged(int value);
    void onTorqueDirectEditingFinished();
    void onApplyTorqueClicked();
    void onTorqueRangeChanged();

    void onVelocitySliderValueChanged(int value);
    void onVelocitySliderReleased();
    void onVelocityDirectValueChanged(int value);
    void onVelocityDirectEditingFinished();
    void onApplyVelocityClicked();
    void onVelocityTorqueLimitValueChanged(int value);
    void onVelocityTorqueLimitEditingFinished();
    void onApplyVelocityTorqueLimitClicked();
    void onVelocityRangeChanged();

    void onEstopClicked();
    void sendPendingTorqueCommand();
    void sendPendingVelocityCommand();
    void updateChipStatus();

    void onWorkerConnected(bool ok, quint32 chipId, const QString &message);
    void onWorkerStatusReady(const Tmc6460QtInterface::RunStatus &runStatus);
    void onWorkerCommandDone(const QString &action, bool ok);
    void onWorkerStallStateChanged(bool stalled, const QString &message);

    void appendLog(const QString &message);
    void showError(const QString &message);

private:
    static constexpr const char *DEFAULT_COM_PORT = "COM6";
    static constexpr int DEFAULT_BAUD_RATE = 115200;
    static constexpr int STATUS_TIMER_MS = 1000;
    static constexpr int COMMAND_DEBOUNCE_MS = 100;
    static constexpr int MIN_ALLOWED_VALUE = -10000000;
    static constexpr int MAX_ALLOWED_VALUE =  10000000;
    static constexpr int DEFAULT_VELMIN_RAW = -4000000;
    static constexpr int DEFAULT_VELMAX_RAW = 4000000;
    static constexpr int DEFAULT_VELOCITY_RAW = 4000000;
    static constexpr int DEFAULT_TORQUEMIN_VALUE = -3000;
    static constexpr int DEFAULT_TORQUEMAX_VALUE = 3000;
    // This is the torque/current limit used while running in velocity mode.
    // It is not a torque-mode target. Change this default as needed for testing.
    static constexpr int DEFAULT_VELOCITY_TORQUE_LIMIT_MIN = 0;
    static constexpr int DEFAULT_VELOCITY_TORQUE_LIMIT_MAX = 10000;
    static constexpr int DEFAULT_VELOCITY_TORQUE_LIMIT_RAW = 1000;

    QThread workerThread;
    bool shutdownDone = false;
    MotorWorker *worker = nullptr;

    QLineEdit *portEdit = nullptr;
    QPushButton *connectButton = nullptr;

    QLabel *chipIdValueLabel = nullptr;
    QLabel *statusValueLabel = nullptr;
    QLabel *errorDotLabel = nullptr;
    QLabel *errorTextLabel = nullptr;

    QSpinBox *torqueMinSpin = nullptr;
    QSpinBox *torqueMaxSpin = nullptr;
    QSlider *torqueSlider = nullptr;
    QSpinBox *torqueDirectSpin = nullptr;
    QPushButton *applyTorqueButton = nullptr;
    QLabel *torqueValueLabel = nullptr;

    QSpinBox *velocityMinSpin = nullptr;
    QSpinBox *velocityMaxSpin = nullptr;
    QSlider *velocitySlider = nullptr;
    QSpinBox *velocityDirectSpin = nullptr;
    QPushButton *applyVelocityButton = nullptr;
    QSpinBox *velocityTorqueLimitSpin = nullptr;
    QPushButton *applyVelocityTorqueLimitButton = nullptr;
    QLabel *velocityValueLabel = nullptr;
    QLabel *velocityTorqueLimitValueLabel = nullptr;

    QHash<QString, QLabel *> feedbackLabels;

    QPushButton *estopButton = nullptr;
    QTextEdit *logText = nullptr;
    QFile logFile;

    QTimer statusTimer;
    QTimer torqueCommandTimer;
    QTimer velocityCommandTimer;
    QTimer velocityTorqueLimitCommandTimer;

    int pendingTorque = 0;
    int pendingVelocityTorqueLimitRaw = DEFAULT_VELOCITY_TORQUE_LIMIT_RAW;
    int pendingVelocityDisplayValue = 0;
    qint32 pendingVelocityRaw = 0;
    bool syncingUi = false;
    bool connectedToController = false;
    bool statusRequestPending = false;

    void buildUi();
    void applyStyleSheet();
    QWidget *createTopConnectionRow();
    QWidget *createChipStatusGroup();
    QWidget *createTorqueGroup();
    QWidget *createVelocityGroup();
    QWidget *createLiveFeedbackPanel();
    QWidget *createLogGroup();
    void setupConnections();
    void setupWorkerThread();
    void setupLogFile();

    QLabel *addFeedbackItem(QGridLayout *layout,
                            const QString &key,
                            const QString &title,
                            int row,
                            int columnPair,
                            const QString &defaultValue = "--");
    void setFeedbackValue(const QString &key, const QString &value);

    void scheduleTorqueCommand(int value);
    void scheduleVelocityCommand(int displayVelocity);
    void applyVelocityTorqueLimitNow();
    int rawToVelocityDisplay(qint32 rawVelocity) const;
    qint32 velocityDisplayToRaw(int displayVelocity) const;
    QString velocityUnitText() const;
    QString velocityUnitSuffix() const;
    int defaultVelocityMinDisplay() const;
    int defaultVelocityMaxDisplay() const;
    int defaultVelocityDisplay() const;
    void updateRange(QSpinBox *minSpin, QSpinBox *maxSpin, QSlider *slider, QSpinBox *directSpin, QLabel *valueLabel, const QString &suffix = QString());
    void setConnectedUi(bool connected);
    void setStatusText(const QString &text, bool ok);
    void clearError();
    void logAction(const QString &action);
};

#endif
