#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "Tmc6460QtInterface.h"

#include <QMainWindow>
#include <QTimer>

class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QTextEdit;
class QLineEdit;
class QWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override = default;

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
    void onVelocityRangeChanged();

    void onEstopClicked();
    void sendPendingTorqueCommand();
    void sendPendingVelocityCommand();
    void updateChipStatus();
    void appendLog(const QString &message);
    void showError(const QString &message);

private:
    static constexpr const char *DEFAULT_COM_PORT = "COM6";
    static constexpr int DEFAULT_BAUD_RATE = 115200;
    static constexpr int STATUS_TIMER_MS = 1000;
    static constexpr int COMMAND_DEBOUNCE_MS = 100;
    static constexpr int MIN_ALLOWED_VALUE = -10000000;
    static constexpr int MAX_ALLOWED_VALUE =  10000000;
    static constexpr int DEFAULT_VELMIN_VALUE = -4000000;
    static constexpr int DEFAULT_VELMAX_VALUE = 4000000;
    static constexpr int DEFAULT_TORQUEMIN_VALUE = 0;
    static constexpr int DEFAULT_TORQUEMAX_VALUE = 8000;

    Tmc6460QtInterface tmc;

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
    QLabel *velocityValueLabel = nullptr;

    QPushButton *estopButton = nullptr;
    QTextEdit *logText = nullptr;

    QTimer statusTimer;
    QTimer torqueCommandTimer;
    QTimer velocityCommandTimer;

    int pendingTorque = 0;
    int pendingVelocity = 0;
    bool syncingUi = false;

    void buildUi();
    void applyStyleSheet();
    QWidget *createConnectionGroup();
    QWidget *createStatusAndEstopRow();
    QWidget *createChipStatusGroup();
    QWidget *createEstopPanel();
    QWidget *createTorqueGroup();
    QWidget *createVelocityGroup();
    QWidget *createLogGroup();
    void setupConnections();

    void scheduleTorqueCommand(int value);
    void scheduleVelocityCommand(int value);
    void updateRange(QSpinBox *minSpin, QSpinBox *maxSpin, QSlider *slider, QSpinBox *directSpin, QLabel *valueLabel);
    void setConnectedUi(bool connected);
    void setStatusText(const QString &text, bool ok);
    void clearError();
};

#endif
