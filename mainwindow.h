#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>

class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QSpinBox;
class QSlider;
class QGroupBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void connectAndInitialize();
    void emergencyStop();

    void applyTorque();
    void applyVelocity();

    void onTorqueSliderChanged(int value);
    void onVelocitySliderChanged(int value);

    void readSerialData();
    void handleSerialError(QSerialPort::SerialPortError error);

    void pollStatus();
    void sendPendingVelocityAfterDelay();

private:
    void buildUi();
    void applyStyle();

    QGroupBox *createConnectionGroup();
    QGroupBox *createStatusGroup();
    QGroupBox *createEmergencyGroup();
    QGroupBox *createTorqueGroup();
    QGroupBox *createVelocityGroup();
    QGroupBox *createLogGroup();

    void sendCommand(const QString &command);
    void logMessage(const QString &message);

    void setStatusDisconnected();
    void setStatusConnected();
    void setStatusInitialized();
    void setStatusError(const QString &errorText);

    void setErrorState(bool error, const QString &text);
    bool isDirectionReversal(int newVelocity) const;

private:
    QSerialPort *serial;
    QTimer *statusTimer;
    QTimer *reverseDelayTimer;

    QLineEdit *portEdit;
    QPushButton *connectInitButton;

    QLabel *chipIdValueLabel;
    QLabel *statusLampLabel;
    QLabel *statusTextLabel;
    QLabel *errorLampLabel;
    QLabel *errorTextLabel;

    QPushButton *estopButton;

    QSpinBox *torqueMinSpin;
    QSpinBox *torqueMaxSpin;
    QSpinBox *torqueDirectSpin;
    QSlider *torqueSlider;
    QLabel *torqueValueLabel;
    QPushButton *applyTorqueButton;

    QSpinBox *velocityMinSpin;
    QSpinBox *velocityMaxSpin;
    QSpinBox *velocityDirectSpin;
    QSlider *velocitySlider;
    QLabel *velocityValueLabel;
    QPushButton *applyVelocityButton;

    QTextEdit *logText;

    bool initialized;
    int currentVelocity;
    int pendingVelocity;
};

#endif
