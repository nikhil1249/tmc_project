#include "mainwindow.h"

#include <QApplication>
#include <QDateTime>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSerialPortInfo>
#include <QSlider>
#include <QSpinBox>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    serial(new QSerialPort(this)),
    statusTimer(new QTimer(this)),
    reverseDelayTimer(new QTimer(this)),
    initialized(false),
    currentVelocity(0),
    pendingVelocity(0)
{
    buildUi();
    applyStyle();

    connect(serial, &QSerialPort::readyRead,
            this, &MainWindow::readSerialData);

    connect(serial, &QSerialPort::errorOccurred,
            this, &MainWindow::handleSerialError);

    connect(statusTimer, &QTimer::timeout,
            this, &MainWindow::pollStatus);

    connect(reverseDelayTimer, &QTimer::timeout,
            this, &MainWindow::sendPendingVelocityAfterDelay);

    reverseDelayTimer->setSingleShot(true);

    setStatusDisconnected();
    setErrorState(false, "No error");

    setWindowTitle("TMC6460 Motor Control");
    resize(1280, 720);
}

MainWindow::~MainWindow()
{
    if (serial->isOpen()) {
        sendCommand("VEL 0");
        serial->close();
    }
}

void MainWindow::buildUi()
{
    QWidget *central = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    mainLayout->setContentsMargins(18, 16, 18, 16);
    mainLayout->setSpacing(14);

    QLabel *titleLabel = new QLabel("TMC6460 Motor Control");
    QFont titleFont;
    titleFont.setPointSize(22);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setMinimumHeight(38);

    mainLayout->addWidget(titleLabel);
    mainLayout->addWidget(createConnectionGroup());

    QHBoxLayout *statusEmergencyLayout = new QHBoxLayout;
    statusEmergencyLayout->setSpacing(14);
    statusEmergencyLayout->addWidget(createStatusGroup(), 3);
    statusEmergencyLayout->addWidget(createEmergencyGroup(), 1);

    mainLayout->addLayout(statusEmergencyLayout);
    mainLayout->addWidget(createTorqueGroup());
    mainLayout->addWidget(createVelocityGroup());
    mainLayout->addWidget(createLogGroup(), 1);

    setCentralWidget(central);
}

QGroupBox *MainWindow::createConnectionGroup()
{
    QGroupBox *group = new QGroupBox("Connection");
    QHBoxLayout *layout = new QHBoxLayout(group);

    layout->setContentsMargins(14, 22, 14, 14);
    layout->setSpacing(12);

    QLabel *portLabel = new QLabel("Bridge COM Port:");
    portEdit = new QLineEdit;
    portEdit->setText("COM6");
    portEdit->setFixedWidth(130);
    portEdit->setMinimumHeight(34);

    connectInitButton = new QPushButton("Connect + Initialize");
    connectInitButton->setMinimumHeight(36);
    connectInitButton->setFixedWidth(190);

    layout->addWidget(portLabel);
    layout->addWidget(portEdit);
    layout->addWidget(connectInitButton);
    layout->addStretch(1);

    connect(connectInitButton, &QPushButton::clicked,
            this, &MainWindow::connectAndInitialize);

    return group;
}

QGroupBox *MainWindow::createStatusGroup()
{
    QGroupBox *group = new QGroupBox("Chip Status");
    QGridLayout *layout = new QGridLayout(group);

    layout->setContentsMargins(14, 26, 14, 18);
    layout->setHorizontalSpacing(14);
    layout->setVerticalSpacing(16);

    QLabel *chipIdLabel = new QLabel("Chip ID:");
    chipIdValueLabel = new QLabel("----");
    chipIdValueLabel->setObjectName("chipValue");

    QLabel *statusLabel = new QLabel("Status:");
    statusLampLabel = new QLabel;
    statusLampLabel->setFixedSize(13, 13);

    statusTextLabel = new QLabel("Disconnected");
    statusTextLabel->setMinimumWidth(140);
    statusTextLabel->setAlignment(Qt::AlignCenter);
    statusTextLabel->setObjectName("statusText");

    QLabel *errorLabel = new QLabel("Error:");
    errorLampLabel = new QLabel;
    errorLampLabel->setFixedSize(13, 13);

    errorTextLabel = new QLabel("No error");

    layout->addWidget(chipIdLabel,       0, 0);
    layout->addWidget(chipIdValueLabel,  0, 1);
    layout->addWidget(statusLabel,       0, 2);
    layout->addWidget(statusLampLabel,   0, 3);
    layout->addWidget(statusTextLabel,   0, 4);

    layout->addWidget(errorLabel,        1, 0);
    layout->addWidget(errorLampLabel,    1, 1);
    layout->addWidget(errorTextLabel,    1, 2, 1, 3);

    layout->setColumnStretch(5, 1);

    return group;
}

QGroupBox *MainWindow::createEmergencyGroup()
{
    QGroupBox *group = new QGroupBox("Emergency Stop");
    QVBoxLayout *layout = new QVBoxLayout(group);

    layout->setContentsMargins(16, 26, 16, 18);

    estopButton = new QPushButton("E-STOP");
    estopButton->setObjectName("estopButton");
    estopButton->setMinimumHeight(82);
    estopButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    layout->addWidget(estopButton);

    connect(estopButton, &QPushButton::clicked,
            this, &MainWindow::emergencyStop);

    return group;
}

QGroupBox *MainWindow::createTorqueGroup()
{
    QGroupBox *group = new QGroupBox("Torque Control");
    QVBoxLayout *main = new QVBoxLayout(group);

    main->setContentsMargins(14, 24, 14, 14);
    main->setSpacing(10);

    QHBoxLayout *top = new QHBoxLayout;
    top->setSpacing(10);

    torqueMinSpin = new QSpinBox;
    torqueMaxSpin = new QSpinBox;
    torqueDirectSpin = new QSpinBox;

    torqueMinSpin->setRange(-1000000, 1000000);
    torqueMaxSpin->setRange(-1000000, 1000000);
    torqueDirectSpin->setRange(-1000000, 1000000);

    torqueMinSpin->setValue(0);
    torqueMaxSpin->setValue(3000);
    torqueDirectSpin->setValue(0);

    torqueMinSpin->setFixedWidth(120);
    torqueMaxSpin->setFixedWidth(120);
    torqueDirectSpin->setFixedWidth(150);

    torqueMinSpin->setMinimumHeight(32);
    torqueMaxSpin->setMinimumHeight(32);
    torqueDirectSpin->setMinimumHeight(32);

    applyTorqueButton = new QPushButton("Apply Torque");
    applyTorqueButton->setMinimumHeight(36);
    applyTorqueButton->setFixedWidth(140);

    top->addWidget(new QLabel("Min:"));
    top->addWidget(torqueMinSpin);
    top->addSpacing(8);
    top->addWidget(new QLabel("Max:"));
    top->addWidget(torqueMaxSpin);
    top->addSpacing(8);
    top->addWidget(new QLabel("Direct Value:"));
    top->addStretch(1);
    top->addWidget(torqueDirectSpin);
    top->addWidget(new QLabel("RAW"));
    top->addWidget(applyTorqueButton);

    QHBoxLayout *bottom = new QHBoxLayout;
    bottom->setSpacing(14);

    torqueSlider = new QSlider(Qt::Horizontal);
    torqueSlider->setRange(0, 3000);
    torqueSlider->setValue(0);
    torqueSlider->setMinimumHeight(34);

    QLabel *valueText = new QLabel("Value:");
    torqueValueLabel = new QLabel("0");
    torqueValueLabel->setObjectName("valueNumber");
    torqueValueLabel->setMinimumWidth(70);

    bottom->addWidget(torqueSlider, 1);
    bottom->addWidget(valueText);
    bottom->addWidget(torqueValueLabel);

    main->addLayout(top);
    main->addLayout(bottom);

    connect(torqueSlider, &QSlider::valueChanged,
            this, &MainWindow::onTorqueSliderChanged);

    connect(torqueDirectSpin, qOverload<int>(&QSpinBox::valueChanged),
            torqueSlider, &QSlider::setValue);

    connect(applyTorqueButton, &QPushButton::clicked,
            this, &MainWindow::applyTorque);

    connect(torqueMinSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this]() {
        torqueSlider->setRange(torqueMinSpin->value(), torqueMaxSpin->value());
    });

    connect(torqueMaxSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this]() {
        torqueSlider->setRange(torqueMinSpin->value(), torqueMaxSpin->value());
    });

    return group;
}

QGroupBox *MainWindow::createVelocityGroup()
{
    QGroupBox *group = new QGroupBox("Velocity Control");
    QVBoxLayout *main = new QVBoxLayout(group);

    main->setContentsMargins(14, 24, 14, 14);
    main->setSpacing(10);

    QHBoxLayout *top = new QHBoxLayout;
    top->setSpacing(10);

    velocityMinSpin = new QSpinBox;
    velocityMaxSpin = new QSpinBox;
    velocityDirectSpin = new QSpinBox;

    velocityMinSpin->setRange(-5000000, 5000000);
    velocityMaxSpin->setRange(-5000000, 5000000);
    velocityDirectSpin->setRange(-5000000, 5000000);

    velocityMinSpin->setValue(-100000);
    velocityMaxSpin->setValue(4000000);
    velocityDirectSpin->setValue(0);

    velocityMinSpin->setFixedWidth(130);
    velocityMaxSpin->setFixedWidth(130);
    velocityDirectSpin->setFixedWidth(160);

    velocityMinSpin->setMinimumHeight(32);
    velocityMaxSpin->setMinimumHeight(32);
    velocityDirectSpin->setMinimumHeight(32);

    applyVelocityButton = new QPushButton("Apply Velocity");
    applyVelocityButton->setMinimumHeight(36);
    applyVelocityButton->setFixedWidth(150);

    top->addWidget(new QLabel("Min:"));
    top->addWidget(velocityMinSpin);
    top->addSpacing(8);
    top->addWidget(new QLabel("Max:"));
    top->addWidget(velocityMaxSpin);
    top->addSpacing(8);
    top->addWidget(new QLabel("Direct Value:"));
    top->addStretch(1);
    top->addWidget(velocityDirectSpin);
    top->addWidget(new QLabel("Internal"));
    top->addWidget(applyVelocityButton);

    QHBoxLayout *bottom = new QHBoxLayout;
    bottom->setSpacing(14);

    velocitySlider = new QSlider(Qt::Horizontal);
    velocitySlider->setRange(-100000, 4000000);
    velocitySlider->setValue(0);
    velocitySlider->setMinimumHeight(34);

    QLabel *valueText = new QLabel("Value:");
    velocityValueLabel = new QLabel("0");
    velocityValueLabel->setObjectName("valueNumber");
    velocityValueLabel->setMinimumWidth(80);

    bottom->addWidget(velocitySlider, 1);
    bottom->addWidget(valueText);
    bottom->addWidget(velocityValueLabel);

    main->addLayout(top);
    main->addLayout(bottom);

    connect(velocitySlider, &QSlider::valueChanged,
            this, &MainWindow::onVelocitySliderChanged);

    connect(velocityDirectSpin, qOverload<int>(&QSpinBox::valueChanged),
            velocitySlider, &QSlider::setValue);

    connect(applyVelocityButton, &QPushButton::clicked,
            this, &MainWindow::applyVelocity);

    connect(velocityMinSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this]() {
        velocitySlider->setRange(velocityMinSpin->value(), velocityMaxSpin->value());
    });

    connect(velocityMaxSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this]() {
        velocitySlider->setRange(velocityMinSpin->value(), velocityMaxSpin->value());
    });

    return group;
}

QGroupBox *MainWindow::createLogGroup()
{
    QGroupBox *group = new QGroupBox("UART / Register Log");
    QVBoxLayout *layout = new QVBoxLayout(group);

    layout->setContentsMargins(14, 24, 14, 14);

    logText = new QTextEdit;
    logText->setReadOnly(true);
    logText->setMinimumHeight(190);
    logText->setObjectName("logText");

    layout->addWidget(logText);

    return group;
}

void MainWindow::applyStyle()
{
    qApp->setStyleSheet(
        "QWidget {"
        "    font-family: Segoe UI;"
        "    font-size: 10pt;"
        "    color: #06142E;"
        "}"

        "QMainWindow {"
        "    background-color: #F4F7FB;"
        "}"

        "QGroupBox {"
        "    background-color: #FFFFFF;"
        "    border: 1px solid #CBD3DF;"
        "    border-radius: 8px;"
        "    margin-top: 12px;"
        "    font-weight: 700;"
        "}"

        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    subcontrol-position: top left;"
        "    left: 18px;"
        "    padding: 0px 6px;"
        "    background-color: #F4F7FB;"
        "    color: #000000;"
        "}"

        "QLineEdit, QSpinBox {"
        "    background-color: #FFFFFF;"
        "    border: 1px solid #C5D0DF;"
        "    border-radius: 5px;"
        "    padding: 4px 8px;"
        "    min-height: 24px;"
        "}"

        "QLineEdit:focus, QSpinBox:focus {"
        "    border: 1px solid #2F66E8;"
        "}"

        "QPushButton {"
        "    background-color: #2F66E8;"
        "    color: white;"
        "    border: none;"
        "    border-radius: 6px;"
        "    font-weight: 700;"
        "    padding: 8px 16px;"
        "}"

        "QPushButton:hover {"
        "    background-color: #2554C7;"
        "}"

        "QPushButton:pressed {"
        "    background-color: #1F45A5;"
        "}"

        "QPushButton:disabled {"
        "    background-color: #AAB3C2;"
        "    color: #EEF1F5;"
        "}"

        "QPushButton#estopButton {"
        "    background-color: #EF233C;"
        "    border: 3px solid #B5152A;"
        "    border-radius: 34px;"
        "    font-size: 24pt;"
        "    font-weight: 900;"
        "    color: white;"
        "}"

        "QPushButton#estopButton:hover {"
        "    background-color: #D91E36;"
        "}"

        "QLabel#chipValue {"
        "    color: #174CFF;"
        "    font-weight: 800;"
        "    font-size: 11pt;"
        "}"

        "QLabel#statusText {"
        "    background-color: #FFF4F4;"
        "    border: 1px solid #FFA8A8;"
        "    border-radius: 6px;"
        "    color: #D7263D;"
        "    font-weight: 800;"
        "    padding: 8px 14px;"
        "}"

        "QLabel#valueNumber {"
        "    color: #174CFF;"
        "    font-size: 14pt;"
        "    font-weight: 900;"
        "}"

        "QTextEdit#logText {"
        "    background-color: #0B1325;"
        "    color: #D9F99D;"
        "    border-radius: 6px;"
        "    border: 1px solid #0B1325;"
        "    font-family: Consolas;"
        "    font-size: 10pt;"
        "}"

        "QSlider::groove:horizontal {"
        "    height: 7px;"
        "    background: #D4DAE3;"
        "    border-radius: 3px;"
        "}"

        "QSlider::sub-page:horizontal {"
        "    background: #2F66E8;"
        "    border-radius: 3px;"
        "}"

        "QSlider::handle:horizontal {"
        "    background: #2F66E8;"
        "    border: 1px solid #1743AA;"
        "    width: 20px;"
        "    height: 20px;"
        "    margin: -8px 0px;"
        "    border-radius: 10px;"
        "}"
        );
}

void MainWindow::connectAndInitialize()
{
    if (serial->isOpen()) {
        sendCommand("VEL 0");
        serial->close();
        statusTimer->stop();
        initialized = false;
        currentVelocity = 0;

        setStatusDisconnected();
        logMessage("Disconnected.");
        connectInitButton->setText("Connect + Initialize");
        return;
    }

    const QString portName = portEdit->text().trimmed();

    if (portName.isEmpty()) {
        setStatusError("COM port empty");
        logMessage("ERROR: COM port is empty.");
        return;
    }

    serial->setPortName(portName);
    serial->setBaudRate(115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite)) {
        setStatusError("Open failed");
        logMessage("ERROR: Could not open " + portName + ": " + serial->errorString());
        return;
    }

    serial->clear();

    setStatusConnected();
    setErrorState(false, "No error");
    logMessage("Connected to " + portName + " at 115200 baud.");

    sendCommand("VEL 0");
    sendCommand("INIT");

    initialized = true;
    currentVelocity = 0;
    pendingVelocity = 0;

    setStatusInitialized();
    connectInitButton->setText("Disconnect");

    statusTimer->start(1000);
}

void MainWindow::emergencyStop()
{
    reverseDelayTimer->stop();

    if (serial->isOpen()) {
        sendCommand("VEL 0");
        sendCommand("ESTOP");
    }

    initialized = false;
    currentVelocity = 0;
    pendingVelocity = 0;

    velocitySlider->setValue(0);
    velocityDirectSpin->setValue(0);

    setStatusError("E-STOP active");
    logMessage("E-STOP activated. Click Connect + Initialize again before running.");
}

void MainWindow::applyTorque()
{
    if (!serial->isOpen() || !initialized) {
        logMessage("ERROR: Not connected / not initialized.");
        return;
    }

    const int torque = torqueDirectSpin->value();

    sendCommand("TORQUE " + QString::number(torque));
    logMessage("Torque applied: " + QString::number(torque));
}

void MainWindow::applyVelocity()
{
    if (!serial->isOpen() || !initialized) {
        logMessage("ERROR: Not connected / not initialized.");
        return;
    }

    const int newVelocity = velocityDirectSpin->value();
    pendingVelocity = newVelocity;

    if (isDirectionReversal(newVelocity)) {
        logMessage("Direction reversal detected. Sending VEL 0 first to avoid jerk.");

        sendCommand("VEL 0");
        currentVelocity = 0;

        reverseDelayTimer->start(700);
        return;
    }

    sendCommand("VEL " + QString::number(newVelocity));
    currentVelocity = newVelocity;

    logMessage("Velocity applied: " + QString::number(newVelocity));
}

void MainWindow::sendPendingVelocityAfterDelay()
{
    if (!serial->isOpen() || !initialized) {
        return;
    }

    sendCommand("VEL " + QString::number(pendingVelocity));
    currentVelocity = pendingVelocity;

    logMessage("Velocity applied after reversal delay: " + QString::number(currentVelocity));
}

void MainWindow::onTorqueSliderChanged(int value)
{
    torqueDirectSpin->blockSignals(true);
    torqueDirectSpin->setValue(value);
    torqueDirectSpin->blockSignals(false);

    torqueValueLabel->setText(QString::number(value));
}

void MainWindow::onVelocitySliderChanged(int value)
{
    velocityDirectSpin->blockSignals(true);
    velocityDirectSpin->setValue(value);
    velocityDirectSpin->blockSignals(false);

    velocityValueLabel->setText(QString::number(value));
}

void MainWindow::readSerialData()
{
    const QByteArray data = serial->readAll();

    if (data.isEmpty()) {
        return;
    }

    const QString text = QString::fromLocal8Bit(data);

    logText->moveCursor(QTextCursor::End);
    logText->insertPlainText(text);
    logText->moveCursor(QTextCursor::End);

    if (text.contains("CHIP.ID", Qt::CaseInsensitive) ||
        text.contains("CHIP ID", Qt::CaseInsensitive)) {

        int index = text.indexOf("0x", 0, Qt::CaseInsensitive);
        if (index >= 0 && text.length() >= index + 10) {
            chipIdValueLabel->setText(text.mid(index, 10));
        } else {
            chipIdValueLabel->setText("Detected");
        }
    }

    if (text.contains("ERROR", Qt::CaseInsensitive) ||
        text.contains("FAULT", Qt::CaseInsensitive) ||
        text.contains("FAIL", Qt::CaseInsensitive)) {
        setErrorState(true, "Check log");
    }

    if (text.contains("GDRV_ON", Qt::CaseInsensitive) ||
        text.contains("READY", Qt::CaseInsensitive) ||
        text.contains("Initialized", Qt::CaseInsensitive)) {
        setStatusInitialized();
    }
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError) {
        return;
    }

    if (error == QSerialPort::ResourceError) {
        setStatusError("Serial disconnected");
        logMessage("ERROR: Serial disconnected unexpectedly.");

        statusTimer->stop();
        serial->close();
        initialized = false;
        connectInitButton->setText("Connect + Initialize");
        return;
    }

    logMessage("Serial error: " + serial->errorString());
}

void MainWindow::pollStatus()
{
    if (serial->isOpen()) {
        sendCommand("STATUS");
    }
}

void MainWindow::sendCommand(const QString &command)
{
    if (!serial->isOpen()) {
        logMessage("ERROR: Serial port not open.");
        return;
    }

    const QString tx = command.trimmed() + "\n";

    serial->write(tx.toUtf8());
    serial->flush();

    logMessage("TX: " + command.trimmed());
}

void MainWindow::logMessage(const QString &message)
{
    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");

    logText->append("[" + timestamp + "] " + message);
    logText->verticalScrollBar()->setValue(logText->verticalScrollBar()->maximum());
}

void MainWindow::setStatusDisconnected()
{
    statusLampLabel->setStyleSheet(
        "background-color: #D7263D;"
        "border-radius: 6px;"
        );

    statusTextLabel->setText("Disconnected");
    statusTextLabel->setStyleSheet(
        "background-color: #FFF4F4;"
        "border: 1px solid #FFA8A8;"
        "border-radius: 6px;"
        "color: #D7263D;"
        "font-weight: 800;"
        "padding: 8px 14px;"
        );
}

void MainWindow::setStatusConnected()
{
    statusLampLabel->setStyleSheet(
        "background-color: #F59E0B;"
        "border-radius: 6px;"
        );

    statusTextLabel->setText("Connected");
    statusTextLabel->setStyleSheet(
        "background-color: #FFF7E6;"
        "border: 1px solid #FBCB7A;"
        "border-radius: 6px;"
        "color: #B45309;"
        "font-weight: 800;"
        "padding: 8px 14px;"
        );
}

void MainWindow::setStatusInitialized()
{
    statusLampLabel->setStyleSheet(
        "background-color: #16A34A;"
        "border-radius: 6px;"
        );

    statusTextLabel->setText("Initialized");
    statusTextLabel->setStyleSheet(
        "background-color: #ECFDF3;"
        "border: 1px solid #86EFAC;"
        "border-radius: 6px;"
        "color: #15803D;"
        "font-weight: 800;"
        "padding: 8px 14px;"
        );
}

void MainWindow::setStatusError(const QString &errorText)
{
    statusLampLabel->setStyleSheet(
        "background-color: #D7263D;"
        "border-radius: 6px;"
        );

    statusTextLabel->setText(errorText);
    statusTextLabel->setStyleSheet(
        "background-color: #FFF4F4;"
        "border: 1px solid #FFA8A8;"
        "border-radius: 6px;"
        "color: #D7263D;"
        "font-weight: 800;"
        "padding: 8px 14px;"
        );

    setErrorState(true, errorText);
}

void MainWindow::setErrorState(bool error, const QString &text)
{
    if (error) {
        errorLampLabel->setStyleSheet(
            "background-color: #D7263D;"
            "border-radius: 6px;"
            );
    } else {
        errorLampLabel->setStyleSheet(
            "background-color: #16A34A;"
            "border-radius: 6px;"
            );
    }

    errorTextLabel->setText(text);
}

bool MainWindow::isDirectionReversal(int newVelocity) const
{
    if (currentVelocity == 0 || newVelocity == 0) {
        return false;
    }

    return ((currentVelocity > 0) != (newVelocity > 0));
}
