#include "mainwindow.h"

#include <QDateTime>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTextEdit>
#include <QStyle>
#include <QVBoxLayout>
#include <QtGlobal>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    buildUi();
    applyStyleSheet();
    setupConnections();
    setConnectedUi(false);
}

void MainWindow::buildUi()
{
    setWindowTitle("TMC6460 Motor Control");
    resize(1120, 760);

    QWidget *central = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(16, 16, 16, 16);
    mainLayout->setSpacing(12);

    QLabel *title = new QLabel("TMC6460 Motor Control");
    title->setObjectName("titleLabel");
    mainLayout->addWidget(title);

    mainLayout->addWidget(createConnectionGroup());
    mainLayout->addWidget(createStatusAndEstopRow());
    mainLayout->addWidget(createTorqueGroup());
    mainLayout->addWidget(createVelocityGroup());
    mainLayout->addWidget(createLogGroup(), 1);

    setCentralWidget(central);
}

QWidget *MainWindow::createConnectionGroup()
{
    QGroupBox *group = new QGroupBox("Connection");
    QHBoxLayout *layout = new QHBoxLayout(group);
    layout->setContentsMargins(16, 22, 16, 16);
    layout->setSpacing(10);

    layout->addWidget(new QLabel("Arduino Bridge COM Port:"));

    portEdit = new QLineEdit(DEFAULT_COM_PORT);
    portEdit->setMaximumWidth(160);
    layout->addWidget(portEdit);

    connectButton = new QPushButton("Connect + Initialize");
    connectButton->setMinimumWidth(190);
    layout->addWidget(connectButton);

    layout->addStretch(1);
    return group;
}

QWidget *MainWindow::createStatusAndEstopRow()
{
    QWidget *row = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(12);

    QWidget *chipStatus = createChipStatusGroup();
    QWidget *estopPanel = createEstopPanel();

    layout->addWidget(chipStatus, 3);
    layout->addWidget(estopPanel, 1);

    return row;
}

QWidget *MainWindow::createChipStatusGroup()
{
    QGroupBox *group = new QGroupBox("Chip Status");
    group->setMinimumHeight(135);

    QGridLayout *layout = new QGridLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);
    layout->setHorizontalSpacing(14);
    layout->setVerticalSpacing(10);

    layout->addWidget(new QLabel("Chip ID:"), 0, 0);
    chipIdValueLabel = new QLabel("----");
    chipIdValueLabel->setObjectName("blueValueLabel");
    layout->addWidget(chipIdValueLabel, 0, 1);

    layout->addWidget(new QLabel("Status:"), 0, 2);
    statusValueLabel = new QLabel("Disconnected");
    statusValueLabel->setObjectName("statusBad");
    layout->addWidget(statusValueLabel, 0, 3);

    layout->addWidget(new QLabel("Error:"), 1, 0);
    errorDotLabel = new QLabel("●");
    errorDotLabel->setObjectName("dotGreen");
    layout->addWidget(errorDotLabel, 1, 1);

    errorTextLabel = new QLabel("No error");
    errorTextLabel->setWordWrap(true);
    layout->addWidget(errorTextLabel, 1, 2, 1, 2);

    layout->setColumnStretch(4, 1);
    return group;
}

QWidget *MainWindow::createEstopPanel()
{
    QGroupBox *group = new QGroupBox("Emergency Stop");
    group->setMinimumHeight(135);

    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(16, 26, 16, 16);
    layout->setSpacing(8);

    estopButton = new QPushButton("E-STOP");
    estopButton->setObjectName("estopButton");
    estopButton->setMinimumHeight(82);
    estopButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(estopButton);

    return group;
}

QWidget *MainWindow::createTorqueGroup()
{
    QGroupBox *group = new QGroupBox("Torque Control");
    QGridLayout *layout = new QGridLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);
    layout->setHorizontalSpacing(12);
    layout->setVerticalSpacing(12);

    torqueMinSpin = new QSpinBox;
    torqueMinSpin->setRange(MIN_ALLOWED_VALUE, MAX_ALLOWED_VALUE);
    torqueMinSpin->setValue(0);
    torqueMinSpin->setMinimumWidth(130);

    torqueMaxSpin = new QSpinBox;
    torqueMaxSpin->setRange(MIN_ALLOWED_VALUE, MAX_ALLOWED_VALUE);
    torqueMaxSpin->setValue(3000);
    torqueMaxSpin->setMinimumWidth(130);

    torqueDirectSpin = new QSpinBox;
    torqueDirectSpin->setRange(0, 3000);
    torqueDirectSpin->setValue(0);
    torqueDirectSpin->setMinimumWidth(150);
    torqueDirectSpin->setKeyboardTracking(false);   // valueChanged is emitted after Enter/focus-out, not on every digit

    applyTorqueButton = new QPushButton("Apply Torque");
    applyTorqueButton->setMinimumWidth(130);

    torqueSlider = new QSlider(Qt::Horizontal);
    torqueSlider->setRange(0, 3000);
    torqueSlider->setValue(0);
    torqueSlider->setTracking(true);      // display live value while sliding
    torqueSlider->setTickPosition(QSlider::TicksBelow);
    torqueSlider->setTickInterval(250);

    torqueValueLabel = new QLabel("0");
    torqueValueLabel->setObjectName("blueValueLabel");
    torqueValueLabel->setMinimumWidth(120);

    layout->addWidget(new QLabel("Min:"), 0, 0);
    layout->addWidget(torqueMinSpin, 0, 1);
    layout->addWidget(new QLabel("Max:"), 0, 2);
    layout->addWidget(torqueMaxSpin, 0, 3);
    layout->addWidget(new QLabel("Direct Value:"), 0, 4);
    layout->addWidget(torqueDirectSpin, 0, 5);
    layout->addWidget(new QLabel("RAW"), 0, 6);
    layout->addWidget(applyTorqueButton, 0, 7);

    layout->addWidget(torqueSlider, 1, 0, 1, 6);
    layout->addWidget(new QLabel("Value:"), 1, 6);
    layout->addWidget(torqueValueLabel, 1, 7);

    layout->setColumnStretch(4, 1);
    return group;
}

QWidget *MainWindow::createVelocityGroup()
{
    QGroupBox *group = new QGroupBox("Velocity Control");
    QGridLayout *layout = new QGridLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);
    layout->setHorizontalSpacing(12);
    layout->setVerticalSpacing(12);

    velocityMinSpin = new QSpinBox;
    velocityMinSpin->setRange(MIN_ALLOWED_VALUE, MAX_ALLOWED_VALUE);
    velocityMinSpin->setValue(-100000);
    velocityMinSpin->setMinimumWidth(130);

    velocityMaxSpin = new QSpinBox;
    velocityMaxSpin->setRange(MIN_ALLOWED_VALUE, MAX_ALLOWED_VALUE);
    velocityMaxSpin->setValue(4000000);
    velocityMaxSpin->setMinimumWidth(130);

    velocityDirectSpin = new QSpinBox;
    velocityDirectSpin->setRange(-100000, 4000000);
    velocityDirectSpin->setValue(0);
    velocityDirectSpin->setMinimumWidth(150);
    velocityDirectSpin->setKeyboardTracking(false); // send only when Enter/focus-out or Apply is clicked

    applyVelocityButton = new QPushButton("Apply Velocity");
    applyVelocityButton->setMinimumWidth(140);

    velocitySlider = new QSlider(Qt::Horizontal);
    velocitySlider->setRange(-100000, 4000000);
    velocitySlider->setValue(0);
    velocitySlider->setTracking(true);    // display live value while sliding
    velocitySlider->setTickPosition(QSlider::TicksBelow);
    velocitySlider->setTickInterval(500000);

    velocityValueLabel = new QLabel("0");
    velocityValueLabel->setObjectName("blueValueLabel");
    velocityValueLabel->setMinimumWidth(140);

    layout->addWidget(new QLabel("Min:"), 0, 0);
    layout->addWidget(velocityMinSpin, 0, 1);
    layout->addWidget(new QLabel("Max:"), 0, 2);
    layout->addWidget(velocityMaxSpin, 0, 3);
    layout->addWidget(new QLabel("Direct Value:"), 0, 4);
    layout->addWidget(velocityDirectSpin, 0, 5);
    layout->addWidget(new QLabel("Internal"), 0, 6);
    layout->addWidget(applyVelocityButton, 0, 7);

    layout->addWidget(velocitySlider, 1, 0, 1, 6);
    layout->addWidget(new QLabel("Value:"), 1, 6);
    layout->addWidget(velocityValueLabel, 1, 7);

    layout->setColumnStretch(4, 1);
    return group;
}

QWidget *MainWindow::createLogGroup()
{
    QGroupBox *group = new QGroupBox("UART / Register Log");
    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);

    logText = new QTextEdit;
    logText->setReadOnly(true);
    logText->setMinimumHeight(170);
    layout->addWidget(logText);

    return group;
}

void MainWindow::applyStyleSheet()
{
    setStyleSheet(
        "QMainWindow { background-color: #F5F7FA; }"
        "QWidget { font-family: Segoe UI; font-size: 14px; color: #111827; }"
        "#titleLabel { font-size: 28px; font-weight: bold; color: #111827; padding: 4px; }"
        "QGroupBox { background-color: #FFFFFF; border: 1px solid #D1D5DB; border-radius: 10px; margin-top: 12px; font-size: 17px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0px 8px; left: 12px; }"
        "QLineEdit, QSpinBox { background-color: #FFFFFF; border: 1px solid #CBD5E1; border-radius: 6px; padding: 5px; font-size: 15px; }"
        "QPushButton { background-color: #2563EB; color: white; border-radius: 7px; padding: 8px 16px; font-weight: bold; }"
        "QPushButton:hover { background-color: #1D4ED8; }"
        "QPushButton:pressed { background-color: #1E40AF; }"
        "#estopButton { background-color: #EF233C; color: white; font-size: 30px; font-weight: bold; border-radius: 34px; border: 2px solid #B91C1C; }"
        "#estopButton:hover { background-color: #DC2626; }"
        "#estopButton:pressed { background-color: #991B1B; }"
        "QSlider::groove:horizontal { height: 8px; background: #D1D5DB; border-radius: 4px; }"
        "QSlider::handle:horizontal { background: #2563EB; border: 2px solid #1D4ED8; width: 22px; height: 22px; margin: -8px 0; border-radius: 11px; }"
        "QSlider::sub-page:horizontal { background: #2563EB; border-radius: 4px; }"
        "#blueValueLabel { color: #1D4ED8; font-size: 20px; font-weight: bold; }"
        "#statusGood { background-color: #ECFDF5; color: #16A34A; border: 1px solid #BBF7D0; border-radius: 8px; padding: 8px 20px; font-size: 17px; font-weight: bold; }"
        "#statusBad { background-color: #FEF2F2; color: #DC2626; border: 1px solid #FCA5A5; border-radius: 8px; padding: 8px 20px; font-size: 17px; font-weight: bold; }"
        "#dotGreen { color: #16A34A; font-size: 26px; }"
        "#dotRed { color: #DC2626; font-size: 26px; }"
        "QTextEdit { background-color: #0F172A; color: #E5E7EB; border-radius: 8px; font-family: Consolas; font-size: 12px; }"
        );
}

void MainWindow::setupConnections()
{
    connect(connectButton, &QPushButton::clicked, this, &MainWindow::connectAndInitialize);

    connect(&tmc, &Tmc6460QtInterface::logMessage, this, &MainWindow::appendLog);
    connect(&tmc, &Tmc6460QtInterface::errorChanged, this, &MainWindow::showError);

    connect(torqueSlider, &QSlider::valueChanged, this, &MainWindow::onTorqueSliderValueChanged);
    connect(torqueSlider, &QSlider::sliderReleased, this, &MainWindow::onTorqueSliderReleased);
    connect(torqueDirectSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onTorqueDirectValueChanged);
    connect(torqueDirectSpin, &QSpinBox::editingFinished, this, &MainWindow::onTorqueDirectEditingFinished);
    connect(applyTorqueButton, &QPushButton::clicked, this, &MainWindow::onApplyTorqueClicked);
    connect(torqueMinSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onTorqueRangeChanged);
    connect(torqueMaxSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onTorqueRangeChanged);

    connect(velocitySlider, &QSlider::valueChanged, this, &MainWindow::onVelocitySliderValueChanged);
    connect(velocitySlider, &QSlider::sliderReleased, this, &MainWindow::onVelocitySliderReleased);
    connect(velocityDirectSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onVelocityDirectValueChanged);
    connect(velocityDirectSpin, &QSpinBox::editingFinished, this, &MainWindow::onVelocityDirectEditingFinished);
    connect(applyVelocityButton, &QPushButton::clicked, this, &MainWindow::onApplyVelocityClicked);
    connect(velocityMinSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onVelocityRangeChanged);
    connect(velocityMaxSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onVelocityRangeChanged);

    connect(estopButton, &QPushButton::clicked, this, &MainWindow::onEstopClicked);

    torqueCommandTimer.setSingleShot(true);
    velocityCommandTimer.setSingleShot(true);
    connect(&torqueCommandTimer, &QTimer::timeout, this, &MainWindow::sendPendingTorqueCommand);
    connect(&velocityCommandTimer, &QTimer::timeout, this, &MainWindow::sendPendingVelocityCommand);

    connect(&statusTimer, &QTimer::timeout, this, &MainWindow::updateChipStatus);
}

void MainWindow::connectAndInitialize()
{
    clearError();
    connectButton->setEnabled(false);

    if (!tmc.openPort(portEdit->text().trimmed(), DEFAULT_BAUD_RATE))
    {
        setConnectedUi(false);
        connectButton->setEnabled(true);
        return;
    }

    if (!tmc.initializeMotor())
    {
        setStatusText("Initialization failed", false);
        connectButton->setEnabled(true);
        return;
    }

    quint32 chipId = 0;
    if (tmc.readChipId(&chipId))
    {
        chipIdValueLabel->setText(QString("0x%1").arg(chipId, 8, 16, QLatin1Char('0')).toUpper());
    }

    setConnectedUi(true);
    statusTimer.start(STATUS_TIMER_MS);
    connectButton->setEnabled(true);
}

void MainWindow::onTorqueSliderValueChanged(int value)
{
    torqueValueLabel->setText(QString::number(value));

    QSignalBlocker blocker(torqueDirectSpin);
    torqueDirectSpin->setValue(value);
}

void MainWindow::onTorqueSliderReleased()
{
    scheduleTorqueCommand(torqueSlider->value());
}

void MainWindow::onTorqueDirectValueChanged(int value)
{
    if (syncingUi)
    {
        return;
    }

    {
        QSignalBlocker blocker(torqueSlider);
        torqueSlider->setValue(value);
    }

    torqueValueLabel->setText(QString::number(value));
}

void MainWindow::onTorqueDirectEditingFinished()
{
    scheduleTorqueCommand(torqueDirectSpin->value());
}

void MainWindow::onApplyTorqueClicked()
{
    scheduleTorqueCommand(torqueDirectSpin->value());
}

void MainWindow::onTorqueRangeChanged()
{
    updateRange(torqueMinSpin, torqueMaxSpin, torqueSlider, torqueDirectSpin, torqueValueLabel);
}

void MainWindow::onVelocitySliderValueChanged(int value)
{
    velocityValueLabel->setText(QString::number(value));

    QSignalBlocker blocker(velocityDirectSpin);
    velocityDirectSpin->setValue(value);
}

void MainWindow::onVelocitySliderReleased()
{
    scheduleVelocityCommand(velocitySlider->value());
}

void MainWindow::onVelocityDirectValueChanged(int value)
{
    if (syncingUi)
    {
        return;
    }

    {
        QSignalBlocker blocker(velocitySlider);
        velocitySlider->setValue(value);
    }

    velocityValueLabel->setText(QString::number(value));
}

void MainWindow::onVelocityDirectEditingFinished()
{
    scheduleVelocityCommand(velocityDirectSpin->value());
}

void MainWindow::onApplyVelocityClicked()
{
    scheduleVelocityCommand(velocityDirectSpin->value());
}

void MainWindow::onVelocityRangeChanged()
{
    updateRange(velocityMinSpin, velocityMaxSpin, velocitySlider, velocityDirectSpin, velocityValueLabel);
}

void MainWindow::onEstopClicked()
{
    clearError();
    torqueCommandTimer.stop();
    velocityCommandTimer.stop();

    syncingUi = true;
    {
        QSignalBlocker b1(torqueSlider);
        QSignalBlocker b2(torqueDirectSpin);
        QSignalBlocker b3(velocitySlider);
        QSignalBlocker b4(velocityDirectSpin);

        torqueSlider->setValue(0);
        torqueDirectSpin->setValue(0);
        velocitySlider->setValue(0);
        velocityDirectSpin->setValue(0);
    }
    syncingUi = false;

    torqueValueLabel->setText("0");
    velocityValueLabel->setText("0");

    if (tmc.emergencyStop())
    {
        setStatusText("E-STOP", false);
        showError("Emergency stop active. Driver disabled.");
    }
}

void MainWindow::scheduleTorqueCommand(int value)
{
    if (!tmc.isOpen())
    {
        return;
    }

    pendingTorque = value;
    torqueCommandTimer.start(COMMAND_DEBOUNCE_MS);
}

void MainWindow::scheduleVelocityCommand(int value)
{
    if (!tmc.isOpen())
    {
        return;
    }

    pendingVelocity = value;
    velocityCommandTimer.start(COMMAND_DEBOUNCE_MS);
}

void MainWindow::sendPendingTorqueCommand()
{
    if (tmc.isBusy())
    {
        torqueCommandTimer.start(COMMAND_DEBOUNCE_MS);
        return;
    }

    clearError();
    tmc.setTorqueTarget(pendingTorque);
}

void MainWindow::sendPendingVelocityCommand()
{
    if (tmc.isBusy())
    {
        velocityCommandTimer.start(COMMAND_DEBOUNCE_MS);
        return;
    }

    clearError();
    tmc.setVelocityTarget(pendingVelocity);
}

void MainWindow::updateChipStatus()
{
    if (!tmc.isOpen() || tmc.isBusy())
    {
        return;
    }

    quint32 chipId = 0;
    quint32 statusFlags = 0;

    if (tmc.readChipId(&chipId))
    {
        chipIdValueLabel->setText(QString("0x%1").arg(chipId, 8, 16, QLatin1Char('0')).toUpper());
    }

    if (tmc.readChipStatusFlags(&statusFlags))
    {
        setStatusText(QString("Connected  STATUS=0x%1").arg(statusFlags, 8, 16, QLatin1Char('0')).toUpper(), true);
    }
}

void MainWindow::appendLog(const QString &message)
{
    const QString line = QString("[%1] %2")
    .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"))
        .arg(message);
    logText->append(line);
}

void MainWindow::showError(const QString &message)
{
    errorDotLabel->setObjectName("dotRed");
    errorDotLabel->style()->unpolish(errorDotLabel);
    errorDotLabel->style()->polish(errorDotLabel);
    errorTextLabel->setText(message);
}

void MainWindow::clearError()
{
    errorDotLabel->setObjectName("dotGreen");
    errorDotLabel->style()->unpolish(errorDotLabel);
    errorDotLabel->style()->polish(errorDotLabel);
    errorTextLabel->setText("No error");
}

void MainWindow::updateRange(QSpinBox *minSpin, QSpinBox *maxSpin, QSlider *slider, QSpinBox *directSpin, QLabel *valueLabel)
{
    int minValue = minSpin->value();
    int maxValue = maxSpin->value();

    if (minValue >= maxValue)
    {
        maxValue = minValue + 1;
        QSignalBlocker blocker(maxSpin);
        maxSpin->setValue(maxValue);
    }

    const int oldValue = directSpin->value();
    const int clippedValue = qBound(minValue, oldValue, maxValue);

    syncingUi = true;
    {
        QSignalBlocker b1(slider);
        QSignalBlocker b2(directSpin);
        slider->setRange(minValue, maxValue);
        directSpin->setRange(minValue, maxValue);
        slider->setValue(clippedValue);
        directSpin->setValue(clippedValue);
    }
    syncingUi = false;

    valueLabel->setText(QString::number(clippedValue));
}

void MainWindow::setConnectedUi(bool connected)
{
    if (connected)
    {
        setStatusText("Connected", true);
    }
    else
    {
        setStatusText("Disconnected", false);
        chipIdValueLabel->setText("----");
    }
}

void MainWindow::setStatusText(const QString &text, bool ok)
{
    statusValueLabel->setText(QString("●  %1").arg(text));
    statusValueLabel->setObjectName(ok ? "statusGood" : "statusBad");
    statusValueLabel->style()->unpolish(statusValueLabel);
    statusValueLabel->style()->polish(statusValueLabel);
}
