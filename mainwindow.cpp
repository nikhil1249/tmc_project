#include "mainwindow.h"
#include "MotorWorker.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTextEdit>
#include <QStyle>
#include <QVBoxLayout>
#include <QtGlobal>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    qRegisterMetaType<Tmc6460QtInterface::RunStatus>("Tmc6460QtInterface::RunStatus");

    buildUi();
    applyStyleSheet();
    setupLogFile();
    setupWorkerThread();
    setupConnections();
    setConnectedUi(false);
}

MainWindow::~MainWindow()
{
    statusTimer.stop();
    torqueCommandTimer.stop();
    velocityCommandTimer.stop();

    emit workerShutdown();
    workerThread.quit();
    workerThread.wait(1500);

    if (logFile.isOpen())
    {
        logFile.close();
    }
}

void MainWindow::buildUi()
{
    setWindowTitle("TMC6460 Motor Control");
    resize(1120, 840);

    QWidget *central = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(16, 14, 16, 16);
    mainLayout->setSpacing(10);

    QLabel *title = new QLabel("TMC6460 Motor Control");
    title->setObjectName("titleLabel");
    mainLayout->addWidget(title);

    // Row-1: connection and E-STOP together
    mainLayout->addWidget(createTopConnectionRow());

    // Row-2: chip status only
    mainLayout->addWidget(createChipStatusGroup());

    mainLayout->addWidget(createTorqueGroup());
    mainLayout->addWidget(createVelocityGroup());
    mainLayout->addWidget(createLogGroup(), 1);

    setCentralWidget(central);
}

QWidget *MainWindow::createTopConnectionRow()
{
    QGroupBox *group = new QGroupBox("Connection / Safety");
    QGridLayout *layout = new QGridLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);
    layout->setHorizontalSpacing(12);
    layout->setVerticalSpacing(8);

    layout->addWidget(new QLabel("Bridge COM Port:"), 0, 0);

    portEdit = new QLineEdit(DEFAULT_COM_PORT);
    portEdit->setMaximumWidth(160);
    layout->addWidget(portEdit, 0, 1);

    connectButton = new QPushButton("Connect + Initialize");
    connectButton->setMinimumWidth(190);
    layout->addWidget(connectButton, 0, 2);

    layout->addItem(new QSpacerItem(20, 10, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 3);

    estopButton = new QPushButton("E-STOP");
    estopButton->setObjectName("estopButton");
    estopButton->setMinimumSize(210, 70);
    layout->addWidget(estopButton, 0, 4);

    layout->setColumnStretch(3, 1);
    return group;
}

QWidget *MainWindow::createChipStatusGroup()
{
    QGroupBox *group = new QGroupBox("Chip Status / Live Feedback");
    QGridLayout *layout = new QGridLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);
    layout->setHorizontalSpacing(18);
    layout->setVerticalSpacing(10);

    QWidget *statusPanel = new QWidget(group);
    QGridLayout *statusLayout = new QGridLayout(statusPanel);
    statusLayout->setContentsMargins(0, 0, 0, 0);
    statusLayout->setHorizontalSpacing(12);
    statusLayout->setVerticalSpacing(10);

    statusLayout->addWidget(new QLabel("Chip ID:"), 0, 0);
    chipIdValueLabel = new QLabel("----");
    chipIdValueLabel->setObjectName("blueValueLabel");
    statusLayout->addWidget(chipIdValueLabel, 0, 1);

    statusLayout->addWidget(new QLabel("Status:"), 0, 2);
    statusValueLabel = new QLabel("Disconnected");
    statusValueLabel->setObjectName("statusBad");
    statusLayout->addWidget(statusValueLabel, 0, 3);

    statusLayout->addWidget(new QLabel("Error:"), 1, 0);
    errorDotLabel = new QLabel("●");
    errorDotLabel->setObjectName("dotGreen");
    statusLayout->addWidget(errorDotLabel, 1, 1);

    errorTextLabel = new QLabel("No error");
    errorTextLabel->setWordWrap(true);
    statusLayout->addWidget(errorTextLabel, 1, 2, 1, 2);
    statusLayout->setColumnStretch(4, 1);

    layout->addWidget(statusPanel, 0, 0);
    layout->addWidget(createLiveFeedbackPanel(), 0, 1);
    layout->setColumnStretch(0, 1);
    layout->setColumnStretch(1, 2);

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
    torqueMinSpin->setValue(DEFAULT_TORQUEMIN_VALUE);
    torqueMinSpin->setMinimumWidth(130);

    torqueMaxSpin = new QSpinBox;
    torqueMaxSpin->setRange(MIN_ALLOWED_VALUE, MAX_ALLOWED_VALUE);
    torqueMaxSpin->setValue(DEFAULT_TORQUEMAX_VALUE);
    torqueMaxSpin->setMinimumWidth(130);

    torqueDirectSpin = new QSpinBox;
    torqueDirectSpin->setRange(DEFAULT_TORQUEMIN_VALUE, DEFAULT_TORQUEMAX_VALUE);
    torqueDirectSpin->setValue(0);
    torqueDirectSpin->setMinimumWidth(150);
    torqueDirectSpin->setKeyboardTracking(false);

    applyTorqueButton = new QPushButton("Apply Torque");
    applyTorqueButton->setMinimumWidth(130);

    torqueSlider = new QSlider(Qt::Horizontal);
    torqueSlider->setRange(DEFAULT_TORQUEMIN_VALUE, DEFAULT_TORQUEMAX_VALUE);
    torqueSlider->setValue(0);
    torqueSlider->setTracking(true);
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
    velocityMinSpin->setRange(-20000, 20000);
    velocityMinSpin->setValue(DEFAULT_VELMIN_RPM);
    velocityMinSpin->setSuffix(" rpm");
    velocityMinSpin->setMinimumWidth(130);

    velocityMaxSpin = new QSpinBox;
    velocityMaxSpin->setRange(-20000, 20000);
    velocityMaxSpin->setValue(DEFAULT_VELMAX_RPM);
    velocityMaxSpin->setSuffix(" rpm");
    velocityMaxSpin->setMinimumWidth(130);

    velocityDirectSpin = new QSpinBox;
    velocityDirectSpin->setRange(DEFAULT_VELMIN_RPM, DEFAULT_VELMAX_RPM);
    velocityDirectSpin->setValue(0);
    velocityDirectSpin->setSuffix(" rpm");
    velocityDirectSpin->setMinimumWidth(150);
    velocityDirectSpin->setKeyboardTracking(false);

    applyVelocityButton = new QPushButton("Apply Velocity");
    applyVelocityButton->setMinimumWidth(140);

    velocitySlider = new QSlider(Qt::Horizontal);
    velocitySlider->setRange(DEFAULT_VELMIN_RPM, DEFAULT_VELMAX_RPM);
    velocitySlider->setValue(0);
    velocitySlider->setTracking(true);
    velocitySlider->setTickPosition(QSlider::TicksBelow);
    velocitySlider->setTickInterval(500);

    velocityValueLabel = new QLabel("0 rpm");
    velocityValueLabel->setObjectName("blueValueLabel");
    velocityValueLabel->setMinimumWidth(140);

    layout->addWidget(new QLabel("Min:"), 0, 0);
    layout->addWidget(velocityMinSpin, 0, 1);
    layout->addWidget(new QLabel("Max:"), 0, 2);
    layout->addWidget(velocityMaxSpin, 0, 3);
    layout->addWidget(new QLabel("Direct Value:"), 0, 4);
    layout->addWidget(velocityDirectSpin, 0, 5);
    layout->addWidget(new QLabel("RPM"), 0, 6);
    layout->addWidget(applyVelocityButton, 0, 7);

    layout->addWidget(velocitySlider, 1, 0, 1, 6);
    layout->addWidget(new QLabel("Value:"), 1, 6);
    layout->addWidget(velocityValueLabel, 1, 7);

    layout->setColumnStretch(4, 1);
    return group;
}

QWidget *MainWindow::createLiveFeedbackPanel()
{
    QWidget *panel = new QWidget;
    QGridLayout *layout = new QGridLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setHorizontalSpacing(18);
    layout->setVerticalSpacing(8);

    // Scalable feedback layout: to add a new feedback item later, add one more line here.
    addFeedbackItem(layout, "current_mA", "Current Iq:", 0, 0, "-- mA");
    addFeedbackItem(layout, "flux_raw", "Flux raw:", 0, 1);
    addFeedbackItem(layout, "velocity_calc", "Velocity actual:", 0, 2);
    addFeedbackItem(layout, "torque_raw", "Torque raw:", 0, 3);

    for (int i = 0; i < 8; ++i)
    {
        layout->setColumnStretch(i, 1);
    }

    return panel;
}

QWidget *MainWindow::createLogGroup()
{
    QGroupBox *group = new QGroupBox("UART / Register Log");
    QVBoxLayout *layout = new QVBoxLayout(group);
    layout->setContentsMargins(16, 24, 16, 16);

    logText = new QTextEdit;
    logText->setReadOnly(true);
    logText->setMinimumHeight(260);
    logText->setLineWrapMode(QTextEdit::NoWrap);
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
        "QPushButton:disabled { background-color: #94A3B8; color: #E5E7EB; }"
        "#estopButton { background-color: #EF233C; color: white; font-size: 30px; font-weight: bold; border-radius: 32px; border: 2px solid #B91C1C; }"
        "#estopButton:hover { background-color: #DC2626; }"
        "#estopButton:pressed { background-color: #991B1B; }"
        "QSlider::groove:horizontal { height: 8px; background: #D1D5DB; border-radius: 4px; }"
        "QSlider::handle:horizontal { background: #2563EB; border: 2px solid #1D4ED8; width: 22px; height: 22px; margin: -8px 0; border-radius: 11px; }"
        "QSlider::sub-page:horizontal { background: #2563EB; border-radius: 4px; }"
        "#blueValueLabel { color: #1D4ED8; font-size: 20px; font-weight: bold; }"
        "#feedbackValueLabel { color: #0F766E; font-size: 20px; font-weight: bold; }"
        "#feedbackTitleLabel { color: #111827; font-size: 14px; font-weight: normal; }"
        "#statusGood { background-color: #ECFDF5; color: #16A34A; border: 1px solid #BBF7D0; border-radius: 8px; padding: 8px 20px; font-size: 17px; font-weight: bold; }"
        "#statusBad { background-color: #FEF2F2; color: #DC2626; border: 1px solid #FCA5A5; border-radius: 8px; padding: 8px 20px; font-size: 17px; font-weight: bold; }"
        "#dotGreen { color: #16A34A; font-size: 26px; }"
        "#dotRed { color: #DC2626; font-size: 26px; }"
        "QTextEdit { background-color: #0F172A; color: #E5E7EB; border-radius: 8px; font-family: Consolas; font-size: 12px; }"
    );
}

void MainWindow::setupWorkerThread()
{
    worker = new MotorWorker;
    worker->moveToThread(&workerThread);

    connect(&workerThread, &QThread::finished, worker, &QObject::deleteLater);

    connect(this, &MainWindow::workerConnectAndInitialize,
            worker, &MotorWorker::connectAndInitialize, Qt::QueuedConnection);
    connect(this, &MainWindow::workerReadStatus,
            worker, &MotorWorker::readStatus, Qt::QueuedConnection);
    connect(this, &MainWindow::workerApplyTorque,
            worker, &MotorWorker::applyTorque, Qt::QueuedConnection);
    connect(this, &MainWindow::workerApplyVelocityRpm,
            worker, &MotorWorker::applyVelocityRpm, Qt::QueuedConnection);
    connect(this, &MainWindow::workerEmergencyStop,
            worker, &MotorWorker::emergencyStop, Qt::QueuedConnection);
    connect(this, &MainWindow::workerShutdown,
            worker, &MotorWorker::shutdown, Qt::QueuedConnection);

    connect(worker, &MotorWorker::connected,
            this, &MainWindow::onWorkerConnected, Qt::QueuedConnection);
    connect(worker, &MotorWorker::statusReady,
            this, &MainWindow::onWorkerStatusReady, Qt::QueuedConnection);
    connect(worker, &MotorWorker::commandDone,
            this, &MainWindow::onWorkerCommandDone, Qt::QueuedConnection);
    connect(worker, &MotorWorker::logMessage,
            this, &MainWindow::appendLog, Qt::QueuedConnection);
    connect(worker, &MotorWorker::errorChanged,
            this, &MainWindow::showError, Qt::QueuedConnection);

    workerThread.start();
}

void MainWindow::setupConnections()
{
    connect(connectButton, &QPushButton::clicked, this, &MainWindow::connectAndInitialize);

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

void MainWindow::setupLogFile()
{
    const QString logDirPath = QCoreApplication::applicationDirPath() + "/logs";
    QDir().mkpath(logDirPath);

    const QString fileName = QString("tmc6460_%1.log")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

    logFile.setFileName(logDirPath + "/" + fileName);
    if (logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        appendLog(QString("Log file: %1").arg(logFile.fileName()));
    }
}

QLabel *MainWindow::addFeedbackItem(QGridLayout *layout,
                                    const QString &key,
                                    const QString &title,
                                    int row,
                                    int columnPair,
                                    const QString &defaultValue)
{
    QLabel *titleLabel = new QLabel(title);
    titleLabel->setObjectName("feedbackTitleLabel");

    QLabel *valueLabel = new QLabel(defaultValue);
    valueLabel->setObjectName("feedbackValueLabel");
    valueLabel->setMinimumWidth(110);

    const int col = columnPair * 2;
    layout->addWidget(titleLabel, row, col);
    layout->addWidget(valueLabel, row, col + 1);

    feedbackLabels.insert(key, valueLabel);
    return valueLabel;
}

void MainWindow::setFeedbackValue(const QString &key, const QString &value)
{
    QLabel *label = feedbackLabels.value(key, nullptr);
    if (label != nullptr)
    {
        label->setText(value);
    }
}

void MainWindow::connectAndInitialize()
{
    clearError();
    logAction(QString("Connect + Initialize requested on %1").arg(portEdit->text().trimmed()));

    connectButton->setEnabled(false);
    setStatusText("Connecting...", false);

    emit workerConnectAndInitialize(portEdit->text().trimmed(), DEFAULT_BAUD_RATE);
}

void MainWindow::onWorkerConnected(bool ok, quint32 chipId, const QString &message)
{
    connectButton->setEnabled(true);

    if (!ok)
    {
        setConnectedUi(false);
        setStatusText("Initialization failed", false);
        showError(message.isEmpty() ? QStringLiteral("Connection / initialization failed") : message);
        return;
    }

    chipIdValueLabel->setText(QString("0x%1").arg(chipId, 8, 16, QLatin1Char('0')).toUpper());
    setConnectedUi(true);
    appendLog(message);
    statusTimer.start(STATUS_TIMER_MS);
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

    QSignalBlocker blocker(torqueSlider);
    torqueSlider->setValue(value);
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

void MainWindow::onVelocitySliderValueChanged(int rpm)
{
    if (syncingUi)
    {
        return;
    }

    pendingVelocityRpm = rpm;
    velocityValueLabel->setText(QString("%1 rpm").arg(rpm));

    QSignalBlocker blocker(velocityDirectSpin);
    velocityDirectSpin->setValue(rpm);
}

void MainWindow::onVelocitySliderReleased()
{
    scheduleVelocityCommand(velocitySlider->value());
}

void MainWindow::onVelocityDirectValueChanged(int rpm)
{
    if (syncingUi)
    {
        return;
    }

    pendingVelocityRpm = rpm;

    QSignalBlocker blocker(velocitySlider);
    velocitySlider->setValue(rpm);
    velocityValueLabel->setText(QString("%1 rpm").arg(rpm));
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
    updateRange(velocityMinSpin, velocityMaxSpin, velocitySlider, velocityDirectSpin, velocityValueLabel, QStringLiteral(" rpm"));
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
    velocityValueLabel->setText("0 rpm");

    logAction("E-STOP requested");
    emit workerEmergencyStop();

    setStatusText("E-STOP", false);
    showError("Emergency stop active. Driver disable requested.");
}

void MainWindow::scheduleTorqueCommand(int value)
{
    if (!connectedToController)
    {
        return;
    }

    pendingTorque = value;
    torqueCommandTimer.start(COMMAND_DEBOUNCE_MS);
}

void MainWindow::scheduleVelocityCommand(int rpm)
{
    if (!connectedToController)
    {
        return;
    }

    pendingVelocityRpm = rpm;
    velocityCommandTimer.start(COMMAND_DEBOUNCE_MS);
}

void MainWindow::sendPendingTorqueCommand()
{
    logAction(QString("Apply torque %1").arg(pendingTorque));
    emit workerApplyTorque(pendingTorque);
}

void MainWindow::sendPendingVelocityCommand()
{
    const qint32 rawVelocity = Tmc6460QtInterface::rpmToVelocityRaw(pendingVelocityRpm);

    // Avoid status polling while the ramp is writing velocity targets.
    // This keeps the UART stream clean and prevents partial/stale replies.
    statusTimer.stop();
    statusRequestPending = false;

    logAction(QString("Velocity target %1 rpm -- raw %2")
                  .arg(pendingVelocityRpm)
                  .arg(rawVelocity));

    emit workerApplyVelocityRpm(pendingVelocityRpm);
}

void MainWindow::updateChipStatus()
{
    if (!connectedToController || statusRequestPending)
    {
        return;
    }

    statusRequestPending = true;
    emit workerReadStatus();
}

void MainWindow::onWorkerStatusReady(const Tmc6460QtInterface::RunStatus &runStatus)
{
    statusRequestPending = false;

    const quint32 statusFlags = runStatus.chipStatusFlags;
    // setStatusText(QString("Connected  STATUS=0x%1")
    //                   .arg(statusFlags, 8, 16, QLatin1Char('0'))
    //                   .toUpper(),
    //               true);

    clearError();

    setFeedbackValue("current_mA", QString("%1 mA").arg(runStatus.torqueCurrentMilliAmp));
    setFeedbackValue("flux_raw", QString::number(runStatus.fluxActualRaw));
    setFeedbackValue("torque_raw", QString::number(runStatus.torqueActualRaw));

    setFeedbackValue("velocity_calc",
                     QString("%1 rpm")
                         .arg(runStatus.velocityActualRpm));

}

void MainWindow::onWorkerCommandDone(const QString &action, bool ok)
{
    appendLog(QString("ACTION_RESULT: %1 : %2").arg(action, ok ? "OK" : "FAILED"));

    if (ok)
    {
        clearError();
    }
    else
    {
        showError(QString("%1 failed").arg(action));
    }

    if (connectedToController && !statusTimer.isActive())
    {
        statusTimer.start(STATUS_TIMER_MS);
    }
}

qint32 MainWindow::calculateVelocityFromPosition(quint32 positionActualRaw)
{
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    qint32 velocityRawPerSecond = 0;

    if (hasLastPositionActual)
    {
        const qint64 dtMs = nowMs - lastPositionTimeMs;
        if (dtMs > 0)
        {
            const qint32 positionDelta = static_cast<qint32>(positionActualRaw - lastPositionActualRaw);
            velocityRawPerSecond = static_cast<qint32>((static_cast<qint64>(positionDelta) * 1000) / dtMs);
        }
    }

    lastPositionActualRaw = positionActualRaw;
    lastPositionTimeMs = nowMs;
    hasLastPositionActual = true;

    return velocityRawPerSecond;
}

void MainWindow::appendLog(const QString &message)
{
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
    const QString line = QString("[%1] %2").arg(timestamp, message);

    if (logText != nullptr)
    {
        logText->append(line.toHtmlEscaped());
    }

    if (logFile.isOpen())
    {
        logFile.write(line.toUtf8());
        logFile.write("\n");
        logFile.flush();
    }
}

void MainWindow::logAction(const QString &action)
{
    appendLog(QString("ACTION: %1").arg(action));
}

void MainWindow::showError(const QString &message)
{
    statusRequestPending = false;

    if (message.trimmed().isEmpty())
    {
        clearError();
        return;
    }

    errorDotLabel->setObjectName("dotRed");
    errorDotLabel->style()->unpolish(errorDotLabel);
    errorDotLabel->style()->polish(errorDotLabel);

    errorTextLabel->setText(message);
    appendLog(QString("ERROR: %1").arg(message));
}

void MainWindow::clearError()
{
    errorDotLabel->setObjectName("dotGreen");
    errorDotLabel->style()->unpolish(errorDotLabel);
    errorDotLabel->style()->polish(errorDotLabel);
    errorTextLabel->setText("No error");
}

void MainWindow::updateRange(QSpinBox *minSpin,
                             QSpinBox *maxSpin,
                             QSlider *slider,
                             QSpinBox *directSpin,
                             QLabel *valueLabel,
                             const QString &suffix)
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

    valueLabel->setText(QString("%1%2").arg(clippedValue).arg(suffix));
}

void MainWindow::setConnectedUi(bool connected)
{
    connectedToController = connected;
    statusRequestPending = false;

    connectButton->setEnabled(true);
    applyTorqueButton->setEnabled(connected);
    applyVelocityButton->setEnabled(connected);
    estopButton->setEnabled(connected);

    if (connected)
    {
        setStatusText("Connected", true);
        return;
    }

    statusTimer.stop();
    setStatusText("Disconnected", false);
    chipIdValueLabel->setText("----");

    const QStringList keys = feedbackLabels.keys();
    for (const QString &key : keys)
    {
        setFeedbackValue(key, key == "current_mA" ? "-- mA" : "--");
    }

    hasLastPositionActual = false;
    lastPositionActualRaw = 0;
    lastPositionTimeMs = 0;
}

void MainWindow::setStatusText(const QString &text, bool ok)
{
    statusValueLabel->setText(QString("●  %1").arg(text));
    statusValueLabel->setObjectName(ok ? "statusGood" : "statusBad");
    statusValueLabel->style()->unpolish(statusValueLabel);
    statusValueLabel->style()->polish(statusValueLabel);
}
