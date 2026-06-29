#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "MotorWorker.h"

#include <QApplication>
#include <QCloseEvent>
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
    , ui(new Ui::MainWindow)
{
    qRegisterMetaType<Tmc6460QtInterface::RunStatus>("Tmc6460QtInterface::RunStatus");

    pendingVelocityRaw = DEFAULT_VELOCITY_RAW;
    pendingVelocityDisplayValue = defaultVelocityDisplay();

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
    velocityTorqueLimitCommandTimer.stop();

    if (!shutdownDone && worker != nullptr && workerThread.isRunning())
    {
        shutdownDone = true;
        QMetaObject::invokeMethod(worker,
                                  "shutdown",
                                  Qt::BlockingQueuedConnection);
    }

    workerThread.quit();
    workerThread.wait(3000);

    if (logFile.isOpen())
    {
        logFile.close();
    }

    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    statusTimer.stop();
    torqueCommandTimer.stop();
    velocityCommandTimer.stop();
    velocityTorqueLimitCommandTimer.stop();

    if (!shutdownDone && worker != nullptr && workerThread.isRunning())
    {
        shutdownDone = true;
        appendLog(QStringLiteral("GUI close requested. Sending safe motor shutdown."));
        QMetaObject::invokeMethod(worker,
                                  "shutdown",
                                  Qt::BlockingQueuedConnection);
    }

    event->accept();
}


void MainWindow::buildUi()
{
    ui->setupUi(this);

    ui->chipIdValueLabel->setObjectName("blueValueLabel");
    ui->statusValueLabel->setObjectName("statusBad");
    ui->errorDotLabel->setObjectName("dotGreen");

    ui->torqueValueLabel->setObjectName("blueValueLabel");
    ui->velocityValueLabel->setObjectName("blueValueLabel");

    ui->currentTitleLabel->setObjectName("feedbackTitleLabel");
    ui->fluxTitleLabel->setObjectName("feedbackTitleLabel");
    ui->velocityActualTitleLabel->setObjectName("feedbackTitleLabel");
    ui->torqueActualTitleLabel->setObjectName("feedbackTitleLabel");

    ui->currentValueLabel->setObjectName("feedbackValueLabel");
    ui->fluxValueLabel->setObjectName("feedbackValueLabel");
    ui->velocityActualValueLabel->setObjectName("feedbackValueLabel");
    ui->torqueActualValueLabel->setObjectName("feedbackValueLabel");

    feedbackLabels.insert(QStringLiteral("current_mA"), ui->currentValueLabel);
    feedbackLabels.insert(QStringLiteral("flux_raw"), ui->fluxValueLabel);
    feedbackLabels.insert(QStringLiteral("velocity_calc"), ui->velocityActualValueLabel);
    feedbackLabels.insert(QStringLiteral("torque_raw"), ui->torqueActualValueLabel);

    ui->portEdit->setText(QString::fromLatin1(DEFAULT_COM_PORT));

    ui->velocityMinSpin->setValue(defaultVelocityMinDisplay());
    ui->velocityMaxSpin->setValue(defaultVelocityMaxDisplay());
    ui->velocityDirectSpin->setRange(defaultVelocityMinDisplay(), defaultVelocityMaxDisplay());
    ui->velocityDirectSpin->setValue(defaultVelocityDisplay());
    ui->velocitySlider->setRange(defaultVelocityMinDisplay(), defaultVelocityMaxDisplay());
    ui->velocitySlider->setValue(defaultVelocityDisplay());
    ui->velocityValueLabel->setText(QString("%1 %2").arg(defaultVelocityDisplay()).arg(velocityUnitText()));

    ui->velocityTorqueLimitSpin->setValue(DEFAULT_VELOCITY_TORQUE_LIMIT_RAW);
    pendingVelocityTorqueLimitRaw = DEFAULT_VELOCITY_TORQUE_LIMIT_RAW;
}

void MainWindow::applyStyleSheet()
{
    setStyleSheet(
        "QMainWindow { background-color: #F5F7FA; }"
        "QWidget { font-family: Segoe UI; font-size: 12px; color: #111827; }"
        "#titleLabel { font-size: 24px; font-weight: bold; color: #111827; padding: 4px; }"
        "QGroupBox { background-color: #FFFFFF; border: 1px solid #D1D5DB; border-radius: 10px; margin-top: 12px; font-size: 14px; font-weight: bold; }"
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
        "#blueValueLabel { color: #1D4ED8; font-size: 16px; font-weight: bold; }"
        "#feedbackValueLabel { color: #0F766E; font-size: 16px; font-weight: bold; }"
        "#feedbackTitleLabel { color: #111827; font-size: 14px; font-weight: normal; }"
        "#statusGood { background-color: #ECFDF5; color: #16A34A; border: 1px solid #BBF7D0; border-radius: 8px; padding: 6px 12px; font-size: 13px; font-weight: bold; }"
        "#statusBad { background-color: #FEF2F2; color: #DC2626; border: 1px solid #FCA5A5; border-radius: 8px; padding: 6px 12px; font-size: 13px; font-weight: bold; }"
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
    connect(this, &MainWindow::workerReadRunStatusSnapshot,
            worker, &MotorWorker::readRunStatusSnapshot, Qt::QueuedConnection);
    connect(this, &MainWindow::workerApplyTorque,
            worker, &MotorWorker::applyTorque, Qt::QueuedConnection);
    connect(this, &MainWindow::workerApplyVelocityTorqueLimit,
            worker, &MotorWorker::applyVelocityTorqueLimitRaw, Qt::QueuedConnection);
    connect(this, &MainWindow::workerApplyVelocityLimitRaw,
            worker, &MotorWorker::applyVelocityLimitRaw, Qt::QueuedConnection);
    connect(this, &MainWindow::workerApplyVelocityRaw,
            worker, &MotorWorker::applyVelocityRaw, Qt::QueuedConnection);
    connect(this, &MainWindow::workerStartAutoCalibration,
            worker, &MotorWorker::startAutoCalibrationFromButton, Qt::QueuedConnection);
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
    connect(worker, &MotorWorker::stallStateChanged,
            this, &MainWindow::onWorkerStallStateChanged, Qt::QueuedConnection);
    connect(worker, &MotorWorker::logMessage,
            this, &MainWindow::appendLog, Qt::QueuedConnection);
    connect(worker, &MotorWorker::errorChanged,
            this, &MainWindow::showError, Qt::QueuedConnection);

    workerThread.start();
}

void MainWindow::setupConnections()
{
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::connectAndInitialize);
    connect(ui->calibrationButton, &QPushButton::clicked, this, &MainWindow::onCalibrationClicked);
    connect(ui->readRunStatusButton, &QPushButton::clicked, this, &MainWindow::onReadRunStatusClicked);

    connect(ui->torqueSlider, &QSlider::valueChanged, this, &MainWindow::onTorqueSliderValueChanged);
    connect(ui->torqueSlider, &QSlider::sliderReleased, this, &MainWindow::onTorqueSliderReleased);
    connect(ui->torqueDirectSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onTorqueDirectValueChanged);
    connect(ui->torqueDirectSpin, &QSpinBox::editingFinished, this, &MainWindow::onTorqueDirectEditingFinished);
    connect(ui->applyTorqueButton, &QPushButton::clicked, this, &MainWindow::onApplyTorqueClicked);
    connect(ui->torqueMinSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onTorqueRangeChanged);
    connect(ui->torqueMaxSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onTorqueRangeChanged);

    connect(ui->velocitySlider, &QSlider::valueChanged, this, &MainWindow::onVelocitySliderValueChanged);
    connect(ui->velocitySlider, &QSlider::sliderReleased, this, &MainWindow::onVelocitySliderReleased);
    connect(ui->velocityDirectSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onVelocityDirectValueChanged);
    connect(ui->velocityDirectSpin, &QSpinBox::editingFinished, this, &MainWindow::onVelocityDirectEditingFinished);
    connect(ui->applyVelocityButton, &QPushButton::clicked, this, &MainWindow::onApplyVelocityClicked);
    connect(ui->velocityCwButton, &QPushButton::clicked, this, &MainWindow::onVelocityCwClicked);
    connect(ui->velocityCcwButton, &QPushButton::clicked, this, &MainWindow::onVelocityCcwClicked);
    connect(ui->velocityTorqueLimitSpin, qOverload<int>(&QSpinBox::valueChanged),
            this, &MainWindow::onVelocityTorqueLimitValueChanged);
    connect(ui->velocityTorqueLimitSpin, &QSpinBox::editingFinished,
            this, &MainWindow::onVelocityTorqueLimitEditingFinished);
    connect(ui->applyVelocityTorqueLimitButton, &QPushButton::clicked,
            this, &MainWindow::onApplyVelocityTorqueLimitClicked);
    connect(ui->velocityMinSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onVelocityRangeChanged);
    connect(ui->velocityMaxSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onVelocityRangeChanged);

    connect(ui->estopButton, &QPushButton::clicked, this, &MainWindow::onEstopClicked);

    torqueCommandTimer.setSingleShot(true);
    velocityCommandTimer.setSingleShot(true);
    velocityTorqueLimitCommandTimer.setSingleShot(true);
    connect(&torqueCommandTimer, &QTimer::timeout, this, &MainWindow::sendPendingTorqueCommand);
    connect(&velocityCommandTimer, &QTimer::timeout, this, &MainWindow::sendPendingVelocityCommand);
    connect(&velocityTorqueLimitCommandTimer, &QTimer::timeout, this, &MainWindow::applyVelocityTorqueLimitNow);

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
    statusTimer.stop();
    statusRequestPending = false;

    clearError();
    logAction(QString("Connect + Initialize requested on %1").arg(ui->portEdit->text().trimmed()));

    ui->connectButton->setEnabled(false);
    setStatusText("Connecting...", false);

    emit workerConnectAndInitialize(ui->portEdit->text().trimmed(), DEFAULT_BAUD_RATE);
}

void MainWindow::onWorkerConnected(bool ok, quint32 chipId, const QString &message)
{
    ui->connectButton->setEnabled(true);

    if (!ok)
    {
        setConnectedUi(false);
        setStatusText("Initialization failed", false);
        showError(message.isEmpty() ? QStringLiteral("Connection / initialization failed") : message);
        return;
    }

    QString tempString = "0x";
    ui->chipIdValueLabel->setText(tempString + QString("%1").arg(chipId, 8, 16, QLatin1Char('0')).toUpper());
    setConnectedUi(true);
    appendLog(message);
    statusTimer.start(STATUS_TIMER_MS);
}

void MainWindow::onTorqueSliderValueChanged(int value)
{
    ui->torqueValueLabel->setText(QString::number(value));

    QSignalBlocker blocker(ui->torqueDirectSpin);
    ui->torqueDirectSpin->setValue(value);
}

void MainWindow::onTorqueSliderReleased()
{
    scheduleTorqueCommand(ui->torqueSlider->value());
}

void MainWindow::onTorqueDirectValueChanged(int value)
{
    if (syncingUi)
    {
        return;
    }

    QSignalBlocker blocker(ui->torqueSlider);
    ui->torqueSlider->setValue(value);
    ui->torqueValueLabel->setText(QString::number(value));
}

void MainWindow::onTorqueDirectEditingFinished()
{
    scheduleTorqueCommand(ui->torqueDirectSpin->value());
}

void MainWindow::onApplyTorqueClicked()
{
    scheduleTorqueCommand(ui->torqueDirectSpin->value());
}

void MainWindow::onTorqueRangeChanged()
{
    updateRange(ui->torqueMinSpin, ui->torqueMaxSpin, ui->torqueSlider, ui->torqueDirectSpin, ui->torqueValueLabel);
}

void MainWindow::onVelocitySliderValueChanged(int displayVelocity)
{
    if (syncingUi)
    {
        return;
    }

    pendingVelocityDisplayValue = displayVelocity;
    pendingVelocityRaw = velocityDisplayToRaw(displayVelocity);
    ui->velocityValueLabel->setText(QString("%1 %2").arg(displayVelocity).arg(velocityUnitText()));

    QSignalBlocker blocker(ui->velocityDirectSpin);
    ui->velocityDirectSpin->setValue(displayVelocity);
}

void MainWindow::onVelocitySliderReleased()
{
    scheduleVelocityCommand(ui->velocitySlider->value());
}

void MainWindow::onVelocityDirectValueChanged(int displayVelocity)
{
    if (syncingUi)
    {
        return;
    }

    pendingVelocityDisplayValue = displayVelocity;
    pendingVelocityRaw = velocityDisplayToRaw(displayVelocity);

    QSignalBlocker blocker(ui->velocitySlider);
    ui->velocitySlider->setValue(displayVelocity);
    ui->velocityValueLabel->setText(QString("%1 %2").arg(displayVelocity).arg(velocityUnitText()));
}

void MainWindow::onVelocityDirectEditingFinished()
{
    scheduleVelocityCommand(ui->velocityDirectSpin->value());
}

void MainWindow::onApplyVelocityClicked()
{
    scheduleVelocityCommand(ui->velocityDirectSpin->value());
}

void MainWindow::onVelocityCwClicked()
{
    applyVelocityFromDirectWithDirection(+1);
}

void MainWindow::onVelocityCcwClicked()
{
    applyVelocityFromDirectWithDirection(-1);
}


void MainWindow::onVelocityTorqueLimitValueChanged(int value)
{
    pendingVelocityTorqueLimitRaw = qBound(DEFAULT_VELOCITY_TORQUE_LIMIT_MIN,
                                           value,
                                           DEFAULT_VELOCITY_TORQUE_LIMIT_MAX);

    // Same behavior as velocity Min/Max: changing this field must update the
    // controller limit register, but must not resend velocity target.
    // The timer prevents many UART writes while using spin arrows.
    if (connectedToController && !syncingUi)
    {
        velocityTorqueLimitCommandTimer.start(COMMAND_DEBOUNCE_MS);
    }
}

void MainWindow::onVelocityTorqueLimitEditingFinished()
{
    velocityTorqueLimitCommandTimer.stop();
    applyVelocityTorqueLimitNow();
}

void MainWindow::onApplyVelocityTorqueLimitClicked()
{
    velocityTorqueLimitCommandTimer.stop();
    applyVelocityTorqueLimitNow();
}

void MainWindow::onVelocityRangeChanged()
{
    updateRange(ui->velocityMinSpin,
                ui->velocityMaxSpin,
                ui->velocitySlider,
                ui->velocityDirectSpin,
                ui->velocityValueLabel,
                velocityUnitSuffix());

    pendingVelocityDisplayValue = ui->velocityDirectSpin->value();
    pendingVelocityRaw = velocityDisplayToRaw(pendingVelocityDisplayValue);

    // Changing Min/Max must not resend FOC_PID_VELOCITY_TARGET.
    // It only updates the GUI range and the controller velocity limit register.
    const qint32 minRaw = velocityDisplayToRaw(ui->velocityMinSpin->value());
    const qint32 maxRaw = velocityDisplayToRaw(ui->velocityMaxSpin->value());
    const qint32 absLimitRaw = qMax(qAbs(minRaw), qAbs(maxRaw));

    if (connectedToController && absLimitRaw > 0)
    {
        statusTimer.stop();
        statusRequestPending = false;
        logAction(QString("Velocity range changed: minRaw=%1 maxRaw=%2, updating FOC_PID_VELOCITY_LIMIT=%3")
                  .arg(minRaw)
                  .arg(maxRaw)
                  .arg(absLimitRaw));
        emit workerApplyVelocityLimitRaw(absLimitRaw);
    }
}

void MainWindow::onEstopClicked()
{
    clearError();
    torqueCommandTimer.stop();
    velocityCommandTimer.stop();
    velocityTorqueLimitCommandTimer.stop();

    syncingUi = true;
    {
        QSignalBlocker b1(ui->torqueSlider);
        QSignalBlocker b2(ui->torqueDirectSpin);
        QSignalBlocker b3(ui->velocitySlider);
        QSignalBlocker b4(ui->velocityDirectSpin);

        ui->torqueSlider->setValue(0);
        ui->torqueDirectSpin->setValue(0);
        ui->velocitySlider->setValue(0);
        ui->velocityDirectSpin->setValue(0);
    }
    syncingUi = false;

    ui->torqueValueLabel->setText("0");
    ui->velocityValueLabel->setText(QString("0 %1").arg(velocityUnitText()));
    pendingVelocityDisplayValue = 0;
    pendingVelocityRaw = 0;

    logAction("E-STOP requested");
    emit workerEmergencyStop();

    setStatusText("E-STOP", false);
    showError("Safe E-STOP hold active. Driver remains enabled to hold the load.");
}


void MainWindow::onReadRunStatusClicked()
{
    if (!connectedToController)
    {
        appendLog("RUN_STATUS[MANUAL]: skipped - not connected");
        return;
    }

    statusTimer.stop();
    statusRequestPending = false;
    appendLog("ACTION: Manual run-status snapshot requested");
    emit workerReadRunStatusSnapshot(QStringLiteral("MANUAL"));

    if (!statusTimer.isActive())
    {
        statusTimer.start(STATUS_TIMER_MS);
    }
}

void MainWindow::scheduleTorqueCommand(int value)
{
    if (!connectedToController)
    {
        return;
    }

    pendingTorque = qBound(DEFAULT_TORQUEMIN_VALUE, value, DEFAULT_TORQUEMAX_VALUE);
    torqueCommandTimer.start(COMMAND_DEBOUNCE_MS);
}

void MainWindow::scheduleVelocityCommand(int displayVelocity)
{
    if (!connectedToController)
    {
        return;
    }

    const int minValue = ui->velocityMinSpin->value();
    const int maxValue = ui->velocityMaxSpin->value();
    pendingVelocityDisplayValue = qBound(minValue, displayVelocity, maxValue);
    pendingVelocityRaw = velocityDisplayToRaw(pendingVelocityDisplayValue);

    // Clear any previously latched stall/error indication as soon as a new
    // velocity command is requested. The worker will emit a fresh
    // "Monitoring..." or "STALL DETECTED..." state after the command.
    clearError();
    setFeedbackValue("stall_status", pendingVelocityRaw == 0 ? "Not active" : "Commanding...");
    setStatusText(pendingVelocityRaw == 0 ? "Connected" : "Commanding velocity", true);

    velocityCommandTimer.start(COMMAND_DEBOUNCE_MS);
}

void MainWindow::onCalibrationClicked()
{
    clearError();

    const qint32 calibrationVelocityRaw = DEFAULT_AUTO_TORQUE_LEARN_VELOCITY_RAW;

    appendLog(QStringLiteral("Calibration requested by button. Auto-run velocity raw=%1. Ensure load is mechanically safe before starting.")
              .arg(calibrationVelocityRaw));

    emit workerStartAutoCalibration(calibrationVelocityRaw);
}

void MainWindow::sendPendingTorqueCommand()
{
    logAction(QString("Apply torque %1").arg(pendingTorque));
    emit workerApplyTorque(pendingTorque);
}

void MainWindow::sendPendingVelocityCommand()
{
    const qint32 rawVelocity = pendingVelocityRaw;

    // Avoid status polling while the optional ramp is writing velocity targets.
    // This keeps the UART stream clean and prevents partial/stale replies.
    statusTimer.stop();
    statusRequestPending = false;

    const int torqueLimitRaw = qBound(DEFAULT_VELOCITY_TORQUE_LIMIT_MIN,
                                      pendingVelocityTorqueLimitRaw,
                                      DEFAULT_VELOCITY_TORQUE_LIMIT_MAX);

    logAction(QString("Velocity target %1=%2 convertedRaw=%3 with torque limit raw=%4")
                  .arg(velocityUnitText())
                  .arg(pendingVelocityDisplayValue)
                  .arg(rawVelocity)
                  .arg(torqueLimitRaw));

    emit workerApplyVelocityTorqueLimit(torqueLimitRaw);
    emit workerApplyVelocityRaw(rawVelocity);
}

void MainWindow::applyVelocityFromDirectWithDirection(int directionSign)
{
    if (!connectedToController)
    {
        return;
    }

    const int magnitude = qAbs(ui->velocityDirectSpin->value());
    const int requestedVelocity = (directionSign >= 0) ? magnitude : -magnitude;
    const int minValue = ui->velocityMinSpin->value();
    const int maxValue = ui->velocityMaxSpin->value();
    const int boundedVelocity = qBound(minValue, requestedVelocity, maxValue);

    syncingUi = true;
    {
        QSignalBlocker b1(ui->velocityDirectSpin);
        QSignalBlocker b2(ui->velocitySlider);
        ui->velocityDirectSpin->setValue(boundedVelocity);
        ui->velocitySlider->setValue(boundedVelocity);
        ui->velocityValueLabel->setText(QString("%1 %2").arg(boundedVelocity).arg(velocityUnitText()));
    }
    syncingUi = false;

    logAction(QString("Velocity %1 button clicked, direct magnitude=%2, applying target=%3")
              .arg(directionSign >= 0 ? "CW" : "CCW")
              .arg(magnitude)
              .arg(boundedVelocity));

    scheduleVelocityCommand(boundedVelocity);
}

void MainWindow::applyVelocityTorqueLimitNow()
{
    if (!connectedToController)
    {
        return;
    }

    const int torqueLimitRaw = qBound(DEFAULT_VELOCITY_TORQUE_LIMIT_MIN,
                                      pendingVelocityTorqueLimitRaw,
                                      DEFAULT_VELOCITY_TORQUE_LIMIT_MAX);

    statusTimer.stop();
    statusRequestPending = false;

    logAction(QString("Velocity torque/flux limit raw=%1, updating FOC_PID_TORQUE_FLUX_LIMITS")
              .arg(torqueLimitRaw));
    emit workerApplyVelocityTorqueLimit(torqueLimitRaw);
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

    // const quint32 statusFlags = runStatus.chipStatusFlags;
    // setStatusText(QString("Connected  STATUS=0x%1")
    //                   .arg(statusFlags, 8, 16, QLatin1Char('0'))
    //                   .toUpper(),
    //               true);

    clearError();

    setFeedbackValue("current_mA", QString("%1 mA").arg(runStatus.torqueCurrentMilliAmp));
    setFeedbackValue("flux_raw", QString::number(runStatus.fluxActualRaw));
    setFeedbackValue("torque_raw", QString::number(runStatus.torqueActualRaw));
    setFeedbackValue("velocity_calc", QString("%1 raw").arg(runStatus.velocityActualRaw));

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

void MainWindow::onWorkerStallStateChanged(bool stalled, const QString &message)
{
    if (stalled)
    {
        if (message.contains(QStringLiteral("END STOP"), Qt::CaseInsensitive) ||
            message.contains(QStringLiteral("MECHANICAL END"), Qt::CaseInsensitive) ||
            message.contains(QStringLiteral("POSITION_LIMIT"), Qt::CaseInsensitive))
        {
            // Mechanical end stop at 0/90 degree is an expected stop, not a fault.
            setStatusText("END FOUND", true);
            clearError();
        }
        else
        {
            setStatusText("STALL DETECTED", false);
            showError(message);
        }
        return;
    }

    clearError();

    if (message.startsWith(QStringLiteral("Monitoring"), Qt::CaseInsensitive) ||
        message.startsWith(QStringLiteral("Commanding"), Qt::CaseInsensitive))
    {
        setStatusText("Motor running", true);
    }
    else if (connectedToController)
    {
        setStatusText("Connected", true);
    }
}


int MainWindow::rawToVelocityDisplay(qint32 rawVelocity) const
{
    return static_cast<int>(qBound<qint32>(MIN_ALLOWED_VALUE, rawVelocity, MAX_ALLOWED_VALUE));
}

qint32 MainWindow::velocityDisplayToRaw(int displayVelocity) const
{
    return qBound<qint32>(MIN_ALLOWED_VALUE,
                          static_cast<qint32>(displayVelocity),
                          MAX_ALLOWED_VALUE);
}

QString MainWindow::velocityUnitText() const
{
    return QStringLiteral("raw");
}

QString MainWindow::velocityUnitSuffix() const
{
    return QStringLiteral(" raw");
}

int MainWindow::defaultVelocityMinDisplay() const
{
    return DEFAULT_VELMIN_RAW;
}

int MainWindow::defaultVelocityMaxDisplay() const
{
    return DEFAULT_VELMAX_RAW;
}

int MainWindow::defaultVelocityDisplay() const
{
    return DEFAULT_VELOCITY_RAW;
}

void MainWindow::appendLog(const QString &message)
{
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
    const QString line = QString("[%1] %2").arg(timestamp, message);

    if (ui->logText != nullptr)
    {
        ui->logText->append(line.toHtmlEscaped());
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

    ui->errorDotLabel->setObjectName("dotRed");
    ui->errorDotLabel->style()->unpolish(ui->errorDotLabel);
    ui->errorDotLabel->style()->polish(ui->errorDotLabel);

    ui->errorTextLabel->setText(message);
    appendLog(QString("ERROR: %1").arg(message));
}

void MainWindow::clearError()
{
    ui->errorDotLabel->setObjectName("dotGreen");
    ui->errorDotLabel->style()->unpolish(ui->errorDotLabel);
    ui->errorDotLabel->style()->polish(ui->errorDotLabel);
    ui->errorTextLabel->setText("No error");
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

    ui->connectButton->setEnabled(true);
    ui->applyTorqueButton->setEnabled(connected);
    ui->applyVelocityButton->setEnabled(connected);
    ui->velocityCwButton->setEnabled(connected);
    ui->velocityCcwButton->setEnabled(connected);
    ui->applyVelocityTorqueLimitButton->setEnabled(connected);
    if (ui->readRunStatusButton != nullptr)
    {
        ui->readRunStatusButton->setEnabled(connected);
    }
    ui->calibrationButton->setEnabled(connected);
    ui->estopButton->setEnabled(connected);

    if (connected)
    {
        setStatusText("Connected", true);
        return;
    }

    statusTimer.stop();
    setStatusText("Disconnected", false);
    ui->chipIdValueLabel->setText("----");

    const QStringList keys = feedbackLabels.keys();
    for (const QString &key : keys)
    {
        if (key == "current_mA")
            setFeedbackValue(key, "-- mA");
        else if (key == "stall_status")
            setFeedbackValue(key, "Not active");
        else
            setFeedbackValue(key, "--");
    }

}

void MainWindow::setStatusText(const QString &text, bool ok)
{
    ui->statusValueLabel->setText(QString("●  %1").arg(text));
    ui->statusValueLabel->setObjectName(ok ? "statusGood" : "statusBad");
    ui->statusValueLabel->style()->unpolish(ui->statusValueLabel);
    ui->statusValueLabel->style()->polish(ui->statusValueLabel);
}
