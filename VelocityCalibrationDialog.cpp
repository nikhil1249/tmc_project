#include "VelocityCalibrationDialog.h"

#include <QAbstractItemView>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QtGlobal>
#include <cmath>

VelocityCalibrationDialog::VelocityCalibrationDialog(double currentSlope,
                                                     double currentIntercept,
                                                     QWidget *parent)
    : QDialog(parent),
      slope(currentSlope),
      intercept(currentIntercept),
      equationValid(std::abs(currentSlope) > 1e-12)
{
    setWindowTitle(QStringLiteral("Velocity RPM Equation"));
    resize(760, 520);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    QLabel *infoLabel = new QLabel(
        QStringLiteral("Enter the equation directly, or add measured samples and derive it. "
                       "The main GUI can then run in either RAW mode or RPM mode. "
                       "Motor core logic still sends RAW velocity to TMC6460. Equation: rpm = m * raw + c."));
    infoLabel->setWordWrap(true);
    mainLayout->addWidget(infoLabel);

    QGridLayout *manualLayout = new QGridLayout;
    slopeSpin = new QDoubleSpinBox;
    slopeSpin->setDecimals(12);
    slopeSpin->setRange(-1000000.0, 1000000.0);
    slopeSpin->setValue(slope);
    slopeSpin->setMinimumWidth(160);

    interceptSpin = new QDoubleSpinBox;
    interceptSpin->setDecimals(12);
    interceptSpin->setRange(-1000000.0, 1000000.0);
    interceptSpin->setValue(intercept);
    interceptSpin->setMinimumWidth(160);

    manualLayout->addWidget(new QLabel(QStringLiteral("Slope m:")), 0, 0);
    manualLayout->addWidget(slopeSpin, 0, 1);
    manualLayout->addWidget(new QLabel(QStringLiteral("Intercept c:")), 0, 2);
    manualLayout->addWidget(interceptSpin, 0, 3);
    mainLayout->addLayout(manualLayout);

    QGridLayout *inputLayout = new QGridLayout;

    rawSpin = new QSpinBox;
    rawSpin->setRange(-10000000, 10000000);
    rawSpin->setValue(4000000);
    rawSpin->setMinimumWidth(140);

    rpmSpin = new QSpinBox;
    rpmSpin->setRange(-100000, 100000);
    rpmSpin->setValue(0);
    rpmSpin->setMinimumWidth(140);

    currentSpin = new QDoubleSpinBox;
    currentSpin->setRange(0.0, 10000.0);
    currentSpin->setDecimals(2);
    currentSpin->setSuffix(QStringLiteral(" mA"));
    currentSpin->setValue(0.0);
    currentSpin->setMinimumWidth(140);

    QPushButton *addButton = new QPushButton(QStringLiteral("Add Sample"));
    QPushButton *removeButton = new QPushButton(QStringLiteral("Remove Selected"));
    QPushButton *clearButton = new QPushButton(QStringLiteral("Clear"));

    inputLayout->addWidget(new QLabel(QStringLiteral("Raw velocity:")), 0, 0);
    inputLayout->addWidget(rawSpin, 0, 1);
    inputLayout->addWidget(new QLabel(QStringLiteral("Actual RPM:")), 0, 2);
    inputLayout->addWidget(rpmSpin, 0, 3);
    inputLayout->addWidget(new QLabel(QStringLiteral("Current:")), 0, 4);
    inputLayout->addWidget(currentSpin, 0, 5);
    inputLayout->addWidget(addButton, 0, 6);
    inputLayout->addWidget(removeButton, 1, 5);
    inputLayout->addWidget(clearButton, 1, 6);
    mainLayout->addLayout(inputLayout);

    sampleTable = new QTableWidget(0, 3, this);
    sampleTable->setHorizontalHeaderLabels(QStringList()
                                           << QStringLiteral("Raw velocity")
                                           << QStringLiteral("Actual RPM")
                                           << QStringLiteral("Current mA"));
    sampleTable->horizontalHeader()->setStretchLastSection(true);
    sampleTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    sampleTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    sampleTable->setSelectionMode(QAbstractItemView::SingleSelection);
    mainLayout->addWidget(sampleTable, 1);

    QGridLayout *equationLayout = new QGridLayout;
    QPushButton *deriveButton = new QPushButton(QStringLiteral("Derive Equation From Samples"));
    applyButton = new QPushButton(QStringLiteral("Apply Equation"));
    equationLabel = new QLabel;
    equationLabel->setWordWrap(true);
    statusLabel = new QLabel(QStringLiteral("Edit slope/intercept directly, or add at least two samples and derive equation."));
    statusLabel->setWordWrap(true);

    equationLayout->addWidget(deriveButton, 0, 0);
    equationLayout->addWidget(applyButton, 0, 1);
    equationLayout->addWidget(equationLabel, 1, 0, 1, 2);
    equationLayout->addWidget(statusLabel, 2, 0, 1, 2);
    mainLayout->addLayout(equationLayout);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, this);
    mainLayout->addWidget(buttonBox);

    connect(addButton, &QPushButton::clicked, this, [this]() { addSample(); });
    connect(removeButton, &QPushButton::clicked, this, [this]() { removeSelectedSample(); });
    connect(clearButton, &QPushButton::clicked, this, [this]() { clearSamples(); });
    connect(deriveButton, &QPushButton::clicked, this, [this]() { deriveEquation(); });
    connect(applyButton, &QPushButton::clicked, this, [this]() { applyEquation(); });
    connect(slopeSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { onManualEquationChanged(); });
    connect(interceptSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { onManualEquationChanged(); });
    connect(buttonBox, &QDialogButtonBox::rejected, this, &VelocityCalibrationDialog::reject);

    updateEquationLabel();
}

void VelocityCalibrationDialog::onManualEquationChanged()
{
    slope = slopeSpin->value();
    intercept = interceptSpin->value();
    equationValid = std::abs(slope) > 1e-12;
    updateEquationLabel();
}

void VelocityCalibrationDialog::addSample()
{
    Sample sample;
    sample.raw = rawSpin->value();
    sample.rpm = rpmSpin->value();
    sample.currentMilliAmp = currentSpin->value();
    appendSampleToTable(sample);
    statusLabel->setText(QStringLiteral("Sample added. Click Derive Equation when you have at least two samples."));
}

void VelocityCalibrationDialog::removeSelectedSample()
{
    const int row = sampleTable->currentRow();
    if (row >= 0)
    {
        sampleTable->removeRow(row);
        equationValid = std::abs(slopeSpin->value()) > 1e-12;
        statusLabel->setText(QStringLiteral("Sample removed. Derive equation again or use manual slope/intercept."));
    }
}

void VelocityCalibrationDialog::clearSamples()
{
    sampleTable->setRowCount(0);
    equationValid = std::abs(slopeSpin->value()) > 1e-12;
    statusLabel->setText(QStringLiteral("Samples cleared. Manual equation is still available."));
}

QVector<VelocityCalibrationDialog::Sample> VelocityCalibrationDialog::samplesFromTable() const
{
    QVector<Sample> samples;
    for (int row = 0; row < sampleTable->rowCount(); ++row)
    {
        Sample sample;
        sample.raw = sampleTable->item(row, 0)->text().toDouble();
        sample.rpm = sampleTable->item(row, 1)->text().toDouble();
        sample.currentMilliAmp = sampleTable->item(row, 2)->text().toDouble();
        samples.append(sample);
    }
    return samples;
}

void VelocityCalibrationDialog::appendSampleToTable(const Sample &sample)
{
    const int row = sampleTable->rowCount();
    sampleTable->insertRow(row);
    sampleTable->setItem(row, 0, new QTableWidgetItem(QString::number(sample.raw, 'f', 0)));
    sampleTable->setItem(row, 1, new QTableWidgetItem(QString::number(sample.rpm, 'f', 0)));
    sampleTable->setItem(row, 2, new QTableWidgetItem(QString::number(sample.currentMilliAmp, 'f', 2)));
}

void VelocityCalibrationDialog::deriveEquation()
{
    const QVector<Sample> samples = samplesFromTable();
    if (samples.size() < 2)
    {
        equationValid = std::abs(slopeSpin->value()) > 1e-12;
        statusLabel->setText(QStringLiteral("Need at least two samples."));
        return;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;

    for (const Sample &sample : samples)
    {
        sumX += sample.raw;
        sumY += sample.rpm;
        sumXX += sample.raw * sample.raw;
        sumXY += sample.raw * sample.rpm;
    }

    const double n = static_cast<double>(samples.size());
    const double denominator = (n * sumXX) - (sumX * sumX);

    if (std::abs(denominator) < 1e-9)
    {
        equationValid = false;
        statusLabel->setText(QStringLiteral("Cannot derive equation. Raw velocity values must not all be same."));
        return;
    }

    slope = ((n * sumXY) - (sumX * sumY)) / denominator;
    intercept = (sumY - (slope * sumX)) / n;
    equationValid = std::abs(slope) > 1e-12;

    slopeSpin->setValue(slope);
    interceptSpin->setValue(intercept);
    updateEquationLabel();
    statusLabel->setText(QStringLiteral("Equation derived. Click Apply Equation to use RPM mode in main GUI."));
}

void VelocityCalibrationDialog::applyEquation()
{
    slope = slopeSpin->value();
    intercept = interceptSpin->value();
    equationValid = std::abs(slope) > 1e-12;

    if (!equationValid)
    {
        statusLabel->setText(QStringLiteral("Invalid equation. Slope must not be zero."));
        return;
    }

    accept();
}

void VelocityCalibrationDialog::updateEquationLabel()
{
    equationLabel->setText(QStringLiteral("Equation: rpm = %1 * raw + %2")
                           .arg(slope, 0, 'g', 12)
                           .arg(intercept, 0, 'g', 12));
}
