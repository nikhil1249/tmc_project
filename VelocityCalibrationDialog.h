#ifndef VELOCITYCALIBRATIONDIALOG_H
#define VELOCITYCALIBRATIONDIALOG_H

#include <QDialog>
#include <QVector>

class QLabel;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QTableWidget;

class VelocityCalibrationDialog : public QDialog
{
public:
    explicit VelocityCalibrationDialog(double currentSlope,
                                       double currentIntercept,
                                       QWidget *parent = nullptr);

    double appliedSlope() const { return slope; }
    double appliedIntercept() const { return intercept; }

private:
    struct Sample
    {
        double raw = 0.0;
        double rpm = 0.0;
        double currentMilliAmp = 0.0;
    };

    QSpinBox *rawSpin = nullptr;
    QSpinBox *rpmSpin = nullptr;
    QDoubleSpinBox *currentSpin = nullptr;
    QDoubleSpinBox *slopeSpin = nullptr;
    QDoubleSpinBox *interceptSpin = nullptr;
    QTableWidget *sampleTable = nullptr;
    QLabel *equationLabel = nullptr;
    QLabel *statusLabel = nullptr;
    QPushButton *applyButton = nullptr;

    double slope = 0.0;
    double intercept = 0.0;
    bool equationValid = false;

    void addSample();
    void removeSelectedSample();
    void clearSamples();
    void deriveEquation();
    void applyEquation();
    void onManualEquationChanged();

    QVector<Sample> samplesFromTable() const;
    void appendSampleToTable(const Sample &sample);
    void updateEquationLabel();
};

#endif
