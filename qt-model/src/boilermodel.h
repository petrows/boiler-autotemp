#ifndef BOILERMODEL_H
#define BOILERMODEL_H

#include <QObject>
#include <QVector>

struct ParamChange
{
	double tms;
	double pressureCoef;
	double tempCurrent;
	ParamChange(double tm = 0, double pressure = 0, double temp = 0) : tms(tm), pressureCoef(pressure), tempCurrent(temp) {}
};

class BoilerModel
{
public:
    BoilerModel();

    double simStep;
    double simTime;

	double pressureCoef;

	double tempOutput;  // Real temp
	double tempSet;    // 'ideal' temp
	double tempCurrent;    // Selected by user
	int servoSet;    // servo pos

    int simStepsCount;

	QVector<double> xTime;

	QVector<double> yTempOutput;
	QVector<double> yTempCurrent;
	QVector<double> yTempSet;
	QVector<double> yServoPos;
	QVector<double> yPressure;

	QVector<ParamChange> paramsChanged;

	void reset();
	void calc();
	void setServo(int servoPers);
};

#endif // BOILERMODEL_H
