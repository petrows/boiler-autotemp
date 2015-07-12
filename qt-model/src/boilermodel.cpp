#include "boilermodel.h"
#include <cmath>
#include <QDebug>

BoilerModel::BoilerModel()
{
	reset();
}

void BoilerModel::reset()
{
	paramsChanged.clear();

	simTime = 300.0;

	tempOutput = 25;
	tempCurrent = 40;
	pressureCoef = 1.0;
}

void BoilerModel::calc()
{
	setServo(50);

	simStepsCount = simTime;

	qDebug() << "Sim steps " << simStepsCount;

	xTime.resize(simStepsCount);
	yTempOutput.resize(simStepsCount);
	yTempCurrent.resize(simStepsCount);
	yTempSet.resize(simStepsCount);
	yServoPos.resize(simStepsCount);
	yPressure.resize(simStepsCount);

	int normalFixDeep = 5;

	int heaterDelay = 20;
	QList<double> heaterDelayed;
	for (int x=0; x<heaterDelay; x++) heaterDelayed.push_back(tempOutput);

	QVector<double> normalValues(normalFixDeep);
	normalValues.fill(0);

	int servoFreq = 5;
	int servoWait = 25;

	double tempReal = tempOutput;

	double Pk = 1.08;
	double Ik = 0.12;
	double Dk = 2.43;

	double ItPrev = 0.0;
	double ErrorPrev = 0.0;

	for (int step=0; step<simStepsCount; step++)
	{
		double tms = step;

		// Check the current 'ideal' temp and
		// calc new temp value if needed
		double tempApply = 0.0; // No change
		double tempDelta = std::abs(tempReal - tempSet);
		double stepDir = (tempReal < tempSet) ? 1.0 : -1.0;
		if (0.1 < tempDelta)
		{
			tempApply = 0.3 * tempDelta;
			tempApply = tempApply * stepDir;

			tempReal = tempReal + tempApply;
		}

		heaterDelayed.push_front(tempReal);
		tempOutput = heaterDelayed.last();
		heaterDelayed.removeLast();

		normalValues.push_back(tempApply);
		normalValues.pop_front();
		yTempOutput[step] = tempOutput;
		yTempSet[step] = tempSet;
		yTempCurrent[step] = tempCurrent;
		yServoPos[step] = servoSet;
		yPressure[step] = pressureCoef * 100.0;
		xTime[step] = tms;

		// Check algo
		servoWait--;
		if (servoWait <= 0)
		{
			servoWait = servoFreq; // x second
			
			double curError = tempCurrent - tempOutput;

			double Pt = Pk * curError;
			double It = ItPrev + (Ik * curError);
			ItPrev = It;
			double Dt = Dk * (curError - ErrorPrev);

			ErrorPrev = curError;

			double Ut = Pt
					+ It
					+ Dt;
					;
			
			setServo(servoSet + Ut);
		}

		// Changed smth?
		for (QVector<ParamChange>::iterator it = paramsChanged.begin(); it != paramsChanged.end(); it++)
		{
			ParamChange z = *it;
			if (z.tms <= tms)
			{
				// Some new
				if (0 != z.pressureCoef) pressureCoef = z.pressureCoef;
				if (0 != z.tempCurrent) tempCurrent = z.tempCurrent;

				paramsChanged.erase(it, it);
			}
		}
		setServo(servoSet);
	}
}

void BoilerModel::setServo(int servoPers)
{
	servoSet = servoPers;
	if (servoSet > 100) servoSet = 100;
	if (servoSet < 0) servoSet = 0;
	tempSet = (30.0 * pressureCoef) + 20.0 * ((double)servoSet/100.0);
}
