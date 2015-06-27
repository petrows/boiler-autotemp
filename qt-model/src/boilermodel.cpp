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

	simTime = 90.0;
	simStep = 0.1;

	tempOutput = 25;
	tempCurrent = 40;
	pressureCoef = 1.0;
}

void BoilerModel::calc()
{
	setServo(50);

	simStepsCount = ceil(simTime / simStep);

	qDebug() << "Sim steps " << simStepsCount;

	xTime.resize(simStepsCount);
	yTempOutput.resize(simStepsCount);
	yTempCurrent.resize(simStepsCount);
	yTempSet.resize(simStepsCount);
	yServoPos.resize(simStepsCount);
	yPressure.resize(simStepsCount);

	int normalFixDeep = 5;

	QVector<double> normalValues(normalFixDeep);
	normalValues.fill(0);

	int servoWait = 0.0 * (1.0/simStep);

	double Pk = 8.0;
	double Ik = 0.0;
	double Dk = 1.0;

	double ItPrev = 0.0;
	double ErrorPrev = 0.0;

	for (int step=0; step<simStepsCount; step++)
	{
		double tms = simStep * (double)step;

		// Check the current 'ideal' temp and
		// calc new temp value if needed
		double tempApply = 0.0; // No change
		double tempDelta = std::abs(tempOutput - tempSet);
		double stepDir = (tempOutput < tempSet) ? 1.0 : -1.0;
		if (0.1 < tempDelta)
		{
			tempApply = 0.2 * (1.0 * (tempDelta/10.0) );
			tempApply = tempApply * stepDir;

			// Calc normalzied value
			double lastSum = 0;
			for (int x=0; x<normalFixDeep; x++) lastSum += normalValues[x];
			tempApply = (lastSum + tempApply) / ((double)normalFixDeep + 1.0);

			tempOutput = tempOutput + tempApply;
			// qDebug() << tempOutput;
		}

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
		if (servoWait < 0)
		{
			servoWait = 3.0 * (1.0/simStep); // x second
			
			double curError = tempCurrent - tempOutput;

			double Pt = Pk * curError;
			double It = ItPrev - (Ik * curError);
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
