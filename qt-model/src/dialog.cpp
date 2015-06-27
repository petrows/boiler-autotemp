#include "boilermodel.h"
#include "dialog.h"
#include "ui_dialog.h"

#include <cmath>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);


    /*
    //
    double simTime = 5.0;
    double simStep = 0.01; // 100ms
    int simSteps = simTime / simStep;
    QVector<double> x(simSteps), y(simSteps), yTemp(simSteps), yTempSin(simSteps), yTempSin2(simSteps);
    QVector<double> xLabels;

    double tempSpeed = 2.0; // C/sec
    double tempSet = 10.0; // C/sec

    for (int i=0; i<simSteps; i++)
    {
        if (i%10 == 0) xLabels.append(i);
        x[i] = (double)i * simStep;
        y[i] = cos(x[i]);

        double yTempCurrent = tempSpeed * (simStep * i);
        if (yTempCurrent > tempSet) yTempCurrent = tempSet;
        yTemp[i] = yTempCurrent;

        double tempSin = 20 + (10 * (1.0-sin((M_PI/2.0) + x[i])));
        double tempSin2 = 20 + (10 * ( 0.0 - sin((M_PI/2.0) + (x[i]*x[i]))));

        // yTempSin[i] = std::min(20.0, tempSin);
        if (x[i] < (M_PI)) yTempSin[i] = tempSin;
        yTempSin2[i] = tempSin2;
    }

    QCPGraph * gr1 = ui->plot->addGraph();
    gr1->setData(x, y);

    QCPGraph * gr2 = ui->plot->addGraph();
    gr2->setData(x, yTempSin);
    QPen p2;
    p2.setColor(Qt::red);
    p2.setWidth(2);
    gr2->setPen(p2);

    QCPGraph * gr3 = ui->plot->addGraph();
    gr3->setData(x, yTempSin2);
    QPen p3;
    p3.setColor(Qt::green);
    p3.setWidth(2);
    gr3->setPen(p3);

    ui->plot->xAxis->setRange(0, simTime);
    //ui->plot->xAxis->setAutoTicks(false);
    //ui->plot->xAxis->setTickVector(xLabels);
    ui->plot->yAxis->setRange(0, 50);
    ui->plot->yAxis->setAutoTickStep(false);
    ui->plot->yAxis->setTickStep(5);
    //ui->plot->yAxis->setTickVectorLabels(xLabels);
    ui->plot->replot();
    ui->plot->savePdf("graph.pdf", false, 800, 600);*/

	//mkSensor();
	// mkSensorModel();
	mkGraph();
}

Dialog::~Dialog()
{
	delete ui;
}

void Dialog::mkSensorModel()
{
	QCPGraph * gr;
	QPen pn;

	double simTime = 60.0;
	double simStepTime = 0.1;

	double tempStart = 25.0;
	double tempCurrent = tempStart;
	double tempSet = 35.0;
	double tempDeltaPrev = 0.0;

	int normalFixDeep = 5;

	int steps = simTime / simStepTime;

	QVector<double> xTime(steps), yTemp(steps), yTempSet(steps);
	QVector<double> normalValues(normalFixDeep);
	normalValues.fill(0);

	QVector<double> tempLabels;
	for (int x=10; x<=50; x+=5) tempLabels.push_back(x);

	QVector<double> timeLabels;
	for (int x=0; x<=simTime; x+=5) timeLabels.push_back(x);

	for (int x=0; x<steps; x++)
	{
		// Calc new temp value
		double tempApply = 0.0; // No change
		double tempDelta = std::abs(tempCurrent - tempSet);
		double stepDir = (tempCurrent < tempSet) ? 1.0 : -1.0;
		if (0.1 < tempDelta)
		{
			if (tempDelta > 10)
			{
				tempApply = 0.19 * 1.0;
			} else {
				tempApply = 0.2 * (1.0 * (tempDelta/10.0) );
			}

			// if (tempApply < 0.01) tempApply = 0.01;

			// qDebug() << tempApply;

			tempApply = tempApply * stepDir;

			// Calc normalzied value
			double lastSum = 0;
			for (int x=0; x<normalFixDeep; x++) lastSum += normalValues[x];
			tempApply = (lastSum + tempApply) / ((double)normalFixDeep + 1.0);

			tempDeltaPrev = tempApply;
			tempCurrent = tempCurrent + tempApply;
			qDebug() << tempCurrent;
		}

		normalValues.push_back(tempApply);
		normalValues.pop_front();

		xTime[x] = (double)x * simStepTime;

		if (10.0 == xTime[x]) tempSet = 40.0;
		if (40.0 == xTime[x]) tempSet = 30.0;

		yTempSet[x] = tempSet;
		yTemp[x] = tempCurrent;
	}

	gr = ui->plot->addGraph();
	gr->setData(xTime, yTemp);
	pn.setColor(Qt::red);
	pn.setWidth(3);
	gr->setPen(pn);
	gr->setName("Реальное значение");

	gr = ui->plot->addGraph();
	gr->setData(xTime, yTempSet);
	pn.setColor(0x008080);
	pn.setWidth(1);
	pn.setStyle(Qt::DashLine);
	gr->setPen(pn);
	gr->setName("Конечное значение");




	ui->plot->xAxis->setLabel("Время, сек");
	ui->plot->xAxis->setRange(0, simTime);
	ui->plot->xAxis->setAutoTicks(false);
	ui->plot->xAxis->setTickVector(timeLabels);

	ui->plot->yAxis->setLabel("Температура, °C");
	ui->plot->yAxis->setRange(20, 50);
	ui->plot->yAxis->setAutoTicks(false);
	ui->plot->yAxis->setTickVector(tempLabels);

	ui->plot->legend->setVisible(true);

	ui->plot->replot();
	ui->plot->savePdf("graph-boiler.pdf", false, 800, 400);
}

void Dialog::mkSensor()
{

    QCPGraph * gr;
    QPen pn;





    int steps = (20) + 1;
    double stepTemp = 0.47;

    QVector<double> xLabels;
    QVector<double> yLabels;
    QVector<double> xD(steps), yD(steps),yDlin(steps);

    for (int x=0; x<steps; x++)
    {
        //xD[x] = 30.0 + (stepTemp * (double)x) + 0.5 * ((double)(100 - qrand()%200) / 100.0);
        xD[x] = 30.0 + x;
        yD[x] = 158 + floor( ((double)x / 0.47) + 1.5 * ((double)(100 - qrand()%200) / 100.0) );
        yDlin[x] = 158.0 + ((double)x / 0.47);
        //if (((int)yD[x])%50 == 0) yLabels.push_back(yD[x]);
        //if (((int)xD[x])%10 == 0) xLabels.push_back(xD[x]);
    }

    for (int x=150; x<=210; x+=10)
    {
        yLabels.push_back(x);
    }

    for (int x=30; x<=50; x+=1)
    {
        xLabels.push_back(x);
    }

    qDebug() << yD;
    qDebug() << xD;

    gr = ui->plot->addGraph();
    gr->setData(xD, yD);
    pn.setColor(Qt::red);
    pn.setWidth(3);
    gr->setPen(pn);

    gr = ui->plot->addGraph();
    gr->setData(xD, yDlin);
    pn.setColor(0x008080);
    pn.setWidth(1);
    pn.setStyle(Qt::DashLine);
    gr->setPen(pn);

    ui->plot->xAxis->setLabel("Температура, °C");
    ui->plot->xAxis->setRange(30, 50);
    ui->plot->xAxis->setAutoTicks(false);
    ui->plot->xAxis->setTickVector(xLabels);
    ui->plot->yAxis->setLabel("Сигнал АЦП");
    ui->plot->yAxis->setRange(145, 215);
    ui->plot->yAxis->setAutoTicks(false);
    ui->plot->yAxis->setTickVector(yLabels);

    ui->plot->replot();
	ui->plot->savePdf("graph-sensor.pdf", false, 800, 400);
}

void Dialog::mkGraph()
{
	QCPGraph * gr;
	QPen pn;

	QVector<double> timeLabels, tempLabels, drosselLabels;
	for (int x=0; x<mdl.simTime; x+=10) timeLabels.push_back(x);
	for (int x=0; x<=80; x+=5) tempLabels.push_back(x);
	for (int x=0; x<=120; x+=10) drosselLabels.push_back(x);

	gr = ui->plot->addGraph();
	gr->setData(mdl.xTime, mdl.yTempOutput);
	pn.setColor(Qt::red);
	pn.setWidth(3);
	gr->setPen(pn);
	gr->setName("Реальное значение, °C");

	gr = ui->plot->addGraph();
	gr->setData(mdl.xTime, mdl.yTempSet);
	pn.setColor(0x0000FF);
	pn.setWidth(2);
	pn.setStyle(Qt::DotLine);
	gr->setPen(pn);
	gr->setName("Конечное значение, °C");

	gr = ui->plot->addGraph();
	gr->setData(mdl.xTime, mdl.yTempCurrent);
	pn.setColor(0x808000);
	pn.setWidth(2);
	pn.setStyle(Qt::SolidLine);
	gr->setPen(pn);
	gr->setName("Желаемая температура, °C");

	gr = ui->plot->addGraph(0, ui->plot->yAxis2);
	gr->setData(mdl.xTime, mdl.yServoPos);
	pn.setColor(0x006600);
	pn.setWidth(2);
	pn.setStyle(Qt::DotLine);
	gr->setPen(pn);
	gr->setName("Дроссель, %");

	gr = ui->plot->addGraph(0, ui->plot->yAxis2);
	gr->setData(mdl.xTime, mdl.yPressure);
	pn.setColor(0x800080);
	pn.setWidth(2);
	pn.setStyle(Qt::SolidLine);
	gr->setPen(pn);
	gr->setName("Сила нагрева, %");

	ui->plot->xAxis->setLabel("Время, сек");
	ui->plot->xAxis->setRange(0, mdl.simTime);
	ui->plot->xAxis->setAutoTicks(false);
	ui->plot->xAxis->setTickVector(timeLabels);
	//ui->plot->xAxis->setAutoTickCount(15);
	ui->plot->xAxis->setSubTickCount(10);

	ui->plot->yAxis->setLabel("Температура, °C");
	ui->plot->yAxis->setRange(20, 80);
	ui->plot->yAxis->setAutoTicks(false);
	ui->plot->yAxis->setTickVector(tempLabels);
	ui->plot->yAxis->setSubTickCount(5);

	ui->plot->yAxis2->setLabel("Параметры, %");
	ui->plot->yAxis2->setRange(0, 150);
	ui->plot->yAxis2->setVisible(true);
	ui->plot->yAxis2->setAutoTicks(false);
	ui->plot->yAxis2->setTickVector(drosselLabels);

	ui->plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);
	ui->plot->legend->setVisible(true);

	mdl.reset();
	mdl.tempCurrent = 40;
	mdl.pressureCoef = 1.0;
	repaintGraph("ideal");

	mdl.reset();
	mdl.tempCurrent = 35;
	mdl.pressureCoef = 1.2;
	mdl.paramsChanged.push_back(ParamChange(20.0, 0, 40));
	repaintGraph("hotrun");

	mdl.reset();
	mdl.tempCurrent = 35;
	mdl.pressureCoef = 1.2;
	mdl.paramsChanged.push_back(ParamChange(20.0, 0, 30));
	repaintGraph("hotrun-low");
// return;

	mdl.reset();
	mdl.tempCurrent = 45;
	mdl.pressureCoef = 0.5;
	mdl.paramsChanged.push_back(ParamChange(20.0, 0, 40));
	mdl.paramsChanged.push_back(ParamChange(30.0, 1.2, 0));
	repaintGraph("plow-hi");

	mdl.reset();
	mdl.tempCurrent = 45;
	mdl.pressureCoef = 0.5;
	mdl.paramsChanged.push_back(ParamChange(30.0, 0.8, 0));
	repaintGraph("plow");

	mdl.reset();
	mdl.tempOutput = 40;
	mdl.tempCurrent = 30;
	mdl.paramsChanged.push_back(ParamChange(50.0, 0.8, 0));
	repaintGraph("hotstart");

	mdl.reset();
	mdl.simTime = 180;
	mdl.tempCurrent = 30;
	mdl.paramsChanged.push_back(ParamChange(20.0, 0.8, 0));
	mdl.paramsChanged.push_back(ParamChange(40.0, 0, 40));
	mdl.paramsChanged.push_back(ParamChange(50.0, 0.9, 0));
	mdl.paramsChanged.push_back(ParamChange(90.0, 1.1, 0));
	mdl.paramsChanged.push_back(ParamChange(130.0, 1.0, 0));
	repaintGraph("longrun");

	// ui->plot->savePdf("graph-test.pdf", false, 800, 600);

	// qDebug() << QImageWriter::supportedImageFormats();
}

void Dialog::repaintGraph(QString name)
{
	mdl.calc();

	QVector<double> timeLabels;
	for (int x=0; x<mdl.simTime; x+=10) timeLabels.push_back(x);
	ui->plot->xAxis->setTickVector(timeLabels);
	ui->plot->xAxis->setRange(0, mdl.simTime);

	ui->plot->graph(0)->setData(mdl.xTime, mdl.yTempOutput);
	ui->plot->graph(1)->setData(mdl.xTime, mdl.yTempSet);
	ui->plot->graph(2)->setData(mdl.xTime, mdl.yTempCurrent);
	ui->plot->graph(3)->setData(mdl.xTime, mdl.yServoPos);
	ui->plot->graph(4)->setData(mdl.xTime, mdl.yPressure);
	ui->plot->replot();

	QString fname = QString("graph-%1").arg(name);

	ui->plot->savePdf(fname + ".pdf", false, 800, 600);
	// ui->plot->saveJpg(fname + ".jpg", false, 1600, 1200);
}
