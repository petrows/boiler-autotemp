#include "dialogrecorder.h"
#include "ui_dialogrecorder.h"
#include <time.h>
#include "QDebug"

DialogRecorder::DialogRecorder(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::DialogRecorder)
{
	portDataIndex = 0;
	ui->setupUi(this);

	connect(&port, &QSerialPort::readyRead, this, &DialogRecorder::onPortRead);

	QPen pn;
	QVector<double> timeLabels, tempLabels, drosselLabels;
	for (int x=0; x<displaySteps; x+=10) timeLabels.push_back(x);
	for (int x=0; x<=80; x+=5) tempLabels.push_back(x);
	for (int x=0; x<=100; x+=10) drosselLabels.push_back(x);

	ui->plot->xAxis->setLabel("Время, сек");
	ui->plot->xAxis->setRange(0, displaySteps);
	ui->plot->xAxis->setAutoTicks(false);
	ui->plot->xAxis->setTickVector(timeLabels);
	ui->plot->xAxis->setSubTickCount(10);

	ui->plot->yAxis->setLabel("Температура, °C");
	ui->plot->yAxis->setRange(20, 70);
	ui->plot->yAxis->setAutoTicks(false);
	ui->plot->yAxis->setTickVector(tempLabels);
	ui->plot->yAxis->setSubTickCount(5);

	ui->plot->yAxis2->setLabel("Параметры, %");
	ui->plot->yAxis2->setRange(0, 100);
	ui->plot->yAxis2->setVisible(true);
	ui->plot->yAxis2->setAutoTicks(false);
	ui->plot->yAxis2->setTickVector(drosselLabels);

	grTempCels = ui->plot->addGraph(0, ui->plot->yAxis);
	pn.setColor(Qt::red);
	pn.setWidth(3);
	grTempCels->setPen(pn);
	grTempCels->setName("Датчик, °C");

	grTempSet = ui->plot->addGraph(0, ui->plot->yAxis);
	pn.setColor(Qt::blue);
	pn.setWidth(1);
	grTempSet->setPen(pn);
	grTempSet->setName("Выбрано, °C");

	grServo = ui->plot->addGraph(0, ui->plot->yAxis2);
	pn.setColor(0x006600);
	pn.setWidth(2);
	pn.setStyle(Qt::DotLine);
	grServo->setPen(pn);
	grServo->setName("Дроссель, %");
}

DialogRecorder::~DialogRecorder()
{
	delete ui;
}

void DialogRecorder::on_serialStart_clicked()
{
	if (port.isOpen())
	{
		// Stop the process
		port.close();
		logFile.close();
		dataList.clear();
		ui->plot->replot();
		ui->serialDevice->setDisabled(false);
		ui->serialOutput->setDisabled(false);
		return;
	}

	port.setPortName(ui->serialDevice->text());
	port.setBaudRate(9600);
	if (!port.open(QIODevice::ReadOnly))
	{
		QMessageBox::warning(this, "Error!", "Error opening port!");
		return;
	}

	// Open log file
	QString fName = QString("%1-%2.csv").arg(ui->serialOutput->text(), time(NULL));
	qDebug() << fName;
	logFile.setFileName(fName);
	if (!logFile.open(QIODevice::WriteOnly))
	{
		QMessageBox::warning(this, "Error!", "Error opening output file!");
		return;
	}

	ui->serialDevice->setDisabled(true);
	ui->serialOutput->setDisabled(true);
}

void DialogRecorder::onPortRead()
{
	// Get bytes?...
	dataBuffer = dataBuffer + port.readAll();

	if (dataBuffer.size() < 5) return;
	qDebug() << "Get bytes: " << port.bytesAvailable();

	// Search the pkg start mark (0xFF)
	int pkgStartIndex = dataBuffer.indexOf(0xFF, 0);
	if (-1 == pkgStartIndex)
	{
		// Garbage?
		dataBuffer.clear();
		return;
	}

	// Cut garbage from the start
	if (pkgStartIndex > 0)
	{
		dataBuffer.remove(0, pkgStartIndex);
	}

	if (dataBuffer.length() < 5)
	{
		// Some part truncated...
		return;
	}

	SerialPortData portData(portDataIndex);

	portData.selected_cels = dataBuffer[1];
	portData.sensor_cels = dataBuffer[2];
	portData.sensor_raw = dataBuffer[3];
	portData.servo = dataBuffer[4];
	dataList.push_back(portData);

	QStringList csvList;
	csvList << QString::number(portData.selected_cels);
	csvList << QString::number(portData.sensor_cels);
	csvList << QString::number(portData.sensor_raw);
	csvList << QString::number(portData.servo);
	csvList << "\n";
	QString csvLine = csvList.join(",");

	logFile.write(csvLine.toUtf8());

	ui->lcdServo->display(portData.servo);
	ui->lcdTempRaw->display(portData.sensor_raw);
	ui->lcdTempC->display(portData.sensor_cels);
	ui->lcdTempSet->display(portData.selected_cels);

	portDataIndex++;
	dataBuffer.remove(0, 5); // Cut off this

	qDebug() << "Temp = " << portData.sensor_cels;

	if (dataBuffer.size() >= 5) onPortRead(); // Try to read again

	// Update plot
	onUpdatePlot();
}

void DialogRecorder::onUpdatePlot()
{
	QVector<double> indexData;
	QVector<double> tempSensorData;
	QVector<double> tempSelectedData;
	QVector<double> servoData;

	for (int x=dataList.size()-1, n=0; x>0 && n<displaySteps; x--, n++)
	{
		indexData.append(n);
		tempSensorData.push_back(dataList.at(x).sensor_cels);
		tempSelectedData.push_back(dataList.at(x).selected_cels);
		servoData.push_back(dataList.at(x).servo);
	}

	grTempCels->setData(indexData, tempSensorData);
	grTempSet->setData(indexData, tempSelectedData);
	grServo->setData(indexData, servoData);

	ui->plot->replot();
}
