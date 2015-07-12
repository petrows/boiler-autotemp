#ifndef DIALOGRECORDER_H
#define DIALOGRECORDER_H

#include <qcustomplot.h>
#include <QDialog>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <types.h>

namespace Ui {
class DialogRecorder;
}

class DialogRecorder : public QDialog
{
	Q_OBJECT

public:
	explicit DialogRecorder(QWidget *parent = 0);
	~DialogRecorder();

private slots:
	void on_serialStart_clicked();

	void onPortRead();
	void onUpdatePlot();

private:
	const static int displaySteps = 100;

	Ui::DialogRecorder *ui;
	QSerialPort port;
	QByteArray dataBuffer;
	int portDataIndex;

	QCPGraph * grTempCels;
	QCPGraph * grServo;
	QCPGraph * grTempSet;

	QList< SerialPortData > dataList;
};

#endif // DIALOGRECORDER_H
