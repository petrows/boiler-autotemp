#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "boilermodel.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

    void mkSensorModel();
    void mkSensor();

	void mkGraph();
	void repaintGraph(QString name);

private:
    Ui::Dialog *ui;
	BoilerModel mdl;
};

#endif // DIALOG_H
