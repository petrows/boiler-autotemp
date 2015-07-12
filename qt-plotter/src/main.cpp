#include "dialogrecorder.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	DialogRecorder w;
    w.show();

    return a.exec();
}
