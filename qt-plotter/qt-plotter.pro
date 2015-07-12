#-------------------------------------------------
#
# Project created by QtCreator 2014-12-13T00:33:15
#
#-------------------------------------------------

QT       += core gui
QT += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = qt-plotter
TEMPLATE = app


SOURCES += src/main.cpp \
    src/dialogrecorder.cpp \
    src/qcustomplot.cpp

INCLUDEPATH += src/

FORMS += \
    src/dialogrecorder.ui

HEADERS += \
    src/dialogrecorder.h \
    src/qcustomplot.h \
    src/types.h



