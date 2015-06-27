#-------------------------------------------------
#
# Project created by QtCreator 2014-12-13T00:33:15
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = qt-model
TEMPLATE = app


SOURCES += src/main.cpp\
src/dialog.cpp \
    src/qcustomplot.cpp \
    src/boilermodel.cpp

HEADERS  += src/dialog.h \
    src/qcustomplot.h \
    src/boilermodel.h

INCLUDEPATH += src/

FORMS    += src/dialog.ui
