#-------------------------------------------------
#
# Project created by QtCreator 2019-03-22T11:12:06
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_ui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qnode.cpp

HEADERS  += mainwindow.h \
    qnode.h

FORMS    += mainwindow.ui


INCLUDEPATH += \
                /opt/ros/kinetic/include\
                /usr/include
