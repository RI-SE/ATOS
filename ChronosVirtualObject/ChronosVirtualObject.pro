#-------------------------------------------------
#
# Project created by QtCreator 2017-09-07T13:17:09
#
#-------------------------------------------------

QT       += core gui
QT       += network
QT       += websockets

CONFIG += c++11 console
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = ChronosVirtualObject
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
        locpoint.cpp \
        packet.cpp \
        #packetinterface.cpp \
        tcpserversimple.cpp \
        vbytearray.cpp \
        utility.cpp \
        mapwidget.cpp \
        carinfo.cpp \
        perspectivepixmap.cpp \
        osmclient.cpp \
        osmtile.cpp \
    virtualobject.cpp \
    qcustomplot.cpp \
    isocom.cpp

HEADERS += \
        mainwindow.h \
        locpoint.h \
        packet.h \
        #packetinterface.h \
        tcpserversimple.h \
        vbytearray.h \
        datatypes.h \
        utility.h \
        carinfo.h \
        perspectivepixmap.h \
        osmclient.h \
        osmtile.h \
        mapwidget.h \
    virtualobject.h \
    qcustomplot.h \
    isocom.h

FORMS += \
        mainwindow.ui

RESOURCES += \
    resources.qrc
