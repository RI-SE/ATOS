QT += core websockets
QT -= gui

CONFIG += c++11

LIBS += -lrt

TARGET = websocket_visualization
#CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    visualizationserver.cpp \
    generator.cpp \
    util.c

HEADERS += \
    visualizationserver.h \
    generator.h \
    util.h

DISTFILES +=
