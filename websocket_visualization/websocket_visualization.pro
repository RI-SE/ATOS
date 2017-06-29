QT += core websockets
QT -= gui

CONFIG += c++11

LIBS += ../server/build/libutil.a
LIBS += -lrt

TARGET = websocket_visualization
#CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    visualizationserver.cpp \
    generator.cpp

HEADERS += \
    visualizationserver.h \
    generator.h \
    ../server/inc/util.h

DISTFILES +=
