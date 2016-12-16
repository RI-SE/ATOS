QT += core websockets
QT -= gui

CONFIG += c++11

TARGET = websocket_visualization
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    visualizationserver.cpp \
    generator.cpp

HEADERS += \
    visualizationserver.h \
    generator.h

DISTFILES +=
