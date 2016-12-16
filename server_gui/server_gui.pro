#-------------------------------------------------
#
# Project created by QtCreator 2016-03-11T14:49:19
#
#-------------------------------------------------

QT       += core
QT       += widgets
QT       += gui
QT       += network
QT       += opengl
QT       += websockets

# OpenGL support
DEFINES += HAS_OPENGL

TARGET = server_gui
TEMPLATE = app

contains(DEFINES, HAS_ASSIMP) {
    LIBS += -lassimp
}

SOURCES += main.cpp\
        mainwindow.cpp \
    utility.cpp \
    mapwidget.cpp \
    carinfo.cpp \
    locpoint.cpp \
    perspectivepixmap.cpp \
    osmclient.cpp \
    osmtile.cpp

HEADERS  += mainwindow.h \
    datatypes.h \
    utility.h \
    carinfo.h \
    locpoint.h \
    perspectivepixmap.h \
    osmclient.h \
    osmtile.h \
    mapwidget.h

FORMS    += mainwindow.ui

contains(DEFINES, HAS_OPENGL) {
    SOURCES += orientationwidget.cpp
    HEADERS += orientationwidget.h
}

RESOURCES += \
    resources.qrc
