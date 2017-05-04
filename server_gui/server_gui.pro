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
#QT       += websockets

# OpenGL support
DEFINES += HAS_OPENGL

TARGET = server_gui
TEMPLATE = app

release_win {
    DESTDIR = build/win
    OBJECTS_DIR = build/win/obj
    MOC_DIR = build/win/obj
    RCC_DIR = build/win/obj
    UI_DIR = build/win/obj
}

release_lin {
    # http://micro.nicholaswilson.me.uk/post/31855915892/rules-of-static-linking-libstdc-libc-libgcc
    # http://insanecoding.blogspot.se/2012/07/creating-portable-linux-binaries.html
    QMAKE_LFLAGS += -static-libstdc++ -static-libgcc
    DESTDIR = build/lin
    OBJECTS_DIR = build/lin/obj
    MOC_DIR = build/lin/obj
    RCC_DIR = build/lin/obj
    UI_DIR = build/lin/obj
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
