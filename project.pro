TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /usr/local/include/eigen3
LIBS += -lopencv_core

HEADERS += MotionModel.h
SOURCES += MotionModel.cpp


