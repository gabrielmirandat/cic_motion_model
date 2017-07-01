TEMPLATE = app
CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11
CONFIG -= app_bundle qt

INCLUDEPATH += /usr/include/eigen3
LIBS += -lopencv_core -lboost_iostreams -lboost_system -lboost_filesystem

HEADERS += MotionModel.h
SOURCES += MotionModel.cpp


