#To make a project qtable, just add a .pro file with the following tags
#run with  qmake -o Makefile trabalho.pro to generate the makefile
#run make

OBJECTS_DIR= generated_files #Intermediate object files directory
MOC_DIR    = generated_files #Intermediate moc files directory
INCLUDEPATH = include                  \
              /opt/ros/indigo/include  \
              /usr/include/eigen3      \
              gnuplot-iostream-master
DEPENDPATH = include

QMAKE_CXXFLAGS += -std=c++11
CONFIG -= app_bundle qt

HEADERS +=  include/MotionModel.h \
            include/Node.h


SOURCES +=  src/MotionModel.cpp  \
            src/Node.cpp \
            main.cpp

LIBS    += -lopencv_core -lboost_iostreams -lboost_system -lboost_filesystem
