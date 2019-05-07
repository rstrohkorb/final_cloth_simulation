TARGET=Test
SOURCES+= main.cpp \
          ../clothSim/src/Cloth.cpp \
          ../clothSim/src/MassPoint.cpp \
          ../clothSim/src/BVTree.cpp \
          ../clothSim/src/SphereBV.cpp \
          ../clothSim/src/SphereObj.cpp

LIBS+= -lgtest
INCLUDEPATH+= ../clothSim/include

# Following code written by Jon Macey
include($$(HOME)/NGL/UseNGL.pri)
