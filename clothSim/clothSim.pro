TARGET=clothSim
SOURCES+=src/main.cpp \
         src/Cloth.cpp \
         src/MassPoint.cpp \
         src/NGLScene.cpp \
         src/NGLSceneMouseControls.cpp \
         src/BVTree.cpp \
         src/SphereBV.cpp \
         src/SphereObj.cpp

HEADERS+= include/MassPoint.h \
          include/Cloth.h \
          include/UnorderedPair.h \
          include/NGLScene.h \
          include/WindowParams.h \
          include/BVTree.h \
          include/SphereBV.h \
          include/SphereObj.h

OTHER_FILES+= shaders/*.glsl

INCLUDEPATH+= include

cache()

# Following code written by Jon Macey
include($(HOME)/NGL/UseNGL.pri)
