QT += core
QT -= gui

CONFIG += c++11

TARGET = test_warping
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    mesh.cpp \
    meshflow.cpp \
    quad.cpp

INCLUDEPATH += "/usr/local/include"
INCLUDEPATH += "/home/eric/SuperLU/SuperLU_5.2.0/include"
INCLUDEPATH += "/home/eric/Eigen-3.3.4/Eigen"
INCLUDEPATH += "/usr/include/suitesparse"
INCLUDEPATH += "/home/eric/ceres/ceres-solver-1.13.0/include"
INCLUDEPATH += "/home/eric/ceres/ceres-solver-1.13.0/config"
LIBS += `pkg-config --libs opencv`
LIBS += -llapack -lblas -larmadillo -lsuperlu


QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS +=  -fopenmp
LIBS += -lgomp -lpthread

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/lapack/ -llapack

INCLUDEPATH += $$PWD/../../../../usr/lib/lapack
DEPENDPATH += $$PWD/../../../../usr/lib/lapack

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/lapack/liblapack.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/lapack/ -llapack

INCLUDEPATH += $$PWD/../../../../usr/lib/lapack
DEPENDPATH += $$PWD/../../../../usr/lib/lapack

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/libblas/ -lblas

INCLUDEPATH += $$PWD/../../../../usr/lib/libblas
DEPENDPATH += $$PWD/../../../../usr/lib/libblas

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/libblas/libblas.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/libblas/ -lblas

INCLUDEPATH += $$PWD/../../../../usr/lib/libblas
DEPENDPATH += $$PWD/../../../../usr/lib/libblas

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/ -lopenblas

INCLUDEPATH += $$PWD/../../../../usr/include
DEPENDPATH += $$PWD/../../../../usr/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/libopenblas.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/ -lopenblas

INCLUDEPATH += $$PWD/../../../../usr/include
DEPENDPATH += $$PWD/../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../ceres/ceres-solver-1.13.0/lib/ -lceres

INCLUDEPATH += $$PWD/../../ceres/ceres-solver-1.13.0/include
DEPENDPATH += $$PWD/../../ceres/ceres-solver-1.13.0/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../ceres/ceres-solver-1.13.0/lib/libceres.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lgflags

INCLUDEPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libgflags.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lglog

INCLUDEPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libglog.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lsuitesparseconfig

INCLUDEPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libsuitesparseconfig.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lspqr

INCLUDEPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libspqr.a

unix:!macx: LIBS += -L$$PWD/../../../../usr/lib/x86_64-linux-gnu/ -lcholmod

INCLUDEPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/../../../../usr/lib/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/lib/x86_64-linux-gnu/libcholmod.a


#unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lsuperlu

#INCLUDEPATH += $$PWD/../../../../usr/local/include
#DEPENDPATH += $$PWD/../../../../usr/local/include

#unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../usr/local/lib/libsuperlu.a

unix:!macx: LIBS += -L$$PWD/../../SuperLU/SuperLU_5.2.0/lib/ -lsuperlu

INCLUDEPATH += $$PWD/../../SuperLU/SuperLU_5.2.0/include
DEPENDPATH += $$PWD/../../SuperLU/SuperLU_5.2.0/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../SuperLU/SuperLU_5.2.0/lib/libsuperlu.a

HEADERS += \
    mesh.h \
    quad.h \
    meshflow.h

DISTFILES +=
