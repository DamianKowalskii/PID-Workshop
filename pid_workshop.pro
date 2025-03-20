QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17 -Wall -Wextra
QMAKE_CXXFLAGS += -Wa,-mbig-obj

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    qcustomplot/qcustomplot.cpp \
    src/model/autotuning/autotuner.cpp \
    src/model/autotuning/population.c \
    src/model/car.cpp \
    src/model/pid.cpp

HEADERS += \
    include/model/autotuning/autotuner.h \
    include/model/autotuning/autotuning_config.h \
    include/model/autotuning/population.h \
    include/model/car.h \
    include/model/car_parameters.h \
    include/model/pid.h \
    include/model/simulation.h \
    include/model/simulation_config.h \
    include/view/gui.h \
    mainwindow.h \
    qcustomplot/qcustomplot.h

INCLUDEPATH += \
    include/model/autotuning \
     include/model \
     include/view
FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
