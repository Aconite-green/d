QT       += core gui charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    drum.cpp \
    src/CanManager.cpp \
    src/CommandParser.cpp \
    src/DrumRobot.cpp \
    src/HomeManager.cpp \
    src/Motor.cpp \
    src/PathManager.cpp \
    src/RL_assign.cpp \
    src/Sensor.cpp \
    src/TaskUtility.cpp \
    src/TestManager.cpp

HEADERS += \
    drum.h \
    include/managers/CanManager.hpp \
    include/managers/HomeManager.hpp \
    include/managers/PathManager.hpp \
    include/managers/RL_assign.hpp \
    include/managers/TestManager.hpp \
    include/motors/CommandParser.hpp \
    include/motors/Motor.hpp \
    include/tasks/DrumRobot.hpp \
    include/tasks/SystemState.hpp \
    include/tasks/TaskUtility.hpp \
    include/usbio/Global.hpp \
    include/usbio/ICPDAS_USBIO.hpp \
    include/usbio/SenSor.hpp \
    include/usbio/ThreadFun.hpp \
    include/usbio/Timer.hpp \
    include/usbio/USBIO_Comm.hpp \
    include/usbio/USBIO_Device.hpp \
    include/usbio/USBIO_Object_Layout.hpp

FORMS += \
    drum.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    res.qrc

DISTFILES += \
    include/managers/codeConfession.txt \
    include/managers/rT.txt

unix:!macx: LIBS += -L$$PWD/lib/ -lUSBIO_64

INCLUDEPATH += $$PWD/include/usbio
DEPENDPATH += $$PWD/include/usbio
