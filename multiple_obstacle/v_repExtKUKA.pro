# Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.2.3 rev4 on December 21st 2015

QT -= core
QT -= gui

TARGET = v_repExtKUKA
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
CONFIG += shared
INCLUDEPATH += "../include"

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

win32 {
    DEFINES += WIN_VREP
}

macx {
    DEFINES += MAC_VREP
}

unix:!macx {
    DEFINES += LIN_VREP
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

HEADERS += \
    v_repExtKUKA.h \
    ../include/luaFunctionData.h \
    ../include/luaFunctionDataItem.h \
    ../include/v_repLib.h \  
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/flaccoController.h \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/Manipulator.h \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/path_trajectory.h \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/Task.h \


SOURCES += \
    v_repExtKUKA.cpp \
    ../common/luaFunctionData.cpp \
    ../common/luaFunctionDataItem.cpp \
    ../common/v_repLib.cpp \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/Manipulator.cpp \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/flaccoController.cpp \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/v_rep_skeleton_integration_different_2_task_multiple_obstacles_3/path_trajectory.cpp \

INCLUDEPATH += /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/common \
/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi \
/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/include/stack