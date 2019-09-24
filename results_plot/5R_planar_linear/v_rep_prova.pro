TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += shared

DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY

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

unix:!macx {
    LIBS += -lrt
    LIBS += -pthread
}

SOURCES += \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApi.c \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApiPlatform.c \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/common/shared_memory.c \
    /home/stefano/Robotic2_project/plot_result/v_rep_5R/v_rep_prova.cpp \

HEADERS +=\
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApi.h \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApiPlatform.h \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApiInternal.h \
    /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/include/shared_memory.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

INCLUDEPATH += /home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/include \
/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi \
/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/include/stack