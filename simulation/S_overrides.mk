# Generalized S_override.mk file
include $(JEOD_HOME)/bin/jeod/generic_S_overrides.mk

TRICK_USER_LINK_LIBS += -lparquet -larrow -pthread

SIMULATION_ROOT=../

# ------------------------------------------
# Debug / Release switch
# ------------------------------------------
DEBUG ?= 0

ifeq ($(DEBUG),1)
    TRICK_CXXFLAGS += -g -O0 -DDEBUG
else
    TRICK_CXXFLAGS += -O2 -DNDEBUG
endif

TRICK_CXXFLAGS += -I${SIMULATION_ROOT}/models
TRICK_CXXFLAGS += -I${SIMULATION_ROOT}/fsw
TRICK_CXXFLAGS += -I${TRICK_HOME}/include
TRICK_CXXFLAGS += -std=c++17

# Add models path to CP/ICG include search
TRICK_SFLAGS += -I${SIMULATION_ROOT}/models
TRICK_ICGFLAGS += -I${SIMULATION_ROOT}/models