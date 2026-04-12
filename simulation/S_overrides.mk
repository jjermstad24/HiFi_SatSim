# Generalized S_override.mk file
include $(JEOD_HOME)/bin/jeod/generic_S_overrides.mk

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