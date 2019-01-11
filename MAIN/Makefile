###############################################################################
# compiler setting
###############################################################################
CC       ?= gcc
CXX      ?= g++
LDFLAGS  ?=
DEFINES  = -DAPP_VERSION=\"1.2.2993\"
CFLAGS   += -g -Wall $(DEFINES) -pthread
CXXFLAGS += $(CFLAGS) -Weffc++
INCPATH  = -I. -I../G2ArgHandle -I../G2DeviceHandle -I../G2Procedure -I../Log

###############################################################################
# source files setting
###############################################################################
CXX_SOURCES  = $(shell find ./ -name "*.cpp")
CXX_OBJS     = $(patsubst %.cpp,%.o,$(wildcard $(CXX_SOURCES)))
BIN_DIR      = ../bin
EXEC_SOURCES = $(shell find . -name "*.cpp")
EXEC         = $(subst .,$(BIN_DIR),$(basename $(EXEC_SOURCES)))
OBJS_DIR     = ../out
ALL_OBJS     = $(shell find $(OBJS_DIR) -name "*.o")

###############################################################################
.PHONY : all install clean clean_all
###############################################################################
all: $(BIN_DIR) $(EXEC)

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

%.o:%.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) $< -o $@

$(BIN_DIR)/%:%.o
	$(CXX) $< -o $@ $(ALL_OBJS) $(CXXFLAGS) $(LDFLAGS)
	@echo 'Finished building: $@'

###############################################################################
clean:
	@rm -vfr $(CXX_OBJS)

clean_all: clean
	@rm -vfr $(EXEC) bin
###############################################################################
