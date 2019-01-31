###############################################################################
# compiler setting
###############################################################################
CC       ?= gcc
CXX      ?= g++
CPPFLAGS ?=
LDFLAGS  ?=
CFLAGS   += -g -Wall -pthread
CXXFLAGS += -g -Wall -pthread -Weffc++
DEFINES  = -DAPP_VERSION=\"1.2.3080\" $(USER_DEFINE)
INCPATH  = -I. -I../G2ArgHandle -I../G2DeviceHandle -I../G2Procedure -I../Log

###############################################################################
# source files setting
###############################################################################
CXX_SOURCES     = $(shell find ./ -name "*.cpp")
CXX_OBJS        = $(patsubst %.cpp,%.o,$(wildcard $(CXX_SOURCES)))
BIN_DIR         = ../bin
EXEC_SOURCES    = $(shell find . -name "*.cpp")
EXEC            = $(subst .,$(BIN_DIR),$(basename $(EXEC_SOURCES)))
ALL_OBJS        = $(shell find ../* -name "*.o" -not -path "*../MAIN/*")
INC_H_FILES     = $(shell find ../* -name "*.h")

###############################################################################
.PHONY : all install clean clean_all
###############################################################################
all: bin_dir $(EXEC)

bin_dir:
	@mkdir -p $(BIN_DIR)

%.o:%.cpp $(INC_H_FILES)
	@echo $(CXX) -M $(CXXFLAGS) $(CPPFLAGS) $(INCPATH) $(DEFINES) $<
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $(INCPATH) $(DEFINES) $< -o $@

$(BIN_DIR)/%:%.o $(ALL_OBJS) $(INC_H_FILES)
	$(CXX) $< -o $@ $(ALL_OBJS) $(CXXFLAGS) $(LDFLAGS)
	@echo '=============== Finished building: $@ ==============='

###############################################################################
clean:
	@rm -vfr $(CXX_OBJS)

clean_all: clean
	@rm -vfr $(EXEC)
###############################################################################
