###############################################################################
# compiler setting
###############################################################################
CC       ?= gcc
CXX      ?= g++
CPPFLAGS ?= -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE
CFLAGS   += -g -Wall
CXXFLAGS += -g -Wall -Weffc++
INCPATH  = -I.
DEFINES  = $(USER_DEFINE)

###############################################################################
# source files setting
###############################################################################
C_SOURCES        = $(shell find . -name "*.c")
CXX_SOURCES      = $(shell find . -name "*.cpp")
C_OBJS           = $(patsubst %.c,%.o,$(wildcard $(C_SOURCES)))
CXX_OBJS         = $(patsubst %.cpp,%.o,$(wildcard $(CXX_SOURCES)))
OBJS             = $(CXX_OBJS) $(C_OBJS)
INC_H_FILES      = $(shell find ../* -name "*.h")

###############################################################################
.PHONY : all clean clean_all
###############################################################################
all: $(CXX_OBJS) $(C_OBJS)

%.o:%.c $(INC_H_FILES)
	@echo $(CC) -M $(CFLAGS) $(CPPFLAGS) $(DEFINES) $(INCPATH) $<
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $(DEFINES) $(INCPATH) $< -o $@

%.o:%.cpp $(INC_H_FILES)
	@echo $(CXX) -M $(CXXFLAGS) $(CPPFLAGS) $(DEFINES) $(INCPATH) $<
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $(DEFINES) $(INCPATH) $< -o $@
###############################################################################
clean:
	@rm -vfr $(OBJS)

clean_all:
	@rm -vfr $(OBJS)
###############################################################################
