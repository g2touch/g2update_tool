###############################################################################
# compiler setting
###############################################################################
CC       ?= gcc
CXX      ?= g++
CFLAGS   += -g -Wall
CXXFLAGS += $(CFLAGS) -Weffc++
INCPATH  = -I.

###############################################################################
# source files setting
###############################################################################
C_SOURCES        = $(shell find . -name "*.c")
CXX_SOURCES      = $(shell find . -name "*.cpp")
C_OBJS           = $(patsubst %.c,%.o,$(wildcard $(C_SOURCES)))
CXX_OBJS         = $(patsubst %.cpp,%.o,$(wildcard $(CXX_SOURCES)))
OBJS             = $(CXX_OBJS) $(C_OBJS)
INC_H_PATHS      = $(shell find ../* -name "*.h")

###############################################################################
.PHONY : all clean clean_all
###############################################################################
all: $(CXX_OBJS) $(C_OBJS)

%.o:%.c $(INC_H_PATHS)
	@echo $(CC) -M $(CXXFLAGS) $(INCPATH) $<
	$(CC) -c $(CFLAGS) $(INCPATH) $< -o $@

%.o:%.cpp $(INC_H_PATHS)
	@echo $(CXX) -M $(CXXFLAGS) $(INCPATH) $<
	$(CXX) -c $(CXXFLAGS) $(INCPATH) $< -o $@
###############################################################################
clean:
	@rm -vfr $(OBJS)

clean_all:
	@rm -vfr $(OBJS)
###############################################################################
