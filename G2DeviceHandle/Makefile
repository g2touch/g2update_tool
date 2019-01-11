###############################################################################
# compiler setting
###############################################################################
CC       ?= gcc
CXX      ?= g++
CFLAGS   += -g -Wall
CXXFLAGS += $(CFLAGS) -Weffc++
INCPATH  = -I. -I../G2ArgHandle -I../Log

###############################################################################
# source files setting
###############################################################################
C_SOURCES        = $(shell find . -name "*.c")
CXX_SOURCES      = $(shell find . -name "*.cpp")
C_OBJS           = $(patsubst %.c,%.o,$(wildcard $(C_SOURCES)))
CXX_OBJS         = $(patsubst %.cpp,%.o,$(wildcard $(CXX_SOURCES)))
OBJS             = $(C_OBJS) $(CXX_OBJS)
OBJS_DIR         = ../out

###############################################################################
.PHONY : all clean clean_all
###############################################################################
all: $(OBJS_DIR) $(CXX_OBJS) $(C_OBJS)

%.o:%.c
	$(CC) -c $(CFLAGS) $(INCPATH) $< -o $@

%.o:%.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) $< -o $(OBJS_DIR)/$@

$(OBJS_DIR):
	@mkdir -p $(OBJS_DIR)
###############################################################################
clean:
	@rm -vfr $(OBJS_DIR)/$(OBJS)

clean_all:
	@rm -vfr $(OBJS_DIR)/$(OBJS)
###############################################################################
