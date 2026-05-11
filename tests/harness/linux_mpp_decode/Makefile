CXX      = g++
CC       = gcc
ROOT     = ../../..
SELF_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
# -I$(SELF_DIR) so <devioctl.h> resolves to our stub regardless of make invocation dir
CFLAGS   = -O2 -Wall -I$(SELF_DIR) -I$(ROOT) -I$(ROOT)/mft -include $(SELF_DIR)/winshim.h
CXXFLAGS = $(CFLAGS) -std=c++17

OBJS_CXX = parser_glue.o regbuilder_h264.o h264_packed_tables.o dpb.o
OBJS_C   = linux_mpp_decode.o mpp_svc.o

all: linux_mpp_decode

parser_glue.o:        $(ROOT)/mft/parser_glue.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
regbuilder_h264.o:    $(ROOT)/mft/regbuilder_h264.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
h264_packed_tables.o: $(ROOT)/mft/h264_packed_tables.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
dpb.o:                $(ROOT)/mft/dpb.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

linux_mpp_decode: $(OBJS_CXX) $(OBJS_C)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f linux_mpp_decode *.o
