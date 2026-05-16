CXX      = g++
CC       = gcc
ROOT     = ../../..
SELF_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
# -I$(SELF_DIR) so <devioctl.h> resolves to our stub regardless of make invocation dir
CFLAGS   = -O2 -Wall -I$(SELF_DIR) -I$(ROOT) -I$(ROOT)/mft -include $(SELF_DIR)/winshim.h
CXXFLAGS = $(CFLAGS) -std=c++17

OBJS_CXX = au_iter.o parser_glue.o parser_glue_h265.o avcc_to_annexb.o \
           regbuilder_h264.o regbuilder_h265.o \
           h264_packed_tables.o h265_packed_tables.o dpb.o \
           decode_engine.o backend_linux.o linux_mpp_decode.o
OBJS_C   = mpp_svc.o

all: linux_mpp_decode

au_iter.o:             $(ROOT)/mft/au_iter.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
backend_linux.o:       $(ROOT)/mft/engine/backend_linux.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
decode_engine.o:       $(ROOT)/mft/engine/decode_engine.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
mpp_svc.o:             $(ROOT)/mft/engine/mpp_svc.c
	$(CC)  $(CFLAGS)   -c $< -o $@
parser_glue.o:         $(ROOT)/mft/parser_glue.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
parser_glue_h265.o:    $(ROOT)/mft/parser_glue_h265.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
avcc_to_annexb.o:      $(ROOT)/mft/avcc_to_annexb.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
regbuilder_h264.o:     $(ROOT)/mft/regbuilder_h264.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
regbuilder_h265.o:     $(ROOT)/mft/regbuilder_h265.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
h264_packed_tables.o:  $(ROOT)/mft/h264_packed_tables.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
h265_packed_tables.o:  $(ROOT)/mft/h265_packed_tables.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
dpb.o:                 $(ROOT)/mft/dpb.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

linux_mpp_decode: $(OBJS_CXX) $(OBJS_C)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f linux_mpp_decode *.o
