# Digital voice decoder library (P25 Phase 1, DMR, M17, YSF, D-STAR, NXDN), linked into radiod
CXX ?= c++
CC ?= cc
AR ?= ar
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -MMD -MP
CFLAGS ?= -O2 -g
CFLAGS += -std=gnu11 -Wall -MMD -MP
CPPFLAGS += -I. -Iupstream -Iupstream/imbe_vocoder -Iupstream/libm17

BUILD_DIR := build
SOURCES := dv_api.cpp dv_auto.cpp dv_frontend.cpp dv_ambe.cpp fsk4.cpp \
	p25_rx.cpp p25_voice.cpp upstream/p25_framer.cc upstream/bch.cc \
	$(wildcard upstream/imbe_vocoder/*.cc) \
	dmr_rx.cpp upstream/p25p2_vf.cc upstream/rs.cc \
	upstream/bptc19696.cc upstream/hamming.cc upstream/golay2087.cc \
	upstream/software_imbe_decoder.cc upstream/imbe_decoder.cc \
	upstream/mbelib.c upstream/ambe.c \
	m17_rx.cpp ysf_rx.cpp dstar_rx.cpp nxdn_rx.cpp
# libm17 has files with the same name in different directories, so its
# objects keep its directory layout
LIBM17_SOURCES := $(wildcard upstream/libm17/*.c upstream/libm17/*/*.c)
LIBM17_OBJECTS := $(patsubst upstream/libm17/%.c,$(BUILD_DIR)/libm17/%.o,$(LIBM17_SOURCES))
OBJECTS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(basename $(notdir $(SOURCES))))) $(LIBM17_OBJECTS)
LIBRARY := $(BUILD_DIR)/libdv.a

vpath %.cpp .
vpath %.cc upstream upstream/imbe_vocoder
vpath %.c upstream

.PHONY: all clean
all: $(LIBRARY)

$(LIBRARY): $(OBJECTS)
	rm -f $@
	$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.cc | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/libm17/%.o: upstream/libm17/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $@

# Unmodified OP25 source: decode_tap() takes a parameter it doesn't use
$(BUILD_DIR)/software_imbe_decoder.o: CXXFLAGS += -Wno-unused-parameter

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
