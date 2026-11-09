# Digital voice decoder library (P25 Phase 1, DMR), linked into radiod
CXX ?= c++
CC ?= cc
AR ?= ar
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -MMD -MP
CFLAGS ?= -O2 -g
CFLAGS += -std=gnu11 -Wall -MMD -MP
CPPFLAGS += -I. -Iupstream -Iupstream/imbe_vocoder

BUILD_DIR := build
SOURCES := dv_api.cpp dv_frontend.cpp fsk4.cpp \
	p25_rx.cpp p25_voice.cpp upstream/p25_framer.cc upstream/bch.cc \
	$(wildcard upstream/imbe_vocoder/*.cc) \
	dmr_rx.cpp upstream/p25p2_vf.cc upstream/rs.cc \
	upstream/software_imbe_decoder.cc upstream/imbe_decoder.cc \
	upstream/mbelib.c upstream/ambe.c
OBJECTS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(basename $(notdir $(SOURCES)))))
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

$(BUILD_DIR):
	mkdir -p $@

# Unmodified OP25 source: decode_tap() takes a parameter it doesn't use
$(BUILD_DIR)/software_imbe_decoder.o: CXXFLAGS += -Wno-unused-parameter

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
