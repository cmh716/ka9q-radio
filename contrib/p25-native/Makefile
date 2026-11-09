# Native P25 Phase 1 decoder library, linked into radiod
CXX ?= c++
AR ?= ar
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -MMD -MP
CPPFLAGS += -I. -Iupstream -Iupstream/imbe_vocoder

BUILD_DIR := build
SOURCES := p25_rx.cpp p25_voice.cpp fsk4.cpp upstream/p25_framer.cc upstream/bch.cc \
	$(wildcard upstream/imbe_vocoder/*.cc)
OBJECTS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(basename $(notdir $(SOURCES)))))
LIBRARY := $(BUILD_DIR)/libp25native.a

vpath %.cpp .
vpath %.cc upstream upstream/imbe_vocoder

.PHONY: all clean
all: $(LIBRARY)

$(LIBRARY): $(OBJECTS)
	rm -f $@
	$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.cc | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
