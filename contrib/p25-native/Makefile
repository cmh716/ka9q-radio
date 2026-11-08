CXX ?= c++
CC ?= cc
AR ?= ar
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -Wpedantic -pthread
CPPFLAGS += -I. -Iupstream -Iupstream/imbe_vocoder

BUILD_DIR := build
IMBE_SOURCES := $(wildcard upstream/imbe_vocoder/*.cc)
IMBE_OBJECTS := $(patsubst upstream/imbe_vocoder/%.cc,$(BUILD_DIR)/%.o,$(IMBE_SOURCES))
OBJECTS := $(BUILD_DIR)/imbe.o $(BUILD_DIR)/phase1.o $(BUILD_DIR)/fsk4.o \
	$(BUILD_DIR)/c4fm.o $(BUILD_DIR)/cqpsk.o \
	$(BUILD_DIR)/p25_native_api.o \
	$(BUILD_DIR)/p25_framer.o $(BUILD_DIR)/bch.o $(IMBE_OBJECTS)
LIBRARY := $(BUILD_DIR)/libp25native.a
TEST := $(BUILD_DIR)/test_imbe
C_TEST := $(BUILD_DIR)/test_c_api
PHASE1_TEST := $(BUILD_DIR)/test_phase1
FSK4_TEST := $(BUILD_DIR)/test_fsk4
C4FM_TEST := $(BUILD_DIR)/test_c4fm
CQPSK_TEST := $(BUILD_DIR)/test_cqpsk
C4FM_C_TEST := $(BUILD_DIR)/test_c4fm_api
C4FM_FIXTURE_TOOL := $(BUILD_DIR)/generate_c4fm_dibits
OFFLINE_RUNNER := $(BUILD_DIR)/p25-offline
PCAP_RUNNER := $(BUILD_DIR)/p25-pcap
PCAP_FIXTURE_TOOL := $(BUILD_DIR)/generate_c4fm_pcap
WAV_TEST := $(BUILD_DIR)/test_wav

.PHONY: all test clean fixture-tool offline pcap

all: $(LIBRARY)

test: $(TEST) $(C_TEST) $(PHASE1_TEST) $(FSK4_TEST) $(C4FM_TEST) \
	$(CQPSK_TEST) $(C4FM_C_TEST) $(OFFLINE_RUNNER) $(PCAP_RUNNER) \
	$(PCAP_FIXTURE_TOOL) $(WAV_TEST)
	./$(TEST)
	./$(C_TEST)
	./$(PHASE1_TEST)
	./$(FSK4_TEST)
	./$(C4FM_TEST)
	./$(CQPSK_TEST)
	./$(C4FM_C_TEST)
	./$(OFFLINE_RUNNER) fixtures/op25-c4fm-48k.s16be \
		$(BUILD_DIR)/test-c4fm.wav 0x293
	./$(PCAP_FIXTURE_TOOL) fixtures/op25-c4fm-48k.s16be \
		$(BUILD_DIR)/test-c4fm.pcap
	./$(PCAP_RUNNER) $(BUILD_DIR)/test-c4fm.pcap \
		$(BUILD_DIR)/test-c4fm-pcap.wav 0x293 0x1234abcd
	cmp $(BUILD_DIR)/test-c4fm.wav $(BUILD_DIR)/test-c4fm-pcap.wav
	./$(PCAP_RUNNER) $(BUILD_DIR)/test-c4fm.pcap \
		- 0x293 0x1234abcd
	./$(WAV_TEST)

fixture-tool: $(C4FM_FIXTURE_TOOL)

offline: $(OFFLINE_RUNNER)

pcap: $(PCAP_RUNNER)

$(LIBRARY): $(OBJECTS)
	$(AR) rcs $@ $^

$(TEST): $(BUILD_DIR)/test_imbe.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(C_TEST): $(BUILD_DIR)/test_c_api.o $(LIBRARY)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(PHASE1_TEST): $(BUILD_DIR)/test_phase1.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(FSK4_TEST): $(BUILD_DIR)/test_fsk4.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(C4FM_TEST): $(BUILD_DIR)/test_c4fm.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(CQPSK_TEST): $(BUILD_DIR)/test_cqpsk.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(C4FM_C_TEST): $(BUILD_DIR)/test_c4fm_api.o $(LIBRARY)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD_DIR)/imbe.o: imbe.cpp imbe.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/phase1.o: phase1.cpp phase1.h imbe.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/fsk4.o: fsk4.cpp fsk4.h upstream/fsk4_mmse_taps.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/c4fm.o: c4fm.cpp c4fm.h fsk4.h phase1.h \
	upstream/frame_sync_magics.h upstream/p25_c4fm_rx_taps.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/cqpsk.o: cqpsk.cpp cqpsk.h c4fm.h phase1.h \
	upstream/gnuradio_mmse_taps.h upstream/p25_c4fm_rx_taps.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/p25_native_api.o: p25_native_api.cpp p25_native_api.h \
	c4fm.h cqpsk.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/p25_framer.o: upstream/p25_framer.cc
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/bch.o: upstream/bch.cc
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/test_c_api.o: test_c_api.c imbe.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) -std=c11 -Wall -Wextra -Wpedantic -c -o $@ $<

$(BUILD_DIR)/test_c4fm_api.o: test_c4fm_api.c p25_native_api.h phase1.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CPPFLAGS) -std=c11 -Wall -Wextra -Wpedantic -c -o $@ $<

$(BUILD_DIR)/test_phase1.o: test_phase1.cpp phase1.h test_frame_builder.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/test_fsk4.o: test_fsk4.cpp fsk4.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/test_c4fm.o: test_c4fm.cpp c4fm.h upstream/frame_sync_magics.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/test_cqpsk.o: test_cqpsk.cpp cqpsk.h test_frame_builder.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/test_wav.o: test_wav.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(C4FM_FIXTURE_TOOL): tools/generate_c4fm_dibits.cpp test_frame_builder.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $<

$(OFFLINE_RUNNER): p25_offline.cpp $(LIBRARY)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(PCAP_RUNNER): p25_pcap.cpp $(LIBRARY)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(PCAP_FIXTURE_TOOL): tools/generate_c4fm_pcap.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $<

$(WAV_TEST): $(BUILD_DIR)/test_wav.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD_DIR)/test_imbe.o: test_imbe.cpp imbe.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: upstream/imbe_vocoder/%.cc
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR)
