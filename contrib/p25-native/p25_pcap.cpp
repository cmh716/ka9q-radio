#include "c4fm.h"

#include <array>
#include <cerrno>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

struct file_closer {
    void operator()(FILE *file) const
    {
        if (file != nullptr)
            std::fclose(file);
    }
};

using file_ptr = std::unique_ptr<FILE, file_closer>;

struct wav_output {
    FILE *file;
    uint64_t data_bytes = 0;
    bool discard = false;
};

struct rtp_packet {
    const uint8_t *payload = nullptr;
    size_t payload_size = 0;
    uint32_t ssrc = 0;
    uint32_t timestamp = 0;
    uint16_t sequence = 0;
};

enum class packet_result {
    ignored,
    malformed,
    rtp
};

uint16_t read_be16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                 data[1]);
}

uint32_t read_be32(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

uint16_t read_u16(const uint8_t *data, bool little_endian)
{
    if (little_endian)
        return static_cast<uint16_t>(data[0] |
                                     (static_cast<uint16_t>(data[1]) << 8));
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                 data[1]);
}

uint32_t read_u32(const uint8_t *data, bool little_endian)
{
    if (little_endian)
        return static_cast<uint32_t>(data[0]) |
               (static_cast<uint32_t>(data[1]) << 8) |
               (static_cast<uint32_t>(data[2]) << 16) |
               (static_cast<uint32_t>(data[3]) << 24);
    return read_be32(data);
}

void store_le16(uint8_t *destination, uint16_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
}

void store_le32(uint8_t *destination, uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
    destination[2] = static_cast<uint8_t>(value >> 16);
    destination[3] = static_cast<uint8_t>(value >> 24);
}

bool write_wav_header(FILE *file, uint32_t pcm_bytes)
{
    std::array<uint8_t, 44> header{};
    std::memcpy(header.data(), "RIFF", 4);
    store_le32(header.data() + 4, pcm_bytes + 36);
    std::memcpy(header.data() + 8, "WAVEfmt ", 8);
    store_le32(header.data() + 16, 16);
    store_le16(header.data() + 20, 1);
    store_le16(header.data() + 22, 1);
    store_le32(header.data() + 24, 8000);
    store_le32(header.data() + 28, 16000);
    store_le16(header.data() + 32, 2);
    store_le16(header.data() + 34, 16);
    std::memcpy(header.data() + 36, "data", 4);
    store_le32(header.data() + 40, pcm_bytes);
    return std::fwrite(header.data(), 1, header.size(), file) == header.size();
}

int write_pcm(void *user_data, uint16_t, uint8_t, uint32_t, uint32_t,
              const int16_t pcm[P25_IMBE_PCM_SAMPLES])
{
    auto *output = static_cast<wav_output *>(user_data);
    constexpr uint64_t block_bytes = P25_IMBE_PCM_SAMPLES * sizeof(int16_t);
    if (output->data_bytes >
        std::numeric_limits<uint32_t>::max() - 36 - block_bytes)
        return -1;
    if (output->discard) {
        output->data_bytes += block_bytes;
        return 0;
    }

    std::array<uint8_t, block_bytes> bytes{};
    for (size_t i = 0; i < P25_IMBE_PCM_SAMPLES; ++i)
        store_le16(bytes.data() + 2 * i, static_cast<uint16_t>(pcm[i]));
    if (std::fwrite(bytes.data(), 1, bytes.size(), output->file) != bytes.size())
        return -1;
    output->data_bytes += block_bytes;
    return 0;
}

bool parse_integer(const char *text, uint32_t max_value, uint32_t *value)
{
    char *end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > max_value)
        return false;
    *value = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_pcap_header(FILE *file, bool *little_endian, uint32_t *snaplen,
                       uint32_t *linktype)
{
    std::array<uint8_t, 24> header{};
    if (std::fread(header.data(), 1, header.size(), file) != header.size())
        return false;

    const std::array<uint8_t, 4> magic{
        header[0], header[1], header[2], header[3]};
    if (magic == std::array<uint8_t, 4>{0xd4, 0xc3, 0xb2, 0xa1} ||
        magic == std::array<uint8_t, 4>{0x4d, 0x3c, 0xb2, 0xa1}) {
        *little_endian = true;
    } else if (magic == std::array<uint8_t, 4>{0xa1, 0xb2, 0xc3, 0xd4} ||
               magic == std::array<uint8_t, 4>{0xa1, 0xb2, 0x3c, 0x4d}) {
        *little_endian = false;
    } else {
        return false;
    }

    if (read_u16(header.data() + 4, *little_endian) != 2 ||
        read_u16(header.data() + 6, *little_endian) != 4)
        return false;
    *snaplen = read_u32(header.data() + 16, *little_endian);
    *linktype = read_u32(header.data() + 20, *little_endian);
    return *snaplen != 0 && *linktype == 1;
}

packet_result parse_ethernet_ipv4_udp(const uint8_t *frame, size_t length,
                                     rtp_packet *packet)
{
    if (length < 14)
        return packet_result::malformed;
    size_t offset = 14;
    uint16_t ether_type = read_be16(frame + 12);
    for (unsigned vlan_depth = 0;
         (ether_type == 0x8100 || ether_type == 0x88a8) && vlan_depth < 2;
         ++vlan_depth) {
        if (length < offset + 4)
            return packet_result::malformed;
        ether_type = read_be16(frame + offset + 2);
        offset += 4;
    }
    if (ether_type != 0x0800)
        return packet_result::ignored;
    if (length < offset + 20)
        return packet_result::malformed;

    const uint8_t *ip = frame + offset;
    const size_t ip_header_size = static_cast<size_t>(ip[0] & 0x0f) * 4;
    const size_t ip_total_size = read_be16(ip + 2);
    if ((ip[0] >> 4) != 4 || ip_header_size < 20 ||
        ip_total_size < ip_header_size || length < offset + ip_total_size)
        return packet_result::malformed;
    if (ip[9] != 17)
        return packet_result::ignored;
    if ((read_be16(ip + 6) & 0x3fff) != 0)
        return packet_result::ignored;

    const uint8_t *udp = ip + ip_header_size;
    const size_t ip_payload_size = ip_total_size - ip_header_size;
    if (ip_payload_size < 8)
        return packet_result::malformed;
    const size_t udp_size = read_be16(udp + 4);
    if (udp_size < 8 || udp_size > ip_payload_size)
        return packet_result::malformed;

    const uint8_t *rtp = udp + 8;
    const size_t rtp_size = udp_size - 8;
    if (rtp_size < 12)
        return packet_result::ignored;
    if ((rtp[0] >> 6) != 2)
        return packet_result::ignored;
    size_t header_size = 12 + 4 * (rtp[0] & 0x0f);
    if (header_size > rtp_size)
        return packet_result::malformed;
    if ((rtp[0] & 0x10) != 0) {
        if (rtp_size - header_size < 4)
            return packet_result::malformed;
        const size_t extension_size =
            4 + 4 * static_cast<size_t>(read_be16(rtp + header_size + 2));
        if (extension_size > rtp_size - header_size)
            return packet_result::malformed;
        header_size += extension_size;
    }

    size_t padding_size = 0;
    if ((rtp[0] & 0x20) != 0) {
        padding_size = rtp[rtp_size - 1];
        if (padding_size == 0 || padding_size > rtp_size - header_size)
            return packet_result::malformed;
    }
    const size_t payload_size = rtp_size - header_size - padding_size;
    if (payload_size == 0 || payload_size % 4 != 0)
        return packet_result::malformed;

    packet->sequence = read_be16(rtp + 2);
    packet->timestamp = read_be32(rtp + 4);
    packet->ssrc = read_be32(rtp + 8);
    packet->payload = rtp + header_size;
    packet->payload_size = payload_size;
    return packet_result::rtp;
}

bool parse_nac(const char *text, uint16_t *nac)
{
    uint32_t value;
    if (!parse_integer(text, 0x0fff, &value))
        return false;
    *nac = static_cast<uint16_t>(value);
    return true;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 5) {
        std::fprintf(stderr,
                     "usage: p25-pcap INPUT.pcap OUTPUT.wav|- [NAC [SSRC]]\n"
                     "Reads classic Ethernet/IPv4/UDP PCAP with 48 kHz "
                     "s16be IQ in RTP.\n"
                     "Use '-' to discard decoded PCM and report statistics only.\n"
                     "NAC defaults to 0 (accept any); SSRC defaults to the "
                     "first RTP stream.\n");
        return 2;
    }
    const bool discard_pcm = std::strcmp(argv[2], "-") == 0;
    if (!discard_pcm && std::strcmp(argv[1], argv[2]) == 0) {
        std::fprintf(stderr, "input and output paths must differ\n");
        return 2;
    }

    uint16_t nac = 0;
    if (argc >= 4 && !parse_nac(argv[3], &nac)) {
        std::fprintf(stderr, "invalid NAC: %s\n", argv[3]);
        return 2;
    }
    uint32_t requested_ssrc = 0;
    if (argc == 5 && !parse_integer(argv[4],
                                    std::numeric_limits<uint32_t>::max(),
                                    &requested_ssrc)) {
        std::fprintf(stderr, "invalid SSRC: %s\n", argv[4]);
        return 2;
    }
    bool have_ssrc = argc == 5;
    uint32_t selected_ssrc = requested_ssrc;

    file_ptr input(std::fopen(argv[1], "rb"));
    if (!input) {
        std::fprintf(stderr, "cannot open PCAP input %s: %s\n", argv[1],
                     std::strerror(errno));
        return 1;
    }
    struct stat input_stat;
    if (fstat(fileno(input.get()), &input_stat) != 0) {
        std::fprintf(stderr, "cannot inspect PCAP input %s: %s\n", argv[1],
                     std::strerror(errno));
        return 1;
    }
    file_ptr output_file;
    if (!discard_pcm) {
        struct stat output_stat;
        if (stat(argv[2], &output_stat) == 0) {
            if (input_stat.st_dev == output_stat.st_dev &&
                input_stat.st_ino == output_stat.st_ino) {
                std::fprintf(stderr,
                             "input and output refer to the same file\n");
                return 2;
            }
        } else if (errno != ENOENT) {
            std::fprintf(stderr, "cannot inspect WAV output %s: %s\n",
                         argv[2], std::strerror(errno));
            return 1;
        }
        output_file.reset(std::fopen(argv[2], "wb+"));
        if (!output_file) {
            std::fprintf(stderr, "cannot open WAV output %s: %s\n", argv[2],
                         std::strerror(errno));
            return 1;
        }
        if (!write_wav_header(output_file.get(), 0)) {
            std::fprintf(stderr, "cannot write WAV header: %s\n",
                         std::strerror(errno));
            return 1;
        }
    }

    bool little_endian = false;
    uint32_t snaplen = 0;
    uint32_t linktype = 0;
    if (!parse_pcap_header(input.get(), &little_endian, &snaplen, &linktype)) {
        std::fprintf(stderr,
                     "unsupported or malformed PCAP header "
                     "(expected classic PCAP, Ethernet link type)\n");
        return 1;
    }

    wav_output wav{output_file.get(), 0, discard_pcm};
    uint64_t packets_seen = 0;
    uint64_t packets_selected = 0;
    uint64_t malformed_packets = 0;
    uint64_t lost_packets = 0;
    uint64_t timestamp_discontinuities = 0;
    uint64_t samples = 0;
    uint64_t symbols = 0;
    uint64_t syncs = 0;
    float offset_hz = 0.0f;
    bool have_sequence = false;
    uint16_t previous_sequence = 0;
    uint32_t expected_timestamp = 0;

    try {
        p25_c4fm_decoder_impl decoder(write_pcm, &wav);
        if (decoder.set_expected_nac(nac) != P25_PHASE1_OK) {
            std::fprintf(stderr, "cannot configure expected NAC\n");
            return 1;
        }

        for (;;) {
            std::array<uint8_t, 16> record_header{};
            const size_t header_bytes =
                std::fread(record_header.data(), 1, record_header.size(),
                           input.get());
            if (header_bytes == 0 && std::feof(input.get()))
                break;
            if (header_bytes != record_header.size()) {
                std::fprintf(stderr, "truncated PCAP record header\n");
                return 1;
            }

            const uint32_t captured_size =
                read_u32(record_header.data() + 8, little_endian);
            const uint32_t original_size =
                read_u32(record_header.data() + 12, little_endian);
            constexpr uint32_t kMaxCapturedPacket = 16 * 1024 * 1024;
            if (captured_size > snaplen || captured_size > kMaxCapturedPacket ||
                captured_size > original_size) {
                std::fprintf(stderr, "invalid PCAP packet length\n");
                return 1;
            }
            std::vector<uint8_t> frame(captured_size);
            if (captured_size != 0 &&
                std::fread(frame.data(), 1, captured_size, input.get()) !=
                    captured_size) {
                std::fprintf(stderr, "truncated PCAP packet data\n");
                return 1;
            }

            ++packets_seen;
            rtp_packet packet;
            const packet_result parsed = parse_ethernet_ipv4_udp(
                frame.data(), frame.size(), &packet);
            if (parsed == packet_result::malformed) {
                ++malformed_packets;
                continue;
            }
            if (parsed != packet_result::rtp)
                continue;
            if (!have_ssrc) {
                selected_ssrc = packet.ssrc;
                have_ssrc = true;
            }
            if (packet.ssrc != selected_ssrc)
                continue;

            if (have_sequence) {
                const uint16_t sequence_delta =
                    static_cast<uint16_t>(packet.sequence - previous_sequence);
                if (sequence_delta == 0 || sequence_delta >= 0x8000)
                    continue;
                if (sequence_delta > 1)
                    lost_packets += sequence_delta - 1;
                if (packet.timestamp != expected_timestamp)
                    ++timestamp_discontinuities;
            }

            const size_t complex_samples = packet.payload_size / 4;
            const int result = decoder.process_s16be(
                packet.payload, complex_samples);
            if (result != P25_PHASE1_OK) {
                std::fprintf(stderr, "decoder stopped with error %d\n", result);
                return 1;
            }
            previous_sequence = packet.sequence;
            expected_timestamp =
                packet.timestamp + static_cast<uint32_t>(complex_samples);
            have_sequence = true;
            ++packets_selected;
        }

        if (std::ferror(input.get())) {
            std::fprintf(stderr, "error reading PCAP input: %s\n",
                         std::strerror(errno));
            return 1;
        }
        if (packets_selected == 0) {
            std::fprintf(stderr, "no matching RTP IQ packets found\n");
            return 1;
        }
        if (!discard_pcm) {
            if (std::fseek(output_file.get(), 0, SEEK_SET) != 0 ||
                !write_wav_header(output_file.get(),
                                  static_cast<uint32_t>(wav.data_bytes)) ||
                std::fflush(output_file.get()) != 0) {
                std::fprintf(stderr, "cannot finalize WAV output: %s\n",
                             std::strerror(errno));
                return 1;
            }
        }

        p25_phase1_stats stats{};
        if (decoder.get_stats(&samples, &symbols, &stats) != P25_PHASE1_OK) {
            std::fprintf(stderr, "cannot read decoder statistics\n");
            return 1;
        }
        syncs = decoder.sync_count();
        offset_hz = decoder.frequency_offset_hz();
        std::fprintf(stderr,
                     "SSRC=%" PRIu32 " packets=%" PRIu64
                     "/%" PRIu64 " lost=%" PRIu64
                     " timestamp_discontinuities=%" PRIu64
                     " malformed=%" PRIu64 "\n",
                     selected_ssrc, packets_selected, packets_seen, lost_packets,
                     timestamp_discontinuities, malformed_packets);
        std::fprintf(stderr,
                     "IQ samples=%" PRIu64 " symbols=%" PRIu64
                     " syncs=%" PRIu64 " frames=%" PRIu64
                     " voice_codewords=%" PRIu64 " BCH_errors=%u"
                     " offset_hz=%.1f PCM_bytes=%" PRIu64 "\n",
                     samples, symbols, syncs, stats.complete_frames,
                     stats.voice_codewords_decoded, stats.last_bch_errors,
                     offset_hz, wav.data_bytes);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "PCAP decoding failed: %s\n", error.what());
        return 1;
    }
    return 0;
}
