#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kSsrc = 0x1234abcd;
constexpr size_t kSamplesPerPacket = 512;

void put_be16(std::vector<uint8_t>& bytes, size_t offset, uint16_t value)
{
    bytes[offset] = static_cast<uint8_t>(value >> 8);
    bytes[offset + 1] = static_cast<uint8_t>(value);
}

void put_be32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value)
{
    bytes[offset] = static_cast<uint8_t>(value >> 24);
    bytes[offset + 1] = static_cast<uint8_t>(value >> 16);
    bytes[offset + 2] = static_cast<uint8_t>(value >> 8);
    bytes[offset + 3] = static_cast<uint8_t>(value);
}

void put_le16(std::array<uint8_t, 24>& bytes, size_t offset, uint16_t value)
{
    bytes[offset] = static_cast<uint8_t>(value);
    bytes[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void put_le32(std::array<uint8_t, 24>& bytes, size_t offset, uint32_t value)
{
    bytes[offset] = static_cast<uint8_t>(value);
    bytes[offset + 1] = static_cast<uint8_t>(value >> 8);
    bytes[offset + 2] = static_cast<uint8_t>(value >> 16);
    bytes[offset + 3] = static_cast<uint8_t>(value >> 24);
}

bool write_record(FILE *output, const std::vector<uint8_t>& frame,
                  uint32_t seconds)
{
    std::array<uint8_t, 16> header{};
    header[0] = static_cast<uint8_t>(seconds);
    header[1] = static_cast<uint8_t>(seconds >> 8);
    header[2] = static_cast<uint8_t>(seconds >> 16);
    header[3] = static_cast<uint8_t>(seconds >> 24);
    const uint32_t frame_size = static_cast<uint32_t>(frame.size());
    header[8] = static_cast<uint8_t>(frame_size);
    header[9] = static_cast<uint8_t>(frame_size >> 8);
    header[10] = static_cast<uint8_t>(frame_size >> 16);
    header[11] = static_cast<uint8_t>(frame_size >> 24);
    header[12] = header[8];
    header[13] = header[9];
    header[14] = header[10];
    header[15] = header[11];
    return std::fwrite(header.data(), 1, header.size(), output) ==
               header.size() &&
           std::fwrite(frame.data(), 1, frame.size(), output) == frame.size();
}

std::vector<uint8_t> ethernet_packet(const uint8_t *payload,
                                     size_t payload_size,
                                     uint16_t sequence,
                                     uint32_t timestamp,
                                     uint32_t ssrc,
                                     size_t packet_index)
{
    const bool vlan = packet_index % 2 != 0;
    const bool csrc = packet_index % 3 == 1;
    const bool extension = packet_index % 4 == 2;
    const bool padding = packet_index % 5 == 3;
    const size_t ethernet_size = vlan ? 18 : 14;
    const size_t ip_size = 20;
    const size_t udp_size = 8;
    const size_t rtp_header_size =
        12 + (csrc ? 4 : 0) + (extension ? 8 : 0);
    const size_t padding_size = padding ? 4 : 0;
    const size_t ip_payload_size =
        udp_size + rtp_header_size + payload_size + padding_size;
    const size_t frame_size = ethernet_size + ip_size + ip_payload_size;
    std::vector<uint8_t> frame(frame_size, 0);

    for (size_t i = 0; i < 6; ++i) {
        frame[i] = static_cast<uint8_t>(0x10 + i);
        frame[6 + i] = static_cast<uint8_t>(0x20 + i);
    }
    if (vlan) {
        put_be16(frame, 12, 0x8100);
        put_be16(frame, 14, 1);
        put_be16(frame, 16, 0x0800);
    } else {
        put_be16(frame, 12, 0x0800);
    }

    const size_t ip_offset = ethernet_size;
    frame[ip_offset] = 0x45;
    put_be16(frame, ip_offset + 2,
             static_cast<uint16_t>(ip_size + ip_payload_size));
    frame[ip_offset + 8] = 64;
    frame[ip_offset + 9] = 17;
    frame[ip_offset + 12] = 239;
    frame[ip_offset + 13] = 17;
    frame[ip_offset + 14] = 235;
    frame[ip_offset + 15] = 129;
    frame[ip_offset + 16] = 192;
    frame[ip_offset + 17] = 168;
    frame[ip_offset + 18] = 1;
    frame[ip_offset + 19] = 2;

    const size_t udp_offset = ip_offset + ip_size;
    put_be16(frame, udp_offset, 5004);
    put_be16(frame, udp_offset + 2, 5004);
    put_be16(frame, udp_offset + 4,
             static_cast<uint16_t>(ip_payload_size));

    const size_t rtp_offset = udp_offset + udp_size;
    frame[rtp_offset] = static_cast<uint8_t>(
        0x80 | (padding ? 0x20 : 0) | (extension ? 0x10 : 0) |
        (csrc ? 1 : 0));
    frame[rtp_offset + 1] = 96;
    put_be16(frame, rtp_offset + 2, sequence);
    put_be32(frame, rtp_offset + 4, timestamp);
    put_be32(frame, rtp_offset + 8, ssrc);
    size_t rtp_payload_offset = rtp_offset + 12;
    if (csrc) {
        put_be32(frame, rtp_payload_offset, 0x01020304);
        rtp_payload_offset += 4;
    }
    if (extension) {
        put_be16(frame, rtp_payload_offset, 0xbede);
        put_be16(frame, rtp_payload_offset + 2, 1);
        frame[rtp_payload_offset + 4] = 0x10;
        frame[rtp_payload_offset + 5] = 0xaa;
        frame[rtp_payload_offset + 6] = 0xbb;
        frame[rtp_payload_offset + 7] = 0xcc;
        rtp_payload_offset += 8;
    }
    std::copy(payload, payload + payload_size,
              frame.begin() + static_cast<std::ptrdiff_t>(rtp_payload_offset));
    if (padding)
        frame.back() = static_cast<uint8_t>(padding_size);
    return frame;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::fprintf(stderr, "usage: generate_c4fm_pcap INPUT.s16be OUTPUT.pcap\n");
        return 2;
    }
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) {
        std::fprintf(stderr, "cannot open IQ fixture %s\n", argv[1]);
        return 1;
    }
    const std::vector<uint8_t> iq(
        (std::istreambuf_iterator<char>(input)),
        std::istreambuf_iterator<char>());
    if (iq.empty() || iq.size() % 4 != 0) {
        std::fprintf(stderr, "IQ fixture is empty or has a partial sample\n");
        return 1;
    }

    FILE *output = std::fopen(argv[2], "wb");
    if (output == nullptr) {
        std::perror("cannot open PCAP output");
        return 1;
    }
    std::array<uint8_t, 24> global_header{};
    global_header[0] = 0xd4;
    global_header[1] = 0xc3;
    global_header[2] = 0xb2;
    global_header[3] = 0xa1;
    put_le16(global_header, 4, 2);
    put_le16(global_header, 6, 4);
    put_le32(global_header, 16, 65535);
    put_le32(global_header, 20, 1);
    bool ok = std::fwrite(global_header.data(), 1, global_header.size(),
                          output) == global_header.size();

    const std::array<uint8_t, 4> wrong_stream_payload{};
    uint32_t timestamp = 0xffffe000;
    uint16_t sequence = 65520;
    size_t packet_index = 0;
    if (ok) {
        const auto wrong = ethernet_packet(
            wrong_stream_payload.data(), wrong_stream_payload.size(),
            sequence++, timestamp, 0x0badf00d, packet_index++);
        ok = write_record(output, wrong, 1);
    }

    const size_t complex_samples = iq.size() / 4;
    for (size_t offset = 0; ok && offset < complex_samples;) {
        const size_t samples =
            std::min(kSamplesPerPacket, complex_samples - offset);
        const size_t byte_offset = offset * 4;
        const size_t byte_count = samples * 4;
        const auto frame = ethernet_packet(
            iq.data() + byte_offset, byte_count, sequence++, timestamp,
            kSsrc, packet_index++);
        ok = write_record(output, frame, 1 + packet_index);
        timestamp += static_cast<uint32_t>(samples);
        offset += samples;
    }
    if (std::fclose(output) != 0)
        ok = false;
    if (!ok) {
        std::fprintf(stderr, "failed to write PCAP fixture\n");
        return 1;
    }
    return 0;
}
