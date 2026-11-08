#include "fsk4.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

std::vector<float> make_input(size_t count)
{
    constexpr float levels[] = {-3.0f, -1.0f, 1.0f, 3.0f};
    std::vector<float> samples(count);
    for (size_t i = 0; i < count; ++i) {
        const size_t symbol = i / 5;
        samples[i] = levels[(symbol * 3 + symbol / 7) % 4];
    }
    return samples;
}

std::vector<float> process_partitioned(const std::vector<float>& input,
                                       const std::vector<size_t>& blocks)
{
    p25_fsk4_demod demod(24000.0f, 4800.0f);
    std::vector<float> symbols(input.size());
    std::vector<float> block_output(input.size());
    size_t input_offset = 0;
    size_t output_offset = 0;
    for (size_t block_size : blocks) {
        const size_t count = std::min(block_size, input.size() - input_offset);
        const size_t produced =
            demod.process(input.data() + input_offset, count, block_output.data());
        std::copy(block_output.begin(), block_output.begin() + produced,
                  symbols.begin() + output_offset);
        input_offset += count;
        output_offset += produced;
        if (input_offset == input.size())
            break;
    }
    if (input_offset < input.size()) {
        const size_t count = input.size() - input_offset;
        const size_t produced =
            demod.process(input.data() + input_offset, count, block_output.data());
        std::copy(block_output.begin(), block_output.begin() + produced,
                  symbols.begin() + output_offset);
        output_offset += produced;
    }
    symbols.resize(output_offset);
    return symbols;
}

} // namespace

int main()
{
    const std::vector<float> input = make_input(24000);
    const std::vector<float> contiguous = process_partitioned(input, {input.size()});
    const std::vector<float> irregular =
        process_partitioned(input, {1, 7, 29, 2, 513, 4, 1024, 83});

    require(contiguous.size() > 4700 && contiguous.size() < 4900,
            "symbol recovery rate is close to 4800 symbols/second");
    require(contiguous.size() == irregular.size(),
            "arbitrary block partition preserves output count");
    size_t positive_symbols = 0;
    size_t negative_symbols = 0;
    for (size_t i = 0; i < contiguous.size(); ++i)
    {
        require(std::isfinite(contiguous[i]), "recovered symbols are finite");
        if (contiguous[i] > 0.0f)
            ++positive_symbols;
        else if (contiguous[i] < 0.0f)
            ++negative_symbols;
        require(std::fabs(contiguous[i] - irregular[i]) < 1e-6f,
                "arbitrary block partition preserves symbol values");
    }
    require(positive_symbols > 2000 && negative_symbols > 2000,
            "recovered symbols preserve the four-level signal polarity");

    p25_fsk4_demod demod(24000.0f, 4800.0f);
    std::vector<float> first(input.size());
    std::vector<float> repeated(input.size());
    const size_t first_count = demod.process(input.data(), input.size(), first.data());
    demod.reset();
    const size_t repeated_count =
        demod.process(input.data(), input.size(), repeated.data());
    require(first_count == repeated_count, "reset restores FSK4 timing state");
    for (size_t i = 0; i < first_count; ++i)
        require(std::fabs(first[i] - repeated[i]) < 1e-6f,
                "reset reproduces the same recovered symbols");

    bool invalid_rate_threw = false;
    try {
        p25_fsk4_demod invalid(4800.0f, 9600.0f);
    } catch (const std::invalid_argument&) {
        invalid_rate_threw = true;
    }
    require(invalid_rate_threw, "invalid sample rate is rejected");

    std::puts("native OP25 FSK4 timing loop tests passed");
    return 0;
}
