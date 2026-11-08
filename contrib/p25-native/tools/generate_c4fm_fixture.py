#!/usr/bin/env python3
"""Generate and OP25-reference-check the native C4FM IQ fixture."""

import argparse
import math
import pathlib
import sys

import numpy as np


SYNC = 0x5575F5FF77FF
INPUT_RATE = 48000
IF_RATE = 24000
SYMBOL_RATE = 4800


def load_op25(apps_path):
    apps_path = pathlib.Path(apps_path).resolve()
    sys.path.insert(0, str(apps_path / "tx"))
    sys.path.insert(0, str(apps_path))
    from gnuradio import analog, blocks, gr
    import op25_c4fm_mod
    import p25_demodulator_dev

    return analog, blocks, gr, op25_c4fm_mod, p25_demodulator_dev


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--op25-apps", required=True,
                        help="OP25 gr-op25_repeater/apps directory")
    parser.add_argument("--dibits", required=True,
                        help="one-byte-per-dibit input fixture")
    parser.add_argument("--output", required=True,
                        help="output complex-float IQ fixture")
    parser.add_argument("--s16be-output",
                        help="also write interleaved signed 16-bit big-endian IQ")
    args = parser.parse_args()

    analog, blocks, gr, op25_c4fm_mod, p25_demodulator_dev = load_op25(
        args.op25_apps)
    dibits = np.fromfile(args.dibits, dtype=np.uint8)
    if dibits.size == 0 or np.any(dibits > 3):
        raise ValueError("dibit fixture must contain values in [0, 3]")

    source = blocks.vector_source_b(dibits.tolist(), False)
    modulator = op25_c4fm_mod.p25_mod_bf(output_sample_rate=INPUT_RATE)
    fm = analog.frequency_modulator_fc(
        2.0 * math.pi * 12500.0 / INPUT_RATE * 1.5)
    iq_sink = blocks.vector_sink_c()
    transmitter = gr.top_block()
    transmitter.connect(source, modulator, fm, iq_sink)
    transmitter.run()

    iq = np.asarray(iq_sink.data(), dtype=np.complex64)
    if iq.size != dibits.size * INPUT_RATE // SYMBOL_RATE:
        raise RuntimeError("OP25 transmitter returned an unexpected sample count")
    iq.tofile(args.output)

    if args.s16be_output:
        packed = np.empty((iq.size, 2), dtype=">i2")
        packed[:, 0] = np.clip(np.rint(iq.real * 32767.0),
                               -32768, 32767).astype(np.int16)
        packed[:, 1] = np.clip(np.rint(iq.imag * 32767.0),
                               -32768, 32767).astype(np.int16)
        packed.tofile(args.s16be_output)
        packed_float = packed.astype(np.float32) / 32768.0
        reference_iq = packed_float[:, 0] + 1j * packed_float[:, 1]
    else:
        reference_iq = iq

    rx_source = blocks.vector_source_c(reference_iq.tolist(), False)
    receiver = p25_demodulator_dev.p25_demod_cb(
        input_rate=INPUT_RATE, if_rate=IF_RATE, demod_type="fsk4",
        symbol_rate=SYMBOL_RATE)
    dibit_sink = blocks.vector_sink_b()
    reference = gr.top_block()
    reference.connect(rx_source, receiver, dibit_sink)
    reference.run()

    expected_sync = np.array(
        [(SYNC >> (46 - 2 * i)) & 3 for i in range(24)], dtype=np.uint8)
    recovered = np.asarray(dibit_sink.data(), dtype=np.uint8)
    sync_positions = [
        i for i in range(recovered.size - expected_sync.size + 1)
        if np.array_equal(recovered[i:i + expected_sync.size], expected_sync)
    ]
    if len(sync_positions) != 2:
        raise RuntimeError(
            f"OP25 reference did not recover both sync words: {sync_positions}")
    print(f"wrote {iq.size} complex samples; OP25 sync at {sync_positions}")


if __name__ == "__main__":
    main()
