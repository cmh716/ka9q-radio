#!/bin/sh
# Download the public test recordings (the Trunk Recorder Pro regression
# corpus, https://trunkrecorder.pro/corpus), check them by SHA-256 and
# convert them to 48 kHz cf32 in tests/data. Needs curl and python3 with
# numpy and scipy (python3-numpy python3-scipy).
# SPDX-License-Identifier: GPL-3.0-or-later
set -e
cd "$(dirname "$0")"
mkdir -p data
BASE=https://iq.trunkrecorder.pro/blobs
# name  sha256 of source-0.sigmf-data
while read -r name sha; do
  [ -f "data/$name.c64" ] && continue
  echo "fetching $name"
  curl -sSf -o "data/$name.cf32" "$BASE/$sha"
  echo "$sha  data/$name.cf32" | sha256sum -c --quiet
  # The recordings are 37.5 kHz; the decoders take 48 kHz (x 32/25)
  python3 -I -c 'import sys,numpy as np; from scipy.signal import resample_poly
x=np.fromfile(sys.argv[1],dtype=np.complex64); resample_poly(x,32,25).astype(np.complex64).tofile(sys.argv[2])' \
    "data/$name.cf32" "data/$name.c64"
  rm "data/$name.cf32"
done <<LIST
voice-p25 563babe45445f873a0f8a89fff8b317b7485753e3c155cafdbc33fee8d1a2222
voice-p25-phase2 f5ff4c24433936bf08645a27a0ef7b10327b2391a1ead25e80552dbd6e6a56c2
voice-dmr c030eddcc5a0d694c59146ff135b3bc0f9c115371c33d417657fe5655e222083
control-p25 e67b5d317e2c38efec65c7e2aab37d9cfbc54e4c7526356bd006b86ccf4907f1
LIST
echo "test recordings ready in $(pwd)/data"
