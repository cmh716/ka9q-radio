# Curated frequency catalog

Use a spreadsheet editor for `master.csv` and `inbox.csv`. Downloads only propose
entries; only Master is exported. This starter uses Python's standard library.
It does not program radios or change running receivers.

## Files

- `master.csv`: the channels you selected. Edit names, groups, profiles, and notes here.
- `inbox.csv`: downloaded candidates and proposed updates. Set **Decision** to
  `Keep`, `Ignore`, or `Undecided` (the default). You may edit the candidate before Keep.
- `ignored.csv`: remembered exclusions. Remove a row here to reconsider it on the next import.
- `source-rules.csv`: filename-to-Service/Group classification, separate from downloads.
- `import-state.json`: source snapshots used to compare imports. Keep this alongside the CSVs.
- `device-profiles.csv`: per-radio hardware/default settings (template; not yet an exporter input).
- `memory-assignments.csv`: channel-to-device bank/memory assignments (template).
- `backups/`: recovery copies made before applying decisions or migrating schemas.

Close the CSV in your spreadsheet before running import/apply; reopen it afterward.
Save CSV as UTF-8, keep the header names and IDs unchanged, and use decimal MHz.
Do not run multiple catalog commands simultaneously. Back up the whole catalog
folder (or track it in Git); don't back up just the master spreadsheet.

## First review

Your initialized workspace is `~/Frequencies/catalog`. The master starts empty.

1. Open `~/Frequencies/catalog/inbox.csv` in LibreOffice Calc, Excel, or your preferred editor.
2. Filter by Service, Name, frequency, or Source. Mark a few channels **Keep**.
3. Mark unwanted listings **Ignore**; leave everything else **Undecided**.
4. Review the proposed Name, Group, and Profiles, then save and close the CSV.
5. Apply the decisions:

```sh
python3 ~/ka9q-radio/contrib/frequency-catalog.py \
  --workspace ~/Frequencies/catalog apply
```

Keep adds a new channel, or accepts a proposed update. Ignore remembers a new
listing as unwanted. Ignoring an update rejects the change and keeps the existing
master channel; it does not remove that channel. Undecided entries stay in the inbox.

Repeat imports after downloading fresh source files:

```sh
python3 ~/ka9q-radio/contrib/frequency-catalog.py \
  --workspace ~/Frequencies/catalog import ~/Frequencies/downloads/*.csv
```

Imports retain pending edits when the source is unchanged. Changed proposals are
reset to Undecided. Upstream changes never directly rewrite Master. When staging
updates, a personally changed field is retained instead of the upstream value;
`Source Changes` still shows the upstream change. Missing source entries never
remove master channels. Imports do not edit the downloaded files.

## Columns and classification

- **Service** describes the listing: Ham, FRS, GMRS, Fire Dispatch, etc.
- **Group** is your listening destination: `ham`, `gmrs-frs`, `murs`, `cb`, etc.
  Use lowercase letters/digits and hyphens, without `.local`. The radiod bridge
  maps this to the Tag column so rr2radiod generates the matching multicast stream.
- **Profiles** lists export memberships, separated by semicolons, e.g. `uhf;scanner`.
  `*` includes every profile. Blank means no profile. Initial hf/vhf/uhf suggestions
  use RX frequency bands; rr2radiod still checks the real receiver coverage.
- **Receive Only** defaults to `yes`. It is retained for future device exporters;
  this starter's only exporter is receive-only radiod, so it never programs TX.
- **RX/TX MHz** are the authoritative frequencies as viewed from your radio. A blank
  TX frequency means unknown/not provided, not an instruction to transmit simplex.
- **Offset MHz** is calculated as TX minus RX. Do not edit it independently. It is
  recomputed when rows are loaded/written; it is a CSV value, not a live spreadsheet formula.
- **Operation** is Unknown, Simplex, Repeater, Split, or Receive-only. Imports start
  Unknown; they do not infer an intended transmit setup from an input frequency.
- **RX Mode / TX Mode** are separate. RX mode comes from the source; TX mode remains
  blank until explicitly chosen. Unsupported modes require correction before Keep.
- **Bandwidth kHz** is an optional positive numeric value; it is not an Icom filter number.
- **RX Squelch Type / Value** control receiving: Carrier, CTCSS, DCS, Digital, or Unknown.
  **TX Access Type / Value** describe repeater access: None, CTCSS, DCS, Digital, or Unknown.
  CTCSS values are Hz; DCS values are three-digit octal strings, e.g. `023`.
  Import DCS value columns as spreadsheet text to preserve leading zeros.
- **RX/TX DCS Polarity** independently specify Normal or Inverted.
- **DMR Color Code / Timeslot / Talkgroup**, **P25 NAC**, **NXDN RAN**, and
  **D-STAR UR / RPT1 / RPT2** hold mode-specific settings. NAC is hexadecimal;
  DMR timeslot is 1 or 2. Leave unknown values blank.
- **Source Mode / Source RX Tone / Source TX Tone** retain the original imported
  values for reference. Operational settings above are authoritative; these source
  columns do not override personal choices.
- **ID** is an internal key; do not edit it.

FRS.csv and GMRS.csv are separate services with a shared `gmrs-frs` group by
starter rule. MURS.csv goes to `murs`, CB.csv to `cb`, and amateur downloads to
`ham`. Other RadioReference entries initially use their Tag as service/group.
Personal group choices can be changed in Master at any time.

## Export and receiver integration

Export only the selected channels for a profile:

```sh
python3 ~/ka9q-radio/contrib/frequency-catalog.py \
  --workspace ~/Frequencies/catalog export --profile uhf \
  -o ~/Frequencies/catalog/exports/uhf.csv
```

The export is an intermediate RadioReference-shaped CSV, not an Icom or scanner
programming file. The complete master retains TX and other personal information;
this bridge is specifically for receiver configurations. Overlapping same-group
FM/FM Narrow entries are combined using the widest listed receiver mode. Entries
at the same frequency assigned to different groups are rejected for review rather
than silently letting input order choose the multicast group.

Once Master contains what you want, the intermediate export replaces the
`downloads/*.csv` argument in your existing rr2radiod command. For example:

```sh
# This REPLACES the generated conventional channel file. Review Master first.
python3 ~/ka9q-radio/contrib/rr2radiod.py \
  --base ~/Frequencies/uhf.d \
  -o ~/Frequencies/uhf.d/50-conventional.conf \
  -n ~/Frequencies/names/uhf-names.csv \
  ~/Frequencies/catalog/exports/uhf.csv
```

Do not feed both the selected export and the original downloads into rr2radiod:
that would reintroduce unwanted channels. Move personal corrections from the old
`freq-overrides.csv` into Master as you select entries; the starter does not apply
that old file automatically. The initial setup does not replace any receiver files.

## Current limits

This is a starter, not a complete universal radio database. Stable IDs include
source filename, label (or description for nameless entries), RX frequency, mode,
tone, TX frequency, and agency/location. A change to one of those identifiers, or
renaming a downloaded file, appears as a **new candidate**, leaving the old Master
entry intact; reconcile it manually instead of automatically replacing a channel.
Description updates to a named entry are proposed as updates. Ignoring an entry is
sticky for that ID. Keep filenames consistent between download refreshes.

Cross-source duplicates remain separate in Master with provenance; the receiver
export combines equivalent entries. Actual device export formats,
additional digital import mappings, geographic filtering, automatic missing-entry reports, and an
interactive import UI can be added later. No transmit configuration is inferred.

Run regression checks with:

```sh
python3 contrib/frequency-catalog-test.py
```

## Transmitter schema (version 2)

Existing catalogs can be backed up and upgraded explicitly:

```sh
python3 ~/ka9q-radio/contrib/frequency-catalog.py \
  --workspace ~/Frequencies/catalog migrate
```

The migration preserves IDs, review decisions, raw source values, and custom
columns. Legacy Mode maps to RX Mode. Known PL/DPL/NAC/DMR slot spellings are
parsed; ambiguous access strings remain Unknown for review. No TX frequency,
TX mode, or repeater operation is inferred from a blank value. The migration also
normalizes the import snapshots so the next refresh does not reset existing reviews.
Running it again is safe and creates another recovery snapshot. It can also refresh
saved Offset MHz values after you edit RX/TX frequencies directly in a spreadsheet.

To mark a channel for transmit use (`Receive Only = no`), explicitly enter its TX
frequency, TX mode, and operation. Simplex requires equal RX/TX frequencies;
Repeater/Split require distinct frequencies. This flag records intent; it does not
assert a radio's transmit capability or create a transmit programming file.

Values remain separate from activation. For example, `TX Access Type = None` with
`TX Access Value = 67.0` retains a dormant tone value without activating it.
Likewise, Carrier receive squelch does not enable a stored receive tone.
The radiod bridge uses the structured receive fields, and reports an error instead
of silently dropping unknown squelch or inverted receive DCS it cannot represent.
Existing rr2radiod limitations still apply: it currently uses DMR slot but not color
code/talkgroup filters, and has no NXDN RAN or D-STAR routing configuration export.
Those fields remain available in Master for future device exporters.

Device profiles keep model/filter/tuning-step/preamp/attenuator/antenna/IP+/power
settings outside the channel master. Memory assignments reference a **Channel ID**
and **Device**, then specify bank, memory number, short name, favorite, and scan
status. The two files start with headers only; do not guess Icom numeric settings.
Neither file is consumed by the current receiver exporter. A verified non-simplex
SDR-Control example is still needed before implementing that application's exporter.

For daily review you can hide transmitter, digital, and Source columns in your
spreadsheet and focus on Decision, Name, RX MHz, RX Mode, Service, Group, and Profiles.
