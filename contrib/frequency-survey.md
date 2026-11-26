# Checking frequency lists on the air

These tools check downloaded frequency lists against what your receiver actually
hears, before anything goes into your radiod config. Nothing is added to the
config automatically: you decide in the [frequency catalog](frequency-catalog.md).

| Program | What it does | Writes |
|---|---|---|
| `activity` | Watches the whole front end (a Fobos: about 422 to 469 MHz) and logs every transmission. With `-w`, it opens a temporary channel on any listed frequency it hears (the on-demand audit). | `activity.csv`, `activity-coverage.csv`, `activity-summary.csv` |
| `tone-survey` | Listens to the audit's channels and records what each transmission carried: CTCSS tone, DCS code, or digital mode and IDs. | `tone-survey.csv` |
| `frequency-survey.py` | Reports on a list: whether each frequency was heard, and whether its listed tone matches. With `--unknowns`, reports busy frequencies that no list has. | `<list>-survey.csv`, `unknowns.csv`, `unknowns-all.csv` |

See `activity(1)` and `tone-survey(1)` for every option.

## Where the lists go

The commands below expect this layout. Each tool takes a folder and reads
every `.csv` in it, so adding a list just means saving it in the right folder.

```
~/Frequencies/downloads/          conventional lists: RadioReference county downloads,
                                  RepeaterBook exports
~/Frequencies/downloads/sites/    trunked system site downloads (trs_sites_*.csv)
~/Frequencies/surveys/            reports (keep them out of downloads/)
~/Frequencies/catalog/master.csv  your catalog
```

Readable formats:

| Format | Frequency column |
|---|---|
| RadioReference conventional download | `Frequency Output` |
| RadioReference trunked site download | `Frequencies`, then every column after it, one frequency each (`452.0125c` is a control channel) |
| RepeaterBook export | `Output Freq` |
| The catalog (`master.csv`, `inbox.csv`) | `RX MHz` |

Keep RadioReference talkgroup downloads (`trs_tg_*.csv`) out of these folders.
They have no frequencies, and activity refuses a list without a frequency column.

## Running

activity and tone-survey run all the time. Start them once:

```sh
activity -v -c ~/Frequencies/uhf.d -l ~/activity.csv -s ~/activity-summary.csv \
  -w ~/Frequencies/catalog/master.csv -w ~/Frequencies/downloads -w ~/Frequencies/downloads/sites \
  -a tone-survey.local -r ~/tone-survey.csv fobos-uhf.local

tone-survey -o ~/tone-survey.csv tone-survey.local
```

- `-w`: what to audit (a list, or a folder of them).
- `-a`: where the audit's channels send audio (tone-survey listens there).
- `-r`: tone-survey's results. A frequency with 10 identified transmissions
  (`-N`) is done, and its channel goes to frequencies that still need one.
- `-A`: audit channels at once (16 by default). Each costs radiod some CPU.

After adding or removing a list, restart activity: it reads the lists only at
start. tone-survey needn't restart. Both keep their results across restarts.

## Reports

Run these whenever you like. A day or two of data is enough for busy
frequencies; give rarely used ones a week.

A list, frequency by frequency:

```sh
cd ~/ka9q-radio/contrib
./frequency-survey.py -a ~/activity.csv --tones ~/tone-survey.csv \
  -o ~/Frequencies/surveys/NiagaraCounty-survey.csv ~/Frequencies/downloads/NiagaraCounty.csv
```

Without `-o`, the report goes beside the list (`NiagaraCounty-survey.csv`), in
`downloads/`, where activity would read it as a list. Give `-o`.

Each frequency gets:

- **Survey**: `RF activity near frequency`, `no RF activity in N days monitored`,
  or `not covered by activity` (outside the receiver's range). Rare-use
  services (fire, EMS, tactical) are flagged so silence isn't held against them.
- **Transmissions**, **Airtime s**, **First/Last heard**, **Busiest hours**,
  **Pattern** (regular intervals: telemetry).
- **Heard**: everything tone-survey heard there, with counts.
- **Tone check**: `confirmed`, `differs: heard X (listed Y)`,
  `heard, less often than X` (a shared frequency), `no tone heard`,
  `digital: DMR TG 1234` or `DMR (no voice)` (encrypted or data), or
  `not surveyed yet`.

Busy frequencies that no list has:

```sh
./frequency-survey.py -a ~/activity.csv --unknowns ~/Frequencies/surveys/unknowns.csv \
  ~/Frequencies/catalog/master.csv ~/Frequencies/downloads ~/Frequencies/downloads/sites
```

`unknowns.csv` holds the 40 most worth looking up, ranked by airtime
(`--shortlist` sets how many). `unknowns-all.csv` holds every one, each with a
reason when it was left out:

- heard too seldom (under 30 a day monitored)
- the receiver's own centre frequency (its DC spur)
- splatter from a strong configured channel
- 433.92 MHz unlicensed devices
- the input of a repeater whose output is listed
- data or telemetry (short bursts, regular intervals)
- one event with a listed frequency or a stronger unknown (frequencies that
  always start in the same second: one transmitter's spurious emissions, or
  products of strong signals in the receiver)
- the same signal as a busier neighbour, read off its centre

To identify an unknown:

- Look for a trunked system or county list that has it, add that list, and
  rerun: the fastest way, as one site list can account for several unknowns.
- Search RadioReference's FCC database by frequency, within ±12.5 kHz, and
  5 MHz lower for an input (455–460 and 465–470 MHz).
- Use the FCC's ULS for 454.025–454.650 MHz (mostly Part 22 channels).
- Use ISED's Spectrum Management System for Ontario stations.

## Listening to one

A temporary channel, heard in monitor among the group's channels:

```sh
tune -r fobos-uhf.local -s 1454050 -m mixed -f 454m05 -D ham.local -t 90000
```

- `-s`: any SSRC not in use.
- `-m mixed`: FM with no tone squelch, plus every digital mode.
- `-D`: a group your monitor watches.
- `-t`: lifetime in radiod blocks (about 20 ms each). 90000 is about half an hour.

To remove it early:

```sh
tune -r fobos-uhf.local -s 1454050 -t 1
```

## Correcting the catalog

For `master.csv`, the survey can propose corrections from what was heard:

```sh
./frequency-survey.py -a ~/activity.csv --tones ~/tone-survey.csv \
  -o ~/Frequencies/surveys/master-survey.csv \
  --corrections ~/Frequencies/surveys/master-corrections.csv --skip 'GMRS|FRS' \
  ~/Frequencies/catalog/master.csv
```

`--skip` leaves rows alone whose Service, Group or Name match (here, all
FRS and GMRS). Each row of the sheet has an **Action**:

- **change**: one field of one channel, `From` what Master holds `To` the
  correction, with the evidence. A tone heard on 10 or more transmissions,
  at least 60% of those heard, replaces the listed squelch. A digital mode
  heard the same way replaces FM or FMN, with the squelch set to Digital
  (and the NAC, for P25). A CTCSS notch follows the tone.
- **remove**: silent for 2 days or more, and not a rare-use service.
- **listen**: the evidence conflicts (for example, listed with a DCS code
  but heard with no tone, which can mean several users share the channel).
  Listen before deciding. Nothing is applied.
- **wait**: too little heard yet. Nothing is applied.

Review the sheet. Delete any row you don't want, or set its Action to
something else. Then close master.csv in your spreadsheet and apply:

```sh
./frequency-catalog.py --workspace ~/Frequencies/catalog correct ~/Frequencies/surveys/master-corrections.csv
```

It applies only change and remove rows. It refuses the whole sheet, writing
nothing, if any `From` no longer matches Master (a stale sheet) or a
corrected channel fails validation. It backs Master up first, to
`backups/`. Removed channels go to `ignored.csv`, so an import won't propose
them again.

## Then

Add what you've confirmed to the catalog (see
[frequency-catalog.md](frequency-catalog.md)) and generate the radiod config
from it.
