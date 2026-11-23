#!/usr/bin/env python3
"""Curated frequency CSV master, selective import inbox, and radiod bridge.

See contrib/frequency-catalog.md. Standard library only; never changes downloads.
"""
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import tempfile
from decimal import Decimal, InvalidOperation

# Frequencies are authoritative; Offset MHz is a derived display column.
FIELDS = ['ID', 'Name', 'RX MHz', 'TX MHz', 'Offset MHz', 'Operation',
          'RX Mode', 'TX Mode', 'Bandwidth kHz', 'CTCSS Notch Hz',
          'RX Squelch Type', 'RX Squelch Value', 'TX Access Type', 'TX Access Value',
          'RX DCS Polarity', 'TX DCS Polarity',
          'DMR Color Code', 'DMR Timeslot', 'DMR Talkgroup', 'P25 NAC', 'NXDN RAN',
          'D-STAR UR', 'D-STAR RPT1', 'D-STAR RPT2',
          'Service', 'Group', 'Profiles', 'Receive Only', 'Description', 'Source', 'Notes',
          'Source Mode', 'Source RX Tone', 'Source TX Tone']
INBOX = ['Decision', 'Change', 'Source Changes'] + FIELDS
SOURCE_FIELDS = ['Name', 'RX MHz', 'TX MHz', 'RX Mode', 'TX Mode', 'Bandwidth kHz',
                 'RX Squelch Type', 'RX Squelch Value', 'TX Access Type', 'TX Access Value',
                 'RX DCS Polarity', 'TX DCS Polarity', 'DMR Color Code', 'DMR Timeslot',
                 'DMR Talkgroup', 'P25 NAC', 'NXDN RAN', 'Description',
                 'Source Mode', 'Source RX Tone', 'Source TX Tone']
DEVICE_FIELDS = ['Device', 'Model', 'Default Filter', 'Tuning Step kHz', 'Preamp',
                 'Attenuator dB', 'Antenna', 'IP Plus', 'Power W', 'Notes']
MEMORY_FIELDS = ['Channel ID', 'Device', 'Bank', 'Memory Number', 'Short Name',
                 'Favorite', 'Scan', 'Notes']
SCHEMA_VERSION = 2
MODES = {'USB', 'AM', 'FM', 'FMN', 'DMR', 'P25', 'NXDN', 'NXDN48', 'YSF', 'D-STAR', 'LTR'}
RR_FIELDS = ['Frequency Output', 'Frequency Input', 'Alpha Tag', 'Description',
             'Mode', 'PL Output Tone', 'PL Input Tone', 'Agency/Category', 'Tag', 'CTCSS Notch Hz']


def read_csv(path, required=()):
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        if not set(required).issubset(reader.fieldnames or []):
            raise ValueError(f'{path}: missing columns: {", ".join(required)}')
        return [{k: (v or '').strip() for k, v in r.items() if k is not None}
                for r in reader if any(r.values())]


def atomic_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='') as f:
            f.write(text)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def write_csv(path, fields, rows):
    import io
    out = io.StringIO(newline='')
    rows = list(rows)
    if path.name in ('master.csv', 'inbox.csv', 'ignored.csv'):
        fields = list(dict.fromkeys(list(fields) + [k for row in rows for k in row]))
    w = csv.DictWriter(out, fieldnames=fields, extrasaction='ignore')
    w.writeheader()
    w.writerows(rows)
    atomic_text(path, out.getvalue())


def mhz(value, optional=False):
    if optional and not value:
        return ''
    try:
        n = Decimal(value)
        if not n.is_finite() or n <= 0:
            raise InvalidOperation
        return f'{n:.6f}'
    except (InvalidOperation, ValueError):
        raise ValueError(f'invalid frequency in MHz: {value!r}')


def indexed(rows, path):
    result = {}
    for row in rows:
        key = row.get('ID', '')
        if not key or key in result:
            raise ValueError(f'{path}: blank or duplicate ID: {key!r}')
        result[key] = row
    return result


def parse_access(text, receive=False):
    """Translate known source spellings; retain ambiguous values for review."""
    text = text.strip()
    if not text:
        return ('Carrier' if receive else 'Unknown'), '', ''
    if text.upper() in ('OFF', 'NONE', 'CSQ'):
        return ('Carrier' if receive else 'None'), '', ''
    m = re.fullmatch(r'(?:D(\d{3})|(\d{3}) DPL)(?:\s*([NI]))?', text, re.I)
    if m:
        return 'DCS', m[1] or m[2], 'Inverted' if (m[3] or '').upper() == 'I' else 'Normal'
    m = re.fullmatch(r'(\d+(?:\.\d+)?)(?: PL)?', text, re.I)
    if m:
        return 'CTCSS', m[1], ''
    return 'Unknown', text, ''


def upgrade(row):
    """Normalize old rows in memory. Never replace an existing v2 setting."""
    row = dict(row)
    legacy = any(k in row for k in ('Mode', 'RX Tone', 'TX Tone'))
    if legacy:
        mode = row.pop('Mode', '')
        rx_tone, tx_tone = row.pop('RX Tone', ''), row.pop('TX Tone', '')
        row['RX Mode'] = row.get('RX Mode') or mode
        # A listed input frequency is not evidence of the desired transmit mode.
        for field, value in [('Source Mode', mode), ('Source RX Tone', rx_tone), ('Source TX Tone', tx_tone)]:
            row[field] = row.get(field) or value
        for raw, receive, type_field, value_field, polarity_field in (
                (rx_tone, True, 'RX Squelch Type', 'RX Squelch Value', 'RX DCS Polarity'),
                (tx_tone, False, 'TX Access Type', 'TX Access Value', 'TX DCS Polarity')):
            kind, value, polarity = parse_access(raw, receive)
            row[type_field] = row.get(type_field) or kind
            row[value_field] = row.get(value_field) or value
            row[polarity_field] = row.get(polarity_field) or polarity
        # Preserve structured digital access when the listing explicitly gives it.
        if row['RX Mode'] == 'P25':
            m = re.fullmatch(r'([0-9A-Fa-f]{1,3}) NAC', rx_tone)
            if m:
                row['P25 NAC'] = row.get('P25 NAC') or m[1].upper()
                row['RX Squelch Type'], row['RX Squelch Value'] = 'Digital', ''
        if row['RX Mode'] == 'DMR':
            cc = re.search(r'\bCC\s*(\d+)\b', rx_tone, re.I)
            slot = re.search(r'\bSL\s*(\d+)\b', rx_tone, re.I)
            if cc or slot:
                row['RX Squelch Type'], row['RX Squelch Value'] = 'Digital', ''
                if cc: row['DMR Color Code'] = row.get('DMR Color Code') or cc[1]
                if slot: row['DMR Timeslot'] = row.get('DMR Timeslot') or slot[1]
    for field in FIELDS:
        row.setdefault(field, '')
    row['Operation'] = row['Operation'] or 'Unknown'
    row['Receive Only'] = row['Receive Only'] or 'yes'
    if row['RX MHz'] and row['TX MHz']:
        mhz(row['RX MHz'])
        mhz(row['TX MHz'])
        row['Offset MHz'] = f"{Decimal(row['TX MHz']) - Decimal(row['RX MHz']):+.6f}"
    else:
        row['Offset MHz'] = ''
    return row


def access_text(kind, value):
    kind = kind.lower()
    if kind == 'ctcss': return value + ' PL'
    if kind == 'dcs': return value + ' DPL'
    return ''


def receiver_tone(row):
    mode = row['RX Mode'].upper()
    if mode.rstrip('E') == 'P25' and row['P25 NAC']:
        return row['P25 NAC'].removeprefix('0x').removeprefix('0X') + ' NAC'
    if mode == 'DMR':
        return ' '.join(v for v in (('CC ' + row['DMR Color Code']) if row['DMR Color Code'] else '',
                                    ('SL ' + row['DMR Timeslot']) if row['DMR Timeslot'] else '') if v)
    kind, value = row['RX Squelch Type'].lower(), row['RX Squelch Value']
    if kind == 'ctcss': return value + ' PL'
    if kind == 'dcs':
        if row['RX DCS Polarity'].lower() == 'inverted':
            raise ValueError(f'{row["ID"]}: rr2radiod cannot represent inverted receive DCS')
        return value + ' DPL'
    if kind in ('', 'carrier', 'digital'): return ''
    raise ValueError(f'{row["ID"]}: resolve unknown RX squelch before receiver export')


def snapshot(workspace):
    from datetime import datetime, timezone
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    destination = workspace / 'backups' / stamp
    for name in ('master.csv', 'inbox.csv', 'ignored.csv', 'import-state.json',
                 'device-profiles.csv', 'memory-assignments.csv'):
        if (workspace / name).exists():
            atomic_text(destination / name, (workspace / name).read_text())
    return destination


def migrate(args):
    w = args.workspace
    master, ignored, inbox, state = load(w)
    backup = snapshot(w)
    write_csv(w / 'master.csv', FIELDS, master.values())
    write_csv(w / 'ignored.csv', FIELDS, ignored.values())
    write_csv(w / 'inbox.csv', INBOX, inbox.values())
    save_state(w, state)
    for name, fields in [('device-profiles.csv', DEVICE_FIELDS), ('memory-assignments.csv', MEMORY_FIELDS)]:
        if not (w / name).exists():
            write_csv(w / name, fields, [])
    print(f'Migrated to schema {SCHEMA_VERSION}: {len(master)} master, {len(inbox)} inbox, {len(ignored)} ignored. Backup: {backup}')


def load(workspace):
    master = indexed([upgrade(r) for r in read_csv(workspace / 'master.csv', ['ID'])], 'master.csv')
    ignored = indexed([upgrade(r) for r in read_csv(workspace / 'ignored.csv', ['ID'])], 'ignored.csv')
    inbox = indexed([upgrade(r) for r in read_csv(workspace / 'inbox.csv', ['ID', 'Decision'])], 'inbox.csv')
    path = workspace / 'import-state.json'
    state = json.loads(path.read_text()) if path.exists() else {'approved': {}, 'latest': {}}
    for section in ('approved', 'latest'):
        state[section] = {key: upgrade(row) for key, row in state.get(section, {}).items()}
    state['schema_version'] = SCHEMA_VERSION
    return master, ignored, inbox, state


def save_state(workspace, state):
    atomic_text(workspace / 'import-state.json', json.dumps(state, indent=2, sort_keys=True) + '\n')


def source_rows(paths, rules):
    spec = importlib.util.spec_from_file_location('rr2radiod', Path(__file__).with_name('rr2radiod.py'))
    rr = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(rr)
    finally:
        sys.dont_write_bytecode = previous
    found = {}
    for path in paths:
        with path.open(newline='', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            book = 'Output Freq' in (reader.fieldnames or [])
            if not book and 'Frequency Output' not in (reader.fieldnames or []):
                raise ValueError(f'{path}: not a RadioReference or RepeaterBook CSV')
            for line, original in enumerate(reader, 2):
                rows = rr.repeaterbook(original) if book else [original]
                for raw in rows:
                    try:
                        rx = mhz(raw.get('Frequency Output', ''))
                        tx_raw = original.get('Input Freq' if book else 'Frequency Input', '')
                        tx = mhz(tx_raw, True) if tx_raw and Decimal(tx_raw) != 0 else ''
                    except (ValueError, InvalidOperation) as e:
                        raise ValueError(f'{path}:{line}: {e}') from e
                    name = raw.get('Alpha Tag', '').strip()
                    mode = raw.get('Mode', '').upper().strip().replace('DSTAR', 'D-STAR')
                    service = rules.get(path.name, {}).get('Service') or ('Ham' if book else raw.get('Tag') or 'Other')
                    group = rules.get(path.name, {}).get('Group') or rr.slug(service)
                    # Identity includes source, label, frequency and mode. Same-frequency
                    # listings from different services/sources remain separate records.
                    identity = [path.name, name or raw.get('Description', ''), rx, mode,
                                raw.get('PL Output Tone', '').strip(), tx,
                                raw.get('Agency/Category', ''), original.get('Location', '')]
                    key = hashlib.sha256(json.dumps(identity).encode()).hexdigest()[:16]
                    row = dict.fromkeys(FIELDS, '')
                    row.update({'ID': key, 'Name': name, 'RX MHz': rx, 'TX MHz': tx,
                                'Mode': mode, 'RX Tone': raw.get('PL Output Tone', '').strip(),
                                'TX Tone': original.get('Uplink Tone' if book else 'PL Input Tone', '').strip(),
                                'Service': service, 'Group': group,
                                'Receive Only': 'yes', 'Description': raw.get('Description', '').strip(),
                                'Source': path.name})
                    freq = Decimal(rx)
                    row['Profiles'] = 'hf' if freq < 30 else 'vhf' if freq < 300 else 'uhf'
                    if key in found and found[key] != upgrade(row):
                        raise ValueError(f'{path}:{line}: conflicting duplicate listing; resolve the source records first')
                    found[key] = upgrade(row)
    return found


def import_files(args):
    w = args.workspace
    master, ignored, inbox, state = load(w)
    rules_path = w / 'source-rules.csv'
    rules = {r['File']: r for r in read_csv(rules_path, ['File', 'Service', 'Group'])}
    found = source_rows(args.files, rules)
    new_inbox = dict(inbox)  # keep pending entries even if their source disappears
    for key, source in found.items():
        if key in ignored:
            continue
        old = state['approved'].get(key)
        if key in master and old == source:
            continue
        if key in inbox and state['latest'].get(key) == source:
            continue  # don't overwrite spreadsheet edits or pending decisions
        if key in master:
            if old is None:
                continue  # manually maintained master rows are authoritative
            changed = [f for f in SOURCE_FIELDS if old.get(f, '') != source.get(f, '')]
            if not changed:
                continue
            proposal = dict(master[key])
            for field in changed:
                if master[key].get(field, '') == old.get(field, ''):
                    proposal[field] = source[field]
            change = 'Update'
            summary = '; '.join(f'{f}: {old.get(f, "")} -> {source[f]}' for f in changed)
        else:
            proposal, change, summary = dict(source), 'New', ''
        # A changed proposal must be reviewed again, even if Keep was previously typed.
        new_inbox[key] = dict(proposal, Decision='Undecided', Change=change, **{'Source Changes': summary})
        state['latest'][key] = source
    w.mkdir(parents=True, exist_ok=True)
    if not (w / 'master.csv').exists():
        write_csv(w / 'master.csv', FIELDS, [])
    if not (w / 'ignored.csv').exists():
        write_csv(w / 'ignored.csv', FIELDS, [])
    write_csv(w / 'inbox.csv', INBOX, sorted(new_inbox.values(), key=lambda r: (Decimal(r['RX MHz']), r['Name'], r['ID'])))
    save_state(w, state)
    print(f'{len(master)} master entries; {len(new_inbox)} inbox entries; {len(ignored)} ignored. No channels selected automatically.')


def notch_hz(value):
    try:
        n = Decimal(value or '0')
        if not n.is_finite() or (n != 0 and not Decimal(60) <= n <= Decimal(260)):
            raise InvalidOperation
        return format(n.normalize(), 'f') if value else ''
    except (InvalidOperation, ValueError):
        raise ValueError('CTCSS Notch Hz must be blank, 0, or a frequency from 60 to 260 Hz')


def validate(row):
    if not row.get('Name'):
        raise ValueError(f'{row["ID"]}: Name is required')
    mhz(row.get('RX MHz', ''))
    mhz(row.get('TX MHz', ''), True)
    row = upgrade(row)
    mode = row.get('RX Mode', '').upper()
    notch = notch_hz(row['CTCSS Notch Hz'])
    if notch not in ('', '0') and mode not in ('FM', 'FMN'):
        raise ValueError(f'{row["ID"]}: CTCSS Notch Hz requires FM or FMN')
    if mode not in MODES and not (mode.endswith('E') and mode[:-1] in MODES):
        raise ValueError(f'{row["ID"]}: unsupported mode {mode!r}; correct it before Keep')
    if not re.fullmatch(r'[a-z0-9]+(?:-[a-z0-9]+)*', row.get('Group', '')):
        raise ValueError(f'{row["ID"]}: Group must be a name such as ham or gmrs-frs (without .local)')
    if row.get('Receive Only', '').lower() not in ('yes', 'no'):
        raise ValueError(f'{row["ID"]}: Receive Only must be yes or no')

    operation = row['Operation'].lower()
    if operation not in ('unknown', 'simplex', 'repeater', 'split', 'receive-only'):
        raise ValueError(f'{row["ID"]}: Operation must be Unknown, Simplex, Repeater, Split, or Receive-only')
    tx_mode = row['TX Mode'].upper()
    if tx_mode and tx_mode not in MODES:
        raise ValueError(f'{row["ID"]}: unsupported TX Mode {tx_mode!r}')
    if row['Receive Only'].lower() == 'no':
        if not row['TX MHz'] or not tx_mode or operation in ('unknown', 'receive-only'):
            raise ValueError(f'{row["ID"]}: transmit use requires explicit TX MHz, TX Mode, and Operation')
    if operation == 'simplex' and (not row['TX MHz'] or Decimal(row['TX MHz']) != Decimal(row['RX MHz'])):
        raise ValueError(f'{row["ID"]}: Simplex requires equal RX and TX frequencies')
    if operation in ('repeater', 'split') and (not row['TX MHz'] or Decimal(row['TX MHz']) == Decimal(row['RX MHz'])):
        raise ValueError(f'{row["ID"]}: {row["Operation"]} requires distinct RX and TX frequencies')
    for prefix, type_field, value_field in [('RX', 'RX Squelch Type', 'RX Squelch Value'),
                                             ('TX', 'TX Access Type', 'TX Access Value')]:
        kind, value = row[type_field].lower(), row[value_field]
        allowed = ('', 'unknown', 'carrier', 'ctcss', 'dcs', 'digital') if prefix == 'RX' else ('', 'unknown', 'none', 'ctcss', 'dcs', 'digital')
        if kind not in allowed:
            raise ValueError(f'{row["ID"]}: invalid {type_field}: {row[type_field]!r}')
        if kind == 'ctcss':
            try:
                tone = Decimal(value)
                if not tone.is_finite() or tone <= 0: raise InvalidOperation
            except InvalidOperation:
                raise ValueError(f'{row["ID"]}: {value_field} needs a positive CTCSS frequency in Hz')
        if kind == 'dcs' and not re.fullmatch(r'[0-7]{3}', value):
            raise ValueError(f'{row["ID"]}: {value_field} needs a three-digit octal DCS code, e.g. 023')
        if row[prefix + ' DCS Polarity'].lower() not in ('', 'normal', 'inverted'):
            raise ValueError(f'{row["ID"]}: {prefix} DCS Polarity must be Normal or Inverted')
    if row['Bandwidth kHz']:
        try:
            bandwidth = Decimal(row['Bandwidth kHz'])
            if not bandwidth.is_finite() or bandwidth <= 0: raise InvalidOperation
        except InvalidOperation:
            raise ValueError(f'{row["ID"]}: Bandwidth kHz must be positive')
    for field, low, high in [('DMR Color Code', 0, 15), ('DMR Timeslot', 1, 2),
                              ('DMR Talkgroup', 0, 16777215), ('NXDN RAN', 0, 63)]:
        if row[field] and (not re.fullmatch(r'\d+', row[field]) or not low <= int(row[field]) <= high):
            raise ValueError(f'{row["ID"]}: {field} must be {low}..{high}')
    if row['P25 NAC'] and not re.fullmatch(r'(?:0[xX])?[0-9a-fA-F]{1,3}', row['P25 NAC']):
        raise ValueError(f'{row["ID"]}: P25 NAC must be hexadecimal, e.g. 293 or 0x293')


def apply(args):
    w = args.workspace
    master, ignored, inbox, state = load(w)
    pending, kept, rejected = {}, 0, 0
    for key, row in inbox.items():
        decision = row.get('Decision', '').lower()
        clean = upgrade({k: v for k, v in row.items() if k not in ('Decision', 'Change', 'Source Changes')})
        if decision == 'keep':
            validate(clean)
            master[key] = clean
            ignored.pop(key, None)
            state['approved'][key] = state['latest'].get(key, clean)
            kept += 1
        elif decision == 'ignore':
            if key in master:
                # Reject the proposed change, not the channel already selected.
                state['approved'][key] = state['latest'].get(key, clean)
            else:
                ignored[key] = clean
            rejected += 1
        elif decision in ('', 'undecided'):
            pending[key] = row
        else:
            raise ValueError(f'{key}: Decision must be Keep, Ignore, or Undecided')
    # Validate all decisions before writing anything. Keep a recovery snapshot.
    backup = snapshot(w)
    write_csv(w / 'master.csv', FIELDS, master.values())
    write_csv(w / 'ignored.csv', FIELDS, ignored.values())
    write_csv(w / 'inbox.csv', INBOX, pending.values())
    save_state(w, state)
    print(f'Kept {kept}; ignored {rejected}; pending {len(pending)}; master {len(master)}. Backup: {backup}')


def export(args):
    master, _, _, _ = load(args.workspace)
    rows = []
    for entry in master.values():
        validate(entry)
        profiles = re.split(r'[;,\s]+', entry.get('Profiles', ''))
        if args.profile not in profiles and '*' not in profiles:
            continue
        rows.append(dict(zip(RR_FIELDS, [entry['RX MHz'], entry['TX MHz'] or '0.000000',
                    entry['Name'], entry['Description'], entry['RX Mode'], receiver_tone(entry),
                    access_text(entry['TX Access Type'], entry['TX Access Value']), entry['Service'], entry['Group'], notch_hz(entry.get('CTCSS Notch Hz', ''))])))
    # This export is for receive-only radiod: combine overlapping source
    # listings without losing their separate records in the master.
    combined = {}
    destinations = {}
    notch_conflicts = set()
    for row in rows:
        rx, mode, tone, group = (row[f] for f in ('Frequency Output', 'Mode', 'PL Output Tone', 'Tag'))
        key = (rx, 'FM' if mode in ('FM', 'FMN') else mode, tone)
        destinations.setdefault(key, {}).setdefault(group, set()).add(row['Alpha Tag'])
        if key not in combined:
            combined[key] = row
            continue
        previous = combined[key]
        if previous['CTCSS Notch Hz'] != row['CTCSS Notch Hz']:
            notch_conflicts.add(key)
        if mode == 'FM':
            previous['Mode'] = 'FM'  # receive the widest listed analog signal
        for field in ('Alpha Tag', 'Description', 'Agency/Category'):
            if row[field] and row[field] != previous[field]:
                previous[field] += ' / ' + row[field]
    conflicts = []
    for key, groups in sorted(destinations.items()):
        rx, mode, tone = key
        channel = f'{rx} MHz {mode} ({tone or "no receive tone"})'
        if len(groups) > 1:
            details = '; '.join(f'{group} ({", ".join(sorted(names))})'
                                for group, names in sorted(groups.items()))
            conflicts.append(f'{channel} is assigned to different Groups: {details}')
        if key in notch_conflicts:
            conflicts.append(f'{channel} has conflicting CTCSS Notch Hz settings')
    if conflicts:
        raise ValueError('\n'.join(conflicts))
    rows = list(combined.values())
    write_csv(args.output, RR_FIELDS, rows)
    print(f'Exported {len(rows)} selected entries for {args.profile} to {args.output}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workspace', type=Path, required=True, help='folder for master.csv, inbox.csv, ignored.csv and import state')
    sub = ap.add_subparsers(dest='command', required=True)
    imp = sub.add_parser('import', help='stage downloads without selecting channels')
    imp.add_argument('files', type=Path, nargs='+')
    imp.set_defaults(func=import_files)
    migration = sub.add_parser('migrate', help='back up and upgrade catalog columns; create device templates')
    migration.set_defaults(func=migrate)
    accept = sub.add_parser('apply', help='apply Keep/Ignore decisions from inbox.csv')
    accept.set_defaults(func=apply)
    exp = sub.add_parser('export', help='export Master only, as rr2radiod-compatible CSV')
    exp.add_argument('--profile', required=True)
    exp.add_argument('-o', '--output', type=Path, required=True)
    exp.set_defaults(func=export)
    args = ap.parse_args()
    try:
        if args.command == 'export' and args.output.resolve() in {
                (args.workspace / n).resolve() for n in ('master.csv', 'inbox.csv', 'ignored.csv', 'import-state.json', 'source-rules.csv', 'device-profiles.csv', 'memory-assignments.csv')}:
            raise ValueError('export output must not overwrite catalog files')
        args.func(args)
    except (ValueError, OSError, csv.Error) as e:
        ap.exit(1, f'{e}\n')


if __name__ == '__main__':
    main()
