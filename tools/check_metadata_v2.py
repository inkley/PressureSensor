"""Check frozen v2 layouts and baseline-planning or measured-build RAM; not a codec."""
import argparse
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def evaluate(map_path):
    schema = json.loads((ROOT / 'tools/metadata_v2_schema.json').read_text())
    expected = dict(header=96, sample=16, event=64, metadata=256,
                    manifest=256, journal=96)
    for name, size in expected.items():
        layout = schema['layouts'][name]
        offset = 0
        names = set()
        for field in layout['fields']:
            assert field['name'] not in names, 'Duplicate field'
            names.add(field['name'])
            assert field['offset'] == offset and field['bytes'] > 0, name
            offset += field['bytes']
        assert offset == layout['bytes'] == size, name
    assert schema['version'] == 2 and schema['block_bytes'] == 1024
    assert schema['header_bytes'] == 96 and schema['crc_offset'] == 1016
    assert schema['commit_offset'] == 1020
    payload = schema['crc_offset'] - schema['header_bytes']
    pairs, events = payload // 16, payload // 64
    assert pairs == 57 and events == 14
    assert (schema['ram_events'] + events - 1) // events == 3

    raw = map_path.read_bytes()
    text = raw.decode()
    m = re.search(r'^\s*SRAM\s+([\da-fA-F]+)\s+([\da-fA-F]+)\s+'
                  r'([\da-fA-F]+)\s+([\da-fA-F]+)', text, re.M)
    assert m, 'SRAM map missing'
    origin, total, used, free = (int(x, 16) for x in m.groups())
    span = re.search(r'^20000000\s+20000000\s+([\da-fA-F]+)\s+', text, re.M)
    assert span and origin == 0x20000000 and total == 32768
    extent = int(span[1], 16)
    assert extent >= used and used + free == total
    # Old map yields the planning budget; a measured build must prove its actual
    # arena/event symbols and 4 KiB stack rather than subtract the old array twice.
    measured = bool(re.search(r'\b00004000\b[^\n]*capture_arena', text, re.I))
    if measured:
        assert re.search(r'\b00000800\b[^\n]*m2_events', text, re.I), 'Event array size changed'
        assert re.search(r'^\s*\.stack\s+0\s+[\da-fA-F]+\s+00001000', text, re.M), '4 KiB stack missing'
        budget = dict(linked_occupied_extent=extent)
        planned = extent
    else:
        assert re.search(r'\b00003ff8\b[^\n]*SensorBufferData', text, re.I), 'Unrecognized staging baseline'
        old_array = 4094 * 4
        shared = max(old_array, schema['ram_samples'] * 16)
        budget = dict(other_existing_extent=extent-old_array,
                      shared_staging=shared, event_array=schema['ram_events']*64,
                      metadata_allowance=1024, control_allowance=512,
                      extra_stack_allowance=2048)
        planned = sum(budget.values())
    assert total - planned >= 4096, 'Margin below 4 KiB'
    return dict(design_only=not measured, runtime_stack_unmeasured=True, map_sha256=hashlib.sha256(raw).hexdigest(),
                map_used=used, occupied_extent=extent, sram_total=total,
                budgets=budget, planned=planned, remaining=total-planned,
                measured_capture_pairs=schema['ram_samples'],
                pairs_per_data_block=pairs, events_per_block=events,
                maximum_capture_data_blocks=(schema['ram_samples']+pairs-1)//pairs,
                flash_data_ceiling_pairs=119*pairs)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=ROOT/'Debug/Inkley_PressureSensor.map')
    args = parser.parse_args()
    print(json.dumps(evaluate(args.map), indent=2))
