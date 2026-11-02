"""Provisional capacity arithmetic; no flash writes or firmware changes."""
import json
import math
import re
from pathlib import Path

root = Path(__file__).resolve().parents[1]
text = (root / 'Debug/Inkley_PressureSensor.map').read_text()
linker = (root / 'tm4c123gh6pm.cmd').read_text()
rows = {}
for name in ('FLASH_APP', 'SRAM'):
    m = re.search(r'^\s*'+name+r'\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)', text, re.M)
    if not m:
        raise ValueError(f'Missing {name} map row')
    rows[name] = dict(zip(('origin','length','used','unused'), (int(x,16) for x in m.groups())))
start = int(re.search(r'__flash_log_start__\s*=\s*(0x[0-9a-fA-F]+)', linker)[1],16)
end = int(re.search(r'__flash_log_end__\s*=\s*(0x[0-9a-fA-F]+)', linker)[1],16)
block = 1024
reserved_blocks = {'journal_a':1,'journal_b':1,'session_descriptor':1,'gap_events':4,'recovery_spares':2}
assert start % block == end % block == 0
assert rows['FLASH_APP']['origin'] + rows['FLASH_APP']['length'] <= start
blocks = (end-start)//block
available = blocks-sum(reserved_blocks.values())
header, trailer = 64, 8
payload = block-header-trailer
formats=[]
for label,size in [('raw_only',4),('compact_timed_candidate',8),('explicit_index_timed_candidate',16)]:
    per_block=payload//size
    count=available*per_block
    formats.append(dict(format=label,record_bytes=size,pairs_per_block=per_block,
                        pairs=count,seconds_nominal=count/1000,
                        blocks_for_10s=math.ceil(10000/per_block),
                        blocks_for_30s=math.ceil(30000/per_block),
                        bytes_unused_per_data_block=payload-per_block*size))
print(json.dumps(dict(map=rows,log_start=hex(start),log_end=hex(end),log_bytes=end-start,
                     erase_block_bytes=block,total_blocks=blocks,reserved_blocks=reserved_blocks,
                     data_blocks=available,block_header_bytes=header,block_trailer_bytes=trailer,
                     formats=formats),indent=2))
