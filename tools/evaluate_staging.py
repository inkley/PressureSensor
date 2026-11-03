"""RAM staging sensitivity from current Debug map; not a timing measurement."""
import json,math,re
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'Debug/Inkley_PressureSensor.map').read_text()
m=re.search(r'^\s*SRAM\s+([\da-fA-F]+)\s+([\da-fA-F]+)\s+([\da-fA-F]+)\s+([\da-fA-F]+)',s,re.M)
if not m:raise ValueError('SRAM map missing')
origin,total,used,free=(int(x,16) for x in m.groups())
raw=4094*4;image=1024
pairs_per_block=(1024-64-8)//8
queue_blocks=8
base=used-raw+queue_blocks*1024 # retain existing serialization image and stack
budgets={'base_after_replacing_raw_array_with_queue':base,'selective_sram_code_allowance':8192,
         'additional_stack_allowance':2048,'descriptors_events_diagnostics_allowance':1024}
planned=sum(budgets.values())
optional_dma=2*256*2*4 # two buffers, 256 pairs, two 32-bit FIFO words per pair
scenarios=[]
for seconds in (.015,.04,.5,.5387,.6387):
    pairs=math.ceil(seconds*1000)
    blocks=math.ceil(pairs/pairs_per_block)+2 # conservatively withhold filling and in-flight blocks
    scenarios.append(dict(stall_seconds=seconds,new_pairs=pairs,queue_blocks_including_two_withheld=blocks))
print(json.dumps(dict(sram_total=total,map_used=used,map_unallocated=free,raw_array_bytes=raw,
                     retained_serialization_image_bytes=image,other_current_allocations=used-raw-image,
                     compact_pairs_per_block=pairs_per_block,proposed_queue_blocks=queue_blocks,
                     total_queue_pairs=queue_blocks*pairs_per_block,
                     available_pairs_if_six_blocks_start_free=(queue_blocks-2)*pairs_per_block,
                     planning_budgets=budgets,planned_total=planned,remaining=total-planned,
                     optional_raw_adc_dma_buffers=optional_dma,remaining_with_optional_dma=total-planned-optional_dma,
                     scenarios=scenarios),indent=2))
