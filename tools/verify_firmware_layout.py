"""Check a TI ELF32 image before flashing: partition bounds, vectors, and stack.

Usage: python tools/verify_firmware_layout.py Debug/Inkley_PressureSensor.out
No hardware access and no third-party Python dependencies.
"""
import argparse
from pathlib import Path
import struct


def verify(path):
    data = Path(path).read_bytes()
    if data[:7] != b'\x7fELF\x01\x01\x01':
        raise ValueError('Expected little-endian ELF32')
    header = struct.unpack_from('<HHIIIIIHHHHHH', data, 16)
    _, machine, _, _, phoff, shoff, _, _, phsize, phcount, shsize, shcount, names_index = header
    if machine != 40:
        raise ValueError('Expected ARM executable')
    sections = [struct.unpack_from('<10I', data, shoff+i*shsize) for i in range(shcount)]

    def contents(section):
        return data[section[4]:section[4]+section[5]]

    def string(table, offset):
        return table[offset:table.index(b'\0', offset)].decode()

    names = contents(sections[names_index])
    by_name = {string(names, s[0]): s for s in sections}
    symbols = {}
    for section in sections:
        if section[1] == 2:  # SHT_SYMTAB
            strings = contents(sections[section[6]])
            for offset in range(section[4], section[4]+section[5], section[9]):
                name, value, _, _, _, _ = struct.unpack_from('<IIIBBH', data, offset)
                symbols[string(strings, name)] = value
    expected = {'__flash_log_start__': 0x20000, '__flash_log_end__': 0x40000,
                '__flash_log_size__': 0x20000}
    for name, value in expected.items():
        if symbols.get(name) != value:
            raise ValueError(f'Incorrect partition symbol: {name}')
    for i in range(phcount):
        kind, _, virtual, physical, filesz, memsz, _, _ = struct.unpack_from('<8I', data, phoff+i*phsize)
        if kind != 1 or memsz == 0:
            continue
        if not (0 <= virtual < virtual+memsz <= 0x20000 or
                0x20000000 <= virtual < virtual+memsz <= 0x20008000):
            raise ValueError(f'Load segment outside application flash/SRAM: {virtual:#x}')
        if filesz and not (0 <= physical < physical+filesz <= 0x20000 or
                          0x20000000 <= physical < physical+filesz <= 0x20008000):
            raise ValueError(f'Image payload outside application flash/SRAM: {physical:#x}')
    vectors = by_name['.intvecs']
    stack = by_name['.stack']
    if vectors[3] != 0:
        raise ValueError('Reset vectors must start at zero')
    words = struct.unpack('<'+'I'*(vectors[5]//4), contents(vectors))
    stack_top = stack[3]+stack[5]
    if words[0] != stack_top or stack_top % 8 or symbols['__STACK_TOP'] != stack_top:
        raise ValueError('Initial stack pointer does not match allocated stack end/alignment')
    for index, name in ((1, 'ResetISR'), (15, 'SysTickIntHandler'), (55, 'IntCAN0Handler')):
        if not words[index] & 1 or words[index] != symbols[name] | 1:
            raise ValueError(f'Incorrect interrupt vector for {name}')
    print(f'{path}: PASS; stack={stack[5]} bytes, initial SP={stack_top:#010x}; '
          'vectors valid; no load segment in reserved flash [0x20000, 0x40000).')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path, nargs='+')
    args = parser.parse_args()
    for image in args.image:
        verify(image)
