"""Offline reference codec for proposed log block v1; never accesses hardware."""
import struct
import zlib
from uuid import UUID

SIZE = 1024
HEADER = struct.Struct('<4sHH16sIHHQQI12s')
SAMPLE = struct.Struct('<IHH')
MAGIC = b'IPLG'
VERSION = 1
COMMIT = b'CMIT'
DATA, EVENTS, MANIFEST, JOURNAL = 1, 2, 3, 4
ACTUAL_TIME, NOMINAL_TIME = 1, 2
ALLOWED_FLAGS = 15
RECORD_SIZES = {DATA: 8, EVENTS: 48, MANIFEST: 128, JOURNAL: 64}

def crc32(data):
    return zlib.crc32(data) & 0xffffffff

def encode_block(session, sequence, kind, payload, count, first_slot=0,
                 first_us=0, period_us=1000, flags=ACTUAL_TIME):
    if not isinstance(session, UUID) or session.int == 0:
        raise ValueError('nonzero UUID required')
    if kind not in RECORD_SIZES or not 0 < count <= 952 // RECORD_SIZES[kind]:
        raise ValueError('invalid kind/count')
    if kind in (MANIFEST, JOURNAL) and count != 1:
        raise ValueError('one manifest required')
    if len(payload) != count * RECORD_SIZES[kind]:
        raise ValueError('payload length mismatch')
    if flags & ~ALLOWED_FLAGS or (flags & 3) not in (ACTUAL_TIME, NOMINAL_TIME):
        raise ValueError('invalid flags/time basis')
    if period_us != 1000:
        raise ValueError('v1 requires nominal 1000 us period')
    try:
        header = HEADER.pack(MAGIC, VERSION, kind, session.bytes, sequence, count,
                             flags, first_slot, first_us, period_us, bytes(12))
    except struct.error as exc:
        raise ValueError('header integer out of range') from exc
    body = header + payload + b'\xff' * (952-len(payload))
    block = body + struct.pack('<I', crc32(body)) + COMMIT
    decode_block(block)  # same semantic validation for encoders and readers
    return block

def decode_block(block):
    if len(block) != SIZE or block[1020:] != COMMIT:
        raise ValueError('wrong size or absent commit')
    if crc32(block[:1016]) != struct.unpack_from('<I', block, 1016)[0]:
        raise ValueError('CRC mismatch')
    magic, version, kind, uid, seq, count, flags, slot, time, period, reserved = HEADER.unpack_from(block)
    if magic != MAGIC or version != VERSION or kind not in RECORD_SIZES:
        raise ValueError('unsupported format')
    if uid == bytes(16) or reserved != bytes(12) or period != 1000:
        raise ValueError('invalid header')
    if flags & ~ALLOWED_FLAGS or (flags & 3) not in (ACTUAL_TIME, NOMINAL_TIME):
        raise ValueError('invalid flags/time basis')
    if not 0 < count <= 952 // RECORD_SIZES[kind] or (kind in (MANIFEST, JOURNAL) and count != 1):
        raise ValueError('invalid count')
    end = 64 + count * RECORD_SIZES[kind]
    if block[end:1016] != b'\xff' * (1016-end):
        raise ValueError('invalid padding')
    records = []
    if kind == DATA:
        if slot + count - 1 > 0xffffffffffffffff:
            raise ValueError('slot overflow')
        previous = -1
        for i in range(count):
            delta, p1, p2 = SAMPLE.unpack_from(block, 64+i*8)
            if (i == 0 and delta != 0) or delta <= previous or p1 > 4095 or p2 > 4095:
                raise ValueError('invalid sample')
            if time + delta > 0xffffffffffffffff:
                raise ValueError('timestamp overflow')
            if flags & NOMINAL_TIME and delta != i*1000:
                raise ValueError('invalid nominal timestamp')
            records.append((slot+i, time+delta, p1, p2))
            previous = delta
    return dict(session=UUID(bytes=uid), sequence=seq, kind=kind, count=count,
                flags=flags, first_slot=slot, first_us=time,
                payload=block[64:end], records=records)
