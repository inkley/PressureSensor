"""Format-level tests, not firmware flash/recovery execution tests."""
import struct
import unittest
from uuid import UUID
from log_format import *

UID=UUID('00112233-4455-6677-8899-aabbccddeeff')
def example():
    return encode_block(UID,0,DATA,SAMPLE.pack(0,1234,2345)+SAMPLE.pack(1003,1200,2300),2,10,50000)

def reseal(b):
    b[1016:1020]=struct.pack('<I',crc32(b[:1016]))
    return bytes(b)

class FormatTests(unittest.TestCase):
    def test_crc_check_vector(self):
        self.assertEqual(crc32(b'123456789'),0xcbf43926)
    def test_roundtrip(self):
        b=example()
        self.assertEqual(len(b),1024)
        self.assertEqual(b[8:24],UID.bytes)
        self.assertEqual(decode_block(b)['records'],[(10,50000,1234,2345),(11,51003,1200,2300)])
    def test_corruption_every_byte(self):
        original=example()
        for offset in range(1024):
            b=bytearray(original); b[offset]^=1
            with self.assertRaises(ValueError): decode_block(bytes(b))
    def test_all_torn_prefixes(self):
        b=example()
        for cut in range(1024):
            with self.assertRaises(ValueError): decode_block(b[:cut]+b'\xff'*(1024-cut))
    def test_semantics_even_with_valid_crc(self):
        for offset,value in [(0,b'BAD!'),(4,b'\x02\x00'),(28,b'\xff\xff'),
                             (52,b'\x01'),(64,b'\x01'),(68,b'\xff\xff'),(80,b'\x00')]:
            b=bytearray(example()); b[offset:offset+len(value)]=value
            with self.assertRaises(ValueError): decode_block(reseal(b))
    def test_full_capacity_and_limits(self):
        payload=b''.join(SAMPLE.pack(i*1000,0,4095) for i in range(119))
        self.assertEqual(decode_block(encode_block(UID,1,DATA,payload,119))['count'],119)
        with self.assertRaises(ValueError): encode_block(UID,1,DATA,payload+b'\0'*8,120)
        with self.assertRaises(ValueError): encode_block(UUID(int=0),0,DATA,SAMPLE.pack(0,0,0),1)
    def test_time_overflow_and_nominal_mode(self):
        p=SAMPLE.pack(0,1,2)+SAMPLE.pack(1000,3,4)
        with self.assertRaises(ValueError): encode_block(UID,0,DATA,p,2,first_us=0xffffffffffffffff)
        b=encode_block(UID,0,DATA,p,2,flags=NOMINAL_TIME)
        self.assertEqual(decode_block(b)['flags'],NOMINAL_TIME)
        with self.assertRaises(ValueError): encode_block(UID,0,DATA,SAMPLE.pack(1,1,2),1)
    def test_event_and_manifest_envelopes(self):
        for kind,size in [(EVENTS,48),(MANIFEST,128),(JOURNAL,64)]:
            b=encode_block(UID,2,kind,bytes(size),1)
            self.assertEqual(decode_block(b)['payload'],bytes(size))

if __name__=='__main__': unittest.main()
