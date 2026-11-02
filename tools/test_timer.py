"""Offline timer boundary checks; hardware execution still requires a CCS bench test."""
from pathlib import Path
import unittest

MASK = 0xFFFFFFFF

def due(now, deadline):
    return ((now - deadline) & MASK) < 0x80000000

class TimerTests(unittest.TestCase):
    def test_tick_wrap(self):
        self.assertEqual((MASK + 1) & MASK, 0)
        self.assertEqual(((0xFFFFFFF0 + 32) & MASK), 16)

    def test_deadline_equality_and_order(self):
        self.assertFalse(due(9999, 10000))
        self.assertTrue(due(10000, 10000))
        self.assertTrue(due(10001, 10000))

    def test_deadline_after_wrap(self):
        start = 0xFFFFFF00
        deadline = (start + 10000) & MASK
        for elapsed in (0, 1, 255, 256, 9999):
            self.assertFalse(due((start + elapsed) & MASK, deadline))
        for elapsed in (10000, 10001, 20000):
            self.assertTrue(due((start + elapsed) & MASK, deadline))

    def test_overdue_deadline_before_wrap(self):
        self.assertTrue(due(0x20, 0xFFFFFFF0))
        self.assertFalse(due(0xFFFFFFEF, 0xFFFFFFF0))

    def test_source_counts_tick_before_all_early_returns(self):
        source = (Path(__file__).resolve().parents[1] / 'main.c').read_text()
        handler = source.split('void SysTickIntHandler(void)', 1)[1].split('void Init_ADC()', 1)[0]
        self.assertEqual(handler.count('GlobalTimer++;'), 1)
        self.assertLess(handler.index('GlobalTimer++;'), handler.index('ADCProcessorTrigger('))
        self.assertLess(handler.index('GlobalTimer++;'), handler.index('return;'))
        self.assertIn('(uint32_t)(now - HeartbeatTrigger) < 0x80000000UL', source)

if __name__ == '__main__':
    unittest.main()
