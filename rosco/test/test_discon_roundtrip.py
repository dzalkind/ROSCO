"""
DISCON.IN files written by older ROSCO versions are missing parameters added
since. read_DISCON -> write_DISCON must still work (openfast_io does exactly
this round trip when setting up a simulation).
"""
import os
import tempfile
import unittest

from rosco.toolbox.utilities import read_DISCON, write_DISCON

this_dir = os.path.dirname(os.path.realpath(__file__))
rosco_root = os.path.dirname(os.path.dirname(this_dir))
test_cases = os.path.join(rosco_root, 'Examples', 'Test_Cases')


class TestDISCONRoundTrip(unittest.TestCase):
    def test_roundtrip_preserves_inputs(self):
        src = os.path.join(
            test_cases, 'IEA-15-240-RWT', 'IEA-15-240-RWT-UMaineSemi',
            'IEA-15-240-RWT-UMaineSemi_DISCON.IN')
        turbine = type('', (), {'TurbineName': 'test'})()

        discon_in = read_DISCON(src)
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'DISCON.IN')
            write_DISCON(turbine, None, param_file=out, rosco_vt=discon_in)
            written = read_DISCON(out)

        self.assertEqual([k for k in discon_in if k not in written], [])
        self.assertIn('OutputFormat', written)


if __name__ == '__main__':
    unittest.main()
