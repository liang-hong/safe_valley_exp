#!/usr/bin/env python3
"""Static checks for gps_bias launch wiring and installed script permissions."""
import os
import stat
import unittest
import xml.etree.ElementTree as ET

PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NODE_PATH = os.path.join(PACKAGE_DIR, "scripts", "gps_bias_node.py")
LAUNCH_PATH = os.path.join(PACKAGE_DIR, "launch", "uav_offboard_ego.launch")


class GpsBiasWiringTest(unittest.TestCase):
    def test_node_script_has_git_executable_bits(self):
        mode = stat.S_IMODE(os.stat(NODE_PATH).st_mode)
        self.assertNotEqual(mode & 0o111, 0, "gps_bias_node.py must be executable")

    def test_offboard_launch_starts_configured_gps_bias_node(self):
        root = ET.parse(LAUNCH_PATH).getroot()
        nodes = [node for node in root.findall(".//node")
                 if node.get("pkg") == "safe_valley_exp"
                 and node.get("type") == "gps_bias_node.py"]
        self.assertEqual(len(nodes), 1)
        node = nodes[0]
        self.assertEqual(node.get("name"), "gps_bias_node")
        params = {param.get("name"): param.get("value")
                  for param in node.findall("param")}
        self.assertEqual(params, {
            "~window_s": "10.0",
            "~publish_rate_hz": "1.0",
            "~lockout_s": "3.0",
        })


if __name__ == "__main__":
    unittest.main()
