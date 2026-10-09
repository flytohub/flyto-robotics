import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from flyto_robotics.mobile_provider import CAPABILITY, execute
from flyto_robotics.ros2_adapter import STATUS_SCHEMA


class MobileRos2ProviderTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / "ros2-adapter-status.json"
        self.status = {
            "schema": STATUS_SCHEMA,
            "service": "ros2_readiness_adapter",
            "state": "ready",
            "ready": True,
            "reason": "all_expected_topics_present",
            "missing_topics": [],
            "mismatched_topics": [],
        }
        self.raw = json.dumps(self.status, separators=(",", ":")).encode()
        self.path.write_bytes(self.raw)
        self.request = {
            "schema": "flyto2.execution.v1",
            "invocation_id": "readiness-invocation-01",
            "operation_id": "readiness-operation-01",
            "capability": CAPABILITY,
            "revision": 1,
            "requested_at": "2026-10-10T00:00:00Z",
            "input": {},
        }

    def test_success_is_readable_status_not_robot_mission_completion(self):
        result = execute(self.request, self.path)
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["output"]["graph_state"], "ready")
        self.assertTrue(result["output"]["ready"])
        self.assertEqual(result["evidence"][0]["sha256"], hashlib.sha256(self.raw).hexdigest())
        self.assertNotIn("mission_completed", result["output"])

    def test_unready_graph_remains_successful_read_with_ready_false(self):
        self.status["ready"] = False
        self.status["state"] = "unready"
        self.path.write_text(json.dumps(self.status))
        result = execute(self.request, self.path)
        self.assertEqual(result["status"], "success")
        self.assertFalse(result["output"]["ready"])

    def test_stale_missing_symlink_and_unknown_capability_fail_closed(self):
        os.utime(self.path, (time.time() - 120, time.time() - 120))
        self.assertEqual(execute(self.request, self.path)["status"], "failed")
        self.assertEqual(execute(self.request, self.path.with_name("missing"))["status"], "failed")
        self.path.touch()
        alias = Path(self.folder.name) / "alias"
        alias.symlink_to(self.path)
        self.assertEqual(execute(self.request, alias)["status"], "failed")
        with self.assertRaises(ValueError):
            execute({**self.request, "capability": "motion.advance"}, self.path)

    def test_process_uses_typed_runtime_invocation_without_cloud(self):
        completed = subprocess.run(
            [sys.executable, "-m", "flyto_robotics.mobile_provider",
             "--status-file", str(self.path)],
            input=json.dumps(self.request), text=True, capture_output=True,
            check=False, timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["invocation_id"], self.request["invocation_id"])


if __name__ == "__main__":
    unittest.main()
