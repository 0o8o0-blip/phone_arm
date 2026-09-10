from __future__ import annotations

import json
import queue
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from aiohttp.test_utils import make_mocked_request

from follower.gateway import BrowserPhone, STALE_POSE_TIMEOUT_MS, _RecorderProxy
from server.api import OWNER_TIMEOUT_S, SessionApi, _token_hash
from server.capabilities import issue, verify
from server.relay import ControlPeer, ControlRelay


class CapabilityTests(unittest.TestCase):
    def test_capability_is_valid_until_expiry(self) -> None:
        token = issue("secret", role="arm", session="r_abcdefghijkl", expires_at=200)
        self.assertTrue(
            verify("secret", token, role="arm", session="r_abcdefghijkl", now=199)
        )
        self.assertFalse(
            verify("secret", token, role="arm", session="r_abcdefghijkl", now=200)
        )


class SessionApiTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.api = SessionApi(
            capability_secret="cap-secret",
            turn_secret="turn-secret",
            public_url="https://example.test",
            relay_urls={"europe": "https://eu.test/wt", "asia": "https://asia.test/wt"},
            turn_urls={"europe": "turn:eu.test", "asia": "turn:asia.test"},
            state_file=Path(self.temp_dir.name) / "tokens.json",
            event_log=None,
            follower_timeout_s=20,
            session_lifetime_s=3600,
        )

    def test_disconnected_session_is_kept_until_expiry(self) -> None:
        self.api.followers["current"] = {"seen_at": 0, "expires_at": 101}
        self.api.followers["expired"] = {"seen_at": 99, "expires_at": 100}
        self.api.owners = {"current": {}, "expired": {}}

        self.api._prune_expired(100)

        self.assertIn("current", self.api.followers)
        self.assertIn("current", self.api.owners)
        self.assertNotIn("expired", self.api.followers)
        self.assertNotIn("expired", self.api.owners)

    def test_rate_history_is_pruned_globally(self) -> None:
        self.api.creates_by_address = {
            "old": [1],
            "mixed": [1, 3700],
        }
        self.api._prune_expired(3701)
        self.assertEqual(self.api.creates_by_address, {"mixed": [3700]})

    def test_forwarded_address_is_used_only_from_loopback_proxy(self) -> None:
        direct = SimpleNamespace(
            remote="203.0.113.8", headers={"X-Forwarded-For": "198.51.100.2"}
        )
        proxied = SimpleNamespace(
            remote="127.0.0.1", headers={"X-Forwarded-For": "198.51.100.2, 127.0.0.1"}
        )
        self.assertEqual(self.api._remote_address(direct), "203.0.113.8")
        self.assertEqual(self.api._remote_address(proxied), "198.51.100.2")

    async def test_leader_ownership_is_exclusive_but_expires(self) -> None:
        now = time.time()
        session = "r_abcdefghijkl"
        access = "access-token"
        self.api.tokens = [{
            "name": "invite",
            "session": session,
            "hash": _token_hash(access),
            "expires_at": now + 300,
        }]
        self.api.followers[session] = {
            "session": session,
            "follower_id": "robot",
            "name": "robot",
            "listed": False,
            "video_available": False,
            "seen_at": now,
            "expires_at": now + 600,
        }

        first = make_mocked_request(
            "GET", "/webrtc/config?want_control=1&page_id=first",
            headers={"Authorization": f"Bearer {access}"},
        )
        second = make_mocked_request(
            "GET", "/webrtc/config?want_control=1&page_id=second",
            headers={"Authorization": f"Bearer {access}"},
        )
        first_body = json.loads((await self.api.webrtc_config(first)).body)
        second_body = json.loads((await self.api.webrtc_config(second)).body)
        self.assertEqual(first_body["controlRole"], "controller")
        self.assertEqual(second_body["controlRole"], "viewer")

        self.api.owners[session]["seen_at"] -= OWNER_TIMEOUT_S + 1
        second_body = json.loads((await self.api.webrtc_config(second)).body)
        self.assertEqual(second_body["controlRole"], "controller")


class RelaySessionTests(unittest.TestCase):
    def test_session_is_kept_until_latest_observed_capability_expires(self) -> None:
        relay = ControlRelay(capability_secret="secret")
        session = relay.get_session("r_abcdefghijkl", 101)
        peer = ControlPeer(
            role="phone",
            peer_id="phone-1",
            stream_id=1,
            addr=None,
            send_datagram=lambda _data: None,
        )
        session.phone = peer
        relay.detach_peer(session, peer, "test disconnect")
        relay.prune_expired(100)
        self.assertIs(relay.sessions[session.name], session)

        relay.get_session(session.name, 200)
        relay.prune_expired(101)
        self.assertIn(session.name, relay.sessions)
        relay.prune_expired(200)
        self.assertNotIn(session.name, relay.sessions)


class TrajectoryProxyTests(unittest.TestCase):
    def test_close_flushes_pending_trajectory_batch(self) -> None:
        trajectory_queue: queue.Queue = queue.Queue()
        with patch.dict("os.environ", {"PHONE_ARM_TRAJECTORY_BATCH_INTERVAL_S": "60"}):
            recorder = _RecorderProxy(None, trajectory_queue)
            recorder.record_trajectory({"seq": 1})
            recorder.record_trajectory({"seq": 2})
            recorder.close()
        self.assertEqual(
            trajectory_queue.get_nowait(),
            ("trajectory_batch", [{"seq": 1}, {"seq": 2}]),
        )


class SafetyTimeoutTests(unittest.TestCase):
    def test_stale_pose_is_marked_for_downstream_hold(self) -> None:
        phone = BrowserPhone()
        phone._latest_action_state = {
            "phone.pos": [1, 2, 3],
            "phone.raw_inputs": {},
            "phone.enabled": True,
            "control.source": "phone",
            "_t_recv_ms": time.time() * 1000 - STALE_POSE_TIMEOUT_MS - 1,
            "_pose_valid": True,
        }
        action = phone.get_action()
        self.assertEqual(action["phone.raw_inputs"]["_pose_stale"], 1)
        self.assertTrue(action["phone.enabled"])

    def test_stale_leader_is_disabled_immediately(self) -> None:
        phone = BrowserPhone()
        phone._latest_action_state = {
            "phone.pos": [0, 0, 0],
            "phone.raw_inputs": {},
            "phone.enabled": False,
            "control.source": "leader_arm",
            "leader.positions": {"shoulder_pan": 10},
            "leader.enabled": True,
            "leader.source_id": "leader-1",
            "_t_recv_ms": time.time() * 1000 - STALE_POSE_TIMEOUT_MS - 1,
            "_pose_valid": True,
        }
        action = phone.get_action()
        self.assertEqual(action["phone.raw_inputs"]["_pose_stale"], 1)
        self.assertFalse(action["leader.enabled"])


if __name__ == "__main__":
    unittest.main()
