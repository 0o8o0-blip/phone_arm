# Shared code

Only code with current callers in multiple runnable components belongs here:

- `webtransport.py` is used by the follower gateway and leader controller.
- `leader_protocol.py` defines and validates the wire message used by both the
  leader controller and follower gateway.
- `usb_arm.py` discovers USB serial adapters and prevents the leader and
  follower processes from selecting an arm that is already in use.

Follower mapping, server configuration and browser implementation details stay
with their owning components.
