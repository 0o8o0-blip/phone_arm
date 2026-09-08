# Shared code

Only code with current callers in multiple runnable components belongs here:

- `webtransport.py` is used by the follower gateway, leader controller and
  regional server forwarder.
- `leader_protocol.py` defines and validates the wire message used by both the
  leader controller and follower gateway.

Follower mapping, server configuration and browser implementation details stay
with their owning components.
