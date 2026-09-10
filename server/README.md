# Relay server

This directory contains optional hosted infrastructure. It is not needed on a
controller or follower machine when they use an existing hosted relay.

- `relay.py` carries WebTransport datagrams between one controller and
  one follower in each named session. Disconnected peers may resume the same
  session until its signed expiry; the relay does not delete sessions merely
  because both peers are temporarily absent.
- Each region runs `relay.py`. Controllers and followers independently choose
  their nearest relay. Same-edge peers pair locally; otherwise the controller
  ingress sends latest-only UDP through WireGuard to the arm's relay.
- `api.py` creates anonymous robot sessions and exchanges a single invitation
  capability for scoped control, media and TURN credentials. Video credentials
  are omitted when the follower starts with no camera.
- `capabilities.py` signs and verifies temporary credentials bound to one role
  and one robot session.
- `run.sh` starts the main relay and reads TLS and token configuration from
  environment variables.

Start the main relay from a source checkout:

```sh
./server/run.sh
```

The launcher accepts these environment variables:

- `PHONE_ARM_WT_HOST` and `PHONE_ARM_WT_PORT`
- `PHONE_ARM_WT_CERT` and `PHONE_ARM_WT_KEY`
- `PHONE_ARM_WT_EVENT_LOG`
- `PHONE_ARM_CAPABILITY_SECRET` (required)
- `PHONE_ARM_EDGE_NAME`
- `PHONE_ARM_BACKBONE_HOST`, `PHONE_ARM_BACKBONE_PORT`, and
  `PHONE_ARM_PEER_BACKBONE` (`name=wireguard-ip:port`)

Secret values may be literal tokens or `@/path/to/token-file`. Production uses
one server-only capability-signing secret shared by the API and relay. It is
never copied to followers or controllers.

The inter-region path deliberately has no application ACK, retry, ordered
delivery, or congestion window. Each direction keeps at most one pending
datagram per session; epoch and sequence fields reject reordered state, and
the receiving edge rejects over-age packets. WireGuard supplies encryption
and peer authentication. Commands remain absolute state snapshots, so losing
an older packet is safe and the next packet supersedes it.

```sh
mkdir -p server/secrets
openssl rand -hex 32 > server/secrets/capability.token
PHONE_ARM_CAPABILITY_SECRET=@server/secrets/capability.token ./server/run.sh
```

The default certificate paths are `server/certs/server.crt` and
`server/certs/server.key`. Production hosts should set the certificate and
key variables to their publicly trusted TLS files.

Run a regional relay with:

```sh
python3 -m server.relay --help
```
