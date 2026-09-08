# Relay server

This directory contains optional hosted infrastructure. It is not needed on a
controller or follower machine when they use an existing hosted relay.

- `relay.py` carries WebTransport datagrams between one controller and
  one follower in each named session.
- `forwarder.py` is an optional regional edge that forwards controller traffic
  to the main relay.
- `api.py` creates anonymous robot sessions and exchanges a single invitation
  capability for scoped control, media and TURN credentials.
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
- `PHONE_ARM_CAPABILITY_SECRET` (recommended)
- `PHONE_ARM_WT_PHONE_SECRET` and `PHONE_ARM_WT_ARM_SECRET` (legacy fallback)

Secret values may be literal tokens or `@/path/to/token-file`. Production uses
one server-only capability-signing secret shared by the API and relay. It is
never copied to followers or controllers. The two legacy role secrets remain
available only for local compatibility.

```sh
mkdir -p server/secrets
openssl rand -hex 32 > server/secrets/capability.token
openssl rand -hex 32 > server/secrets/phone.token
openssl rand -hex 32 > server/secrets/arm.token
PHONE_ARM_CAPABILITY_SECRET=@server/secrets/capability.token ./server/run.sh
```

The default certificate paths are `server/certs/server.crt` and
`server/certs/server.key`. Production hosts should set the certificate and
key variables to their publicly trusted TLS files.

Run a regional forwarder with:

```sh
python3 -m server.forwarder --help
```
