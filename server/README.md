# Relay server

This directory contains optional hosted infrastructure. It is not needed on a
controller or follower machine when they use an existing hosted relay.

- `relay.py` carries WebTransport datagrams between one controller and
  one follower in each named session.
- `forwarder.py` is an optional regional edge that forwards controller traffic
  to the main relay.
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
- `PHONE_ARM_WT_PHONE_SECRET` and `PHONE_ARM_WT_ARM_SECRET`

Secret values may be literal tokens or `@/path/to/token-file`. The relay
refuses to start unless both secrets resolve to non-empty values. For a local
checkout, put them in the ignored default locations:

```sh
mkdir -p server/secrets
openssl rand -hex 32 > server/secrets/phone.token
openssl rand -hex 32 > server/secrets/arm.token
```

The default certificate paths are `server/certs/server.crt` and
`server/certs/server.key`. Production hosts should set the certificate and
key variables to their publicly trusted TLS files.

Run a regional forwarder with:

```sh
python3 -m server.forwarder --help
```
