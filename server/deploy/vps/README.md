# VPS deployment

Production configuration for the London service host at `188.166.154.201`.
Followers make outbound connections only; they need no accounts, copied
secrets, SSH keys, open ports or tunnels.

## Services

- `phone-arm-api` anonymously creates robot sessions, maintains heartbeat
  leases, validates invitations and issues temporary service capabilities.
- `phone-arm-wt` routes WebTransport control datagrams by session.
- `caddy` serves the controller and authorizes per-session WHIP/WHEP requests.
- `mediamtx` provides an independent video path for each active robot.
- `coturn` accepts short-lived TURN REST credentials issued by the API.

The Singapore host at `146.190.104.81` runs a symmetric WebTransport edge and
a second coturn instance. Robots and controllers independently select their
nearest control edge. Browser media is TURN-only and receives both regional
servers with the nearest one listed first.

## Session model

`follower/run.sh` anonymously creates a random session. The API returns:

- a private registration capability for heartbeats;
- private arm-relay and video-publish capabilities;
- one two-hour invitation link for the operator.

The invitation is stored by the API as a SHA-256 hash. Its plaintext value
appears after `#access=` in the link, so browsers do not include it in ordinary
HTTP requests or Caddy access logs. After validating it in an Authorization
header, the API issues short-lived controller, video-view and TURN credentials
scoped to that one session.

## Media

Caddy accepts `/media/<session>/whip` and `/media/<session>/whep`, asks the API
to validate the corresponding capability, strips `/media`, and proxies the
request to MediaMTX on `127.0.0.1:8889`. MediaMTX allows matching dynamic
`r_*` paths but remains unreachable directly from the Internet.

The follower publishes H.264 with `ffmpeg` and `whipinto`. Browsers subscribe
using WHEP. Each robot therefore has its own publisher and video path.

## Server-only secrets

These files exist only on the service hosts and are installed with mode 0600:

- `/etc/phone_arm/capability_secret` signs API, relay and media capabilities.
- `/etc/phone_arm/turn_rest_secret` signs temporary TURN credentials. The same
  server-only value must be installed on both coturn hosts because either edge
  can validate credentials issued by the central API.

They are infrastructure keys, not follower setup credentials.

`turnserver.conf.template` records the non-secret London coturn configuration.
During deployment replace `TURN_REST_SECRET_GOES_HERE` with the contents of the
server-only secret, install the rendered file as `/etc/turnserver.conf` with
mode 0600, and restart coturn. Never commit the rendered file.

## Deploy

Deploy the Python entry points, `capabilities.py`, and units to `/opt/phone_arm` and
`/etc/systemd/system`, copy `controllers/phone/` to
`/opt/phone_arm/static/`, validate Caddy before reloading it, and restart API,
relay and MediaMTX in that order. Preserve the previous Caddy and MediaMTX
files as rollback copies during a live cutover.
