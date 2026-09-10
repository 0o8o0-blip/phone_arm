# Asia relay deployment

The Singapore host at `146.190.104.81` runs a complete control relay. Both
controllers and followers can connect to WebTransport on UDP port 4434. Peers
in Asia are paired locally; a controller whose arm is in Europe is bridged to
the London relay using latest-only UDP inside WireGuard.

Deploy all of these together:

- `server/relay.py` as `/opt/phone_arm/relay.py`;
- `server/capabilities.py` as `/opt/phone_arm/capabilities.py`;
- `phone-arm-wt-relay.service` under `/etc/systemd/system/`;
- `Caddyfile` under `/etc/caddy/`.
- `wg0.conf.template` rendered with host-only WireGuard keys as
  `/etc/wireguard/wg0.conf`.

Install the same server-only capability-signing secret used by the API at
`/etc/phone_arm/capability_secret`. After copying the files, reload systemd,
disable the old `phone-arm-wt-forwarder` unit, and enable and restart
`phone-arm-wt-relay`.

Deploy the API and both regional relay changes together, then restart followers
so their heartbeat advertises the arm edge. Sessions created by older follower
processes should be replaced rather than carried across this routing cutover.
The Singapore relay binds UDP port 7443 only on WireGuard address `10.44.0.2`.
Run chrony on both edges so the 150 ms packet-age deadline is meaningful.

Singapore also runs coturn. It uses TURN REST authentication and the same
server-only `static-auth-secret` as London. That secret is never copied to a
robot or controller. The central API turns it into expiring credentials, and
the browser uses the nearer TURN host first for its relay-only media path.
`turnserver.conf.template` records the safe, non-secret portion of this config;
replace its placeholder only on the host and never commit the rendered file.
