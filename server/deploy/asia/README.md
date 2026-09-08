# Asia edge deployment

The Singapore host at `146.190.104.81` is a symmetric regional edge. Both
controllers and followers can connect to WebTransport on UDP port 4434. The
edge preserves the requested `/wt/phone` or `/wt/arm` path and forwards that
session to the central London relay.

Deploy all of these together:

- `server/forwarder.py` as `/opt/phone_arm/webtransport_forwarder.py`;
- `shared/__init__.py` and `shared/webtransport.py` under
  `/opt/phone_arm/shared/`;
- `phone-arm-wt-forwarder.service` under `/etc/systemd/system/`;
- `Caddyfile` under `/etc/caddy/`.

The shared WebTransport module is a runtime dependency, not an optional helper.
After copying it, reload systemd and restart `phone-arm-wt-forwarder`.

Singapore also runs coturn. It uses TURN REST authentication and the same
server-only `static-auth-secret` as London. That secret is never copied to a
robot or controller. The central API turns it into expiring credentials, and
the browser tries the nearer TURN host first when a direct media path fails.
`turnserver.conf.template` records the safe, non-secret portion of this config;
replace its placeholder only on the host and never commit the rendered file.
