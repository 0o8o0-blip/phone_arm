# VPS configuration

Live infrastructure configs for the London relay droplet
(`188.166.154.201`, DO tag `claude-relay`). Kept in-repo so they're
versioned; the VPS filesystem is the source of truth at runtime, this is
just the checked-in copy.

## `Caddyfile`

Deployed at `/etc/caddy/Caddyfile` on the VPS. Serves:
- Static operator page (index.html + app.js) directly from
  `/opt/phone_arm/static/`. Rsynced from the Pi on teleop startup
  (see `follower/run.sh`).
- `/robot/whip*` and `/robot/whep*` -> reverse-proxy to MediaMTX
  (loopback :8889). WHIP is used by the Pi to publish video into the
  SFU; WHEP is used by phones to subscribe.
- Everything else -> reverse-proxy to `https://localhost:8443`, which
  is the bore tunnel terminating against the Pi's HTTPS server.

To redeploy:
```
scp -i ~/.ssh/do_wg_relay server/deploy/vps/Caddyfile root@188.166.154.201:/etc/caddy/Caddyfile
ssh -i ~/.ssh/do_wg_relay root@188.166.154.201 "caddy fmt --overwrite /etc/caddy/Caddyfile && systemctl reload caddy"
```

## `mediamtx.yml.template` + `mediamtx.service`

MediaMTX is the WebRTC SFU we use to distribute the Pi's camera to
operator phones. Pi publishes H.264 via WHIP; phones subscribe via WHEP.

The `.template` is safe to check in; the deployed
`/opt/mediamtx/mediamtx.yml` is chmod 600 with actual passwords
substituted in (the `${MTX_PUBLISH_PW}` etc. placeholders replaced from
`/root/.mediamtx_secrets`).

To redeploy:
```
scp -i ~/.ssh/do_wg_relay server/deploy/vps/mediamtx.yml.template root@188.166.154.201:/opt/mediamtx/mediamtx.yml.template
ssh -i ~/.ssh/do_wg_relay root@188.166.154.201 "
  set -a && . /root/.mediamtx_secrets && set +a
  sed -e 's|\${MTX_PUBLISH_PW}|'\"\$MTX_PUBLISH_PW\"'|' \\
      -e 's|\${MTX_PLAY_PW}|'\"\$MTX_PLAY_PW\"'|' \\
      -e 's|\${TURN_PW}|'\"\$TURN_PW\"'|' \\
      /opt/mediamtx/mediamtx.yml.template > /opt/mediamtx/mediamtx.yml &&
  chmod 600 /opt/mediamtx/mediamtx.yml &&
  systemctl restart mediamtx"
```

Secrets on the Pi (source-of-truth for the credentials, replicated to
VPS) live under `~/.phone_arm_secrets/mediamtx_publish_pw` and
`~/.phone_arm_secrets/mediamtx_play_pw`.

### End-to-end topology when WHEP is enabled

```
Camera (Pi)  --v4l2-->  ffmpeg (H.264, x264enc)
                            |  RTSP UDP loopback
                            v
                        whipinto (Rust WHIP client, live777 project)
                            |  HTTPS + Bearer WHIP_PUB_TOKEN
                            v
                        Caddy on VPS  ------[loopback]-----> MediaMTX (SFU)
                                                                   |
                        Operator phone <--- WebRTC/H.264 <--- MediaMTX
                            ^  ^
                            |  Bearer WHIP_SUB_TOKEN
                            |  WHEP subscribe via `/robot/whep`
                            |
                       (Caddy on VPS)
```

Pi-side toolchain (both packaged as `.deb` from
https://github.com/binbat/live777/releases):
- `whipinto` -- WHIP publisher, ingests RTSP, publishes WebRTC
- `whepfrom` -- WHEP subscriber; useful for smoke-testing without a
   real browser

Install both on a fresh Pi:
```
cd /tmp
wget https://github.com/binbat/live777/releases/download/v0.9.0/whipinto_0.9.0_arm64.deb
wget https://github.com/binbat/live777/releases/download/v0.9.0/whepfrom_0.9.0_arm64.deb
sudo dpkg -i whipinto_0.9.0_arm64.deb whepfrom_0.9.0_arm64.deb
```

`follower/run.sh` spawns `whipinto` + `ffmpeg` as background children
of the teleop shell when `PHONE_ARM_MEDIAMTX_WHEP_URL` is set (which
happens automatically when `~/.phone_arm_secrets/mediamtx_publish_pw`
exists on the Pi). `trap EXIT` tears both down when teleop exits.

Test without hardware: set `PHONE_ARM_WHIP_INPUT_ARGS` to an ffmpeg
input, e.g.:
```
PHONE_ARM_WHIP_INPUT_ARGS="-f lavfi -re -i testsrc2=size=640x480:rate=30" \
  ./follower/run.sh
```

The operator page uses WHEP by default. Both the SFU URL and the subscribe
token are advertised in the Pi's `/webrtc/config` response when
`PHONE_ARM_MEDIAMTX_WHEP_URL` + `PHONE_ARM_MEDIAMTX_PLAY_TOKEN` are set.

## Systemd services on this VPS

- `caddy` -- reverse proxy + Let's Encrypt cert
- `bore-server` -- bore tunnel terminator (Pi connects OUT to this)
- `coturn` -- WebRTC TURN relay used by browser WHEP playback
- `phone-arm-wt` -- WebTransport (QUIC) control relay for pose datagrams
- `mediamtx` -- WebRTC SFU (WHIP ingest, WHEP egress) for robot video

The Singapore forwarder droplet (`146.190.104.81`) runs its own coturn
and the `phone-arm-wt-forwarder` service; those configs aren't checked
in here yet.

The source entry points are `server/run.sh` for the main relay and
`python3 -m server.forwarder` for the optional regional forwarder.
