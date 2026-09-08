#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PYTHON_BIN="${PHONE_ARM_PYTHON:-$HOME/.venv/lerobot/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  echo "Follower Python environment not found: $PYTHON_BIN" >&2
  echo "Set PHONE_ARM_PYTHON to the Python executable containing lerobot." >&2
  exit 1
fi

# Preserve the prior log + trajectory CSV so we have forensics when a session
# crashes, and so each run's data isn't clobbered by the next.
LOG_DIR="${PHONE_ARM_LOG_DIR:-$HOME/phone_arm_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/teleop.log"
TRAJ="$LOG_DIR/teleop_trajectory.csv"
METRICS="$LOG_DIR/system_metrics.csv"
WHIPINTO_LOG="$LOG_DIR/whipinto.log"
WHIP_FFMPEG_LOG="$LOG_DIR/whip_ffmpeg.log"
VIDEO_SUPERVISOR_LOG="$LOG_DIR/video_supervisor.log"
export PHONE_ARM_TRAJECTORY_CSV="${PHONE_ARM_TRAJECTORY_CSV:-$TRAJ}"
export PHONE_ARM_RUN_ID="${PHONE_ARM_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)-$$}"
export PHONE_ARM_RECORDING_DIR="$PWD/teleop_recordings/runs/$PHONE_ARM_RUN_ID"
for path in "$LOG" "$PHONE_ARM_TRAJECTORY_CSV" "$METRICS" \
            "$WHIPINTO_LOG" "$WHIP_FFMPEG_LOG" "$VIDEO_SUPERVISOR_LOG"; do
  if [ -f "$path" ]; then
    ts=$(date -r "$path" +%Y%m%d_%H%M%S 2>/dev/null || date +%Y%m%d_%H%M%S)
    mv "$path" "$path.$ts"
  fi
done

_meminfo_kb() {
  awk -v key="$1:" '$1 == key { print $2; found = 1; exit } END { if (!found) print 0 }' /proc/meminfo
}

_df_avail_kb() {
  df -Pk "$1" 2>/dev/null | awk 'NR == 2 { print $4; found = 1; exit } END { if (!found) print 0 }'
}

_pi_temp_c() {
  local temp=""
  if command -v vcgencmd >/dev/null 2>&1; then
    temp=$(vcgencmd measure_temp 2>/dev/null | sed -n "s/^temp=\([0-9.]*\).*/\1/p" || true)
  fi
  if [ -z "$temp" ] && [ -r /sys/class/thermal/thermal_zone0/temp ]; then
    temp=$(awk '{ printf "%.1f", $1 / 1000.0 }' /sys/class/thermal/thermal_zone0/temp)
  fi
  printf '%s' "$temp"
}

_throttled_flags() {
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd get_throttled 2>/dev/null | sed -n 's/^throttled=//p' || true
  fi
}

_teleop_python_stats() {
  local pids="" rss_kb=0 vmsize_kb=0 threads=0 proc_stats=""
  local proc pid cmd rss vms th
  for proc in /proc/[0-9]*; do
    if ! grep -azqx "PHONE_ARM_RUN_ID=$PHONE_ARM_RUN_ID" "$proc/environ" 2>/dev/null; then
      continue
    fi
    cmd=$(cat "$proc/cmdline" 2>/dev/null | tr '\0' ' ' || true)
    case "$cmd" in
      *"$PYTHON_BIN"*) ;;
      *) continue ;;
    esac
    pid=${proc##*/}
    rss=$(awk '$1 == "VmRSS:" { print $2; found = 1; exit } END { if (!found) print 0 }' "$proc/status" 2>/dev/null || echo 0)
    vms=$(awk '$1 == "VmSize:" { print $2; found = 1; exit } END { if (!found) print 0 }' "$proc/status" 2>/dev/null || echo 0)
    th=$(awk '$1 == "Threads:" { print $2; found = 1; exit } END { if (!found) print 0 }' "$proc/status" 2>/dev/null || echo 0)
    pids="${pids:+$pids;}$pid"
    proc_stats="${proc_stats:+$proc_stats;}$pid:$rss:$vms:$th"
    rss_kb=$((rss_kb + rss))
    vmsize_kb=$((vmsize_kb + vms))
    threads=$((threads + th))
  done
  printf '%s,%s,%s,%s,%s' "$pids" "$rss_kb" "$vmsize_kb" "$threads" "$proc_stats"
}

_start_metrics_logger() {
  set +e
  local metrics_path="$1"
  local interval_s="${PHONE_ARM_METRICS_INTERVAL_S:-5}"
  local sync_rows="${PHONE_ARM_METRICS_SYNC:-1}"
  printf '%s\n' \
    "timestamp_iso,epoch_s,run_id,uptime_s,load1,load5,load15,temp_c,throttled,mem_available_kb,mem_free_kb,swap_free_kb,dirty_kb,writeback_kb,root_avail_kb,log_avail_kb,teleop_pids,teleop_rss_kb,teleop_vmsize_kb,teleop_threads,teleop_proc_stats" \
    >"$metrics_path"
  while true; do
    read -r uptime_s _ </proc/uptime
    read -r load1 load5 load15 _ </proc/loadavg
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
      "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
      "$(date +%s)" \
      "$PHONE_ARM_RUN_ID" \
      "$uptime_s" \
      "$load1" \
      "$load5" \
      "$load15" \
      "$(_pi_temp_c)" \
      "$(_throttled_flags)" \
      "$(_meminfo_kb MemAvailable)" \
      "$(_meminfo_kb MemFree)" \
      "$(_meminfo_kb SwapFree)" \
      "$(_meminfo_kb Dirty)" \
      "$(_meminfo_kb Writeback)" \
      "$(_df_avail_kb /)" \
      "$(_df_avail_kb "$LOG_DIR")" \
      "$(_teleop_python_stats)" \
      >>"$metrics_path"
    if [ "$sync_rows" = "1" ]; then
      sync -d "$metrics_path" 2>/dev/null || true
    fi
    sleep "$interval_s"
  done
}

_video_log() {
  printf '%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" | tee -a "$VIDEO_SUPERVISOR_LOG"
}

_stop_whip_children() {
  local whip_pid="${1:-}" pub_pid="${2:-}"
  if [ -n "$pub_pid" ]; then
    kill "$pub_pid" 2>/dev/null || true
  fi
  if [ -n "$whip_pid" ]; then
    kill "$whip_pid" 2>/dev/null || true
  fi
  [ -n "$pub_pid" ] && wait "$pub_pid" 2>/dev/null || true
  [ -n "$whip_pid" ] && wait "$whip_pid" 2>/dev/null || true
}

_start_whip_publisher_supervisor() {
  set +e
  local dev_args="$1"
  local dev_label="$2"
  local whip_url="$3"
  local token="$4"
  local attempt=0
  local backoff_s="${PHONE_ARM_WHIP_RESTART_INITIAL_S:-1}"
  local max_backoff_s="${PHONE_ARM_WHIP_RESTART_MAX_S:-10}"
  local bind_delay_s="${PHONE_ARM_WHIP_BIND_DELAY_S:-1}"
  local whip_pid=""
  local pub_pid=""
  local wait_status=0
  local exited="unknown"

  trap '_stop_whip_children "$whip_pid" "$pub_pid"; exit 0' TERM INT EXIT

  if ! command -v whipinto >/dev/null 2>&1; then
    _video_log "[whip] whipinto not found; publisher disabled"
    return 0
  fi
  if ! command -v ffmpeg >/dev/null 2>&1; then
    _video_log "[whip] ffmpeg not found; publisher disabled"
    return 0
  fi

  while true; do
    attempt=$((attempt + 1))
    _video_log "[whip] start attempt=$attempt source=$dev_label url=$whip_url"
    {
      printf '\n=== %s attempt=%s source=%s ===\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$attempt" "$dev_label"
    } >>"$WHIPINTO_LOG"
    {
      printf '\n=== %s attempt=%s source=%s ===\n' \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$attempt" "$dev_label"
    } >>"$WHIP_FFMPEG_LOG"

    whipinto --input rtsp-listen://127.0.0.1:8554 \
             --whip "$whip_url" --token "$token" \
             >>"$WHIPINTO_LOG" 2>&1 &
    whip_pid=$!

    # Give whipinto a beat to bind its RTSP listen socket before ffmpeg
    # starts pushing.
    sleep "$bind_delay_s"

    # libx264 baseline needs YUV 4:2:0, but v4l2 MJPG decodes to 4:2:2
    # (yuvj422p). Explicit -vf format=yuv420p downscales the chroma so the
    # baseline profile works; without it libx264 refuses to open with
    # "baseline profile doesn't support 4:2:2".
    ffmpeg -hide_banner -loglevel warning -nostdin \
           $dev_args \
           -map 0:v:0 \
           -vf format=yuv420p \
           -flags +global_header \
           -c:v libx264 -preset ultrafast -tune zerolatency \
             -profile:v baseline -g 15 -keyint_min 15 -sc_threshold 0 \
             -x264-params repeat-headers=1 -b:v 800k \
           -f rtsp -rtsp_transport udp rtsp://127.0.0.1:8554/stream \
           >>"$WHIP_FFMPEG_LOG" 2>&1 &
    pub_pid=$!

    _video_log "[whip] running attempt=$attempt whipinto_pid=$whip_pid ffmpeg_pid=$pub_pid"
    wait -n "$whip_pid" "$pub_pid"
    wait_status=$?
    if ! kill -0 "$whip_pid" 2>/dev/null; then
      exited="whipinto"
    elif ! kill -0 "$pub_pid" 2>/dev/null; then
      exited="ffmpeg"
    else
      exited="unknown"
    fi
    _video_log "[whip] $exited stopped attempt=$attempt status=$wait_status; restarting in ${backoff_s}s"

    _stop_whip_children "$whip_pid" "$pub_pid"
    whip_pid=""
    pub_pid=""
    sleep "$backoff_s"
    if [ "$backoff_s" -lt "$max_backoff_s" ]; then
      backoff_s=$((backoff_s * 2))
      if [ "$backoff_s" -gt "$max_backoff_s" ]; then
        backoff_s="$max_backoff_s"
      fi
    fi
  done
}

# Operator page/signaling are served through the London VPS
# (188.166.154.201). Video remains WebRTC relay-only, but the Pi and phone can
# use different TURN edges: Pi uses London, while the operator phone defaults to
# Singapore below. TURN creds are REQUIRED for video -- the server refuses to
# start video without them. Password is read from ~/.turn_secret (not committed).
# Pi uses London coturn directly (~4ms RTT) for its own video relay candidate.
# The operator's phone hits the Singapore coturn instead -- see
# PHONE_ARM_TURN_URL_PHONE below. Standard TURN permissions handle the
# cross-server traffic: when one side sends to the other's relay-candidate
# address, its coturn forwards UDP from the other coturn's external IP.
export PHONE_ARM_TURN_URL="${PHONE_ARM_TURN_URL:-turn:188.166.154.201:3478?transport=udp}"
export PHONE_ARM_TURN_URL_PHONE="${PHONE_ARM_TURN_URL_PHONE:-turn:146.190.104.81:3478?transport=udp}"
export PHONE_ARM_TURN_USER="${PHONE_ARM_TURN_USER:-teleop}"
export PHONE_ARM_TURN_PW="${PHONE_ARM_TURN_PW:-$(cat "$HOME/.turn_secret")}"

# Control commands ride WebTransport datagrams via the session relay. The
# relay keeps delivery latest-only and independent of the video/signaling
# path. Tokens are kept out of git; override these env vars to test different
# relays.
#
# Pi connects directly to the London relay over its local-ISP path (~4ms RTT).
# The operator's phone hits the Singapore forwarder (see
# PHONE_ARM_SESSION_RELAY_WT_URL_PHONE below) which tunnels into this same
# London relay over a persistent QUIC connection between the two DO
# datacenters.
export PHONE_ARM_SESSION_RELAY_WT_URL="${PHONE_ARM_SESSION_RELAY_WT_URL:-https://188-166-154-201.sslip.io:4433/wt}"
export PHONE_ARM_SESSION_RELAY_WT_URL_PHONE="${PHONE_ARM_SESSION_RELAY_WT_URL_PHONE:-https://146-190-104-81.sslip.io:4434/wt}"
export PHONE_ARM_SESSION_RELAY_SESSION="${PHONE_ARM_SESSION_RELAY_SESSION:-default}"
export PHONE_ARM_SESSION_RELAY_ARM_TOKEN="${PHONE_ARM_SESSION_RELAY_ARM_TOKEN:-$(cat "$HOME/.phone_arm_relay_arm_token")}"
export PHONE_ARM_SESSION_RELAY_PHONE_TOKEN="${PHONE_ARM_SESSION_RELAY_PHONE_TOKEN:-$(cat "$HOME/.phone_arm_relay_phone_token")}"

# MediaMTX SFU for robot video. /webrtc/config advertises the WHEP endpoint
# + subscriber token so the browser can subscribe directly to the SFU on the
# VPS. If the play secret is absent, the browser has no robot-video fallback.
if [ -r "$HOME/.phone_arm_secrets/mediamtx_play_pw" ]; then
  export PHONE_ARM_MEDIAMTX_WHEP_URL="${PHONE_ARM_MEDIAMTX_WHEP_URL:-https://188-166-154-201.sslip.io/robot/whep}"
  export PHONE_ARM_MEDIAMTX_PLAY_TOKEN="${PHONE_ARM_MEDIAMTX_PLAY_TOKEN:-$(cat "$HOME/.phone_arm_secrets/mediamtx_play_pw")}"
fi

# Create one short-lived browser link for this run. The unique run ID avoids
# replacing another operator's still-valid token, while purging keeps the local
# token file from accumulating expired startup entries forever.
echo "[access] creating a two-hour operator link"
"$PYTHON_BIN" -m follower.mint_token purge-expired
"$PYTHON_BIN" -m follower.mint_token mint \
  --name "startup-$PHONE_ARM_RUN_ID" \
  --expires 2h
echo

METRICS_PID=""
if [ "${PHONE_ARM_METRICS:-1}" != "0" ]; then
  _start_metrics_logger "$METRICS" &
  METRICS_PID=$!
  echo "[run] system metrics logging to $METRICS every ${PHONE_ARM_METRICS_INTERVAL_S:-5}s (run_id=$PHONE_ARM_RUN_ID)"
fi

# --- WHIP publisher (only when PHONE_ARM_MEDIAMTX_WHEP_URL is enabled) ---
# ffmpeg pulls MJPG frames off the camera, re-encodes as H.264 baseline
# (Chrome's HW-decode-friendly profile), pushes it over RTSP-UDP to a local
# whipinto instance.
WHIP_SUPERVISOR_PID=""
if [ -n "${PHONE_ARM_MEDIAMTX_WHEP_URL:-}" ] \
   && [ -r "$HOME/.phone_arm_secrets/mediamtx_publish_pw" ]; then
  MTX_PUB=$(cat "$HOME/.phone_arm_secrets/mediamtx_publish_pw")
  # Resolve the default video device via the follower gateway's camera logic
  # uses so the publisher tracks any camera reordering. Import-time
  # warnings go to stdout via third-party libs; take the LAST line only.
  DEV=$("$PYTHON_BIN" -c \
      'from follower import gateway; print(gateway.VIDEO_DEVICE)' 2>/dev/null | tail -1)
  # Override the auto-resolved device for testing without hardware
  # (e.g. `PHONE_ARM_WHIP_INPUT_ARGS="-f lavfi -i testsrc2=size=640x480:rate=30"`).
  if [ -n "${PHONE_ARM_WHIP_INPUT_ARGS:-}" ]; then
    DEV_ARGS="$PHONE_ARM_WHIP_INPUT_ARGS"
  elif [ -n "$DEV" ] && [ -e "$DEV" ]; then
    DEV_ARGS="-f v4l2 -input_format mjpeg -framerate 30 -video_size 640x480 -i $DEV"
  else
    echo "[whip] resolved device \"$DEV\" not present; skipping publisher"
    DEV_ARGS=""
    DEV=""
  fi
  if [ -n "$DEV_ARGS" ]; then
    WHIP_WHIP="${PHONE_ARM_MEDIAMTX_WHIP_URL:-https://188-166-154-201.sslip.io/robot/whip}"
    _start_whip_publisher_supervisor "$DEV_ARGS" "${DEV:-testsrc}" "$WHIP_WHIP" "$MTX_PUB" &
    WHIP_SUPERVISOR_PID=$!
    echo "[whip] supervisor pid=$WHIP_SUPERVISOR_PID log=$VIDEO_SUPERVISOR_LOG"
  fi
fi

_cleanup() {
  if [ -n "$WHIP_SUPERVISOR_PID" ]; then
    kill "$WHIP_SUPERVISOR_PID" 2>/dev/null || true
  fi
  if [ -n "$METRICS_PID" ]; then
    kill "$METRICS_PID" 2>/dev/null || true
    wait "$METRICS_PID" 2>/dev/null || true
  fi
  [ -n "$WHIP_SUPERVISOR_PID" ] && wait "$WHIP_SUPERVISOR_PID" 2>/dev/null || true
}
trap _cleanup EXIT

# Push the operator-facing static assets (index.html + app.js) to the VPS
# before teleop starts. This is intentionally foreground: if the phone loads
# while this is still running, Caddy can serve the previous app.js bundle.
STATIC_APP_SHA="$(sha256sum "$PWD/controllers/phone/app.js" 2>/dev/null | awk '{ print $1 }' || true)"
printf '[static] syncing operator assets to VPS app_schema=20260907b app_sha=%s\n' "${STATIC_APP_SHA:0:12}"
if timeout 15 rsync -e "ssh -i $HOME/.ssh/do_wg_relay -o StrictHostKeyChecking=no -o ConnectTimeout=5" \
      -az --checksum --delete \
      "$PWD/controllers/phone/" \
      root@188.166.154.201:/opt/phone_arm/static/ \
      >/dev/null 2>&1; then
  printf '[static] VPS static sync complete\n'
else
  printf '[static] WARNING: VPS static sync failed; operators may see stale app.js\n'
fi

# follower.main records per-frame phone / desired-EE / measured-EE to
# $PHONE_ARM_TRAJECTORY_CSV.
# Tee output so the user sees it AND it lands in $LOG.
set +e
"$PYTHON_BIN" -u -m follower.main "$@" 2>&1 | tee "$LOG"
status=${PIPESTATUS[0]}
set -e
exit "$status"
