// Phone Arm — WebXR 6DoF controller client.
//
// On Start:
//  1. Requests immersive-ar session with dom-overlay so our HTML controls
//     are available during the AR session.
//  2. Acquires a 'local' reference space so pose is relative to first frame.
//  3. Opens WebTransport control and a separate robot-video PC.
//  4. Each XR animation frame: read viewer pose, send a JSON message
//     matching the lerobot phone schema (phone.pos/phone.rot/raw_inputs/enabled).

const $ = (id) => document.getElementById(id);
const landing = $('landing');
const overlay = $('overlay');
const piCam = $('pi-cam');
const supportEl = $('xr-support');
const startBtn = $('start-btn');
const leaderBtn = $('leader-btn');
const leaderPanel = $('leader-panel');
const leaderCommandEl = $('leader-command');
const copyLeaderBtn = $('copy-leader-btn');
const watchLeaderBtn = $('watch-leader-btn');
const stopBtn = $('stop-btn');
const b1El = $('b1');
const gripperWrap = $('gripper-wrap');
const gripperThumb = $('gripper-thumb');
const statusEl = $('status');
const bannerEl = $('banner');
const followMeterEl = $('follow-meter');
const followValueEl = $('follow-value');
const followFillEl = $('follow-fill');
const logEl = $('log');

// Keep this short because it is also stamped onto control datagrams.
const APP_SCHEMA_ID = '20260907b';
const APP_BOOT_MS = Date.now();
const APP_SCRIPT_SRC = document.currentScript ? document.currentScript.src : '';
const APP_PAGE_ID = (() => {
  try {
    if (crypto && typeof crypto.randomUUID === 'function') {
      return crypto.randomUUID().slice(0, 13);
    }
  } catch (_) {}
  return `${APP_BOOT_MS.toString(36)}-${Math.random().toString(36).slice(2, 7)}`;
})();
const URL_PARAMS = new URLSearchParams(location.search);
const FORCE_VIEWER = URL_PARAMS.get('viewer') === '1' || URL_PARAMS.get('role') === 'viewer';
let xrSupported = false;
let clientRole = FORCE_VIEWER ? 'viewer' : 'unknown';
let _controlClaimActive = false;

function appEventFields(now = Date.now()) {
  return {
    app_schema_id: APP_SCHEMA_ID,
    app_page_id: APP_PAGE_ID,
    app_boot_ms: APP_BOOT_MS,
    app_uptime_ms: now - APP_BOOT_MS,
    app_script_src: APP_SCRIPT_SRC,
    app_role: clientRole,
  };
}

// --- Auth token -----------------------------------------------------------
// Token lifecycle: use only the ?t=TOKEN from the current URL. Do not persist
// worker tokens in browser storage; a bare URL may load the landing page, but
// protected robot endpoints must fail without an explicit token.
let pageAuthToken = '';
(function loadTokenFromUrl() {
  const params = new URLSearchParams(location.search);
  const fromUrl = params.get('t');
  if (fromUrl) {
    pageAuthToken = fromUrl;
    try { localStorage.removeItem('phoneArmToken'); } catch (_) {}
  }
})();
function authToken() { return pageAuthToken; }
function withToken(url) {
  const t = authToken();
  if (!t) return url;
  return url + (url.includes('?') ? '&' : '?') + 't=' + encodeURIComponent(t);
}

function configUrl(extra = {}) {
  const params = new URLSearchParams({
    page_id: APP_PAGE_ID,
    app_v: APP_SCHEMA_ID,
    ...extra,
  });
  return withToken(`/webrtc/config?${params.toString()}`);
}

function setClientRole(role, reason = '') {
  clientRole = role;
  overlay.classList.toggle('viewer', role === 'viewer');
  if (role === 'viewer') {
    setB1(false, reason || 'viewer');
  }
}

// --- Robot camera video + pose control ------------------------------------
let _controlWt = null;       // WebTransport datagram control channel
let _controlWtWriter = null;
let _controlWtReader = null;
let _controlWtReady = false;
let _controlLatestPose = null;          // one app-level latest-state register
let _controlLatestPoseQueuedPerfMs = 0;
let _controlWtFlushWaiting = false;     // true while waiting for writer.ready
let _controlWtFlushTimer = null;
let _controlWtFlushTimerDueMs = 0;
let _controlLastSendPerfMs = 0;
// WebTransport telemetry counters (reset on each new WT session start)
let _controlWtWriteCount = 0;          // total pose write attempts
let _controlDebugPostCount = 0;        // best-effort HTTP debug post attempts
let _controlDebugPostFailCount = 0;
let _controlLastPoseSeqSent = null;
let _controlLastPoseTBrowserMs = null;
let _controlWtWriterBlockedMs = 0;     // cumulative ms spent in writer.ready
let _controlWtWriterBlockedCount = 0;  // # times writer.ready took >5ms to resolve
let _controlWtRecvCount = 0;           // total incoming datagrams (welcome+keepalive+ack)
let _controlWtLastRecvMs = 0;          // Date.now() at last recv (0 = never)
let _controlWtSessionStartMs = 0;      // Date.now() at wt.ready
// Browser/XR pose-source diagnostics. These tell us whether an outage started
// before the network send path (XR/browser stopped generating) or after it.
let _poseSourceStatsTimer = null;
let _poseSourceSessionStartMs = 0;
let _poseSourceFrameCount = 0;
let _poseSourcePoseValidCount = 0;
let _poseSourcePoseInvalidCount = 0;
let _poseSourceGeneratedCount = 0;
let _poseSourceGeneratedB1Count = 0;
let _poseSourceNoControlDropCount = 0;
let _poseSourceBufferedDropCount = 0;
let _poseSourceRateLimitCount = 0;
let _poseSourceWtQueuedCount = 0;
let _poseSourceOversizeDropCount = 0;
let _poseSourceLastFrameWallMs = 0;
let _poseSourceLastFramePerfMs = 0;
let _poseSourceLastGeneratedMs = 0;
let _poseSourceLastSentMs = 0;
let _poseSourceLastNoControlMs = 0;
let _poseSourceLastGapMs = 0;
let _poseSourceMaxGapMs = 0;
let _poseSourceLongGap250Count = 0;
let _poseSourceLongGap1000Count = 0;
let _poseSourceLastStats = null;
// Per-frame timing diagnostics — lets us distinguish "callback fires late
// (external scheduler / thermal / ARCore throttle)" from "callback runs slow
// (our code / GC / chrome internals)". Reset on each post.
let _xrCallbackDurationSumMs = 0;
let _xrCallbackDurationMaxMs = 0;
let _xrCallbackCount = 0;
let _xrCallbackGapSumMs = 0;
let _xrCallbackGapMaxMs = 0;
let _xrLastCallbackPerfMs = 0;
// Main-thread/runtime diagnostics. XR frame gaps tell us that pose production
// slowed; these counters tell us whether Chrome's JS main thread was stalled
// at the same time.
let _mainLongTaskObserver = null;
let _mainLongTaskCount = 0;
let _mainLongTaskTotalMs = 0;
let _mainLongTaskMaxMs = 0;
let _mainLongTaskWindowMaxMs = 0;
let _mainLoopLagTimer = null;
let _mainLoopLagExpectedMs = 0;
let _mainLoopLagCount = 0;
let _mainLoopLagSumMs = 0;
let _mainLoopLagMaxMs = 0;
let _mainLoopLagOver50Count = 0;
let _mainLoopLagOver250Count = 0;
// Compute Pressure API — chrome 125+ on Android exposes CPU pressure as
// nominal/fair/serious/critical. If this transitions while XR rate drops,
// thermal throttling is the cause.
let _cpuPressureLatest = null;          // current pressure state
let _cpuPressureChangeCount = 0;
let _cpuPressureStateCounts = { nominal: 0, fair: 0, serious: 0, critical: 0 };
// Battery state from navigator.getBattery() — discharge rate + charging
// status as a secondary thermal signal.
let _batteryLevel = null;
let _batteryCharging = null;
let _batteryDischargingTimeS = null;
let _controlPoseCount = 0;   // WT pose datagrams accepted for send this session
let _controlShouldReconnect = false;
let _controlStarting = false;
let _controlReconnectDelay = 500;
let _controlReconnectTimer = null;
let _controlHealthTimer = null;
let _controlLastRestartMs = 0;
let _controlConnectStartedMs = 0;
let _controlState = 'down';
let _controlConnectSeq = 0;
let _controlReconnectCount = 0;
let _controlLastReconnectReason = '';
let serverGateState = 'ok';
let serverGateSeverity = 'ok';
let serverGateMessage = '';
let serverGateRequiresReengage = false;
let serverGateReason = '';
let serverGateLastMs = 0;
let robotTrackingErrorM = null;
let robotTrackingCommandErrorM = null;
let robotTrackingGoalErrorM = null;
let robotTrackingAgeMs = null;
let robotTrackingSeq = null;
let robotTrackingPhoneEnabled = false;
let robotTrackingErrorSource = '';
let robotTrackingLastMs = 0;
const ROBOT_TRACKING_WARN_M = 0.03;
const ROBOT_TRACKING_BAD_M = 0.08;
const ROBOT_TRACKING_BAR_MAX_M = 0.12;
const ROBOT_TRACKING_STALE_MS = 1500;
const VIDEO_RECONNECT_MIN_MS = 1000;
const VIDEO_RECONNECT_MAX_MS = 5000;
const video = {
  pc: null,                 // WebRTC video receiver (camera feed from Pi)
  shouldReconnect: false,
  starting: false,
  reconnectTimer: null,
  reconnectDelay: VIDEO_RECONNECT_MIN_MS,
  reconnectCount: 0,
  lastReconnectReason: '',
  state: 'down',
  lastError: '',

  clearReconnectTimer() {
    if (!video.reconnectTimer) return;
    clearTimeout(video.reconnectTimer);
    video.reconnectTimer = null;
  },

  scheduleReconnect(reason, immediate = false) {
    if (!video.shouldReconnect) return;
    video.clearReconnectTimer();
    video.lastReconnectReason = reason || '';
    video.reconnectCount++;
    video.state = 'retry';
    const delayMs = immediate ? 0 : video.reconnectDelay;
    log(`robot video retry in ${delayMs}ms (${reason})`);
    video.reconnectTimer = setTimeout(() => {
      video.reconnectTimer = null;
      video.start();
    }, delayMs);
    if (!immediate && video.reconnectDelay < VIDEO_RECONNECT_MAX_MS) {
      video.reconnectDelay = Math.min(VIDEO_RECONNECT_MAX_MS, video.reconnectDelay * 2);
    }
  },

  closePeer() {
    const pc = video.pc;
    video.pc = null;
    piCam.classList.remove('active');
    if (_statsTimer) { clearInterval(_statsTimer); _statsTimer = null; }
    webrtcStats = null;
    _prevStats = null;
    _displayedRtpTs = null;
    _tOpDisplayedMs = null;
    try { if (pc) pc.close(); } catch (e) {}
    try { piCam.srcObject = null; piCam.load(); } catch (e) {}
  },

  async start() {
    if (video.starting) return;
    video.shouldReconnect = true;
    video.starting = true;
    video.clearReconnectTimer();
    video.state = 'config';
    video.lastError = '';
    try {
      const cfg = await fetch(configUrl({ role: 'viewer' }), { cache: 'no-store' })
        .then(r => {
          if (!r.ok) throw new Error(`config ${r.status}`);
          return r.json();
        });
      if (!cfg.mediamtxWhepUrl || !cfg.mediamtxPlayToken) {
        throw new Error('config missing WHEP video endpoint');
      }
      if (!video.shouldReconnect) return;
      log('robot video: WHEP subscribe (MediaMTX)');
      const ok = await startVideoWHEP(cfg.mediamtxWhepUrl, cfg.mediamtxPlayToken,
                                     cfg.iceServers || [], cfg.iceTransportPolicy || 'relay');
      if (!ok) {
        video.scheduleReconnect(video.lastError || 'WHEP subscribe failed');
      }
    } catch (e) {
      if (!video.shouldReconnect) return;
      video.state = 'error';
      video.lastError = e.message || String(e);
      log(`robot video config failed: ${video.lastError}`);
      video.scheduleReconnect(video.lastError);
    } finally {
      video.starting = false;
    }
  },

  stop() {
    video.shouldReconnect = false;
    video.starting = false;
    video.clearReconnectTimer();
    video.reconnectDelay = VIDEO_RECONNECT_MIN_MS;
    video.lastReconnectReason = '';
    video.state = 'down';
    video.lastError = '';
    video.closePeer();
  },

  statusText() {
    if (webrtcStats) {
      const w = webrtcStats;
      return `vid ${w.fps}fps rtt=${w.rtt} jb=${w.jbuf} dec=${w.decode} jit=${w.jitter}ms`;
    }
    const err = video.lastError ? ` ${video.lastError}` : '';
    return `robot video ${video.state}${err}`;
  },
};
let webrtcStats = null;
let _statsTimer = null;
let _prevStats = null;       // previous cumulative counters, for per-interval deltas
const CONTROL_NO_LINK_MS = 1000;
// relay-webtransport gets a relaxed staleness threshold because the server
// sends a keepalive every 3s -- 8s = 2 missed keepalives + slack.
const CONTROL_RESTART_WT_LINK_MS = 8000;
const CONTROL_CONNECT_TIMEOUT_MS = 10000;
const CONTROL_HEALTH_CHECK_MS = 500;
const SERVER_GATE_STALE_MS = 1500;
const WT_DATAGRAM_MAX_BYTES = 1024;
const WT_OUTGOING_MAX_AGE_MS = 75;
const WT_INCOMING_MAX_AGE_MS = 100;
const CONTROL_TARGET_HZ = 30;
const CONTROL_MIN_SEND_INTERVAL_MS = 1000.0 / CONTROL_TARGET_HZ;
const CLOCK_OFFSET_MAX_RTT_MS = 750;
const CLOCK_OFFSET_MAX_STEP_MS = 100;
let _controlSkippedSinceLastSend = 0;
let _controlTransport = 'relay-webtransport';
const _controlTextEncoder = new TextEncoder();
const _controlTextDecoder = new TextDecoder();
// Operator-display correlation. requestVideoFrameCallback gives us, per
// displayed frame, the source RTP timestamp and browser-clock display time.
// We snapshot the most recent pair and ship it with every pose so the Pi can
// log "operator commanded action X while looking at frame Y."
let _displayedRtpTs = null;        // RTP timestamp of most recently displayed video frame
let _tOpDisplayedMs = null;        // browser Date.now() at the moment that frame was scheduled to display
// Relay-only: WebRTC media always goes through the VPS TURN relay. There is no
// direct/LAN path; the server only ever returns 'relay' here.
let iceTransportPolicy = 'relay';

function pollWebRTCStats() {
  if (!video.pc) return;
  video.pc.getStats().then(s => {
    const cur = { jbd: 0, jbc: 0, dt: 0, dec: 0, fps: 0, rtt: 0, jit: 0, frz: 0,
                  pl: 0, pr: 0 };
    // Chrome getStats exposes decoderImplementation (string like
    // "MediaCodecVideoDecoder" for HW, "openh264"/"libvpx"/etc. for SW)
    // and powerEfficientDecoder (bool). Together they let us tell whether
    // this operator's phone is on the hardware decoder path, which
    // completely changes the thermal picture. Same for packetsLost/
    // packetsReceived -- gives us the real packet-loss numerator for
    // per-hop investigation.
    let lc = null, via = '';
    let decoderImpl = '';
    let powerEfficient = null;
    s.forEach(r => {
      if (r.type === 'inbound-rtp' && r.kind === 'video') {
        cur.jbd = r.jitterBufferDelay || 0; cur.jbc = r.jitterBufferEmittedCount || 0;
        cur.dt = r.totalDecodeTime || 0; cur.dec = r.framesDecoded || 0;
        cur.fps = r.framesPerSecond || 0; cur.jit = r.jitter || 0; cur.frz = r.freezeCount || 0;
        cur.pl = r.packetsLost || 0; cur.pr = r.packetsReceived || 0;
        if (typeof r.decoderImplementation === 'string') decoderImpl = r.decoderImplementation;
        if (typeof r.powerEfficientDecoder === 'boolean') powerEfficient = r.powerEfficientDecoder;
      }
      if (r.type === 'candidate-pair' && (r.nominated || r.selected)) {
        cur.rtt = r.currentRoundTripTime || cur.rtt; lc = r.localCandidateId;
      }
    });
    s.forEach(r => { if (r.id === lc) via = r.candidateType; });
    // Per-interval deltas, not lifetime averages -- so the numbers track CURRENT
    // conditions when you change the lag/playout knobs mid-session.
    const p = _prevStats, prev = webrtcStats;
    const djbc = p ? cur.jbc - p.jbc : 0;
    const ddec = p ? cur.dec - p.dec : 0;
    webrtcStats = {
      rtt: Math.round(cur.rtt * 1000),
      jbuf: djbc > 0 ? Math.round((cur.jbd - p.jbd) / djbc * 1000) : (prev ? prev.jbuf : 0),
      decode: ddec > 0 ? Math.round((cur.dt - p.dt) / ddec * 1000) : (prev ? prev.decode : 0),
      jitter: Math.round(cur.jit * 1000),
      fps: Math.round(cur.fps),
      freezes: cur.frz,
      via: via || '?',
      decoderImpl: decoderImpl || '',
      powerEfficient: powerEfficient == null ? null : (powerEfficient ? 1 : 0),
      packetsLost: cur.pl,
      packetsReceived: cur.pr,
    };
    _prevStats = cur;
    updateViewerStatus();
  }).catch(() => {});
}

function videoStatusText() {
  return video.statusText();
}

function updateViewerStatus() {
  if (clientRole !== 'viewer' || xrSession || !overlay.classList.contains('active')) return;
  statusEl.textContent = `viewer\n${videoStatusText()}`;
}

function startVideo() {
  return video.start();
}

function resetServerGateStatus() {
  serverGateState = 'ok';
  serverGateSeverity = 'ok';
  serverGateMessage = '';
  serverGateRequiresReengage = false;
  serverGateReason = '';
  serverGateLastMs = 0;
}

function noteServerGateStatus(m, now) {
  if (!m || m.server_gate_state == null) return;
  serverGateState = String(m.server_gate_state || 'ok');
  serverGateSeverity = (m.server_gate_severity === 'bad' || m.server_gate_severity === 'warn')
    ? m.server_gate_severity
    : 'ok';
  serverGateMessage = typeof m.server_gate_message === 'string' ? m.server_gate_message : '';
  serverGateRequiresReengage = !!m.server_gate_requires_reengage;
  serverGateReason = typeof m.server_gate_reason === 'string' ? m.server_gate_reason : '';
  serverGateLastMs = now;
}

function finiteNumber(value) {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function resetRobotTrackingStatus() {
  robotTrackingErrorM = null;
  robotTrackingCommandErrorM = null;
  robotTrackingGoalErrorM = null;
  robotTrackingAgeMs = null;
  robotTrackingSeq = null;
  robotTrackingPhoneEnabled = false;
  robotTrackingErrorSource = '';
  robotTrackingLastMs = 0;
  updateRobotTrackingMeter();
}

function noteRobotTrackingStatus(m, now) {
  if (!m || !Object.prototype.hasOwnProperty.call(m, 'robot_tracking_error_m')) return;
  robotTrackingErrorM = finiteNumber(m.robot_tracking_error_m);
  robotTrackingCommandErrorM = finiteNumber(m.robot_tracking_command_error_m);
  robotTrackingGoalErrorM = finiteNumber(m.robot_tracking_goal_error_m);
  robotTrackingAgeMs = finiteNumber(m.robot_tracking_age_ms);
  robotTrackingSeq = m.robot_tracking_seq == null ? null : m.robot_tracking_seq;
  robotTrackingPhoneEnabled = !!m.robot_tracking_phone_enabled;
  robotTrackingErrorSource = typeof m.robot_tracking_error_source === 'string'
    ? m.robot_tracking_error_source
    : '';
  robotTrackingLastMs = now;
}

function updateRobotTrackingMeter() {
  if (!followMeterEl || !followValueEl || !followFillEl) return;
  const now = Date.now();
  const ageMs = robotTrackingLastMs ? now - robotTrackingLastMs : Infinity;
  const feedbackAgeMs = robotTrackingAgeMs == null ? Infinity : robotTrackingAgeMs;
  const fresh = robotTrackingErrorM != null
    && ageMs <= ROBOT_TRACKING_STALE_MS
    && feedbackAgeMs <= ROBOT_TRACKING_STALE_MS;
  let kind = 'idle';
  let label = '--';
  let widthPct = 0;
  if (fresh) {
    const errMm = Math.max(0, robotTrackingErrorM * 1000.0);
    label = `${Math.round(errMm)} mm`;
    widthPct = Math.min(100, Math.max(3, (robotTrackingErrorM / ROBOT_TRACKING_BAR_MAX_M) * 100));
    if (robotTrackingErrorM >= ROBOT_TRACKING_BAD_M) kind = 'bad';
    else if (robotTrackingErrorM >= ROBOT_TRACKING_WARN_M) kind = 'warn';
    else kind = 'ok';
  } else if (b1Held) {
    kind = 'stale';
  }
  followValueEl.textContent = label;
  followFillEl.style.width = `${widthPct.toFixed(0)}%`;
  for (const cls of ['idle', 'ok', 'warn', 'bad', 'stale']) {
    followMeterEl.classList.toggle(cls, cls === kind);
  }
}

// WHEP subscriber (WebRTC-HTTP Egress Protocol, RFC 9725). Standard
// "POST an SDP offer, get an SDP answer" HTTP flow. Auth is a Bearer
// token that Caddy validates before proxying to MediaMTX. The Pi is
// not involved in video signaling at all in this path -- signaling
// terminates on the VPS and media flows SFU->phone through TURN.
async function startVideoWHEP(whepUrl, token, iceServers, policy) {
  let pc = null;
  try {
    video.closePeer();
    iceTransportPolicy = policy;
    video.state = 'ice';
    pc = new RTCPeerConnection({ iceServers, iceTransportPolicy: policy });
    video.pc = pc;
    // Recv-only video track from the SFU.
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.ontrack = (e) => {
      if (pc !== video.pc) return;
      piCam.srcObject = e.streams[0];
      piCam.classList.add('active');
      try { e.receiver.playoutDelayHint = 0; } catch (_) {}
      video.state = 'active';
      video.lastError = '';
      video.reconnectDelay = VIDEO_RECONNECT_MIN_MS;
      video.clearReconnectTimer();
      updateViewerStatus();
      log('robot video active (WHEP)');
      // Preserve per-displayed-frame RTP timing so pose rows can be correlated
      // with operator-side video display timing after a run.
      if (typeof piCam.requestVideoFrameCallback === 'function') {
        const cb = (now, metadata) => {
          if (pc !== video.pc) return;
          if (metadata && metadata.rtpTimestamp != null) {
            _displayedRtpTs = metadata.rtpTimestamp;
            const ref = (metadata.expectedDisplayTime != null)
              ? metadata.expectedDisplayTime : now;
            _tOpDisplayedMs = Date.now() + (ref - performance.now());
          }
          piCam.requestVideoFrameCallback(cb);
        };
        piCam.requestVideoFrameCallback(cb);
      }
    };
    const noteConnectionState = () => {
      if (pc !== video.pc) return;
      const pcState = pc.connectionState || '';
      const iceState = pc.iceConnectionState || '';
      log(`robot video WHEP pc=${pcState || '?'} ice=${iceState || '?'}`);
      if (pcState === 'connected' || iceState === 'connected' || iceState === 'completed') {
        video.state = video.state === 'active' ? 'active' : 'connected';
        video.lastError = '';
        video.reconnectDelay = VIDEO_RECONNECT_MIN_MS;
        video.clearReconnectTimer();
        updateViewerStatus();
      } else if (pcState === 'failed' || iceState === 'failed' || pcState === 'closed') {
        video.state = 'error';
        video.lastError = `pc ${pcState || iceState}`;
        video.closePeer();
        updateViewerStatus();
        video.scheduleReconnect(video.lastError);
      } else if (pcState === 'disconnected' || iceState === 'disconnected') {
        video.state = 'disconnected';
        video.lastError = '';
        updateViewerStatus();
        video.scheduleReconnect('pc disconnected');
      }
    };
    pc.onconnectionstatechange = noteConnectionState;
    pc.oniceconnectionstatechange = noteConnectionState;
    if (_statsTimer) clearInterval(_statsTimer);
    _statsTimer = setInterval(pollWebRTCStats, 1000);

    await pc.setLocalDescription(await pc.createOffer());
    await waitForUsableIce(pc, policy, 'robot video WHEP');
    if (pc !== video.pc) return false;

    video.state = 'offer';
    const resp = await fetch(whepUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/sdp',
        'Authorization': `Bearer ${token}`,
      },
      body: pc.localDescription.sdp,
    });
    if (!resp.ok) throw new Error(`WHEP offer ${resp.status}`);
    if (pc !== video.pc) return false;
    const answerSdp = await resp.text();
    await pc.setRemoteDescription({ type: 'answer', sdp: answerSdp });
    if (pc !== video.pc) return false;
    video.state = 'connecting';
    return true;
  } catch (e) {
    if (!video.shouldReconnect || (pc && pc !== video.pc)) return false;
    video.state = 'error';
    video.lastError = e.message || String(e);
    log(`robot video WHEP err ${video.lastError}`);
    if (!pc || pc === video.pc) video.closePeer();
    updateViewerStatus();
    return false;
  }
}

function noteControlDownlinkRx(now = Date.now()) {
  lastControlRxMs = now;
  _controlWtRecvCount++;
  _controlWtLastRecvMs = now;
}

function noteEdgeAck(m, now) {
  if (m.ack_t == null) return false;
  lastEdgeAckMs = now;
  const rtt = Math.max(0, now - m.ack_t);
  edgeLatencyAvgMs = noteLatencySample(edgeLatencyWindow, rtt);
  latencyAvgMs = edgeLatencyWindow.length ? edgeLatencyAvgMs : robotLatencyAvgMs;
  return true;
}

function noteRobotAck(m, now) {
  if (m.ack_t == null) return false;
  lastRobotAckMs = now;
  const rtt = Math.max(0, now - m.ack_t);
  robotLatencyAvgMs = noteLatencySample(robotLatencyWindow, rtt);
  noteLatencySample(latencyWindow, rtt);
  latencyAvgMs = edgeLatencyWindow.length ? edgeLatencyAvgMs : robotLatencyAvgMs;
  if (m.t_server_recv_ms != null && m.t_server_send_ms != null) {
    const off = ((m.t_server_recv_ms - m.ack_t) + (m.t_server_send_ms - now)) / 2;
    const plausible = rtt <= CLOCK_OFFSET_MAX_RTT_MS
      && (!clockOffsetReady || Math.abs(off - clockOffsetMs) <= CLOCK_OFFSET_MAX_STEP_MS);
    if (plausible) {
      clockOffsetMs = clockOffsetReady ? clockOffsetMs * 0.8 + off * 0.2 : off;
      clockOffsetReady = true;
    }
  }
  noteServerGateStatus(m, now);
  noteRobotTrackingStatus(m, now);
  return true;
}

function handleControlDownlinkMessage(m, now) {
  if (!m || typeof m !== 'object') return;
  if (m.type === 'relay_welcome') {
    log(`control webtransport welcome epoch=${m.epoch}`);
    return;
  }
  if (m.type === 'relay_keepalive') return;
  // edge_ack is app-level relay telemetry. QUIC/WebTransport transport ACKs
  // are separate and happen below this JavaScript-visible protocol.
  if (m.type === 'edge_ack') {
    noteEdgeAck(m, now);
    return;
  }
  noteRobotAck(m, now);
}

// Used by the video WebRTC offer path only; control runs over WebTransport.
function offerHasRelayCandidate(pc) {
  return / typ relay(\s|$)/.test(pc.localDescription?.sdp || '');
}

function waitForUsableIce(pc, policy, label = 'webrtc') {
  const wantsRelay = policy === 'relay';
  return new Promise((res, rej) => {
    const started = performance.now();
    const maxWaitMs = wantsRelay ? 15000 : 1500;
    const check = () => {
      if (wantsRelay && offerHasRelayCandidate(pc)) return done();
      if (pc.iceGatheringState === 'complete') {
        if (wantsRelay) {
          return fail(new Error(`${label} ICE complete without relay candidate`));
        }
        return done();
      }
      if (performance.now() - started >= maxWaitMs) {
        if (wantsRelay && !offerHasRelayCandidate(pc)) {
          return fail(new Error(`${label} no relay ICE candidate after ${maxWaitMs}ms`));
        }
        log(`${label} warning: ICE wait timeout`);
        return done();
      }
    };
    const done = () => {
      pc.removeEventListener('icegatheringstatechange', check);
      clearInterval(timer);
      if (wantsRelay) {
        log(`${label} relay candidate ready (ice=${pc.iceGatheringState})`);
      }
      res();
    };
    const fail = (err) => {
      pc.removeEventListener('icegatheringstatechange', check);
      clearInterval(timer);
      rej(err);
    };
    pc.addEventListener('icegatheringstatechange', check);
    const timer = setInterval(check, 100);
    check();
  });
}

function controlReady() {
  if (_controlTransport === 'relay-webtransport') {
    return _controlWtReady && _controlWt && _controlWtWriter;
  }
  return false;
}

function closeControlWebTransport() {
  _controlWtReady = false;
  if (_controlWtReader) {
    try { _controlWtReader.cancel(); } catch (e) {}
    try { _controlWtReader.releaseLock(); } catch (e) {}
  }
  if (_controlWtWriter) {
    try { _controlWtWriter.releaseLock(); } catch (e) {}
  }
  if (_controlWt) {
    try { _controlWt.close({ closeCode: 0, reason: 'control closed' }); } catch (e) {}
  }
  _controlWt = null;
  _controlWtWriter = null;
  _controlWtReader = null;
  _controlLatestPose = null;
  _controlLatestPoseQueuedPerfMs = 0;
  _controlWtFlushWaiting = false;
  if (_controlWtFlushTimer) {
    clearTimeout(_controlWtFlushTimer);
    _controlWtFlushTimer = null;
  }
  _controlWtFlushTimerDueMs = 0;
  _controlLastSendPerfMs = 0;
  _controlConnectStartedMs = 0;
  _controlState = 'down';
}

function scheduleControlReconnect(reason, immediate = false) {
  if (!_controlShouldReconnect || _controlReconnectTimer || _controlStarting) return;
  _controlReconnectCount++;
  _controlLastReconnectReason = reason;
  const d = immediate ? 0 : _controlReconnectDelay;
  log(`control reconnect in ${d}ms (${reason})`);
  _controlReconnectTimer = setTimeout(() => {
    _controlReconnectTimer = null;
    startControl();
  }, d);
  _controlReconnectDelay = Math.min(_controlReconnectDelay * 2, 4000);
}

function startControl() {
  if (_controlStarting) return;
  _controlShouldReconnect = true;
  if (!_controlHealthTimer) {
    _controlHealthTimer = setInterval(checkControlHealth, CONTROL_HEALTH_CHECK_MS);
  }
  _controlStarting = true;
  // WebTransport is the only control transport we support. A control claimant
  // must be granted the controller role before the server returns the WT URL.
  fetch(configUrl({ want_control: '1' }), { cache: 'no-store' })
    .then(r => {
      if (!r.ok) throw new Error(`config ${r.status}`);
      return r.json();
    })
    .then(cfg => {
      if (cfg.controlRole !== 'controller') {
        _controlStarting = false;
        _controlShouldReconnect = false;
        _controlState = 'viewer';
        if (_controlHealthTimer) {
          clearInterval(_controlHealthTimer);
          _controlHealthTimer = null;
        }
        const reason = cfg.controlDeniedReason || 'viewer';
        setClientRole('viewer', reason);
        log(`control viewer-only (${reason})`);
        return;
      }
      if (!cfg.sessionRelayWtUrl) {
        throw new Error(`config missing sessionRelayWtUrl (controlTransport=${cfg.controlTransport})`);
      }
      _controlTransport = 'relay-webtransport';
      _controlClaimActive = true;
      setClientRole('controller');
      startControlWebTransport(cfg.sessionRelayWtUrl, cfg.sessionRelaySession || 'default');
    })
    .catch((e) => {
      _controlStarting = false;
      log(`control config failed: ${e.message}`);
      scheduleControlReconnect('config failed');
    });
}

async function pollControlWebTransportStats(wt) {
  // Periodically snapshot WT counters + chrome-side QUIC stats (RTT, loss,
  // congestion window). Lets us tell from the operator events.jsonl whether
  // a session close was preceded by (a) writer.ready blocking, (b) silent
  // downlink loss, or (c) chrome-side BWE collapse.
  const intervalMs = 2000;
  let prevSent = 0, prevLost = 0, prevRecv = 0;
  while (wt === _controlWt) {
    const sample = {
      kind: 'wt_stats',
      ageSessionMs: _controlWtSessionStartMs ? (Date.now() - _controlWtSessionStartMs) : null,
      ackAgeMs: Math.round(controlRxAgeMs()),
      poseSent: _controlPoseCount,
      writeAttempts: _controlWtWriteCount,
      writerBlockedCount: _controlWtWriterBlockedCount,
      writerBlockedMs: Math.round(_controlWtWriterBlockedMs),
      recvCount: _controlWtRecvCount,
      lastRecvAgeMs: _controlWtLastRecvMs ? (Date.now() - _controlWtLastRecvMs) : null,
      edgeAckAgeMs: edgeAckAgeMs(),
      edgeRttMs: edgeLatencyWindow.length ? Math.round(edgeLatencyAvgMs) : null,
      robotAckAgeMs: robotAckAgeMs(),
      robotRttMs: robotLatencyWindow.length ? Math.round(robotLatencyAvgMs) : null,
      writerDesiredSize: _controlWtWriter ? _controlWtWriter.desiredSize : null,
      pending: _controlLatestPose ? 1 : 0,
      pendingAgeMs: _controlLatestPose ? (Date.now() - _controlLatestPose.t) : null,
      targetHz: CONTROL_TARGET_HZ,
      serverGateState,
      serverGateSeverity,
      serverGateReason,
      serverGateAgeMs: serverGateLastMs ? Math.round(Date.now() - serverGateLastMs) : null,
      robotTrackingErrorM,
      robotTrackingCommandErrorM,
      robotTrackingGoalErrorM,
      robotTrackingAgeMs,
      robotTrackingPhoneEnabled: robotTrackingPhoneEnabled ? 1 : 0,
      robotTrackingErrorSource,
      robotTrackingAckAgeMs: robotTrackingLastMs ? Math.round(Date.now() - robotTrackingLastMs) : null,
      lastSendAgeMs: _controlLastSendPerfMs
        ? Math.round(performance.now() - _controlLastSendPerfMs) : null,
    };
    // Chrome 124+ exposes WebTransport.getStats() with QUIC-level counters
    if (typeof wt.getStats === 'function') {
      try {
        const s = await wt.getStats();
        if (s) {
          sample.quic = {
            smoothedRtt: s.smoothedRtt ?? null,
            minRtt: s.minRtt ?? null,
            rttVariation: s.rttVariation ?? null,
            estimatedSendRateBps: s.estimatedSendRateBps ?? null,
            datagrams: s.datagrams ? {
              expiredOutgoing: s.datagrams.expiredOutgoing ?? null,
              droppedIncoming: s.datagrams.droppedIncoming ?? null,
              lostOutgoing: s.datagrams.lostOutgoing ?? null,
            } : null,
            packetsSent: s.packetsSent ?? null,
            packetsLost: s.packetsLost ?? null,
            packetsReceived: s.packetsReceived ?? null,
            d_packetsSent: (s.packetsSent ?? 0) - prevSent,
            d_packetsLost: (s.packetsLost ?? 0) - prevLost,
            d_packetsReceived: (s.packetsReceived ?? 0) - prevRecv,
          };
          prevSent = s.packetsSent ?? prevSent;
          prevLost = s.packetsLost ?? prevLost;
          prevRecv = s.packetsReceived ?? prevRecv;
        }
      } catch (_) {}
    }
    try {
      fetch(withToken('/test_event'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        cache: 'no-store',
        keepalive: true,
        body: JSON.stringify(sample),
      }).catch(() => {});
    } catch (_) {}
    await new Promise(r => setTimeout(r, intervalMs));
  }
}

async function readControlWebTransport(wt, reader) {
  try {
    while (wt === _controlWt) {
      const { value, done } = await reader.read();
      if (done) break;
      let text = '';
      try { text = _controlTextDecoder.decode(value); } catch (_) { continue; }
      // Any incoming datagram means the control downlink is alive, even if
      // the message is only a welcome/keepalive and has no pose ack fields.
      const now = Date.now();
      noteControlDownlinkRx(now);
      let m;
      try {
        m = JSON.parse(text);
      } catch (_) {
        continue;
      }
      handleControlDownlinkMessage(m, now);
    }
  } catch (e) {
    if (wt === _controlWt) log(`control webtransport read err: ${e.message || e}`);
  }
}

function startControlWebTransport(wtUrl, sessionName) {
  (async () => {
    closeControlWebTransport();
    if (!('WebTransport' in window)) {
      _controlStarting = false;
      log('control webtransport unsupported on this browser');
      scheduleControlReconnect('webtransport unsupported', true);
      return;
    }
    _controlState = 'connecting';
    _controlConnectStartedMs = Date.now();
    _controlConnectSeq++;
    resetServerGateStatus();
    _controlPoseCount = 0;
    _controlSkippedSinceLastSend = 0;
    _controlLatestPose = null;
    _controlLatestPoseQueuedPerfMs = 0;
    _controlWtFlushWaiting = false;
    if (_controlWtFlushTimer) {
      clearTimeout(_controlWtFlushTimer);
      _controlWtFlushTimer = null;
    }
    _controlWtFlushTimerDueMs = 0;
    _controlLastSendPerfMs = 0;
    _controlWtWriteCount = 0;
    _controlDebugPostCount = 0;
    _controlDebugPostFailCount = 0;
    _controlLastPoseSeqSent = null;
    _controlLastPoseTBrowserMs = null;
    _controlWtWriterBlockedMs = 0;
    _controlWtWriterBlockedCount = 0;
    _controlWtRecvCount = 0;
    _controlWtLastRecvMs = 0;
    _controlWtSessionStartMs = Date.now();
    const wt = new WebTransport(wtUrl, { congestionControl: 'low-latency' });
    _controlWt = wt;
    wt.closed.then((info) => {
      if (wt !== _controlWt) return;
      const sessionMs = Date.now() - _controlWtSessionStartMs;
      const closeDetail = {
        closeCode: info?.closeCode ?? null,
        reason: info?.reason ?? null,
        sessionMs,
        poseSent: _controlPoseCount,
        writeAttempts: _controlWtWriteCount,
        writerBlockedCount: _controlWtWriterBlockedCount,
        writerBlockedMs: _controlWtWriterBlockedMs,
        recvCount: _controlWtRecvCount,
        lastRecvAgeMs: _controlWtLastRecvMs ? (Date.now() - _controlWtLastRecvMs) : null,
        ackAgeMs: Math.round(controlRxAgeMs()),
      };
      _controlWtReady = false;
      _controlState = 'down';
      _controlStarting = false;
      log(`control webtransport closed ${JSON.stringify(closeDetail)}`);
      scheduleControlReconnect('webtransport closed');
    }).catch((e) => {
      if (wt !== _controlWt) return;
      const sessionMs = Date.now() - _controlWtSessionStartMs;
      const closeDetail = {
        errorMessage: String(e?.message || e),
        errorName: String(e?.name || ''),
        sessionMs,
        poseSent: _controlPoseCount,
        writeAttempts: _controlWtWriteCount,
        writerBlockedCount: _controlWtWriterBlockedCount,
        writerBlockedMs: _controlWtWriterBlockedMs,
        recvCount: _controlWtRecvCount,
        lastRecvAgeMs: _controlWtLastRecvMs ? (Date.now() - _controlWtLastRecvMs) : null,
      };
      _controlWtReady = false;
      _controlState = 'error';
      _controlStarting = false;
      log(`control webtransport closed-error ${JSON.stringify(closeDetail)}`);
      scheduleControlReconnect('webtransport error', true);
    });
    try {
      await wt.ready;
      if (wt !== _controlWt) return;
      const dgrams = wt.datagrams;
      try { dgrams.outgoingHighWaterMark = 1; } catch (_) {}
      try { dgrams.incomingHighWaterMark = 1; } catch (_) {}
      try { dgrams.outgoingMaxAge = WT_OUTGOING_MAX_AGE_MS; } catch (_) {}
      try { dgrams.incomingMaxAge = WT_INCOMING_MAX_AGE_MS; } catch (_) {}
      const writable = typeof dgrams.createWritable === 'function'
        ? dgrams.createWritable()
        : dgrams.writable;
      if (!writable || !dgrams.readable) throw new Error('datagram streams unavailable');
      _controlWtWriter = writable.getWriter();
      _controlWtReader = dgrams.readable.getReader();
      _controlWtReady = true;
      _controlState = 'ok';
      _controlReconnectDelay = 500;
      lastControlRxMs = Date.now();
      _controlStarting = false;
      log(
        `control webtransport open (${sessionName}) ` +
        `max=${dgrams.maxDatagramSize || '?'} outAge=${dgrams.outgoingMaxAge ?? '?'}`
      );
      readControlWebTransport(wt, _controlWtReader);
      pollControlWebTransportStats(wt);
    } catch (e) {
      if (wt !== _controlWt) return;
      _controlStarting = false;
      log(`control webtransport open failed: ${e && e.message || e}`);
      closeControlWebTransport();
      scheduleControlReconnect('webtransport open failed', true);
    }
  })().catch((e) => {
    _controlStarting = false;
    log(`control webtransport init ${e && e.message || e}`);
    closeControlWebTransport();
    scheduleControlReconnect('webtransport init failed', true);
  });
}

function restartControl(reason) {
  if (!_controlShouldReconnect || _controlStarting) return;
  const now = Date.now();
  if (now - _controlLastRestartMs < 1000) return;
  _controlLastRestartMs = now;
  _controlReconnectCount++;
  _controlLastReconnectReason = reason;
  log(`control restart (${reason})`);
  _controlReconnectDelay = 500;
  closeControlWebTransport();
  startControl();
}

function checkControlHealth() {
  if (!_controlShouldReconnect) return;
  if (!controlReady()) {
    if (_controlWt && !_controlStarting && _controlConnectStartedMs
        && Date.now() - _controlConnectStartedMs > CONTROL_CONNECT_TIMEOUT_MS) {
      restartControl('connect timeout');
      return;
    }
    if (!_controlWt && !_controlStarting && !_controlReconnectTimer) {
      scheduleControlReconnect('not connected', true);
    }
    return;
  }
  const age = controlRxAgeMs();
  // WT can have one-way zombie state (server keeps sending, client never reads),
  // so downlink staleness is our backstop. Server sends keepalives every 3s,
  // so CONTROL_RESTART_WT_LINK_MS allows for 2 missed keepalives + slack.
  if (age > CONTROL_RESTART_WT_LINK_MS) {
    restartControl(`control downlink stale ${(age / 1000).toFixed(1)}s`);
  }
}

function stopControl() {
  _controlShouldReconnect = false;
  if (_controlReconnectTimer) { clearTimeout(_controlReconnectTimer); _controlReconnectTimer = null; }
  if (_controlHealthTimer) { clearInterval(_controlHealthTimer); _controlHealthTimer = null; }
  closeControlWebTransport();
  lastControlRxMs = 0;
  latencyWindow.length = 0;
  edgeLatencyWindow.length = 0;
  robotLatencyWindow.length = 0;
  latencyAvgMs = 0;
  edgeLatencyAvgMs = 0;
  robotLatencyAvgMs = 0;
  lastEdgeAckMs = 0;
  lastRobotAckMs = 0;
  clockOffsetMs = 0;
  clockOffsetReady = false;
  _controlLastReconnectReason = '';
  resetServerGateStatus();
  resetRobotTrackingStatus();
}

function releaseControlClaim(reason = 'release') {
  if (!_controlClaimActive) return;
  _controlClaimActive = false;
  try {
    fetch(withToken(`/control/release?page_id=${encodeURIComponent(APP_PAGE_ID)}`), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      cache: 'no-store',
      keepalive: true,
      body: JSON.stringify({
        page_id: APP_PAGE_ID,
        reason,
        ...appEventFields(),
        t_browser_ms: Date.now(),
      }),
    }).catch(() => {});
  } catch (_) {}
}

function stopVideo() {
  video.stop();
}

function showLanding(show) {
  landing.style.display = show ? 'flex' : 'none';
}

function startViewer(reason = 'viewer') {
  releaseControlClaim(reason);
  stopControl();
  setClientRole('viewer', reason);
  showLanding(false);
  overlay.classList.add('active');
  statusEl.textContent = 'viewer\nstarting video...';
  bannerEl.classList.remove('show', 'warn', 'bad');
  log(`viewer mode (${reason})`);
  startVideo();
  updateViewerStatus();
}

function stopViewer() {
  releaseControlClaim('viewer_stop');
  stopControl();
  stopVideo();
  setClientRole(FORCE_VIEWER ? 'viewer' : 'unknown');
  overlay.classList.remove('active', 'viewer', 'warn', 'bad');
  showLanding(true);
  startBtn.disabled = false;
}

function log(msg) {
  const ts = new Date().toISOString().slice(11, 19);
  logEl.textContent = `${ts} ${msg}\n` + logEl.textContent;
  logEl.scrollTop = 0;
  try {
    fetch(withToken('/test_event'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      cache: 'no-store',
      keepalive: true,
      body: JSON.stringify({
        kind: 'client_log',
        ...appEventFields(),
        msg,
        ctrl_transport: _controlTransport,
        ctrl_state: _controlState,
        ctrl_ready: controlReady() ? 1 : 0,
        ctrl_connect_seq: _controlConnectSeq,
        ctrl_reconnect_count: _controlReconnectCount,
        ctrl_last_reconnect_reason: _controlLastReconnectReason,
        video_state: video.state,
        video_error: video.lastError,
        video_reconnect_count: video.reconnectCount,
        video_last_reconnect_reason: video.lastReconnectReason,
        t_browser_ms: Date.now(),
      }),
    }).catch(() => {});
  } catch (_) {}
}

// --- Feature detection ----------------------------------------------------
async function checkXR() {
  if (!('xr' in navigator)) {
    supportEl.textContent = 'Viewer mode: robot video only.';
    supportEl.className = 'ok';
    return false;
  }
  try {
    const ok = await navigator.xr.isSessionSupported('immersive-ar');
    if (!ok) {
      supportEl.textContent = 'Viewer mode: robot video only.';
      supportEl.className = 'ok';
      return false;
    }
  } catch (e) {
    supportEl.textContent = 'Viewer mode: robot video only.';
    supportEl.className = 'ok';
    return false;
  }
  supportEl.textContent = FORCE_VIEWER
    ? 'Viewer mode: robot video only.'
    : 'WebXR immersive-ar: supported. This browser can claim control.';
  supportEl.className = 'ok';
  return true;
}

// --- Pose sequence + control downlink tracking ---------------------------
// These are intentionally separate:
// - control RX: any downlink datagram, used only for session liveness
// - edge ack: phone-to-edge app telemetry from the forwarder
// - robot ack: end-to-end pose echo from the Pi, used for robot health/timing
let lastControlRxMs = 0;
let wsPoseSeq = 0;  // monotonic per-pose sequence number (name is legacy;
                    // still stamped on each pose msg and read by the Pi as
                    // a duplicate/reorder detector on the current WT path)
// Latency tracking (sliding window).
const latencyWindow = [];
const edgeLatencyWindow = [];
const robotLatencyWindow = [];
const LATENCY_WINDOW_SIZE = 30;
let latencyAvgMs = 0;
let edgeLatencyAvgMs = 0;
let robotLatencyAvgMs = 0;
let lastEdgeAckMs = 0;
let lastRobotAckMs = 0;
let clockOffsetMs = 0;   // pi_clock - phone_clock, NTP-style from acks
let clockOffsetReady = false;
function controlRxAgeMs() {
  return lastControlRxMs ? Date.now() - lastControlRxMs : 0;
}
function edgeAckAgeMs() {
  return lastEdgeAckMs ? Date.now() - lastEdgeAckMs : null;
}
function robotAckAgeMs() {
  return lastRobotAckMs ? Date.now() - lastRobotAckMs : null;
}
function noteLatencySample(windowRef, rtt) {
  windowRef.push(rtt);
  if (windowRef.length > LATENCY_WINDOW_SIZE) windowRef.shift();
  return windowRef.reduce((a, b) => a + b, 0) / windowRef.length;
}

function resetPoseSourceStats() {
  _poseSourceSessionStartMs = Date.now();
  _poseSourceFrameCount = 0;
  _poseSourcePoseValidCount = 0;
  _poseSourcePoseInvalidCount = 0;
  _poseSourceGeneratedCount = 0;
  _poseSourceGeneratedB1Count = 0;
  _poseSourceNoControlDropCount = 0;
  _poseSourceBufferedDropCount = 0;
  _poseSourceRateLimitCount = 0;
  _poseSourceWtQueuedCount = 0;
  _poseSourceOversizeDropCount = 0;
  _poseSourceLastFrameWallMs = 0;
  _poseSourceLastFramePerfMs = 0;
  _poseSourceLastGeneratedMs = 0;
  _poseSourceLastSentMs = 0;
  _poseSourceLastNoControlMs = 0;
  _poseSourceLastGapMs = 0;
  _poseSourceMaxGapMs = 0;
  _poseSourceLongGap250Count = 0;
  _poseSourceLongGap1000Count = 0;
  _poseSourceLastStats = null;
  _mainLongTaskCount = 0;
  _mainLongTaskTotalMs = 0;
  _mainLongTaskMaxMs = 0;
  _mainLongTaskWindowMaxMs = 0;
  _mainLoopLagCount = 0;
  _mainLoopLagSumMs = 0;
  _mainLoopLagMaxMs = 0;
  _mainLoopLagOver50Count = 0;
  _mainLoopLagOver250Count = 0;
}

function recordPoseSourceFrame(poseValid) {
  const nowWall = Date.now();
  const nowPerf = performance.now();
  if (_poseSourceLastFramePerfMs) {
    const gapMs = nowPerf - _poseSourceLastFramePerfMs;
    _poseSourceLastGapMs = gapMs;
    if (gapMs > _poseSourceMaxGapMs) _poseSourceMaxGapMs = gapMs;
    if (gapMs > 250) _poseSourceLongGap250Count++;
    if (gapMs > 1000) _poseSourceLongGap1000Count++;
  }
  _poseSourceLastFramePerfMs = nowPerf;
  _poseSourceLastFrameWallMs = nowWall;
  _poseSourceFrameCount++;
  if (poseValid) _poseSourcePoseValidCount++;
  else _poseSourcePoseInvalidCount++;
}

function poseSourceSnapshot(reason = 'periodic') {
  const now = Date.now();
  const prev = _poseSourceLastStats || {};
  const net = networkInfoSnapshot();
  const sample = {
    kind: 'pose_source_stats',
    ...appEventFields(now),
    reason,
    t_browser_ms: now,
    ageSessionMs: _poseSourceSessionStartMs ? (now - _poseSourceSessionStartMs) : null,
    xrFrameCount: _poseSourceFrameCount,
    dXrFrameCount: _poseSourceFrameCount - (prev.xrFrameCount || 0),
    poseValidFrames: _poseSourcePoseValidCount,
    dPoseValidFrames: _poseSourcePoseValidCount - (prev.poseValidFrames || 0),
    poseInvalidFrames: _poseSourcePoseInvalidCount,
    dPoseInvalidFrames: _poseSourcePoseInvalidCount - (prev.poseInvalidFrames || 0),
    poseGenerated: _poseSourceGeneratedCount,
    dPoseGenerated: _poseSourceGeneratedCount - (prev.poseGenerated || 0),
    poseGeneratedB1: _poseSourceGeneratedB1Count,
    dPoseGeneratedB1: _poseSourceGeneratedB1Count - (prev.poseGeneratedB1 || 0),
    poseSent: _controlPoseCount,
    dPoseSent: _controlPoseCount - (prev.poseSent || 0),
    poseNoControlDrop: _poseSourceNoControlDropCount,
    dPoseNoControlDrop: _poseSourceNoControlDropCount - (prev.poseNoControlDrop || 0),
    poseBufferedDrop: _poseSourceBufferedDropCount,
    dPoseBufferedDrop: _poseSourceBufferedDropCount - (prev.poseBufferedDrop || 0),
    poseRateLimited: _poseSourceRateLimitCount,
    dPoseRateLimited: _poseSourceRateLimitCount - (prev.poseRateLimited || 0),
    wtQueued: _poseSourceWtQueuedCount,
    dWtQueued: _poseSourceWtQueuedCount - (prev.wtQueued || 0),
    oversizeDrop: _poseSourceOversizeDropCount,
    dOversizeDrop: _poseSourceOversizeDropCount - (prev.oversizeDrop || 0),
    debugPost: _controlDebugPostCount,
    dDebugPost: _controlDebugPostCount - (prev.debugPost || 0),
    debugPostFail: _controlDebugPostFailCount,
    dDebugPostFail: _controlDebugPostFailCount - (prev.debugPostFail || 0),
    lastFrameAgeMs: _poseSourceLastFrameWallMs ? (now - _poseSourceLastFrameWallMs) : null,
    lastGeneratedAgeMs: _poseSourceLastGeneratedMs ? (now - _poseSourceLastGeneratedMs) : null,
    lastSentAgeMs: _poseSourceLastSentMs ? (now - _poseSourceLastSentMs) : null,
    lastNoControlAgeMs: _poseSourceLastNoControlMs ? (now - _poseSourceLastNoControlMs) : null,
    lastFrameGapMs: Math.round(_poseSourceLastGapMs),
    maxFrameGapMs: Math.round(_poseSourceMaxGapMs),
    longFrameGaps250: _poseSourceLongGap250Count,
    longFrameGaps1000: _poseSourceLongGap1000Count,
    wsPoseSeq,
    lastPoseSeqSent: _controlLastPoseSeqSent,
    lastPoseTBrowserMs: _controlLastPoseTBrowserMs,
    b1: b1Held ? 1 : 0,
    docVisibility: document.visibilityState,
    xrVisibility: xrSession ? xrSession.visibilityState : null,
    ctrl_transport: _controlTransport,
    ctrl_state: _controlState,
    ctrl_ready: controlReady() ? 1 : 0,
    ctrl_ack_age_ms: Math.round(controlRxAgeMs()),
    ctrl_edge_ack_age_ms: edgeAckAgeMs() == null ? null : Math.round(edgeAckAgeMs()),
    ctrl_robot_ack_age_ms: robotAckAgeMs() == null ? null : Math.round(robotAckAgeMs()),
    ctrl_edge_rtt_ms: edgeLatencyWindow.length ? Math.round(edgeLatencyAvgMs) : null,
    ctrl_robot_rtt_ms: robotLatencyWindow.length ? Math.round(robotLatencyAvgMs) : null,
    ctrl_connect_seq: _controlConnectSeq,
    ctrl_reconnect_count: _controlReconnectCount,
    ctrl_last_reconnect_reason: _controlLastReconnectReason,
    server_gate_state: serverGateState,
    server_gate_severity: serverGateSeverity,
    server_gate_reason: serverGateReason,
    server_gate_requires_reengage: serverGateRequiresReengage ? 1 : 0,
    server_gate_age_ms: serverGateLastMs ? Math.round(now - serverGateLastMs) : null,
    robot_tracking_error_m: robotTrackingErrorM,
    robot_tracking_command_error_m: robotTrackingCommandErrorM,
    robot_tracking_goal_error_m: robotTrackingGoalErrorM,
    robot_tracking_age_ms: robotTrackingAgeMs,
    robot_tracking_seq: robotTrackingSeq,
    robot_tracking_phone_enabled: robotTrackingPhoneEnabled ? 1 : 0,
    robot_tracking_error_source: robotTrackingErrorSource,
    robot_tracking_ack_age_ms: robotTrackingLastMs ? Math.round(now - robotTrackingLastMs) : null,
    video_state: video.state,
    video_error: video.lastError,
    video_active: video.pc ? 1 : 0,
    video_reconnect_count: video.reconnectCount,
    video_last_reconnect_reason: video.lastReconnectReason,
    wtWriteAttempts: _controlWtWriteCount,
    wtWriterBlockedCount: _controlWtWriterBlockedCount,
    wtWriterBlockedMs: Math.round(_controlWtWriterBlockedMs),
    wtRecvCount: _controlWtRecvCount,
    wtLastRecvAgeMs: _controlWtLastRecvMs ? (now - _controlWtLastRecvMs) : null,
    wtWriterDesiredSize: _controlWtWriter ? _controlWtWriter.desiredSize : null,
    wtPending: _controlLatestPose ? 1 : 0,
    wtPendingAgeMs: _controlLatestPose ? (now - _controlLatestPose.t) : null,
    wtTargetHz: CONTROL_TARGET_HZ,
    wtLastSendAgeMs: _controlLastSendPerfMs
      ? Math.round(performance.now() - _controlLastSendPerfMs) : null,
    // Per-frame timing: distinguishes "ARCore/scheduler is firing the callback
    // late" (gap grows, duration stays small) from "our work in the callback
    // takes too long" (duration grows). Reset each interval so the numbers
    // describe the window, not the session.
    xrCbCount: _xrCallbackCount,
    xrCbDurationAvgMs: _xrCallbackCount > 0 ? (_xrCallbackDurationSumMs / _xrCallbackCount) : null,
    xrCbDurationMaxMs: _xrCallbackDurationMaxMs,
    xrCbGapAvgMs: _xrCallbackCount > 1 ? (_xrCallbackGapSumMs / (_xrCallbackCount - 1)) : null,
    xrCbGapMaxMs: _xrCallbackGapMaxMs,
    mainLongTaskCount: _mainLongTaskCount,
    dMainLongTaskCount: _mainLongTaskCount - (prev.mainLongTaskCount || 0),
    mainLongTaskTotalMs: Math.round(_mainLongTaskTotalMs),
    dMainLongTaskTotalMs: Math.round(
      _mainLongTaskTotalMs - (prev.mainLongTaskTotalMs || 0)),
    mainLongTaskMaxMs: Math.round(_mainLongTaskMaxMs),
    mainLongTaskWindowMaxMs: Math.round(_mainLongTaskWindowMaxMs),
    mainLoopLagCount: _mainLoopLagCount,
    mainLoopLagAvgMs: _mainLoopLagCount > 0
      ? (_mainLoopLagSumMs / _mainLoopLagCount) : null,
    mainLoopLagMaxMs: _mainLoopLagMaxMs,
    mainLoopLagOver50: _mainLoopLagOver50Count,
    mainLoopLagOver250: _mainLoopLagOver250Count,
    // CPU pressure: direct thermal-throttle signal from chrome's Compute
    // Pressure API. Records the current state plus how many times we've seen
    // each state during this session — easy to scan for transitions.
    cpuPressureState: _cpuPressureLatest,
    cpuPressureChangeCount: _cpuPressureChangeCount,
    cpuPressureNominalCount: _cpuPressureStateCounts.nominal,
    cpuPressureFairCount: _cpuPressureStateCounts.fair,
    cpuPressureSeriousCount: _cpuPressureStateCounts.serious,
    cpuPressureCriticalCount: _cpuPressureStateCounts.critical,
    // Battery — secondary thermal signal. Drop in level + discharge rate
    // correlated with XR throttling is consistent with sustained thermal load.
    batteryLevel: _batteryLevel,
    batteryCharging: _batteryCharging,
    batteryDischargingTimeS: _batteryDischargingTimeS,
    ...net,
  };
  if (webrtcStats) {
    sample.wrtc_rtt_ms = webrtcStats.rtt;
    sample.wrtc_jbuf_ms = webrtcStats.jbuf;
    sample.wrtc_decode_ms = webrtcStats.decode;
    sample.wrtc_jitter_ms = webrtcStats.jitter;
    sample.wrtc_fps = webrtcStats.fps;
    sample.wrtc_freezes = webrtcStats.freezes;
    sample.wrtc_via = webrtcStats.via;
    sample.wrtc_decoder_impl = webrtcStats.decoderImpl;
    sample.wrtc_power_efficient = webrtcStats.powerEfficient;
    sample.wrtc_packets_lost = webrtcStats.packetsLost;
    sample.wrtc_packets_received = webrtcStats.packetsReceived;
  }
  // Reset per-frame timing counters so each sample describes the just-elapsed
  // ~2s window, not the cumulative session.
  _xrCallbackDurationSumMs = 0;
  _xrCallbackDurationMaxMs = 0;
  _xrCallbackCount = 0;
  _xrCallbackGapSumMs = 0;
  _xrCallbackGapMaxMs = 0;
  _mainLongTaskWindowMaxMs = 0;
  _mainLoopLagCount = 0;
  _mainLoopLagSumMs = 0;
  _mainLoopLagMaxMs = 0;
  _mainLoopLagOver50Count = 0;
  _mainLoopLagOver250Count = 0;
  _poseSourceLastStats = {
    xrFrameCount: sample.xrFrameCount,
    poseValidFrames: sample.poseValidFrames,
    poseInvalidFrames: sample.poseInvalidFrames,
    poseGenerated: sample.poseGenerated,
    poseGeneratedB1: sample.poseGeneratedB1,
    poseSent: sample.poseSent,
    poseNoControlDrop: sample.poseNoControlDrop,
    poseBufferedDrop: sample.poseBufferedDrop,
    poseRateLimited: sample.poseRateLimited,
    wtQueued: sample.wtQueued,
    oversizeDrop: sample.oversizeDrop,
    debugPost: sample.debugPost,
    debugPostFail: sample.debugPostFail,
    mainLongTaskCount: sample.mainLongTaskCount,
    mainLongTaskTotalMs: sample.mainLongTaskTotalMs,
  };
  return sample;
}

function postPoseSourceStats(reason = 'periodic') {
  try {
    _controlDebugPostCount++;
    const body = JSON.stringify(poseSourceSnapshot(reason));
    fetch(withToken('/test_event'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      cache: 'no-store',
      keepalive: true,
      body,
    }).catch(() => { _controlDebugPostFailCount++; });
  } catch (_) {
    _controlDebugPostFailCount++;
  }
}

function startPoseSourceStats() {
  if (_poseSourceStatsTimer) clearInterval(_poseSourceStatsTimer);
  _poseSourceStatsTimer = setInterval(() => postPoseSourceStats('periodic'), 2000);
}

function startBrowserRuntimeProbes() {
  if (typeof PerformanceObserver === 'function') {
    try {
      _mainLongTaskObserver = new PerformanceObserver((list) => {
        for (const entry of list.getEntries()) {
          const duration = Number(entry.duration) || 0;
          _mainLongTaskCount++;
          _mainLongTaskTotalMs += duration;
          if (duration > _mainLongTaskMaxMs) _mainLongTaskMaxMs = duration;
          if (duration > _mainLongTaskWindowMaxMs) _mainLongTaskWindowMaxMs = duration;
        }
      });
      _mainLongTaskObserver.observe({ entryTypes: ['longtask'] });
    } catch (_) {
      _mainLongTaskObserver = null;
    }
  }

  if (_mainLoopLagTimer) clearInterval(_mainLoopLagTimer);
  const periodMs = 250;
  _mainLoopLagExpectedMs = performance.now() + periodMs;
  _mainLoopLagTimer = setInterval(() => {
    const now = performance.now();
    const lagMs = Math.max(0, now - _mainLoopLagExpectedMs);
    _mainLoopLagExpectedMs += periodMs;
    if (lagMs > periodMs) {
      // If the page was suspended, do not let old expected times produce an
      // artificial staircase of lag samples after resume.
      _mainLoopLagExpectedMs = now + periodMs;
    }
    _mainLoopLagCount++;
    _mainLoopLagSumMs += lagMs;
    if (lagMs > _mainLoopLagMaxMs) _mainLoopLagMaxMs = lagMs;
    if (lagMs > 50) _mainLoopLagOver50Count++;
    if (lagMs > 250) _mainLoopLagOver250Count++;
  }, periodMs);
}

function stopBrowserRuntimeProbes() {
  if (_mainLongTaskObserver) {
    try { _mainLongTaskObserver.disconnect(); } catch (_) {}
    _mainLongTaskObserver = null;
  }
  if (_mainLoopLagTimer) {
    clearInterval(_mainLoopLagTimer);
    _mainLoopLagTimer = null;
  }
}

function networkInfoSnapshot() {
  const c = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
  if (!c) return {};
  return {
    netEffectiveType: c.effectiveType || null,
    netDownlinkMbps: Number.isFinite(c.downlink) ? c.downlink : null,
    netRttMs: Number.isFinite(c.rtt) ? c.rtt : null,
    netSaveData: c.saveData ? 1 : 0,
  };
}

// Compute Pressure API + getBattery() — let us prove (or disprove) that XR
// rate drops because the phone is thermal-throttling. PressureObserver
// reports CPU state transitions (nominal/fair/serious/critical); getBattery
// gives us level + charging context as a secondary signal.
let _pressureObserver = null;
let _batteryTimer = null;
function startThermalProbes() {
  // PressureObserver: chrome 125+ on Android exposes 'cpu' source.
  if (typeof PressureObserver === 'function') {
    try {
      _pressureObserver = new PressureObserver((records) => {
        for (const r of records) {
          const prev = _cpuPressureLatest;
          _cpuPressureLatest = r.state;
          if (prev !== r.state) {
            _cpuPressureChangeCount++;
            // Emit a discrete event for each transition — easy to align
            // against the dXrFrameCount timeline post-hoc.
            try {
              fetch(withToken('/test_event'), {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                cache: 'no-store',
                keepalive: true,
                body: JSON.stringify({
                  kind: 'cpu_pressure_change',
                  from: prev,
                  to: r.state,
                  source: r.source,
                  t_browser_ms: Date.now(),
                }),
              }).catch(() => {});
            } catch (_) {}
          }
          if (r.state in _cpuPressureStateCounts) {
            _cpuPressureStateCounts[r.state]++;
          }
        }
      });
      _pressureObserver.observe('cpu', { sampleInterval: 1000 }).catch((e) => {
        log(`PressureObserver observe failed: ${e.message || e}`);
        _pressureObserver = null;
      });
    } catch (e) {
      log(`PressureObserver init failed: ${e.message || e}`);
      _pressureObserver = null;
    }
  }
  // getBattery(): poll once at session start, then every 30s. Update levels
  // tend to be coarse (1% steps) so high-frequency polling buys nothing.
  if (typeof navigator.getBattery === 'function') {
    const refresh = () => {
      navigator.getBattery().then((b) => {
        _batteryLevel = b.level;
        _batteryCharging = b.charging;
        _batteryDischargingTimeS = isFinite(b.dischargingTime) ? b.dischargingTime : null;
      }).catch(() => {});
    };
    refresh();
    if (_batteryTimer) clearInterval(_batteryTimer);
    _batteryTimer = setInterval(refresh, 30000);
  }
}
function stopThermalProbes() {
  if (_pressureObserver) {
    try { _pressureObserver.disconnect(); } catch (_) {}
    _pressureObserver = null;
  }
  if (_batteryTimer) {
    clearInterval(_batteryTimer);
    _batteryTimer = null;
  }
}

function stopPoseSourceStats(reason = 'stop') {
  if (_poseSourceStatsTimer) {
    clearInterval(_poseSourceStatsTimer);
    _poseSourceStatsTimer = null;
  }
  postPoseSourceStats(reason);
}

function encodeJsonDatagram(msg) {
  const payload = _controlTextEncoder.encode(JSON.stringify(msg));
  return payload.byteLength <= WT_DATAGRAM_MAX_BYTES ? payload : null;
}

function handleControlWebTransportSendError(e) {
  if (_controlTransport === 'relay-webtransport') {
    _controlWtReady = false;
    _controlState = 'send-error';
    log(`control webtransport send err: ${e && e.message || e}`);
    scheduleControlReconnect('webtransport send error', true);
  }
}

function writeControlWebTransportPoseNow(msg, wasPending = false) {
  msg.t_browser_write_ms = Date.now();
  msg.ws_skipped = _controlSkippedSinceLastSend;
  msg.ws_buffered = wasPending ? 1 : 0;
  const payload = encodeJsonDatagram(msg);
  if (!payload) {
    _controlSkippedSinceLastSend++;
    _poseSourceOversizeDropCount++;
    return false;
  }
  try {
    _controlWtWriteCount++;
    _controlWtWriter.write(payload).catch(handleControlWebTransportSendError);
  } catch (e) {
    handleControlWebTransportSendError(e);
    return false;
  }
  _poseSourceWtQueuedCount++;
  _controlPoseCount++;
  _controlLastSendPerfMs = performance.now();
  _poseSourceLastSentMs = Date.now();
  _controlLastPoseSeqSent = msg.seq;
  _controlLastPoseTBrowserMs = msg.t;
  _controlSkippedSinceLastSend = 0;
  return true;
}

function flushControlWebTransportLatest() {
  if (!controlReady() || !_controlWtWriter || !_controlLatestPose) return;
  const desired = _controlWtWriter.desiredSize;
  if (typeof desired === 'number' && desired <= 0) {
    waitForControlWebTransportWriterReady();
    return;
  }
  const nowPerf = performance.now();
  const sendDelayMs = _controlLastSendPerfMs
    ? CONTROL_MIN_SEND_INTERVAL_MS - (nowPerf - _controlLastSendPerfMs)
    : 0;
  if (sendDelayMs > 0) {
    _poseSourceRateLimitCount++;
    scheduleControlWebTransportFlushAfter(sendDelayMs);
    return;
  }
  const msg = _controlLatestPose;
  const queuedPerfMs = _controlLatestPoseQueuedPerfMs;
  _controlLatestPose = null;
  _controlLatestPoseQueuedPerfMs = 0;
  if (_controlWtFlushTimer) {
    clearTimeout(_controlWtFlushTimer);
    _controlWtFlushTimer = null;
    _controlWtFlushTimerDueMs = 0;
  }
  if (Date.now() - msg.t > WT_OUTGOING_MAX_AGE_MS) {
    _controlSkippedSinceLastSend++;
    _poseSourceBufferedDropCount++;
    return;
  }
  writeControlWebTransportPoseNow(
    msg,
    queuedPerfMs > 0 && performance.now() - queuedPerfMs > 1,
  );
}

function scheduleControlWebTransportFlushAfter(delayMs) {
  if (!controlReady() || !_controlWtWriter || !_controlLatestPose) return;
  const dueMs = performance.now() + Math.max(0, delayMs);
  if (_controlWtFlushTimer && _controlWtFlushTimerDueMs <= dueMs + 1) return;
  if (_controlWtFlushTimer) clearTimeout(_controlWtFlushTimer);
  _controlWtFlushTimerDueMs = dueMs;
  _controlWtFlushTimer = setTimeout(() => {
    _controlWtFlushTimer = null;
    _controlWtFlushTimerDueMs = 0;
    flushControlWebTransportLatest();
  }, Math.max(0, delayMs));
}

function waitForControlWebTransportWriterReady() {
  if (_controlWtFlushWaiting || !controlReady() || !_controlWtWriter) return;
  const wt = _controlWt;
  const writer = _controlWtWriter;
  const started = performance.now();
  _controlWtFlushWaiting = true;
  writer.ready.then(() => {
    if (wt !== _controlWt || writer !== _controlWtWriter) return;
    const blockedMs = performance.now() - started;
    if (blockedMs > 5) {
      _controlWtWriterBlockedCount++;
      _controlWtWriterBlockedMs += blockedMs;
    }
    _controlWtFlushWaiting = false;
    flushControlWebTransportLatest();
  }).catch((e) => {
    if (wt !== _controlWt || writer !== _controlWtWriter) return;
    _controlWtFlushWaiting = false;
    handleControlWebTransportSendError(e);
  });
}

// Latest-state pose send to WebTransport. XR frames only update this one
// register. A flush happens when Chrome has writer capacity and the target
// send cadence allows another datagram. Newer poses replace older unsent poses.
function queueControlWebTransportPose(msg) {
  if (!controlReady() || !_controlWtWriter) {
    _controlLatestPose = null;
    _controlLatestPoseQueuedPerfMs = 0;
    _poseSourceNoControlDropCount++;
    _poseSourceLastNoControlMs = Date.now();
    return;
  }
  if (_controlLatestPose) {
    _controlSkippedSinceLastSend++;
    _poseSourceBufferedDropCount++;
  }
  _controlLatestPose = msg;
  _controlLatestPoseQueuedPerfMs = performance.now();
  flushControlWebTransportLatest();
}

function sendPose(pos, quat, raw_inputs, enabled, pose_valid) {
  const now = Date.now();
  _poseSourceGeneratedCount++;
  if (enabled) _poseSourceGeneratedB1Count++;
  _poseSourceLastGeneratedMs = now;
  if (clientRole !== 'controller') {
    return;
  }
  const msg = {
    type: 'pose',
    seq: ++wsPoseSeq,
    t: now,
    pos: [pos.x, pos.y, pos.z],
    rot_xyzw: [quat.x, quat.y, quat.z, quat.w],
    raw_inputs,
    enabled,
    pose_valid: pose_valid !== false,
    off: Math.round(clockOffsetMs),
    off_valid: clockOffsetReady,
    // Last video frame the operator was looking at when this pose was built.
    // Null until requestVideoFrameCallback has fired at least once.
    displayed_rtp_ts: _displayedRtpTs,
    t_op_displayed_ms: _tOpDisplayedMs == null ? null : Math.round(_tOpDisplayedMs),
    app_v: APP_SCHEMA_ID,
    page_id: APP_PAGE_ID,
  };
  // Pose/control transport. WebTransport is the only control transport. If
  // it is not ready, drop immediately rather than queue stale commands.
  if (controlReady()) {
    queueControlWebTransportPose(msg);
  } else {
    _poseSourceNoControlDropCount++;
    _poseSourceLastNoControlMs = now;
  }
}

// --- UI state -------------------------------------------------------------
let b1Held = false;
let b1PointerId = null;
let b1EdgeSeq = 0;
let b1EdgeReason = 'init';
let b1EdgeTimeMs = 0;
// Gripper: ABSOLUTE position the slider holds. 0=closed (left), 1=open (right).
// null until the slider is first touched, so the arm holds the gripper on connect.
let gripperPos = null;

function rawInputs() {
  const inp = {
    b1: (clientRole === 'controller' && b1Held) ? 1 : 0,
    _b1_edge_seq: b1EdgeSeq,
    _b1_edge_reason: b1EdgeReason,
    _b1_edge_t_ms: b1EdgeTimeMs,
  };
  if (gripperPos !== null) inp.a3 = gripperPos;  // absolute 0..1; omitted until touched -> hold
  return inp;
}

function setB1(held, reason = 'unknown') {
  if (clientRole === 'viewer' && held) return;
  if (b1Held === held) return;
  b1Held = held;
  b1EdgeSeq++;
  b1EdgeReason = reason;
  b1EdgeTimeMs = Date.now();
  b1El.classList.toggle('held', held);
  log(`B1 ${held ? 'pressed' : 'released'} (${reason}, seq=${b1EdgeSeq})`);
}

function releaseB1Pointer(e, reason) {
  if (b1PointerId !== e.pointerId) return;
  e.preventDefault();
  b1PointerId = null;
  setB1(false, reason);
  try {
    if (b1El.hasPointerCapture(e.pointerId)) b1El.releasePointerCapture(e.pointerId);
  } catch (_) {}
}

b1El.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  if (b1PointerId !== null) return;
  b1PointerId = e.pointerId;
  try { b1El.setPointerCapture(e.pointerId); } catch (_) {}
  setB1(true, 'pointerdown');
});
b1El.addEventListener('pointerup', (e) => releaseB1Pointer(e, 'pointerup'));
b1El.addEventListener('pointercancel', (e) => releaseB1Pointer(e, 'pointercancel'));
window.addEventListener('pointerup', (e) => releaseB1Pointer(e, 'window-pointerup'));
window.addEventListener('pointercancel', (e) => releaseB1Pointer(e, 'window-pointercancel'));
b1El.addEventListener('lostpointercapture', (e) => {
  if (b1PointerId !== e.pointerId) return;
  b1PointerId = null;
  setB1(false, 'lostpointercapture');
});

function setGripperFromX(clientX) {
  const r = gripperWrap.getBoundingClientRect();
  gripperPos = Math.max(0, Math.min(1, (clientX - r.left) / r.width));  // absolute, holds
  gripperThumb.style.left = `${gripperPos * 100}%`;
}
let gripperActive = false;
gripperWrap.addEventListener('pointerdown', (e) => { gripperWrap.setPointerCapture(e.pointerId); gripperActive = true; setGripperFromX(e.clientX); });
gripperWrap.addEventListener('pointermove', (e) => { if (gripperActive) setGripperFromX(e.clientX); });
gripperWrap.addEventListener('pointerup', () => { gripperActive = false; });   // hold position, don't spring back
gripperWrap.addEventListener('pointercancel', () => { gripperActive = false; });

// --- XR session lifecycle -------------------------------------------------
let xrSession = null;
let xrRefSpace = null;
let xrFpsWindow = { start: performance.now(), n: 0, fps: 0 };
let noPoseSinceMs = 0;

async function startXR() {
  startBtn.disabled = true;
  try {
    const session = await navigator.xr.requestSession('immersive-ar', {
      optionalFeatures: ['dom-overlay', 'local'],
      domOverlay: { root: overlay },
    });
    xrSession = session;
    setClientRole('controller', 'xr_start');
    showLanding(false);
    // Diagnostic + request for ARCore's frame rate. Restored from commit
    // b1d52ee after a working-tree loss. On regular Android Chrome +
    // ARCore, the WebXR spec permits `supportedFrameRates` to be absent
    // or null when the runtime chooses not to enumerate rates -- so a
    // strict-array check false-negatives. Instead log the raw API surface
    // (typeof updateTargetFrameRate + JSON of supportedFrameRates +
    // current frameRate) so we know EXACTLY what the browser exposes,
    // then attempt the call anyway. The promise resolution/rejection
    // tells us definitively whether ARCore accepts the request.
    //
    // Rationale for asking 15 Hz: on constrained hardware (Redmi/Tecno
    // etc.) letting ARCore run its default 30 Hz drives a reactive
    // thermal-throttle cascade (steady early -> abrupt drop to 9 Hz mid-
    // session). A proactive 15 Hz target keeps camera-ISP + SLAM below
    // the throttle threshold for the whole session. On flagships 30 Hz
    // is trivially sustainable but halving still cuts sustained power
    // draw without hurting tele-op feel (RTT dominates, and the Pi-side
    // IK loop runs independently at 60 Hz).
    try {
      const ua = (navigator.userAgent || '').replace(/,/g, ';');
      log(`XR ua: ${ua.slice(0, 200)}`);
      log(`XR frame-rate API: updateTargetFrameRate=${typeof session.updateTargetFrameRate}`
        + ` frameRate=${session.frameRate}`
        + ` supportedFrameRates=${JSON.stringify(session.supportedFrameRates)}`);
      if (typeof session.updateTargetFrameRate === 'function') {
        const wanted = 15;
        let pick = wanted;
        if (Array.isArray(session.supportedFrameRates)
            && session.supportedFrameRates.length > 0) {
          pick = session.supportedFrameRates.reduce((best, r) => {
            return Math.abs(r - wanted) < Math.abs(best - wanted) ? r : best;
          }, session.supportedFrameRates[0]);
        }
        log(`XR updateTargetFrameRate: calling with ${pick}...`);
        session.updateTargetFrameRate(pick).then(() => {
          log(`XR updateTargetFrameRate(${pick}) RESOLVED (now frameRate=${session.frameRate})`);
        }).catch((e) => {
          log(`XR updateTargetFrameRate(${pick}) REJECTED: ${e.name || 'Error'}: ${e.message || e}`);
        });
      } else {
        log(`XR updateTargetFrameRate: method not on session prototype (${typeof session.updateTargetFrameRate})`);
      }
    } catch (e) {
      log(`XR frame-rate diagnostic threw: ${e && e.message || e}`);
    }
    resetPoseSourceStats();
    startPoseSourceStats();
    startBrowserRuntimeProbes();
    startThermalProbes();
    overlay.classList.add('active');
    startControl();
    startVideo();

    // We must provide a WebGL context to the XR session even if we don't
    // render anything ourselves. Create a hidden canvas.
    //
    // Thermal-aware setup:
    //  - powerPreference:'low-power' asks chrome to use the efficiency GPU
    //    where present. On unified-GPU mobile devices this is a hint but
    //    doesn't hurt.
    //  - The XRWebGLLayer's framebufferScaleFactor is set below 1.0 so the
    //    UA-allocated framebuffer is smaller, reducing fragment-shader work
    //    each frame. This is the initial baseline; per-frame we further
    //    adapt via requestViewportScale/recommendedViewportScale so chrome
    //    can dynamically ask us to render even smaller when it detects
    //    thermal or perf pressure. We don't render anything meaningful
    //    ourselves (the "layer" exists only because WebXR requires one),
    //    but chrome still allocates + processes the framebuffer -- so
    //    shrinking it saves real GPU work per frame.
    const canvas = document.createElement('canvas');
    canvas.style.position = 'fixed';
    canvas.style.inset = '0';
    canvas.style.zIndex = '-1';
    document.body.appendChild(canvas);
    const gl = canvas.getContext('webgl', {
      xrCompatible: true,
      powerPreference: 'low-power',
      antialias: false,        // saves fragment-shader work
      depth: false,            // ditto -- we render nothing that needs depth
      alpha: false,
      preserveDrawingBuffer: false,
    });
    await gl.makeXRCompatible();
    session.updateRenderState({
      baseLayer: new XRWebGLLayer(session, gl, {
        // 0.5 halves each dimension -> quarters the fragment count. We're
        // not rendering to it in any way the operator sees; the layer is
        // just a WebXR protocol requirement. So we shrink aggressively.
        framebufferScaleFactor: 0.5,
      }),
    });

    // Try local-floor first (better for hand-held controllers), fall back to local.
    try {
      xrRefSpace = await session.requestReferenceSpace('local-floor');
      log('ref space: local-floor');
    } catch (e) {
      xrRefSpace = await session.requestReferenceSpace('local');
      log('ref space: local (fallback)');
    }

    session.addEventListener('end', () => {
      log(`XR end event  visState=${session.visibilityState}`);
      onXREnd();
    });
    session.addEventListener('visibilitychange', () => {
      log(`XR visibilitychange -> ${session.visibilityState}`);
    });
    session.addEventListener('inputsourceschange', (ev) => {
      log(`XR inputsourceschange add=${ev.added?.length} rem=${ev.removed?.length}`);
    });
    session.requestAnimationFrame(onXRFrame);
    log(`XR started visState=${session.visibilityState}`);
  } catch (e) {
    log(`XR start failed: ${e.message}`);
    if (_poseSourceStatsTimer) stopPoseSourceStats('xr_start_failed');
    releaseControlClaim('xr_start_failed');
    setClientRole(FORCE_VIEWER ? 'viewer' : 'unknown');
    stopControl();
    stopVideo();
    overlay.classList.remove('active');
    showLanding(true);
    startBtn.disabled = false;
  }
}

// Capture page lifecycle events so we know if Chrome is the one ending the
// XR session (visibility hidden / freeze / pagehide).
document.addEventListener('visibilitychange', () => {
  log(`doc visibility -> ${document.visibilityState}`);
  postPoseSourceStats(`doc_visibility_${document.visibilityState}`);
});
document.addEventListener('freeze', () => {
  log('doc freeze');
  postPoseSourceStats('doc_freeze');
});
document.addEventListener('resume', () => {
  log('doc resume');
  postPoseSourceStats('doc_resume');
});
window.addEventListener('pagehide', (e) => {
  log(`pagehide persisted=${e.persisted}`);
  postPoseSourceStats('pagehide');
  releaseControlClaim('pagehide');
});
window.addEventListener('pageshow', (e) => {
  log(`pageshow persisted=${e.persisted}`);
  postPoseSourceStats('pageshow');
});
window.addEventListener('blur', () => {
  log('window blur');
  postPoseSourceStats('window_blur');
});
window.addEventListener('focus', () => {
  log('window focus');
  postPoseSourceStats('window_focus');
});

function onXRFrame(t, frame) {
  const cbStartPerf = performance.now();
  // Measure gap from last callback fire — if ARCore is thermal-throttled, the
  // browser-side onXRFrame fires LESS OFTEN (gap grows from ~33ms to ~67ms to
  // ~83ms in quantized steps as the camera ISP halves its frame rate). Our
  // callback duration should stay small in that case. If the duration is what
  // grows, the bottleneck is on this thread.
  if (_xrLastCallbackPerfMs > 0) {
    const gap = cbStartPerf - _xrLastCallbackPerfMs;
    _xrCallbackGapSumMs += gap;
    if (gap > _xrCallbackGapMaxMs) _xrCallbackGapMaxMs = gap;
  }
  _xrLastCallbackPerfMs = cbStartPerf;
  const session = frame.session;
  session.requestAnimationFrame(onXRFrame);

  const pose = frame.getViewerPose(xrRefSpace);
  recordPoseSourceFrame(!!pose);
  // Dynamic viewport scaling: chrome computes a recommended viewport scale
  // based on its internal thermal/perf heuristics. If we ignore it we're
  // asking chrome to allocate more GPU work than it wants to give us; if
  // we honor it, chrome effectively throttles our GPU work when it detects
  // pressure. Chrome only exposes this on the XRView, once per frame.
  // Guard with feature detection — the property/method are undefined on
  // older browsers or non-WebXR-dynamic-viewport-scaling runtimes.
  if (pose && pose.views) {
    for (const view of pose.views) {
      const recommended = view.recommendedViewportScale;
      if (typeof recommended === 'number'
          && recommended > 0
          && recommended < 1
          && typeof view.requestViewportScale === 'function') {
        try { view.requestViewportScale(recommended); } catch (_) {}
      }
    }
  }
  // Always send a message every frame so:
  //  (a) the control path stays active even when ARCore hasn't locked tracking yet
  //  (b) inputs (B1, gripper) reach the server before SLAM initializes
  // pose_valid=false means the position/rotation fields are stale/identity.
  if (pose) {
    const p = pose.transform.position;
    const q = pose.transform.orientation;
    sendPose(
      p, q,
      rawInputs(),
      b1Held,
      true,
    );
  } else {
    sendPose(
      { x: 0, y: 0, z: 0 },
      { x: 0, y: 0, z: 0, w: 1 },
      rawInputs(),
      b1Held,
      false,
    );
  }

  xrFpsWindow.n++;
  const now = performance.now();
  // Capture callback duration BEFORE any post-sendPose work below so we
  // measure the cost of the work that actually happens every frame on the
  // critical path (pose read + send + bookkeeping). HUD/log lines below are
  // intermittent and shouldn't pollute the steady-state measurement.
  const cbDuration = now - cbStartPerf;
  _xrCallbackDurationSumMs += cbDuration;
  _xrCallbackCount++;
  if (cbDuration > _xrCallbackDurationMaxMs) _xrCallbackDurationMaxMs = cbDuration;
  if (now - xrFpsWindow.start >= 500) {
    xrFpsWindow.fps = (xrFpsWindow.n * 1000 / (now - xrFpsWindow.start));
    xrFpsWindow.start = now;
    xrFpsWindow.n = 0;
  }
  if (pose) {
    const p = pose.transform.position;
    const q = pose.transform.orientation;
    const lagStr = videoStatusText();
    statusEl.textContent =
      `fps=${xrFpsWindow.fps.toFixed(0)}  ctrl=${_controlTransport}:${controlReady() ? 'ok' : _controlState}  ctrl_rtt=${latencyAvgMs.toFixed(0)}ms\n` +
      `${lagStr} ice=${iceTransportPolicy}\n` +
      `b1=${b1Held ? 1 : 0}  grip=${gripperPos == null ? '--' : gripperPos.toFixed(2)}\n` +
      `pos=(${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)})\n` +
      `rot=(${q.x.toFixed(2)}, ${q.y.toFixed(2)}, ${q.z.toFixed(2)}, ${q.w.toFixed(2)})`;
  } else {
    const noPoseForMs = noPoseSinceMs ? Date.now() - noPoseSinceMs : 0;
    statusEl.textContent =
      `fps=${xrFpsWindow.fps.toFixed(0)}  ctrl=${_controlTransport}:${controlReady() ? 'ok' : _controlState}  rtt=${latencyAvgMs.toFixed(0)}ms  ` +
      (noPoseForMs > 2000 ? 'move phone for AR lock' : '(no pose yet)') + '\n' +
      `${videoStatusText()} ice=${iceTransportPolicy}`;
  }
  updateBanner(!!pose);
}

// Banner severity (highest wins). Visible only when not OK so a healthy
// session is uncluttered.
function updateBanner(poseValid) {
  let kind = 'ok';
  let text = '';
  const now = Date.now();
  const ageMs = controlRxAgeMs();
  const ctrlReady = controlReady();
  const serverGateRecent = serverGateLastMs && (now - serverGateLastMs) <= SERVER_GATE_STALE_MS;
  const serverGateActive = serverGateRecent && serverGateState !== 'ok';
  if (poseValid) noPoseSinceMs = 0;
  else if (!noPoseSinceMs) noPoseSinceMs = now;
  if (!ctrlReady) {
    kind = 'bad';
    text = 'CONTROL RECONNECTING...';
  } else if (ageMs > CONTROL_NO_LINK_MS) {
    // We're sending but the server hasn't echoed anything back recently.
    // Connection is half-dead or massively congested.
    kind = 'bad';
    text = `NO LINK (${(ageMs/1000).toFixed(1)}s)`;
  } else if (serverGateActive) {
    kind = serverGateSeverity === 'bad' ? 'bad' : 'warn';
    text = serverGateMessage || 'CONTROL HOLD';
  } else if (!poseValid) {
    kind = 'warn';
    text = (now - noPoseSinceMs > 2000) ? 'MOVE PHONE FOR AR' : 'NO AR TRACKING';
  } else if (ageMs > 300 || latencyAvgMs > 250) {
    kind = 'warn';
    text = `SLOW LINK (${Math.max(ageMs, latencyAvgMs).toFixed(0)}ms)`;
  }
  bannerEl.textContent = text;
  bannerEl.classList.toggle('show', kind !== 'ok');
  bannerEl.classList.toggle('warn', kind === 'warn');
  bannerEl.classList.toggle('bad', kind === 'bad');
  overlay.classList.toggle('warn', kind === 'warn');
  overlay.classList.toggle('bad', kind === 'bad');
  // B1 button colour: held but server-side won't be enabled (no tracking,
  // no link) -> amber, vs held and likely enabled -> green.
  b1El.classList.toggle('not-enabled', b1Held && kind !== 'ok');
  updateRobotTrackingMeter();
}

function onXREnd() {
  releaseControlClaim('xr_end');
  stopPoseSourceStats('xr_end');
  stopBrowserRuntimeProbes();
  stopThermalProbes();
  stopControl();
  stopVideo();
  overlay.classList.remove('active');
  showLanding(true);
  setClientRole(FORCE_VIEWER ? 'viewer' : 'unknown');
  startBtn.disabled = false;
  xrSession = null;
  log('XR session ended');
}

stopBtn.addEventListener('click', (ev) => {
  log(`Stop btn clicked  trusted=${ev.isTrusted} x=${ev.clientX} y=${ev.clientY}`);
  if (xrSession) xrSession.end();
  else stopViewer();
});
// Also catch pointerdown so we know if a finger landed on Stop even if click
// didn't quite fire normally (e.g., during a scroll/drag).
stopBtn.addEventListener('pointerdown', (ev) => {
  log(`Stop btn pointerdown  trusted=${ev.isTrusted} x=${ev.clientX} y=${ev.clientY}`);
});

// --- Boot -----------------------------------------------------------------
function startFromButton() {
  if (FORCE_VIEWER || !xrSupported) {
    startBtn.disabled = true;
    startViewer(FORCE_VIEWER ? 'forced_viewer' : 'no_ar');
    return;
  }
  startXR();
}

async function showLeaderSetup() {
  leaderBtn.disabled = true;
  leaderPanel.classList.add('show');
  leaderCommandEl.textContent = 'Loading command…';
  try {
    const response = await fetch(withToken('/leader/config'), { cache: 'no-store' });
    if (response.status === 401) {
      throw new Error(
        'This page is not authenticated. On the robot server run ' +
        './follower/mint_token.py show global, then open the full URL it prints.'
      );
    }
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const config = await response.json();
    leaderCommandEl.textContent = config.command;
    log(`leader-arm setup ready for session ${config.session}`);
  } catch (e) {
    leaderCommandEl.textContent = `Could not load leader command: ${e.message || e}`;
    log(`leader-arm setup failed: ${e.message || e}`);
  } finally {
    leaderBtn.disabled = false;
  }
}

leaderBtn.addEventListener('click', showLeaderSetup);
copyLeaderBtn.addEventListener('click', async () => {
  const command = leaderCommandEl.textContent || '';
  if (!command || command.startsWith('Loading') || command.startsWith('Could not')) return;
  try {
    await navigator.clipboard.writeText(command);
    copyLeaderBtn.textContent = 'Copied';
    setTimeout(() => { copyLeaderBtn.textContent = 'Copy command'; }, 1500);
  } catch (e) {
    log(`copy failed: ${e.message || e}`);
  }
});
watchLeaderBtn.addEventListener('click', () => startViewer('leader_arm'));

function autoStartViewer(reason) {
  setTimeout(() => {
    if (xrSession || clientRole === 'controller' || video.shouldReconnect || video.starting) return;
    startBtn.disabled = true;
    startViewer(reason);
  }, 0);
}

checkXR().then((ok) => {
  xrSupported = ok;
  startBtn.disabled = false;
  startBtn.textContent = (FORCE_VIEWER || !ok) ? 'Watch Video' : 'Control with phone';
  startBtn.addEventListener('click', startFromButton);
  if (FORCE_VIEWER) {
    autoStartViewer('forced_viewer');
  }
}).catch(() => {
  xrSupported = false;
  supportEl.textContent = 'Viewer mode: robot video only.';
  supportEl.className = 'ok';
  startBtn.disabled = false;
  startBtn.textContent = 'Watch Video';
  startBtn.addEventListener('click', startFromButton);
  if (FORCE_VIEWER) autoStartViewer('forced_viewer');
});
log('page loaded');

// --- Test-only B1 cycler (URL-gated, no-op in production) -----------------
// ?autoB1=on_ms,off_ms[,start_delay_ms]  -> press B1 for on_ms then release for
// off_ms in a loop. Used by the chrome-in-loop synthetic harness to stress the
// real control path the same way an operator cycles B1 hundreds of times. Does
// NOT enter XR (no user gesture from setTimeout), so pose_valid stays 0 and the
// motor stays still; only the WebTransport channel sees the same press/release rate.
(() => {
  const p = new URLSearchParams(location.search);
  const autoB1 = p.get('autoB1');
  if (!autoB1) return;
  const parts = autoB1.split(',').map(Number);
  const onMs = parts[0] || 20000;
  const offMs = parts.length > 1 ? parts[1] : 20000;
  const startDelayMs = parts.length > 2 ? parts[2] : 5000;
  log(`autoB1 enabled: on=${onMs}ms off=${offMs}ms`);
  let phase = 'off';
  const tick = () => {
    if (phase === 'off') {
      setB1(true, 'autoB1');
      phase = 'on';
      setTimeout(tick, onMs);
    } else {
      setB1(false, 'autoB1');
      phase = 'off';
      setTimeout(tick, offMs);
    }
  };
  setTimeout(tick, startDelayMs);
})();

// --- Test-only auto-Start (URL-gated) -------------------------------------
// ?autoStart=1 -> simulate a click on Start after the page is loaded. The
// click is synthetic so WebXR session entry fails (no user activation), but
// startControl() runs anyway since it doesn't require a gesture -> we still
// open the real control path, which is what we want to stress-test.
(() => {
  const p = new URLSearchParams(location.search);
  if (!p.get('autoStart')) return;
  setTimeout(() => {
    log('autoStart triggered');
    try { startControl(); } catch (e) { log(`autoStart failed: ${e.message || e}`); }
  }, 2000);
})();

// --- Test-only auto-Pose (URL-gated) --------------------------------------
// ?autoPose=hz drives a fake 30Hz-default loop that calls sendPose(...) with
// identity rotation + zero translation. Real operators rely on WebXR's
// animation-frame loop to drive sendPose; without WebXR no pose messages
// flow at all. This lets the chrome-in-loop soak push real messages through
// the real WebTransport path using the production app.js code path.
(() => {
  const p = new URLSearchParams(location.search);
  const arg = p.get('autoPose');
  if (!arg) return;
  const hz = parseFloat(arg) || 30;
  const periodMs = 1000.0 / hz;
  const startDelayMs = 3000;
  log(`autoPose enabled at ${hz}Hz`);
  setTimeout(function loop() {
    sendPose(
      { x: 0, y: 0, z: 0 },
      { x: 0, y: 0, z: 0, w: 1 },
      rawInputs(),
      b1Held,
      false,
    );
    setTimeout(loop, periodMs);
  }, startDelayMs);
})();

// --- Test-only auto-Video (URL-gated) -------------------------------------
// ?autoVideo=1 -> open the real video RTCPeerConnection alongside the control
// path. The operator UI normally starts the video PC from inside startXR(); we
// trigger it standalone so the chrome-in-loop soak matches the broken-session
// shape (control WT + video PC simultaneously).
(() => {
  const p = new URLSearchParams(location.search);
  if (!p.get('autoVideo')) return;
  setTimeout(() => {
    log('autoVideo triggered');
    try { startVideo(); } catch (e) { log(`autoVideo failed: ${e.message || e}`); }
  }, 2500);
})();
