// Stable boundary between the shared web controller and native pose sources.
// A normal browser sees an unavailable adapter. The iOS WKWebView host installs
// the `phoneArmNative` message handler and feeds ARKit poses through receivePose.
(function installPhoneArmNativeBridge(global) {
  let poseHandler = null;
  let stateHandler = null;

  function messageHandler() {
    return global.webkit?.messageHandlers?.phoneArmNative || null;
  }

  global.PhoneArmNative = Object.freeze({
    isAvailable() {
      return Boolean(messageHandler());
    },

    setPoseHandler(handler) {
      poseHandler = typeof handler === 'function' ? handler : null;
    },

    setStateHandler(handler) {
      stateHandler = typeof handler === 'function' ? handler : null;
    },

    start() {
      const handler = messageHandler();
      if (!handler) throw new Error('native pose bridge is unavailable');
      handler.postMessage({ type: 'startTracking' });
    },

    stop() {
      const handler = messageHandler();
      if (handler) handler.postMessage({ type: 'stopTracking' });
    },

    receivePose(sample) {
      if (poseHandler) poseHandler(sample || {});
    },

    receiveState(state) {
      if (stateHandler) stateHandler(state || {});
    },
  });
})(window);
