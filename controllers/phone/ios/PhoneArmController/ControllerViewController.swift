import ARKit
import simd
import UIKit
import WebKit

final class ControllerViewController: UIViewController, ARSessionDelegate, WKScriptMessageHandler {
    private let arSession = ARSession()
    private let arQueue = DispatchQueue(label: "phone-arm.arkit", qos: .userInteractive)
    private var webView: WKWebView!
    private var wantsTracking = false
    private var lastFrameTimestamp: TimeInterval = 0
    private var latestPoseJSON: String?
    private var javaScriptSendInFlight = false
    private var lastSentTrackingState = ""

    override func viewDidLoad() {
        super.viewDidLoad()
        title = "Phone Arm"
        view.backgroundColor = .black
        navigationItem.rightBarButtonItem = UIBarButtonItem(
            title: "Change Link",
            style: .plain,
            target: self,
            action: #selector(changeLink)
        )

        let contentController = WKUserContentController()
        contentController.add(self, name: "phoneArmNative")
        let configuration = WKWebViewConfiguration()
        configuration.userContentController = contentController
        configuration.allowsInlineMediaPlayback = true

        webView = WKWebView(frame: .zero, configuration: configuration)
        webView.translatesAutoresizingMaskIntoConstraints = false
        webView.isOpaque = false
        webView.backgroundColor = .black
        view.addSubview(webView)
        NSLayoutConstraint.activate([
            webView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            webView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            webView.topAnchor.constraint(equalTo: view.topAnchor),
            webView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
        ])

        arSession.delegate = self
        arSession.delegateQueue = arQueue
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(appDidEnterBackground),
            name: UIApplication.didEnterBackgroundNotification,
            object: nil
        )
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(appWillEnterForeground),
            name: UIApplication.willEnterForegroundNotification,
            object: nil
        )

        DispatchQueue.main.async { [weak self] in self?.presentLinkPrompt() }
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
        webView?.configuration.userContentController.removeScriptMessageHandler(forName: "phoneArmNative")
    }

    @objc private func changeLink() {
        stopTracking(reason: "link changed")
        presentLinkPrompt()
    }

    private func presentLinkPrompt(message: String? = nil) {
        let alert = UIAlertController(
            title: "Robot invitation link",
            message: message ?? "Paste the complete link printed by follower/run.sh",
            preferredStyle: .alert
        )
        alert.addTextField { field in
            field.placeholder = "https://…/robot/r_…#access=…"
            field.keyboardType = .URL
            field.autocapitalizationType = .none
            field.autocorrectionType = .no
        }
        alert.addAction(UIAlertAction(title: "Open", style: .default) { [weak self, weak alert] _ in
            let value = alert?.textFields?.first?.text ?? ""
            self?.openInvitation(value)
        })
        present(alert, animated: true)
    }

    private func openInvitation(_ value: String) {
        let trimmed = value.trimmingCharacters(in: .whitespacesAndNewlines)
        guard
            let url = URL(string: trimmed),
            url.scheme?.lowercased() == "https",
            url.host != nil,
            URLComponents(url: url, resolvingAgainstBaseURL: false)?.fragment?.contains("access=") == true
        else {
            presentLinkPrompt(message: "Use the complete HTTPS link, including #access=…")
            return
        }
        webView.load(URLRequest(url: url, cachePolicy: .reloadIgnoringLocalCacheData))
    }

    func userContentController(
        _ userContentController: WKUserContentController,
        didReceive message: WKScriptMessage
    ) {
        guard message.name == "phoneArmNative", let body = message.body as? [String: Any] else { return }
        switch body["type"] as? String {
        case "startTracking": startTracking()
        case "stopTracking": stopTracking(reason: "requested by controller")
        default: break
        }
    }

    private func startTracking() {
        guard ARWorldTrackingConfiguration.isSupported else {
            sendState("failed", message: "This iPhone does not support ARKit world tracking")
            return
        }
        wantsTracking = true
        lastFrameTimestamp = 0
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        arSession.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        sendState("starting", message: "Move the phone gently while ARKit finds its position")
    }

    private func stopTracking(reason: String) {
        wantsTracking = false
        arSession.pause()
        lastFrameTimestamp = 0
        DispatchQueue.main.async { [weak self] in
            self?.latestPoseJSON = nil
        }
        sendState("stopped", message: reason)
    }

    @objc private func appDidEnterBackground() {
        guard wantsTracking else { return }
        arSession.pause()
        sendState("suspended", message: "App moved to the background")
    }

    @objc private func appWillEnterForeground() {
        guard wantsTracking else { return }
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        arSession.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        sendState("starting", message: "Tracking restarted; release and reengage B1")
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard wantsTracking else { return }
        // Keep the controller's pose rate bounded and replace stale unsent
        // samples rather than building a JavaScript callback queue.
        guard frame.timestamp - lastFrameTimestamp >= (1.0 / 30.0) else { return }
        lastFrameTimestamp = frame.timestamp

        let transform = frame.camera.transform
        let position = transform.columns.3
        let quaternion = simd_quatf(transform)
        let tracking: Bool
        let trackingName: String
        switch frame.camera.trackingState {
        case .normal:
            tracking = true
            trackingName = "running"
        case .notAvailable:
            tracking = false
            trackingName = "unavailable"
        case .limited(let reason):
            tracking = false
            trackingName = "limited:\(limitedReason(reason))"
        }

        let payload: [String: Any] = [
            "position": ["x": position.x, "y": position.y, "z": position.z],
            "orientation": [
                "x": quaternion.imag.x,
                "y": quaternion.imag.y,
                "z": quaternion.imag.z,
                "w": quaternion.real,
            ],
            "tracking": tracking,
            "state": trackingName,
            "timestamp": frame.timestamp,
        ]
        guard
            let data = try? JSONSerialization.data(withJSONObject: payload),
            let json = String(data: data, encoding: .utf8)
        else { return }
        queuePoseJSON(json, trackingState: trackingName)
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        sendState("failed", message: error.localizedDescription)
    }

    func sessionWasInterrupted(_ session: ARSession) {
        sendState("interrupted", message: "ARKit session interrupted; release B1")
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        guard wantsTracking else { return }
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        sendState("starting", message: "Tracking restarted; reengage B1")
    }

    private func queuePoseJSON(_ json: String, trackingState: String) {
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.latestPoseJSON = json
            if trackingState != self.lastSentTrackingState {
                self.lastSentTrackingState = trackingState
                self.sendState(trackingState, message: nil)
            }
            self.drainLatestPose()
        }
    }

    private func drainLatestPose() {
        guard !javaScriptSendInFlight, let json = latestPoseJSON else { return }
        latestPoseJSON = nil
        javaScriptSendInFlight = true
        webView.evaluateJavaScript("window.PhoneArmNative?.receivePose(\(json))") { [weak self] _, _ in
            guard let self else { return }
            self.javaScriptSendInFlight = false
            self.drainLatestPose()
        }
    }

    private func sendState(_ state: String, message: String?) {
        var payload: [String: Any] = ["state": state]
        if let message { payload["message"] = message }
        guard
            let data = try? JSONSerialization.data(withJSONObject: payload),
            let json = String(data: data, encoding: .utf8)
        else { return }
        DispatchQueue.main.async { [weak self] in
            self?.webView.evaluateJavaScript("window.PhoneArmNative?.receiveState(\(json))")
        }
    }

    private func limitedReason(_ reason: ARCamera.TrackingState.Reason) -> String {
        switch reason {
        case .initializing: return "initializing"
        case .excessiveMotion: return "excessive-motion"
        case .insufficientFeatures: return "insufficient-features"
        case .relocalizing: return "relocalizing"
        @unknown default: return "unknown"
        }
    }
}
