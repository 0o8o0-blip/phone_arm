# Phone controller

Shared web application for phone control and independent WHEP video viewing.
Android browsers supply 6DoF through WebXR. The thin app in `ios/` supplies
ARKit poses through `native-bridge.js`; session handling, controls, safety and
networking stay in this web application.

Controller mode deliberately does not open a video connection. Add
`?viewer=1` to the robot URL to use a separate browser or device as the video
viewer. The root page accepts a complete invitation URL, which also makes an
installed Home Screen web app usable without embedding a session secret in its
manifest.

These files are deployed to the hosted web endpoint alongside the session API.
