# iOS ARKit controller

This is a thin native pose adapter around the shared web controller. It does
not render robot video and does not duplicate session, transport, safety or UI
logic. Swift supplies ARKit camera poses to `controllers/phone/native-bridge.js`;
the hosted web application does everything else.

## Run on a test iPhone

1. Open `PhoneArmController.xcodeproj` in Xcode on macOS.
2. Select the `PhoneArmController` target and choose your signing team.
3. Connect an ARKit-capable iPhone, enable Developer Mode and press Run.
4. Paste the complete two-hour follower invitation link, including the
   `#access=...` fragment.
5. Grant camera access. ARKit uses the rear camera for tracking, but the app
   deliberately does not display or transmit that camera image.

No App Store review is needed for a directly connected development device.
Remote external distribution through TestFlight requires Apple's beta review.

The project is intentionally dependency-free and targets iOS 15 or newer.

