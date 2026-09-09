// The controller requires a live robot session, so requests remain
// network-first. Registration gives browsers an installable app boundary
// without risking stale control code or session configuration.
self.addEventListener('install', () => self.skipWaiting());
self.addEventListener('activate', event => event.waitUntil(self.clients.claim()));
self.addEventListener('fetch', event => event.respondWith(fetch(event.request)));
