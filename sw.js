// Offline app shell. Network-first for this site's files with the cache as
// offline fallback, so deploys reach phones without a version bump. Cross-origin
// requests (api.anthropic.com) and non-GET requests always go to the network.

const VERSION = 'v0.5.0';
const CACHE = `mbot2-${VERSION}`;
const SHELL = [
  './', 'index.html', 'style.css', 'manifest.webmanifest',
  'icons/icon-192.png', 'icons/icon-512.png', 'icons/icon-maskable-512.png', 'icons/favicon-32.png', 'icons/apple-touch-icon.png',
  'js/agent.js', 'js/app.js', 'js/bus.js', 'js/drive.js', 'js/gridmap.js', 'js/joystick.js', 'js/localize.js',
  'js/mapcontrols.js', 'js/mapstore.js', 'js/mapview.js', 'js/motion.js', 'js/navigate.js', 'js/planner.js',
  'js/pose.js', 'js/protocol.js', 'js/radar.js', 'js/robot-ble.js', 'js/robot-sim.js', 'js/scan.js',
  'js/sim-view.js', 'js/tools.js', 'js/tts.js', 'js/voice.js',
];

self.addEventListener('install', (e) => {
  // one missing file must not block the install, so add them one by one
  e.waitUntil(caches.open(CACHE)
    .then((c) => Promise.allSettled(SHELL.map((u) => c.add(new Request(u, { cache: 'reload' })))))
    .then(() => self.skipWaiting()));
});

self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k.startsWith('mbot2-') && k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

self.addEventListener('fetch', (e) => {
  const req = e.request;
  const url = new URL(req.url);
  if (req.method !== 'GET' || url.origin !== self.location.origin) return; // network as usual
  // Network first so a deploy is picked up on the next load; the cache is the
  // offline fallback (and answers if the network takes longer than 4 s).
  e.respondWith((async () => {
    const cache = await caches.open(CACHE);
    // no-cache: revalidate with the server (cheap ETag check) instead of trusting
    // the browser's HTTP cache, which can mix old and new modules after a deploy
    const net = fetch(req, { cache: 'no-cache' }).then((res) => {
      if (res.ok && res.type === 'basic') cache.put(req, res.clone());
      return res;
    });
    const timeout = new Promise((resolve) => setTimeout(resolve, 4000));
    try {
      const res = await Promise.race([net, timeout]);
      if (res) return res;
    } catch { /* offline */ }
    const hit = await cache.match(req, { ignoreSearch: req.mode === 'navigate' })
      ?? (req.mode === 'navigate' ? await cache.match('index.html') : undefined);
    return hit ?? net.catch(() => Response.error());
  })());
});
