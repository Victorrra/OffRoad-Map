/* OffRoad Maps — service worker
 * - App shell (HTML, manifest, icon, Leaflet) is cached so the app opens offline.
 * - Map tiles are served cache-first: first from areas downloaded with
 *   "Свали този район офлайн" (offroad-tiles-v1), then from a small runtime
 *   cache of recently viewed tiles, and only then from the network.
 */

const VERSION = 'v6';
const SHELL_CACHE = 'offroad-shell-' + VERSION;
const TILE_CACHE = 'offroad-tiles-v1';            // filled by downloadArea() in index.html — never auto-deleted
const RUNTIME_TILE_CACHE = 'offroad-tiles-runtime-v1';
const RUNTIME_TILE_LIMIT = 1500;                  // recently viewed tiles kept for spotty coverage
const NETWORK_TIMEOUT_MS = 4000;

const SHELL_FILES = ['./', './index.html', './manifest.json', './icon.png',
                     './vendor/pmtiles.js', './vendor/protomaps-leaflet.js', './router.js'];
const CDN_FILES = [
  'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css',
  'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js',
  'https://unpkg.com/leaflet-rotate@0.2.8/dist/leaflet-rotate-src.js'
];
const TILE_HOSTS = new Set([
  'server.arcgisonline.com',
  'services.arcgisonline.com',
  'a.tile.opentopomap.org',
  'a.tile.openstreetmap.org'
]);

// ---------- Install: cache the app shell ----------
self.addEventListener('install', (event) => {
  event.waitUntil((async () => {
    const cache = await caches.open(SHELL_CACHE);
    await cache.addAll(SHELL_FILES);
    // CDN files one by one, so a single CDN hiccup doesn't break the install
    await Promise.all(CDN_FILES.map(async (url) => {
      try {
        const res = await fetch(url, { mode: 'cors', credentials: 'omit' });
        if (res.ok) await cache.put(url, res);
      } catch (e) { /* will be cached on first use */ }
    }));
    await self.skipWaiting();
  })());
});

// ---------- Activate: drop old shell caches (tile caches are kept) ----------
self.addEventListener('activate', (event) => {
  event.waitUntil((async () => {
    const names = await caches.keys();
    await Promise.all(names
      .filter((n) => n.startsWith('offroad-shell-') && n !== SHELL_CACHE)
      .map((n) => caches.delete(n)));
    await self.clients.claim();
  })());
});

// ---------- Fetch routing ----------
self.addEventListener('fetch', (event) => {
  const req = event.request;
  if (req.method !== 'GET') return;               // Overpass POSTs etc. go straight to the network

  const url = new URL(req.url);

  // The vector map and routing graph are downloaded into OPFS by the page itself —
  // never copy these large files into the Cache API as well.
  if (url.pathname.endsWith('.pmtiles') || url.pathname.endsWith('.bin')) return;

  if (TILE_HOSTS.has(url.hostname)) {
    event.respondWith(tileFirst(req));
    return;
  }
  if (CDN_FILES.includes(req.url)) {
    event.respondWith(cacheFirst(req, SHELL_CACHE));
    return;
  }
  if (url.origin === self.location.origin) {
    event.respondWith(networkFirst(req));
    return;
  }
  // Everything else (Nominatim, Overpass GET...) — default network behaviour
});

// Tiles: downloaded area → recently viewed → network (then remember it)
async function tileFirst(req) {
  const key = req.url;
  const hit = (await (await caches.open(TILE_CACHE)).match(key)) ||
              (await (await caches.open(RUNTIME_TILE_CACHE)).match(key));
  if (hit) return hit;

  try {
    // Fetch with CORS so the response is not opaque: opaque responses can't be
    // status-checked and are counted with a huge padding against the storage quota.
    const res = await fetch(key, { mode: 'cors', credentials: 'omit' });
    if (res.ok) {
      const cache = await caches.open(RUNTIME_TILE_CACHE);
      cache.put(key, res.clone()).then(maybeTrimRuntime).catch(() => {});
    }
    return res;
  } catch (e) {
    try { return await fetch(req); }              // last try with the original (no-cors) request
    catch (e2) { return new Response('', { status: 504, statusText: 'Offline' }); }
  }
}

let putsSinceTrim = 0;
async function maybeTrimRuntime() {
  if (++putsSinceTrim < 50) return;
  putsSinceTrim = 0;
  const cache = await caches.open(RUNTIME_TILE_CACHE);
  const keys = await cache.keys();                // insertion order → oldest first
  const extra = keys.length - RUNTIME_TILE_LIMIT;
  for (let i = 0; i < extra; i++) await cache.delete(keys[i]);
}

async function cacheFirst(req, cacheName) {
  const cache = await caches.open(cacheName);
  const hit = await cache.match(req.url);
  if (hit) return hit;
  try {
    const res = await fetch(req.url, { mode: 'cors', credentials: 'omit' });
    if (res.ok) cache.put(req.url, res.clone()).catch(() => {});
    return res;
  } catch (e) {
    return fetch(req);
  }
}

// App files: try the network (to pick up new versions), fall back to the cache
async function networkFirst(req) {
  const cache = await caches.open(SHELL_CACHE);
  try {
    const res = await withTimeout(fetch(req), NETWORK_TIMEOUT_MS);
    if (res.ok) cache.put(req, res.clone()).catch(() => {});
    return res;
  } catch (e) {
    const hit = await cache.match(req, { ignoreSearch: true });
    if (hit) return hit;
    if (req.mode === 'navigate') {
      const shell = await cache.match('./index.html');
      if (shell) return shell;
    }
    return new Response('Офлайн', { status: 503, headers: { 'Content-Type': 'text/plain; charset=utf-8' } });
  }
}

function withTimeout(promise, ms) {
  return new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error('timeout')), ms);
    promise.then((v) => { clearTimeout(t); resolve(v); }, (e) => { clearTimeout(t); reject(e); });
  });
}
