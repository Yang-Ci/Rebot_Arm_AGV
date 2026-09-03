const CACHE_NAME = 'rebot-mujoco-runtime-v3';
let currentModelVersion = null;
let prunePromise = null;

self.addEventListener('install', () => {
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    Promise.all([
      self.clients.claim(),
      caches.keys().then((names) =>
        Promise.all(names.filter((name) => name.startsWith('rebot-mujoco-') && name !== CACHE_NAME).map((name) => caches.delete(name)))
      )
    ])
  );
});

self.addEventListener('message', (event) => {
  if (event.data?.type !== 'CACHE_APP_SHELL' || !Array.isArray(event.data.urls)) return;
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) =>
      Promise.allSettled(
        [...new Set(event.data.urls)].map(async (value) => {
          const url = new URL(value);
          const scope = new URL(self.registration.scope);
          if (url.origin !== scope.origin || !url.pathname.startsWith(scope.pathname)) return;
          const request = new Request(url.href, { credentials: 'same-origin' });
          if (await cache.match(request)) return;
          const response = await fetch(request, { cache: 'force-cache' });
          if (response.ok) await cache.put(request, response);
        })
      )
    )
  );
});

function isCacheable(requestUrl) {
  const scope = new URL(self.registration.scope);
  if (requestUrl.origin !== scope.origin || !requestUrl.pathname.startsWith(scope.pathname)) return false;
  return requestUrl.pathname.includes('/assets/') || requestUrl.pathname.includes('/models/');
}

function pruneOldModels(cache, version) {
  if (!version || version === currentModelVersion) return prunePromise || Promise.resolve();
  currentModelVersion = version;
  prunePromise = cache.keys().then((requests) =>
    Promise.all(
      requests.map((request) => {
        const url = new URL(request.url);
        if (!url.pathname.includes('/models/')) return false;
        const cachedVersion = url.searchParams.get('v');
        return cachedVersion && cachedVersion !== version ? cache.delete(request) : false;
      })
    )
  );
  return prunePromise;
}

async function cacheFirst(request, event) {
  const cache = await caches.open(CACHE_NAME);
  const url = new URL(request.url);
  await pruneOldModels(cache, url.pathname.includes('/models/') ? url.searchParams.get('v') : null);
  if (url.pathname.includes('/models/') && url.pathname.toLowerCase().endsWith('.gzbin')) {
    const rawUrl = new URL(url.href);
    rawUrl.pathname = rawUrl.pathname.slice(0, -6);
    await cache.delete(rawUrl.href);
  }
  const cached = await cache.match(request);
  if (cached) return cached;
  const response = await fetch(request);
  if (response.ok) {
    event.waitUntil(cache.put(request, response.clone()).catch(() => false));
  }
  return response;
}

async function networkFirstPage(request, event) {
  const cache = await caches.open(CACHE_NAME);
  const fallbackKey = new Request(self.registration.scope);
  try {
    const response = await fetch(request);
    if (response.ok) {
      event.waitUntil(cache.put(fallbackKey, response.clone()).catch(() => false));
    }
    return response;
  } catch (error) {
    return (await cache.match(fallbackKey)) || Promise.reject(error);
  }
}

self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;
  const url = new URL(event.request.url);
  if (event.request.mode === 'navigate' && url.origin === self.location.origin) {
    event.respondWith(networkFirstPage(event.request, event));
    return;
  }
  if (isCacheable(url)) event.respondWith(cacheFirst(event.request, event));
});
