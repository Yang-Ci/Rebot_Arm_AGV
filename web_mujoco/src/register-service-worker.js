const registrationPromise =
  import.meta.env.PROD && 'serviceWorker' in navigator
    ? navigator.serviceWorker
        .register(`${import.meta.env.BASE_URL}sw.js`, {
          scope: import.meta.env.BASE_URL,
          updateViaCache: 'none'
        })
        .catch((error) => {
          console.warn('Service worker registration failed:', error);
          return null;
        })
    : Promise.resolve(null);

export async function persistAppShell() {
  if (!import.meta.env.PROD || !('serviceWorker' in navigator)) return;
  const registration = (await registrationPromise) || (await navigator.serviceWorker.ready);
  if (!registration?.active) return;

  const scopeUrl = new URL(import.meta.env.BASE_URL, location.origin);
  const urls = new Set([scopeUrl.href]);
  performance.getEntriesByType('resource').forEach((entry) => {
    const url = new URL(entry.name);
    if (url.origin !== scopeUrl.origin || !url.pathname.startsWith(scopeUrl.pathname)) return;
    if (!url.pathname.includes('/models/')) urls.add(url.href);
  });
  registration.active.postMessage({ type: 'CACHE_APP_SHELL', urls: [...urls] });
}
