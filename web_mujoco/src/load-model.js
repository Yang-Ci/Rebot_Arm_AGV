const MODEL_DIR = `${import.meta.env.BASE_URL}models`;
const DEFAULT_SCENE_XML = 'rs_grasp_scene.xml';
const DOWNLOAD_CONCURRENCY = 6;
const MODEL_VERSION = typeof __MODEL_VERSION__ === 'undefined' ? 'dev' : __MODEL_VERSION__;
const SUPPORTS_GZIP_STREAM = typeof DecompressionStream === 'function';

function isMeshFile(relative) {
  return /\.(?:stl|obj|msh)$/i.test(relative);
}

function modelUrl(relative, compressed = false) {
  return `${MODEL_DIR}/${relative}${compressed ? '.gzbin' : ''}?v=${MODEL_VERSION}`;
}

export async function loadMujocoModule() {
  const loadMujoco = (await import('@mujoco/mujoco')).default;
  return loadMujoco();
}

function parseIncludesAndMeshes(xmlText) {
  const files = new Set();
  const includeRe = /<include\s+file="([^"]+)"/gi;
  const meshRe = /\sfile="([^"]+\.(?:STL|stl|obj|msh))"/g;
  let match;
  while ((match = includeRe.exec(xmlText))) files.add(match[1]);
  while ((match = meshRe.exec(xmlText))) {
    const relative = match[1].replaceAll('\\', '/').replace(/^\.\//, '');
    files.add(relative.startsWith('meshes/') ? relative : `meshes/${relative}`);
  }
  return [...files];
}

async function fetchBytes(url, onChunk) {
  const response = await fetch(url, { cache: 'default' });
  if (!response.ok) {
    throw new Error(`无法读取 ${url}（HTTP ${response.status}）`);
  }
  const total = Number(response.headers.get('content-length')) || 0;
  const bytes = new Uint8Array(await response.arrayBuffer());
  onChunk?.(bytes.byteLength, total || bytes.byteLength);
  return bytes;
}

async function decompressGzip(bytes) {
  if (bytes[0] !== 0x1f || bytes[1] !== 0x8b) return bytes;
  const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

async function fetchModelBytes(relative, onChunk) {
  const compressed = SUPPORTS_GZIP_STREAM && isMeshFile(relative);
  if (compressed) {
    try {
      const packed = await fetchBytes(modelUrl(relative, true), onChunk);
      return { bytes: await decompressGzip(packed), transferred: packed.byteLength };
    } catch (error) {
      console.warn(`压缩模型读取失败，回退原始资源：${relative}`, error);
    }
  }
  const bytes = await fetchBytes(modelUrl(relative), onChunk);
  return { bytes, transferred: bytes.byteLength };
}

function report(onProgress, key, vars = {}) {
  onProgress?.({ key, vars });
}

function addToVfs(vfs, name, bytes) {
  vfs.addBuffer(name, bytes);
}

function createModel(mujoco, xmlBytes, vfs) {
  const xml = new TextDecoder().decode(xmlBytes);
  const errors = [];
  try {
    const model = mujoco.MjModel.from_xml_string(xml, vfs);
    if (model) return model;
  } catch (error) {
    errors.push(error && error.message ? error.message : String(error));
  }
  throw new Error(`加载 MJCF 失败：${errors.join('；') || '没有可用的加载函数'}`);
}

function createModelFromVfs(mujoco, files, vfs, sceneXmlName) {
  const sceneXml = new TextDecoder().decode(files[sceneXmlName]);
  const model = mujoco.MjModel.from_xml_path(sceneXmlName, vfs);
  if (model) return model;
  try {
    return mujoco.MjModel.from_xml_string(sceneXml, vfs);
  } catch (error) {
    const msg = error && error.message ? error.message : String(error);
        throw new Error(`加载 MJCF 失败：${msg}`);
  }
}

function extractMaterialProps(files, sceneXmlName) {
  const props = [];

  function parseSegment(text) {
    const re = /<material\s+([^>]*)\/?>/gi;
    let m;
    while ((m = re.exec(text))) {
      const attrs = m[1];
      const name = attrs.match(/name="([^"]+)"/)?.[1] ?? '';
      const met = parseFloat(attrs.match(/metallic="([^"]+)"/)?.[1] ?? '');
      const rou = parseFloat(attrs.match(/roughness="([^"]+)"/)?.[1] ?? '');
      props.push({
        name,
        metallic: Number.isFinite(met) ? met : null,
        roughness: Number.isFinite(rou) ? rou : null
      });
    }
  }

  function parseWithIncludes(xmlText) {
    const includeRe = /<include\s+file="([^"]+)"\s*\/?>/gi;
    let last = 0;
    let m;
    while ((m = includeRe.exec(xmlText))) {
      parseSegment(xmlText.slice(last, m.index));
      const file = m[1];
      if (files[file]) {
        const txt = new TextDecoder().decode(files[file]);
        parseWithIncludes(txt.replace(/<\/?mujoco[^>]*>/g, ''));
      }
      last = m.index + m[0].length;
    }
    parseSegment(xmlText.slice(last));
  }

  if (files[sceneXmlName]) {
    parseWithIncludes(new TextDecoder().decode(files[sceneXmlName]));
  }
  return props;
}

export async function loadRsScene(
  mujoco,
  onProgress,
  { sceneXml = DEFAULT_SCENE_XML } = {}
) {
  // Keep the comparatively expensive WASM startup and model downloads in
  // flight together. Meshes are copied into MjVFS as each request completes,
  // avoiding a second retained copy of the complete decompressed asset set.
  const mujocoReady = Promise.resolve(mujoco).then(
    (value) => ({ value }),
    (error) => ({ error })
  );
  const xmlQueue = [sceneXml];
  const seen = new Set();
  const meshFiles = new Set();
  const files = {};
  let loadedBytes = 0;
  let transferredBytes = 0;
  let vfs = null;

  async function requireRuntime() {
    const runtime = await mujocoReady;
    if (runtime.error) throw runtime.error;
    if (!vfs) vfs = new runtime.value.MjVFS();
    return runtime.value;
  }

  try {
    // Resolve the small XML dependency graph first so every mesh is known
    // before starting the parallel download pool.
    while (xmlQueue.length) {
      const relative = xmlQueue.shift();
      if (seen.has(relative)) continue;
      seen.add(relative);
      report(onProgress, 'status.download', { file: relative });
      const { bytes, transferred } = await fetchModelBytes(relative, (received, total) => {
        const mb = total ? `${(received / 1048576).toFixed(1)} / ${(total / 1048576).toFixed(1)} MB` : '';
        report(onProgress, 'status.downloadProgress', { file: relative, mb });
      });
      files[relative] = bytes;
      loadedBytes += bytes.byteLength;
      transferredBytes += transferred;

      const xml = new TextDecoder().decode(bytes);
      for (const extra of parseIncludesAndMeshes(xml)) {
        if (extra.toLowerCase().endsWith('.xml')) {
          if (!seen.has(extra)) xmlQueue.push(extra);
        } else {
          meshFiles.add(extra);
        }
      }
    }

    const pending = [...meshFiles].filter((relative) => !seen.has(relative));
    const totalAssets = seen.size + pending.length;
    let completedAssets = seen.size;
    const activeBytes = new Map();
    let lastProgressAt = 0;

    function reportAssetProgress(force = false) {
      const now = Date.now();
      if (!force && now - lastProgressAt < 80) return;
      lastProgressAt = now;
      const inFlightBytes = [...activeBytes.values()].reduce((sum, value) => sum + value, 0);
      report(onProgress, 'status.loadingAssets', {
        done: completedAssets,
        total: totalAssets,
        mb: ((transferredBytes + inFlightBytes) / 1048576).toFixed(1)
      });
    }

    let nextIndex = 0;
    async function downloadWorker() {
      while (nextIndex < pending.length) {
        const relative = pending[nextIndex];
        nextIndex += 1;
        seen.add(relative);
        activeBytes.set(relative, 0);
        reportAssetProgress(true);
        const { bytes, transferred } = await fetchModelBytes(relative, (received) => {
          activeBytes.set(relative, received);
          reportAssetProgress();
        });
        activeBytes.delete(relative);
        await requireRuntime();
        addToVfs(vfs, relative, bytes);
        loadedBytes += bytes.byteLength;
        transferredBytes += transferred;
        completedAssets += 1;
        reportAssetProgress(true);
      }
    }

    const workerCount = Math.min(DOWNLOAD_CONCURRENCY, pending.length);
    await Promise.all(Array.from({ length: workerCount }, () => downloadWorker()));

    report(onProgress, 'status.preparingRuntime');
    const resolvedMujoco = await requireRuntime();
    Object.entries(files).forEach(([name, bytes]) => addToVfs(vfs, name, bytes));
    const materialProps = extractMaterialProps(files, sceneXml);
    report(onProgress, 'status.compiling', { mb: (loadedBytes / 1048576).toFixed(1) });
    const model = createModelFromVfs(resolvedMujoco, files, vfs, sceneXml);
    const data = new resolvedMujoco.MjData(model);
    resolvedMujoco.mj_forward(model, data);
    return { mujoco: resolvedMujoco, model, data, loadedBytes, files: [...seen], materialProps };
  } finally {
    vfs?.delete();
  }
}
