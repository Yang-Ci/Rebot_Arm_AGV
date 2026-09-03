import langs from './locales/langs.json';
import zh from './locales/zh.json';
import ja from './locales/ja.json';
import en from './locales/en.json';

const STORAGE_KEY = 'web_mujoco.lang';
const TABLES = { zh, ja, en };

export const LANGS = langs;

let lang = 'zh';
const listeners = new Set();

function readStored() {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (LANGS.some((item) => item.id === stored)) return stored;
  } catch {
    /* ignore */
  }
  return 'zh';
}

function langMeta(id = lang) {
  return LANGS.find((item) => item.id === id) || LANGS[0];
}

function tableFor(id) {
  return TABLES[id] || TABLES.zh;
}

export function t(key, vars) {
  const current = tableFor(lang);
  const fallback = TABLES.zh;
  let text = current[key] || fallback[key] || key;
  if (vars) {
    Object.entries(vars).forEach(([name, value]) => {
      text = text.replaceAll(`{${name}}`, String(value));
    });
  }
  return text;
}

export function getLang() {
  return lang;
}

export function setLang(next) {
  if (!LANGS.some((item) => item.id === next) || next === lang) return;
  lang = next;
  try {
    localStorage.setItem(STORAGE_KEY, lang);
  } catch {
    /* ignore */
  }
  document.documentElement.lang = langMeta().html;
  listeners.forEach((fn) => fn(lang));
}

export function onLangChange(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

function labelOf(item) {
  return `${item.flag} ${item.label}`;
}

export function bindLangSwitch(root) {
  if (!root) return;
  const select = root.tagName === 'SELECT' ? root : root.querySelector('select');
  if (!select) return;

  select.replaceChildren(
    ...LANGS.map((item) => {
      const option = document.createElement('option');
      option.value = item.id;
      option.textContent = labelOf(item);
      return option;
    })
  );

  function sync() {
    select.value = lang;
    select.setAttribute('aria-label', t('lang.aria'));
  }

  select.addEventListener('pointerdown', (event) => event.stopPropagation());
  select.addEventListener('click', (event) => event.stopPropagation());
  select.addEventListener('change', () => setLang(select.value));
  onLangChange(sync);
  sync();
}

export function applyStaticI18n() {
  document.querySelectorAll('[data-i18n]').forEach((el) => {
    el.textContent = t(el.dataset.i18n);
  });
  document.querySelectorAll('[data-i18n-title]').forEach((el) => {
    el.title = t(el.dataset.i18nTitle);
  });
  document.querySelectorAll('[data-i18n-aria-label]').forEach((el) => {
    el.setAttribute('aria-label', t(el.dataset.i18nAriaLabel));
  });
}

lang = readStored();
document.documentElement.lang = langMeta().html;
