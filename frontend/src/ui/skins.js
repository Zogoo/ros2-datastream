import * as THREE from 'three';

const el = (id) => document.getElementById(id);
const STORAGE_KEY = 'onsen_skins_v1';

/** Skin pipeline: upload an image (or pick a bundled preset) -> retexture a
 *  whole object class or a single instance -> optionally POST it to the AI
 *  worker so its HSV detection profile is resampled from the new appearance.
 *  Persists in localStorage and survives scene reset. */
export class SkinManager {
  constructor(objects, hud) {
    this.objects = objects;
    this.hud = hud;
    this.instanceMaterials = new Map();

    el('skins-toggle').addEventListener('click', () => {
      el('skins-panel').classList.toggle('hidden');
      this._refreshInstances();
    });
    el('skin-class').addEventListener('change', () => this._refreshInstances());
    el('skin-input').addEventListener('change', (e) => {
      const file = e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => this.applySkin(this._target(), reader.result, el('skin-sync').checked);
      reader.readAsDataURL(file);
      e.target.value = '';
    });
    for (const btn of document.querySelectorAll('.skin-preset')) {
      btn.addEventListener('click', () => {
        this.applySkin(this._target(), PRESETS[btn.dataset.preset](), el('skin-sync').checked);
      });
    }
    el('skin-reset-btn').addEventListener('click', () => this.resetSkin(this._target()));

    this._restore();
  }

  _target() {
    return { cls: el('skin-class').value, instanceId: el('skin-instance').value || null };
  }

  _refreshInstances() {
    const cls = el('skin-class').value;
    const select = el('skin-instance');
    const current = select.value;
    select.innerHTML = '<option value="">all instances</option>';
    for (const item of this.objects.items.filter((i) => i.cls === cls && !i.binned)) {
      const opt = document.createElement('option');
      opt.value = item.id;
      opt.textContent = item.id;
      select.appendChild(opt);
    }
    select.value = [...select.options].some((o) => o.value === current) ? current : '';
  }

  applySkin({ cls, instanceId = null }, dataUrl, syncProfile, persist = true) {
    const img = new Image();
    img.onload = () => {
      const tex = new THREE.Texture(img);
      tex.needsUpdate = true;
      tex.colorSpace = THREE.SRGBColorSpace;
      for (const mat of this._materialsFor(cls, instanceId)) {
        mat.map = tex;
        mat.color.set(0xffffff);
        mat.needsUpdate = true;
      }
      const label = instanceId ?? cls;
      el('skin-status').textContent = `${label}: skin applied`;
      if (persist) this._persist(skinKey(cls, instanceId), dataUrl);
      // Instance skins are visual experiments; only class-wide skins retune detection.
      if (syncProfile && !instanceId) this._syncProfile(cls, dataUrl);
      this.hud?.ticker(`SKIN_APPLIED ${label}`);
    };
    img.src = dataUrl;
  }

  _materialsFor(cls, instanceId) {
    if (!instanceId) return [this.objects.materialFor(cls)];
    const item = this.objects.items.find((i) => i.id === instanceId);
    if (!item) return [];
    if (!this.instanceMaterials.has(instanceId)) {
      const mat = this.objects.materialFor(cls).clone();
      item.mesh.material = mat;
      this.instanceMaterials.set(instanceId, mat);
    }
    return [this.instanceMaterials.get(instanceId)];
  }

  resetSkin({ cls, instanceId = null }) {
    const store = this._load();
    if (instanceId) {
      const item = this.objects.items.find((i) => i.id === instanceId);
      if (item) item.mesh.material = this.objects.materialFor(cls);
      this.instanceMaterials.delete(instanceId);
      delete store[skinKey(cls, instanceId)];
      el('skin-status').textContent = `${instanceId}: default`;
    } else {
      const mat = this.objects.materialFor(cls);
      mat.map = null;
      mat.color.set(this.objects.profiles[cls].color);
      mat.needsUpdate = true;
      delete store[cls];
      el('skin-status').textContent = `${cls}: default`;
      fetch(`/api/profiles/${cls}`, { method: 'DELETE' }).catch(() => {});
    }
    localStorage.setItem(STORAGE_KEY, JSON.stringify(store));
  }

  async _syncProfile(cls, dataUrl) {
    try {
      const blob = await (await fetch(dataUrl)).blob();
      const res = await fetch(`/api/profiles/${cls}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/octet-stream' },
        body: blob,
      });
      const body = await res.json();
      el('skin-status').textContent =
        `${cls}: AI profile updated (hsv ${body.lower?.join(',')} .. ${body.upper?.join(',')})`;
    } catch {
      el('skin-status').textContent = `${cls}: skin applied, AI worker unreachable`;
    }
  }

  _persist(key, dataUrl) {
    const store = this._load();
    store[key] = dataUrl;
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(store));
    } catch {
      /* quota exceeded — skin still applies for this session */
    }
  }

  _load() {
    try {
      return JSON.parse(localStorage.getItem(STORAGE_KEY)) ?? {};
    } catch {
      return {};
    }
  }

  _restore() {
    for (const [key, dataUrl] of Object.entries(this._load())) {
      const [cls, instanceId] = key.split('#');
      this.applySkin({ cls, instanceId: instanceId ?? null }, dataUrl, false, false);
    }
  }
}

const skinKey = (cls, instanceId) => (instanceId ? `${cls}#${instanceId}` : cls);

/** Bundled presets rendered procedurally — no binary assets to ship. */
function presetCanvas(draw) {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  draw(c.getContext('2d'));
  return c.toDataURL('image/png');
}

export const PRESETS = {
  ryokan: () => presetCanvas((ctx) => {
    ctx.fillStyle = '#f5f2ea';
    ctx.fillRect(0, 0, 128, 128);
    ctx.strokeStyle = 'rgba(0,0,0,0.05)';
    for (let i = 0; i < 128; i += 4) {
      ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(128, i); ctx.stroke();
    }
    ctx.fillStyle = '#b03030';
    ctx.fillRect(0, 102, 128, 10);
  }),
  striped: () => presetCanvas((ctx) => {
    for (let i = 0; i < 8; i++) {
      ctx.fillStyle = i % 2 ? '#2a5d9f' : '#f0f0f0';
      ctx.fillRect(0, i * 16, 128, 16);
    }
  }),
  charcoal: () => presetCanvas((ctx) => {
    ctx.fillStyle = '#3a3a3e';
    ctx.fillRect(0, 0, 128, 128);
    ctx.fillStyle = 'rgba(255,255,255,0.04)';
    for (let i = 0; i < 64; i++) {
      ctx.fillRect((i * 37) % 128, (i * 53) % 128, 2, 2);
    }
  }),
};
