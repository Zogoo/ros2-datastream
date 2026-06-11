import * as THREE from 'three';

function canvasTexture(draw, size = 256) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  draw(canvas.getContext('2d'), size);
  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

export function woodTexture() {
  return canvasTexture((ctx, s) => {
    ctx.fillStyle = '#8a6843';
    ctx.fillRect(0, 0, s, s);
    for (let i = 0; i < 18; i++) {
      ctx.fillStyle = `rgba(${60 + Math.random() * 40}, ${40 + Math.random() * 25}, 20, 0.25)`;
      ctx.fillRect(0, i * (s / 18), s, 2 + Math.random() * 3);
    }
    ctx.strokeStyle = 'rgba(40,25,10,0.5)';
    for (let i = 0; i <= 6; i++) {
      ctx.beginPath();
      ctx.moveTo(0, (i * s) / 6);
      ctx.lineTo(s, (i * s) / 6);
      ctx.stroke();
    }
  });
}

export function tileTexture(base = '#b8b4ac', line = '#8d8a83') {
  return canvasTexture((ctx, s) => {
    ctx.fillStyle = base;
    ctx.fillRect(0, 0, s, s);
    ctx.strokeStyle = line;
    ctx.lineWidth = 2;
    const n = 8;
    for (let i = 0; i <= n; i++) {
      ctx.beginPath(); ctx.moveTo((i * s) / n, 0); ctx.lineTo((i * s) / n, s); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, (i * s) / n); ctx.lineTo(s, (i * s) / n); ctx.stroke();
    }
  });
}

export function stoneTexture() {
  return canvasTexture((ctx, s) => {
    ctx.fillStyle = '#6f6a62';
    ctx.fillRect(0, 0, s, s);
    for (let i = 0; i < 220; i++) {
      const g = 90 + Math.random() * 50;
      ctx.fillStyle = `rgba(${g},${g - 6},${g - 12},0.35)`;
      ctx.beginPath();
      ctx.arc(Math.random() * s, Math.random() * s, 1 + Math.random() * 4, 0, Math.PI * 2);
      ctx.fill();
    }
  });
}

export function buildMaterials() {
  const wood = woodTexture();
  const tile = tileTexture();
  const stone = stoneTexture();
  wood.repeat.set(4, 4);
  tile.repeat.set(6, 6);
  stone.repeat.set(8, 8);
  return {
    woodFloor: new THREE.MeshStandardMaterial({ map: wood, roughness: 0.7 }),
    tileFloor: new THREE.MeshStandardMaterial({ map: tile, roughness: 0.4, metalness: 0.05 }),
    wall: new THREE.MeshStandardMaterial({ map: stone, roughness: 0.9 }),
    woodProp: new THREE.MeshStandardMaterial({ color: 0x9a7a52, roughness: 0.75 }),
    counter: new THREE.MeshStandardMaterial({ color: 0xd9d2c4, roughness: 0.5 }),
    locker: new THREE.MeshStandardMaterial({ color: 0x705a40, roughness: 0.65 }),
    water: new THREE.MeshStandardMaterial({
      color: 0x3a6fd9, transparent: true, opacity: 0.65, roughness: 0.15, metalness: 0.3,
    }),
    binTowel: new THREE.MeshStandardMaterial({ color: 0x7a5c3e, roughness: 0.8 }),
    binTrash: new THREE.MeshStandardMaterial({ color: 0x4a4a4a, roughness: 0.8 }),
  };
}
