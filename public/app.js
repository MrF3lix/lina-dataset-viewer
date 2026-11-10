// linasim dataset viewer: scene list + per-scene views (camera, events, LiDAR top view, depth,
// map, info), all following one frame slider. Plain ES module, no build step.

const OBJECT_COLORS = ['#ff5a5a', '#ffb347', '#7fd4ff', '#c58aff', '#8aff9a']
const $ = (sel, el = document) => el.querySelector(sel)
const h = (tag, attrs = {}, ...children) => {
  const el = document.createElement(tag)
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null || v === false) continue
    if (k === 'class') el.className = v
    else if (k === 'style') el.style.cssText = v
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v)
    else if (k === 'html') el.innerHTML = v
    else el.setAttribute(k, v === true ? '' : v)
  }
  for (const c of children.flat()) if (c != null && c !== false) el.append(c instanceof Node ? c : String(c))
  return el
}
const fmt = (v, d = 1) => (v == null || Number.isNaN(v) ? '–' : Number(v).toFixed(d))
const esc = encodeURIComponent
const escHtml = (t) => String(t).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`)

const state = {
  dataset: null,
  query: '',
  split: '',
  segId: null,
  seg: null, // summary of the open segment
  frame: 0,
  playing: false,
  speed: 1,
  show: load('show', { camera: true, events: true, top: true, depth: true, map: true, info: true }),
  overlay: load('overlay', { boxes2d: true, boxes3d: true, mask: false, lidar: false, paths: true, labels: true, coverage: true }),
  eventWindow: 'f1', // fN = N dataset frames, sJ = sub-interval J of the frame
  lidarColor: 'height',
  lidarDot: 2,
}
let views = []

function load(key, fallback) {
  try { return { ...fallback, ...JSON.parse(localStorage.getItem('viewer.' + key) || '{}') } } catch { return fallback }
}
function save(key, value) {
  try { localStorage.setItem('viewer.' + key, JSON.stringify(value)) } catch { /* private window */ }
}

// ---- fetching with a small cache ---------------------------------------------------------

const cache = new Map() // url -> Promise<{ data, headers }>
function fetchAsset(url, kind, signal) {
  const hit = cache.get(url)
  if (hit) {
    cache.delete(url)
    cache.set(url, hit)
    return hit
  }
  const p = fetch(url, { signal }).then(async (r) => {
    if (!r.ok) {
      let msg = r.statusText
      try { msg = (await r.json()).error ?? msg } catch { /* not json */ }
      throw new Error(msg)
    }
    const headers = Object.fromEntries(r.headers.entries())
    if (kind === 'image') return { data: await createImageBitmap(await r.blob()), headers }
    if (kind === 'f32') return { data: new Float32Array(await r.arrayBuffer()), headers }
    return { data: await r.json(), headers }
  })
  p.catch(() => cache.delete(url))
  cache.set(url, p)
  while (cache.size > 300) cache.delete(cache.keys().next().value)
  return p
}

/** One per view: aborts its previous request when a newer one comes in. */
class Latest {
  constructor() { this.ctrl = null; this.pending = 0 }
  async get(url, kind) {
    if (!cache.has(url)) { this.ctrl?.abort(); this.ctrl = new AbortController() }
    this.pending++
    try { return await fetchAsset(url, kind, this.ctrl?.signal) } finally { this.pending-- }
  }
}
const isAbort = (e) => e?.name === 'AbortError'

const api = {
  url: (route, params) => `/api/${route}?` + Object.entries(params).map(([k, v]) => `${k}=${esc(v)}`).join('&'),
  dataset: () => fetch('/api/dataset').then((r) => r.json()),
  segment: (id) => fetchAsset(api.url('segment', { id }), 'json').then((r) => r.data),
}

// ---- geometry ----------------------------------------------------------------------------

/** 3x4 vehicle -> OpenCV camera from the camera (Waymo axes) -> vehicle extrinsic. */
function vehicleToCamera(E) {
  const R = [[E[0], E[1], E[2]], [E[4], E[5], E[6]], [E[8], E[9], E[10]]]
  const t = [E[3], E[7], E[11]]
  // inverse of the rigid transform: R^T, -R^T t (Waymo camera axes x fwd, y left, z up)
  const Ri = [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]]
  const ti = Ri.map((row) => -(row[0] * t[0] + row[1] * t[1] + row[2] * t[2]))
  // OpenCV: x = -y_w, y = -z_w, z = x_w
  return [
    [-Ri[1][0], -Ri[1][1], -Ri[1][2], -ti[1]],
    [-Ri[2][0], -Ri[2][1], -Ri[2][2], -ti[2]],
    [Ri[0][0], Ri[0][1], Ri[0][2], ti[0]],
  ]
}
const apply34 = (M, x, y, z) => [
  M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
  M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
  M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3],
]
const apply44 = (P, x, y, z) => [
  P[0] * x + P[1] * y + P[2] * z + P[3],
  P[4] * x + P[5] * y + P[6] * z + P[7],
  P[8] * x + P[9] * y + P[10] * z + P[11],
]

/** 8 corners of [x, y, z, l, w, h, heading] (index bit 2 = +l, bit 1 = +w, bit 0 = +h). */
function boxCorners(b) {
  const [x, y, z, l, w, hh, yaw] = b
  const c = Math.cos(yaw), s = Math.sin(yaw)
  const out = []
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    const dx = (sx * l) / 2, dy = (sy * w) / 2
    out.push([x + dx * c - dy * s, y + dx * s + dy * c, z + (sz * hh) / 2])
  }
  return out
}
const BOX_EDGES = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]]

function inBox(b, px, py, pz, margin = 0.15) {
  const [x, y, z, l, w, hh, yaw] = b
  const dx = px - x, dy = py - y
  const c = Math.cos(yaw), s = Math.sin(yaw)
  return Math.abs(dx * c + dy * s) <= l / 2 + margin && Math.abs(-dx * s + dy * c) <= w / 2 + margin && Math.abs(pz - z) <= hh / 2 + margin
}

function ramp(v) {
  const t = Math.max(0, Math.min(1, v))
  const r = Math.round(t < 0.5 ? 70 + t * 2 * 30 : 100 + (t - 0.5) * 2 * 155)
  const g = Math.round(t < 0.5 ? 130 + t * 2 * 80 : 210 + (t - 0.5) * 2 * 30)
  const b = Math.round(t < 0.5 ? 230 - t * 2 * 70 : 160 - (t - 0.5) * 2 * 120)
  return `rgb(${r},${g},${b})`
}

function objectColor(name) {
  const idx = Object.keys(state.seg?.objects ?? {}).indexOf(name)
  return OBJECT_COLORS[(idx < 0 ? 0 : idx) % OBJECT_COLORS.length]
}

// ---- drawing shared by the camera and event views --------------------------------------

function drawBoxes(ctx, k, scale = 1) {
  const seg = state.seg
  ctx.save()
  ctx.lineWidth = 2 * scale
  ctx.font = `${Math.round(13 * scale)}px sans-serif`
  if (state.overlay.boxes2d) {
    for (const b of seg.boxes2d[k] ?? []) {
      const color = objectColor(b.object)
      const [x0, y0, x1, y1] = b.bbox
      ctx.strokeStyle = color
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0)
      if (b.mask_bbox) {
        ctx.setLineDash([3 * scale, 3 * scale])
        const [m0, m1, m2, m3] = b.mask_bbox
        ctx.strokeRect(m0, m1, m2 - m0, m3 - m1)
        ctx.setLineDash([])
      }
      if (state.overlay.labels) label(ctx, `${b.object} ${fmt(b.depth, 1)} m`, x0, y0 > 18 * scale ? y0 - 18 * scale : y1, color, scale)
    }
  }
  if (state.overlay.boxes3d && seg.K && seg.extrinsics[k]) {
    const M = vehicleToCamera(seg.extrinsics[k])
    const K = seg.K
    for (const b of seg.boxes3d[k] ?? []) {
      const color = objectColor(b.object)
      const cam = boxCorners(b.box).map(([x, y, z]) => apply34(M, x, y, z))
      ctx.strokeStyle = color
      ctx.lineWidth = 1.5 * scale
      let top = null
      for (const [a, c] of BOX_EDGES) {
        let pa = cam[a], pb = cam[c]
        const near = 0.1
        if (pa[2] < near && pb[2] < near) continue
        if (pa[2] < near || pb[2] < near) {
          const s = (near - pa[2]) / (pb[2] - pa[2])
          const cut = pa.map((v, i) => v + s * (pb[i] - pa[i]))
          if (pa[2] < near) pa = cut
          else pb = cut
        }
        const ua = K[0][0] * pa[0] / pa[2] + K[0][2], va = K[1][1] * pa[1] / pa[2] + K[1][2]
        const ub = K[0][0] * pb[0] / pb[2] + K[0][2], vb = K[1][1] * pb[1] / pb[2] + K[1][2]
        ctx.beginPath(); ctx.moveTo(ua, va); ctx.lineTo(ub, vb); ctx.stroke()
        for (const [u, v] of [[ua, va], [ub, vb]]) if (!top || v < top[1]) top = [u, v]
      }
      if (top && state.overlay.labels && !(seg.boxes2d[k] ?? []).length) {
        label(ctx, `${b.object} ${fmt(Math.hypot(b.box[0], b.box[1], b.box[2]), 1)} m`, top[0], top[1] - 20 * scale, color, scale)
      }
    }
  }
  ctx.restore()
}

function label(ctx, text, x, y, color, scale = 1) {
  const tw = ctx.measureText(text).width + 8 * scale
  ctx.fillStyle = color
  ctx.fillRect(x, y, tw, 18 * scale)
  ctx.fillStyle = '#111'
  ctx.fillText(text, x + 4 * scale, y + 13 * scale)
}

function card(badge, title, extraHead = []) {
  const info = h('span', { class: 'muted small' })
  const msg = h('div', { class: 'view-msg', hidden: true })
  const body = h('div', { class: 'view-body' }, msg)
  const el = h('div', { class: 'view-card' },
    h('div', { class: 'view-head' }, h('span', { class: 'badge ' + badge }, badge), h('span', { class: 'view-title' }, title), info, h('span', { class: 'spacer' }), ...extraHead),
    body)
  const setMsg = (text, err = false) => { msg.hidden = !text; msg.textContent = text ?? ''; msg.className = 'view-msg' + (err ? ' err' : '') }
  return { el, body, info, setMsg }
}

// ---- sensor coverage (the cones of the web app's map) -------------------------------------

// the web app's colours: cameras solid, scanners dashed
const SENSOR_COLORS = { rgb: '#8b5cf6', event: '#ea580c', lidar: '#65a30d', radar: '#0891b2' }
const sensorColor = (s) => SENSOR_COLORS[s.type] ?? '#9b9b9b'
const DEG = Math.PI / 180

/** Position, viewing direction (-Z of the Blender world matrix) and the horizontal window of a
 *  sensor at dataset frame k. `look` is a math angle (from east, counter-clockwise); the window
 *  [a0, a1] in the same convention (positive azimuth of the sensor = to its right). */
function sensorPose(s, k) {
  const m = s.worlds?.[Math.min(k, s.worlds.length - 1)] ?? s.world
  if (!m) return null
  const f = [-m[2], -m[6], -m[10]]
  const horiz = Math.hypot(f[0], f[1])
  const look = horiz > 1e-3 ? Math.atan2(f[1], f[0]) : Math.atan2(m[5], m[1]) // straight up/down: its +Y
  const pitch = Math.atan2(f[2], horiz) / DEG
  const heading = ((90 - look / DEG) % 360 + 360) % 360 // compass, clockwise from north
  const az = s.az ?? [0, 0]
  return { origin: [m[3], m[7], m[11]], look, pitch, heading, a0: look - az[1] * DEG, a1: look - az[0] * DEG }
}

function sensorText(s, p) {
  const fov = s.known ? `${fmt(s.hfov, 1)}° × ${fmt(s.vfov, 1)}°` : 'field of view unknown'
  const dir = p ? `, heading ${fmt(p.heading, 0)}°, pitch ${p.pitch >= 0 ? '+' : ''}${fmt(p.pitch, 0)}°` : ''
  const range = s.range_m ? `, ${s.min_range_m ? `${s.min_range_m}–` : ''}${s.range_m} m` : ''
  return `${s.key} (${s.type ?? '?'}${s.preset ? `, ${s.preset}` : ''}): ${fov}${dir}${range}`
}

/** The horizontal wedge of a sensor as polygon points (scene metres), out to `len`. */
function wedgePoints(p, len) {
  const span = p.a1 - p.a0
  const n = Math.max(2, Math.ceil(span / (4 * DEG)))
  const pts = [[p.origin[0], p.origin[1]]]
  for (let i = 0; i <= n; i++) {
    const a = p.a0 + (span * i) / n
    pts.push([p.origin[0] + Math.cos(a) * len, p.origin[1] + Math.sin(a) * len])
  }
  return pts
}

/** Draw every sensor's coverage into a top-down canvas: wedges to `far` metres (or the sensor's
 *  range when shorter), all-round scanners as a dashed range circle, a view line, a legend. */
function drawCoverage(ctx, toPx, scale, k, far) {
  const sensors = state.seg.coverage?.sensors ?? []
  ctx.save()
  const legend = []
  for (const s of sensors) {
    const p = sensorPose(s, k)
    if (!p) continue
    const color = sensorColor(s)
    const dashed = s.type === 'lidar' || s.type === 'radar'
    ctx.strokeStyle = color
    ctx.fillStyle = color
    ctx.lineWidth = 1.5
    ctx.setLineDash(dashed ? [6, 4] : [])
    const [ox, oy] = toPx(p.origin[0], p.origin[1])
    if (s.known && s.all_round) {
      if (s.range_m) { ctx.beginPath(); ctx.arc(ox, oy, s.range_m * scale, 0, Math.PI * 2); ctx.stroke() }
    } else if (s.known) {
      const len = s.range_m ? Math.min(s.range_m, far) : far
      const pts = wedgePoints(p, len).map(([x, y]) => toPx(x, y))
      ctx.beginPath()
      pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)))
      ctx.closePath()
      ctx.globalAlpha = 0.09; ctx.fill()
      ctx.globalAlpha = 0.85; ctx.stroke()
      ctx.globalAlpha = 1
    }
    // the optical / forward axis, shortened by the pitch like in the web app
    const axis = Math.min(far, 60 / scale + 20) * Math.max(0.15, Math.cos(p.pitch * DEG))
    ctx.setLineDash([3, 3])
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + Math.cos(p.look) * axis * scale, oy - Math.sin(p.look) * axis * scale); ctx.stroke()
    legend.push([color, sensorText(s, p)])
  }
  ctx.restore()
  return legend
}

function drawLegend(ctx, legend, H) {
  ctx.save()
  ctx.font = '15px sans-serif'
  legend.forEach(([color, text], i) => {
    const y = H - 34 - (legend.length - 1 - i) * 20
    ctx.fillStyle = 'rgba(20,20,20,0.75)'
    ctx.fillRect(6, y - 15, ctx.measureText(text).width + 32, 20)
    ctx.fillStyle = color
    ctx.fillRect(10, y - 11, 12, 12)
    ctx.fillStyle = '#d0d0d0'
    ctx.fillText(text, 28, y)
  })
  ctx.restore()
}

// ---- views ------------------------------------------------------------------------------

class CameraView {
  constructor() {
    const seg = state.seg
    const [H, W] = seg.image_shape ?? [360, 640]
    this.W = W; this.H = H
    this.scale = Math.max(1, W / 960)
    this.card = card('rgb', seg.sensors.camera ?? 'camera')
    this.canvas = h('canvas', { width: W, height: H })
    this.card.body.prepend(this.canvas)
    this.el = this.card.el
    this.img = new Latest(); this.mask = new Latest(); this.lidar = new Latest()
  }
  get pending() { return this.img.pending + this.mask.pending + this.lidar.pending > 0 }
  async update(k) {
    const seg = state.seg
    const id = state.segId
    try {
      const [img, mask, pts] = await Promise.all([
        this.img.get(api.url('image', { id, frame: k }), 'image'),
        state.overlay.mask && seg.has.masks ? this.mask.get(api.url('mask', { id, frame: k }), 'image') : null,
        state.overlay.lidar && seg.has.lidar ? this.lidar.get(api.url('lidar', { id, frame: k }), 'f32') : null,
      ])
      if (k !== state.frame || id !== state.segId) return
      const ctx = this.canvas.getContext('2d')
      ctx.drawImage(img.data, 0, 0, this.W, this.H)
      if (mask) ctx.drawImage(mask.data, 0, 0, this.W, this.H)
      if (pts) this.drawLidar(ctx, k, pts.data)
      drawBoxes(ctx, k, this.scale)
      const t = seg.frames.t[k]
      this.card.info.textContent = `frame ${k} · ${fmt(t, 2)} s · sim frame ${seg.frames.sim_frame[k]}${pts ? ` · ${pts.data.length / 6} LiDAR pts` : ''}`
      this.card.setMsg(null)
    } catch (e) {
      if (!isAbort(e)) this.card.setMsg(`frame ${k}: ${e.message}`, true)
    }
  }
  drawLidar(ctx, k, pts) {
    const seg = state.seg
    if (!seg.K || !seg.extrinsics[k]) return
    const M = vehicleToCamera(seg.extrinsics[k])
    const K = seg.K
    const n = pts.length / 6
    const uvz = []
    let zmax = 1
    const boxes = seg.boxes3d[k] ?? []
    for (let i = 0; i < n; i++) {
      const x = pts[i * 6], y = pts[i * 6 + 1], z = pts[i * 6 + 2]
      const [cx, cy, cz] = apply34(M, x, y, z)
      if (cz < 0.05) continue
      const u = K[0][0] * cx / cz + K[0][2], v = K[1][1] * cy / cz + K[1][2]
      if (u < 0 || v < 0 || u >= this.W || v >= this.H) continue
      const obj = boxes.find((b) => inBox(b.box, x, y, z))
      uvz.push([u, v, cz, obj?.object])
      zmax = Math.max(zmax, cz)
    }
    const d = state.lidarDot * this.scale
    for (const [u, v, z, obj] of uvz) {
      ctx.fillStyle = obj ? objectColor(obj) : ramp(1 - z / zmax)
      const s = obj ? d + 1 : d
      ctx.fillRect(u - s / 2, v - s / 2, s, s)
    }
  }
}

class EventView {
  constructor() {
    const seg = state.seg
    const [H, W] = seg.event_shape ?? seg.image_shape ?? [360, 640]
    this.W = W; this.H = H
    this.sameOptics = seg.image_shape && seg.image_shape[0] === H && seg.image_shape[1] === W
    const maxSub = Math.max(1, ...seg.frames.events.map((e) => e.subs))
    const options = [['f1', '1 frame'], ['f2', '2 frames'], ['f4', '4 frames'], ['f8', '8 frames']]
    for (let j = 1; j <= Math.min(maxSub, 40); j++) options.push([`s${j}`, `sub-interval ${j}`])
    const select = h('select', { class: 'small', title: 'events accumulated into the image: whole dataset frames from the current one, or one sub-interval of it', onchange: (e) => { state.eventWindow = e.target.value; this.update(state.frame) } },
      options.map(([v, l]) => h('option', { value: v, selected: v === state.eventWindow }, l)))
    this.card = card('event', seg.sensors.event ?? 'events', [select])
    this.canvas = h('canvas', { width: W, height: H })
    this.card.body.prepend(this.canvas)
    this.el = this.card.el
    this.img = new Latest()
  }
  get pending() { return this.img.pending > 0 }
  async update(k) {
    const seg = state.seg
    const id = state.segId
    const w = state.eventWindow
    const params = w.startsWith('s') ? { id, frame: k, sub: w.slice(1) } : { id, frame: k, frames: w.slice(1) }
    try {
      const r = await this.img.get(api.url('events', params), 'image')
      if (k !== state.frame || id !== state.segId) return
      const ctx = this.canvas.getContext('2d')
      ctx.drawImage(r.data, 0, 0, this.W, this.H)
      if (this.sameOptics) drawBoxes(ctx, k, Math.max(1, this.W / 960))
      const e = seg.frames.events[k]
      this.card.info.textContent = `frame ${k} · ${Number(r.headers['x-event-count'] ?? 0).toLocaleString()} events · ${e?.subs ?? 0} sub-intervals`
      this.card.setMsg(null)
    } catch (e) {
      if (!isAbort(e)) this.card.setMsg(`frame ${k}: ${e.message}`, true)
    }
  }
}

class DepthView {
  constructor() {
    const seg = state.seg
    const [H, W] = seg.image_shape ?? [360, 640]
    this.W = W; this.H = H
    this.card = card('depth', 'depth')
    this.canvas = h('canvas', { width: W, height: H })
    this.card.body.prepend(this.canvas)
    this.el = this.card.el
    this.img = new Latest()
  }
  get pending() { return this.img.pending > 0 }
  async update(k) {
    const id = state.segId
    try {
      const r = await this.img.get(api.url('depth', { id, frame: k }), 'image')
      if (k !== state.frame || id !== state.segId) return
      this.canvas.getContext('2d').drawImage(r.data, 0, 0, this.W, this.H)
      const [lo, hi] = (r.headers['x-depth-range'] ?? '').split(' ').map(Number)
      this.card.info.textContent = `frame ${k} · ${fmt(lo, 1)} – ${fmt(hi, 0)} m (log scale, near = yellow)`
      this.card.setMsg(null)
    } catch (e) {
      if (!isAbort(e)) this.card.setMsg(`frame ${k}: ${e.message}`, true)
    }
  }
}

/** Bird's-eye view in the scene frame (x east, y north): the scan, 3D boxes, paths. */
class TopView {
  constructor() {
    const seg = state.seg
    const color = h('select', { class: 'small', onchange: (e) => { state.lidarColor = e.target.value; this.draw() } },
      [['height', 'by height'], ['range', 'by range'], ['intensity', 'by intensity']].map(([v, l]) => h('option', { value: v, selected: v === state.lidarColor }, l)))
    const zoom = (f) => { this.range = (this.range ?? this.fitRange ?? 50) * f; this.draw() }
    this.card = card('lidar', seg.has.lidar ? `${seg.sensors.lidar} · top view` : 'top view (no LiDAR)', [
      seg.has.lidar ? color : null,
      h('button', { class: 'small', title: 'zoom in', onclick: () => zoom(1 / 1.5) }, '+'),
      h('button', { class: 'small', title: 'zoom out', onclick: () => zoom(1.5) }, '−'),
      h('button', { class: 'small', title: 'fit and re-centre on the sensor', onclick: () => { this.range = null; this.pan = [0, 0]; this.draw() } }, 'fit'),
    ])
    this.canvas = h('canvas', { width: 960, height: 640 })
    this.card.body.classList.add('top')
    this.card.body.prepend(this.canvas)
    this.el = this.card.el
    this.loader = new Latest()
    this.pts = null
    this.k = 0
    this.range = null
    this.fitRange = null
    this.pan = [0, 0]
    this.canvas.addEventListener('wheel', (e) => { e.preventDefault(); zoom(e.deltaY > 0 ? 1.15 : 1 / 1.15) }, { passive: false })
    let drag = null
    this.canvas.addEventListener('pointerdown', (e) => { drag = [e.clientX, e.clientY, ...this.pan]; this.canvas.setPointerCapture(e.pointerId) })
    this.canvas.addEventListener('pointermove', (e) => {
      if (!drag) return
      const s = this.canvas.width / this.canvas.clientWidth / this.scalePx
      this.pan = [drag[2] - (e.clientX - drag[0]) * s, drag[3] + (e.clientY - drag[1]) * s]
      this.draw()
    })
    this.canvas.addEventListener('pointerup', () => { drag = null })
  }
  get pending() { return this.loader.pending > 0 }
  async update(k) {
    const seg = state.seg
    const id = state.segId
    this.k = k
    if (!seg.has.lidar || !seg.frames.lidar_points[k]) { this.pts = null; this.draw(); return }
    try {
      const r = await this.loader.get(api.url('lidar', { id, frame: k }), 'f32')
      if (k !== state.frame || id !== state.segId) return
      this.pts = r.data
      this.card.setMsg(null)
      this.draw()
    } catch (e) {
      if (!isAbort(e)) this.card.setMsg(`scan ${k}: ${e.message}`, true)
    }
  }
  sensorXyz(k) {
    const P = state.seg.poses[k] ?? state.seg.lidar_world[k]
    return P ? [P[3], P[7], P[11]] : [0, 0, 0]
  }
  draw() {
    const seg = state.seg
    const k = this.k
    const c = this.canvas
    const ctx = c.getContext('2d')
    const W = c.width, H = c.height
    const pose = seg.poses[k]
    const centre0 = this.sensorXyz(k)
    // the scan in the scene frame, the points that fall inside a 3D box marked
    const n = this.pts && pose ? this.pts.length / 6 : 0
    const xyz = new Float32Array(n * 3)
    const owner = new Array(n)
    const boxes = seg.boxes3d[k] ?? []
    for (let i = 0; i < n; i++) {
      const x = this.pts[i * 6], y = this.pts[i * 6 + 1], z = this.pts[i * 6 + 2]
      const [X, Y, Z] = apply44(pose, x, y, z)
      xyz[i * 3] = X; xyz[i * 3 + 1] = Y; xyz[i * 3 + 2] = Z
      const b = boxes.find((bb) => inBox(bb.box, x, y, z))
      if (b) owner[i] = b.object
    }
    if (this.fitRange === null || this.fitFor !== seg.segment) {
      const d = []
      for (let i = 0; i < n; i += Math.max(1, Math.floor(n / 2000))) d.push(Math.hypot(xyz[i * 3] - centre0[0], xyz[i * 3 + 1] - centre0[1]))
      for (const path of Object.values(seg.paths)) for (const p of path) d.push(Math.hypot(p[1] - centre0[0], p[2] - centre0[1]) * 1.15)
      d.sort((a, b) => a - b)
      this.fitRange = Math.max(10, d[Math.floor(d.length * 0.95)] ?? 50)
      this.fitFor = seg.segment
    }
    const range = this.range ?? this.fitRange
    const centre = [centre0[0] + this.pan[0], centre0[1] + this.pan[1]]
    const scale = Math.min(W, H) / (2 * range)
    this.scalePx = scale
    const cx = W / 2, cy = H / 2
    const toPx = (x, y) => [cx + (x - centre[0]) * scale, cy - (y - centre[1]) * scale]

    ctx.fillStyle = '#141414'
    ctx.fillRect(0, 0, W, H)
    // range rings around the sensor
    const step = range > 400 ? 100 : range > 100 ? 50 : range > 30 ? 10 : 5
    const [sx, sy] = toPx(centre0[0], centre0[1])
    ctx.strokeStyle = '#2a2a2a'
    ctx.lineWidth = 1
    for (let r = step; r <= range * 2.5; r += step) { ctx.beginPath(); ctx.arc(sx, sy, r * scale, 0, Math.PI * 2); ctx.stroke() }
    // every sensor's coverage, under the points (wedges out past the canvas edge)
    const legend = state.overlay.coverage ? drawCoverage(ctx, toPx, scale, k, (Math.hypot(W, H) / scale) * 1.5) : []
    // points
    if (n) {
      let zmin = Infinity, zmax = -Infinity, imax = 1e-6
      for (let i = 0; i < n; i++) {
        const z = xyz[i * 3 + 2]
        if (z < zmin) zmin = z
        if (z > zmax) zmax = z
        imax = Math.max(imax, this.pts[i * 6 + 3])
      }
      const dot = n > 100_000 ? 1.5 : n > 30_000 ? 2 : n > 8_000 ? 2.5 : 3.5
      for (let i = 0; i < n; i++) {
        if (owner[i]) continue
        const x = xyz[i * 3], y = xyz[i * 3 + 1], z = xyz[i * 3 + 2]
        const [px, py] = toPx(x, y)
        if (px < 0 || py < 0 || px > W || py > H) continue
        const v = state.lidarColor === 'height' ? (z - zmin) / Math.max(1e-6, zmax - zmin)
          : state.lidarColor === 'range' ? Math.hypot(x - centre0[0], y - centre0[1]) / range
            : this.pts[i * 6 + 3] / imax
        ctx.fillStyle = ramp(v)
        ctx.fillRect(px - dot / 2, py - dot / 2, dot, dot)
      }
      for (let i = 0; i < n; i++) {
        if (!owner[i]) continue
        const [px, py] = toPx(xyz[i * 3], xyz[i * 3 + 1])
        ctx.fillStyle = objectColor(owner[i])
        ctx.fillRect(px - 2, py - 2, 4, 4)
      }
    }
    // paths, current positions, 3D boxes
    for (const [name, path] of Object.entries(seg.paths)) {
      const color = objectColor(name)
      if (state.overlay.paths && path.length > 1) {
        ctx.strokeStyle = color; ctx.globalAlpha = 0.5; ctx.lineWidth = 1.5
        ctx.beginPath()
        path.forEach((p, i) => { const [px, py] = toPx(p[1], p[2]); if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py) })
        ctx.stroke()
        ctx.globalAlpha = 1
      }
    }
    for (const b of boxes) {
      const color = objectColor(b.object)
      const g = b.global
      const [px, py] = toPx(g[0], g[1])
      ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.5
      ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2); ctx.stroke()
      if (state.overlay.boxes3d) {
        const corners = boxCorners(g).filter((_, i) => !(i & 1)).map(([x, y]) => toPx(x, y)) // the 4 bottom corners
        ctx.lineWidth = Math.max(1.5, Math.min(3, scale * 0.1))
        ctx.beginPath()
        ;[0, 1, 3, 2].forEach((i, j) => (j === 0 ? ctx.moveTo(...corners[i]) : ctx.lineTo(...corners[i])))
        ctx.closePath(); ctx.stroke()
        const [fx, fy] = toPx(g[0] + Math.cos(g[6]) * g[3] / 2, g[1] + Math.sin(g[6]) * g[3] / 2)
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(fx, fy); ctx.stroke()
      }
      if (state.overlay.labels) {
        ctx.font = '13px sans-serif'
        const dz = g[2] - centre0[2]
        ctx.fillText(`${b.object} ${dz >= 0 ? '+' : ''}${fmt(dz, 1)} m${b.num_points != null ? ` · ${b.num_points} pts` : ''}`, px + 10, py - 8)
      }
    }
    // the scanner: a triangle pointing along the vehicle x axis (its viewing direction)
    const fwd = pose ? Math.atan2(pose[4], pose[0]) : Math.PI / 2
    ctx.save()
    ctx.translate(sx, sy)
    ctx.rotate(Math.PI / 2 - fwd)
    ctx.fillStyle = '#ffffff'
    ctx.beginPath(); ctx.moveTo(0, -11); ctx.lineTo(7, 7); ctx.lineTo(0, 3); ctx.lineTo(-7, 7); ctx.closePath(); ctx.fill()
    ctx.restore()
    ctx.fillStyle = '#9b9b9b'
    ctx.font = '12px sans-serif'
    ctx.fillText('N ↑', 10, 18)
    drawLegend(ctx, legend, H)
    ctx.fillRect(10, H - 16, step * scale, 2)
    ctx.fillText(`${step} m`, 16 + step * scale, H - 11)
    const objPts = owner.filter(Boolean).length
    this.card.info.textContent = seg.has.lidar
      ? `scan ${k} · ${n.toLocaleString()} pts${n ? ` · ${objPts} on objects` : ''} · ±${fmt(range, 0)} m`
      : `±${fmt(range, 0)} m · boxes and paths only`
  }
}

/** Leaflet map: the scene origin, the sensor, the drones' paths and where they are now. */
class MapView {
  constructor() {
    const seg = state.seg
    this.card = card('map', seg.location.name ?? 'map')
    this.card.body.classList.add('map')
    this.div = h('div')
    this.card.body.prepend(this.div)
    this.el = this.card.el
    this.markers = {}
    this.card.info.textContent = seg.location.lat != null ? `${fmt(seg.location.lat, 5)}, ${fmt(seg.location.lon, 5)}` : ''
  }
  get pending() { return false }
  toLatLng(x, y) {
    const { lat, lon } = state.seg.location
    return [lat + y / 111320, lon + x / (111320 * Math.cos((lat * Math.PI) / 180))]
  }
  mount() {
    const seg = state.seg
    if (typeof L === 'undefined') { this.card.setMsg('Leaflet could not be loaded (no internet in the browser?)', true); return }
    if (seg.location.lat == null) { this.card.setMsg('the segment has no location', true); return }
    const map = L.map(this.div, { zoomControl: true, attributionControl: true })
    this.map = map
    const sat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 20, maxNativeZoom: 19, attribution: 'Imagery © Esri' })
    const osm = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 20, maxNativeZoom: 19, attribution: '© OpenStreetMap contributors' })
    sat.addTo(map)
    L.control.layers({ satellite: sat, OpenStreetMap: osm }, {}, { position: 'topright' }).addTo(map)
    const bounds = [this.toLatLng(0, 0)]
    L.circleMarker(this.toLatLng(0, 0), { radius: 3, color: '#ffffff', weight: 1, fillOpacity: 0.6 }).bindTooltip('scene origin').addTo(map)
    for (const s of seg.coverage?.sensors ?? []) {
      const p = sensorPose(s, 0)
      if (p) bounds.push(this.toLatLng(p.origin[0], p.origin[1]))
    }
    this.coverage = L.layerGroup().addTo(map)
    // the cones only need redrawing per frame when a sensor moves
    this.moving = (seg.coverage?.sensors ?? []).some((s) => s.worlds && s.worlds.some((m) => m[3] !== s.worlds[0][3] || m[7] !== s.worlds[0][7] || m[2] !== s.worlds[0][2]))
    this.drawCoverage(state.frame)
    for (const [name, path] of Object.entries(seg.paths)) {
      const color = objectColor(name)
      const lls = path.map((p) => this.toLatLng(p[1], p[2]))
      bounds.push(...lls)
      L.polyline(lls, { color, weight: 2, opacity: 0.8 }).addTo(map)
      this.markers[name] = L.circleMarker(lls[0], { radius: 6, color, weight: 2, fillOpacity: 0.9 }).bindTooltip(name).addTo(map)
    }
    map.fitBounds(L.latLngBounds(bounds).pad(0.4), { maxZoom: 19 })
  }
  /** One wedge per sensor out to past the farthest drone (or the sensor's range when shorter),
   *  a dashed range circle for all-round scanners, the viewing axis and a marker per payload. */
  drawCoverage(k) {
    if (!this.coverage) return
    this.coverage.clearLayers()
    this.drawnFor = k
    const seg = state.seg
    const sensors = seg.coverage?.sensors ?? []
    const payloads = new Map()
    for (const s of sensors) {
      const p = sensorPose(s, k)
      if (!p) continue
      if (!payloads.has(s.payload)) payloads.set(s.payload, { origin: p.origin, lines: [] })
      payloads.get(s.payload).lines.push(sensorText(s, p))
      if (!state.overlay.coverage) continue
      let reach = 40
      for (const path of Object.values(seg.paths)) for (const q of path) reach = Math.max(reach, Math.hypot(q[1] - p.origin[0], q[2] - p.origin[1]) * 1.3)
      const color = sensorColor(s)
      const dash = s.type === 'lidar' || s.type === 'radar' ? '6 4' : undefined
      const tip = sensorText(s, p) + (s.detail ? ` · ${s.detail}` : '') + (s.pose === 'config' ? ' · pose from the scene config' : '')
      if (s.known && s.all_round) {
        if (s.range_m) L.circle(this.toLatLng(p.origin[0], p.origin[1]), { radius: s.range_m, color, weight: 1.5, dashArray: dash, fill: false }).bindTooltip(escHtml(tip), { sticky: true }).addTo(this.coverage)
      } else if (s.known) {
        const len = s.range_m ? Math.min(s.range_m, reach) : reach
        L.polygon(wedgePoints(p, len).map(([x, y]) => this.toLatLng(x, y)), { color, weight: 1.5, fillOpacity: 0.12, dashArray: dash })
          .bindTooltip(escHtml(tip), { sticky: true }).addTo(this.coverage)
      }
      const axis = 40 * Math.max(0.15, Math.cos(p.pitch * DEG))
      L.polyline([this.toLatLng(p.origin[0], p.origin[1]), this.toLatLng(p.origin[0] + Math.cos(p.look) * axis, p.origin[1] + Math.sin(p.look) * axis)],
        { color, weight: 2, dashArray: '4 3' }).bindTooltip(escHtml(tip), { sticky: true }).addTo(this.coverage)
    }
    for (const [name, pl] of payloads) {
      L.circleMarker(this.toLatLng(pl.origin[0], pl.origin[1]), { radius: 6, color: '#ffffff', weight: 2, fillColor: '#a070e0', fillOpacity: 0.9 })
        .bindTooltip(`<b>${escHtml(name)}</b><br>${pl.lines.map(escHtml).join('<br>')}`).addTo(this.coverage)
    }
  }
  redrawCoverage() { this.drawCoverage(state.frame) }
  update(k) {
    if (this.moving && this.drawnFor !== k) this.drawCoverage(k)
    for (const [name, path] of Object.entries(state.seg.paths)) {
      const m = this.markers[name]
      if (!m) continue
      const p = path.find((q) => q[0] === k) ?? path.reduce((best, q) => (Math.abs(q[0] - k) < Math.abs(best[0] - k) ? q : best), path[0])
      m.setLatLng(this.toLatLng(p[1], p[2]))
    }
  }
}

class InfoView {
  constructor() {
    this.objects = h('div')
    this.el = h('div', { class: 'view-card' },
      h('div', { class: 'view-head' }, h('span', { class: 'badge' }, 'info'), h('span', { class: 'view-title' }, 'objects at this frame')),
      this.objects)
  }
  get pending() { return false }
  update(k) {
    const seg = state.seg
    const rows = (seg.boxes3d[k] ?? []).map((b) => {
      const b2 = (seg.boxes2d[k] ?? []).find((x) => x.object === b.object)
      const [x, y, z, l, w, hh, yaw] = b.box
      return h('tr', {},
        h('td', {}, h('span', { class: 'swatch', style: `background:${objectColor(b.object)}` }), b.object, h('span', { class: 'muted' }, ` ${b.class}`)),
        h('td', { class: 'num' }, `[${fmt(x, 1)}, ${fmt(y, 1)}, ${fmt(z, 1)}]`),
        h('td', { class: 'num' }, `${fmt(Math.hypot(x, y, z), 1)} m`),
        h('td', { class: 'num' }, `${fmt(l, 2)}×${fmt(w, 2)}×${fmt(hh, 2)}`),
        h('td', { class: 'num' }, `${fmt((yaw * 180) / Math.PI, 0)}°`),
        h('td', { class: 'num' }, `${fmt(Math.hypot(b.box[7], b.box[8]), 1)} m/s`),
        h('td', { class: 'num' }, b.num_points ?? '–'),
        h('td', {}, b.in_fov == null ? '–' : b.in_fov ? 'yes' : 'no'),
        h('td', { class: 'num' }, b2 ? `${b2.bbox.map((v) => fmt(v, 0)).join(', ')} · vis ${fmt(b2.visibility, 2)}` : '–'))
    })
    this.objects.replaceChildren(rows.length
      ? h('table', {}, h('tr', {}, ...['object', 'centre (vehicle) m', 'dist', 'l×w×h m', 'heading', 'speed', 'LiDAR pts', 'in FOV', '2D box'].map((t) => h('th', {}, t))), ...rows)
      : h('div', { class: 'muted' }, 'no annotated object at this frame'))
  }
}

function metaCard() {
  const seg = state.seg
  const prop = (k, v) => h('tr', {}, h('th', {}, k), h('td', {}, v ?? '–'))
  const ev = seg.frames.events.reduce((s, e) => s + e.count, 0)
  const pts = seg.frames.lidar_points.reduce((s, n) => s + n, 0)
  return h('div', { class: 'view-card' },
    h('div', { class: 'view-head' }, h('span', { class: 'badge' }, 'scene'), h('span', { class: 'view-title' }, seg.segment)),
    h('div', {},
      h('table', {},
        prop('scene', seg.scene),
        prop('split', seg.split),
        prop('location', `${seg.location.name ?? ''} (${fmt(seg.location.lat, 5)}, ${fmt(seg.location.lon, 5)})`),
        prop('local time', seg.datetime.local?.replace('T', ' ')),
        prop('sun', seg.sun.elevation_deg != null ? `${fmt(seg.sun.elevation_deg, 1)}° up, azimuth ${fmt(seg.sun.azimuth_deg, 0)}°` : null),
        prop('frames', `${seg.n_frames} dataset frames of ${seg.n_sim_frames} sim frames at ${seg.fps} fps (${seg.duration_s} s)`),
        prop('camera', seg.sensors.camera ? `${seg.sensors.camera} · ${seg.image_shape?.[1]}×${seg.image_shape?.[0]} · f = ${fmt(seg.K?.[0][0], 1)} px` : null),
        prop('event camera', seg.sensors.event ? `${seg.sensors.event} · ${seg.event_shape?.[1]}×${seg.event_shape?.[0]} · ${seg.substeps} substeps · window ${seg.event_window} · ${ev.toLocaleString()} events` : null),
        prop('LiDAR', seg.sensors.lidar ? `${seg.sensors.lidar} · fov filter ${seg.lidar_fov} · ${pts.toLocaleString()} pts` : null),
        prop('objects', Object.entries(seg.objects).map(([n, o]) => `${n} (${o.class ?? ''}, mask ${o.index ?? '–'})`).join(', ')),
        prop('extras', Object.entries(seg.has).filter(([, v]) => v).map(([k]) => k).join(', ')),
        prop('engine', `${seg.engine ?? ''} · linasim ${seg.linasim_version ?? ''}`),
        prop('source run', h('code', {}, seg.source_run ?? '')),
        prop('file', h('code', {}, seg.file))),
      sensorTable(),
      h('details', {}, h('summary', {}, 'segment_meta.json'), h('pre', {}, JSON.stringify(seg.meta, null, 2)))))
}

function sensorTable() {
  const cov = state.seg.coverage
  if (!cov?.sensors?.length) return null
  const POSE = { recorded: 'recorded in the segment', relative: 'relative to a recorded sensor of the payload', config: 'scene config (static payload)' }
  return h('div', { style: 'margin-top:8px' },
    h('div', { class: 'muted small' }, `sensors at frame 0 (from ${cov.source})`),
    h('table', {},
      h('tr', {}, ...['sensor', 'type', 'FOV h × v', 'heading', 'pitch', 'range', 'pose'].map((t) => h('th', {}, t))),
      cov.sensors.map((s) => {
        const p = sensorPose(s, 0)
        return h('tr', { title: s.detail ?? '' },
          h('td', {}, h('span', { class: 'swatch', style: `background:${sensorColor(s)}` }), s.key),
          h('td', {}, `${s.type ?? '?'}${s.preset ? ` · ${s.preset}` : ''}`),
          h('td', { class: 'num' }, s.known ? `${fmt(s.hfov, 1)}° × ${fmt(s.vfov, 1)}°` : 'unknown'),
          h('td', { class: 'num' }, p ? `${fmt(p.heading, 0)}°` : '–'),
          h('td', { class: 'num' }, p ? `${p.pitch >= 0 ? '+' : ''}${fmt(p.pitch, 0)}°` : '–'),
          h('td', { class: 'num' }, s.range_m ? `${s.min_range_m ? `${s.min_range_m}–` : ''}${s.range_m} m` : '–'),
          h('td', { class: 'muted' }, POSE[s.pose] ?? s.pose ?? ''))
      })))
}

// ---- scene page --------------------------------------------------------------------------

async function openScene(id, frame = 0) {
  stop()
  state.segId = id
  renderSidebar()
  const content = $('#content')
  content.replaceChildren(h('div', { class: 'empty' }, 'loading the segment…'))
  try {
    state.seg = await api.segment(id)
  } catch (e) {
    content.replaceChildren(h('div', { class: 'empty bad' }, `could not open ${id}: ${e.message}`))
    return
  }
  if (state.segId !== id) return
  state.frame = Math.max(0, Math.min(frame, state.seg.n_frames - 1))
  renderScene()
}

function toggle(group, key, text, title, onChange) {
  return h('label', { title }, h('input', {
    type: 'checkbox', checked: state[group][key], onchange: (e) => {
      state[group][key] = e.target.checked
      save(group, state[group])
      onChange?.()
    },
  }), text)
}

function renderScene() {
  const seg = state.seg
  const content = $('#content')
  const n = seg.n_frames
  const slider = h('input', { type: 'range', min: 0, max: Math.max(0, n - 1), value: state.frame, oninput: (e) => { stop(); setFrame(Number(e.target.value)) } })
  const frameLabel = h('span', { class: 'frame-label mono' })
  const playBtn = h('button', { onclick: () => (state.playing ? stop() : play()) }, '▶ play')
  const speed = h('select', { onchange: (e) => { state.speed = Number(e.target.value) } },
    [0.1, 0.25, 0.5, 1, 2, 4].map((s) => h('option', { value: s, selected: s === state.speed }, `${s}×`)))
  const timeline = h('canvas', { height: 14 })
  timeline.addEventListener('click', (e) => { stop(); setFrame(Math.round((e.offsetX / timeline.clientWidth) * (n - 1))) })

  const rebuild = () => renderScene()
  const redraw = () => updateViews()
  const head = h('div', { class: 'scene-head' },
    h('h2', {}, seg.segment),
    h('span', { class: 'muted' }, `${seg.location.name ?? ''} · ${seg.datetime.local?.replace('T', ' ') ?? ''}`),
    h('span', { class: 'muted' }, `${n} frames · ${seg.duration_s} s`),
    seg.split ? h('span', { class: 'badge ' + seg.split }, seg.split) : null,
    h('span', { class: 'spacer' }),
    h('button', { class: 'small', onclick: () => { stop(); location.hash = '' } }, 'all scenes'))
  const has = seg.has
  const toolbar = h('div', { class: 'toolbar' },
    h('span', { class: 'group' }, h('span', { class: 'muted' }, 'views'),
      has.rgb ? toggle('show', 'camera', 'camera', 'the RGB camera frame', rebuild) : null,
      has.events ? toggle('show', 'events', 'events', 'the event camera, accumulated into an image', rebuild) : null,
      toggle('show', 'top', has.lidar ? 'LiDAR top view' : 'top view', 'bird\'s-eye view in the scene frame: the scan, the 3D boxes and the paths', rebuild),
      has.depth ? toggle('show', 'depth', 'depth', 'z-depth of the camera', rebuild) : null,
      toggle('show', 'map', 'map', 'the paths on a satellite map', rebuild),
      toggle('show', 'info', 'info', 'objects at this frame and the scene metadata', rebuild)),
    h('span', { class: 'group' }, h('span', { class: 'muted' }, 'overlays'),
      toggle('overlay', 'boxes2d', '2D boxes', has.boxes2d ? 'the recorded 2D boxes (solid) and mask bounds (dashed)' : 'this segment has no recorded 2D boxes', redraw),
      toggle('overlay', 'boxes3d', '3D boxes', 'the 3D boxes projected into the image and drawn in the top view', redraw),
      has.masks ? toggle('overlay', 'mask', 'masks', 'object-index masks over the camera image', redraw) : null,
      has.lidar ? toggle('overlay', 'lidar', 'LiDAR in camera', 'the scan projected into the camera image, coloured by depth (near = yellow), object points in the object colour', redraw) : null,
      toggle('overlay', 'paths', 'paths', 'the drones\' paths in the top view', redraw),
      toggle('overlay', 'coverage', 'sensor coverage', 'every sensor\'s horizontal field of view and viewing direction on the map and in the top view (dashed: LiDAR and radar)', () => { redraw(); views.forEach((v) => v.redrawCoverage?.()) }),
      toggle('overlay', 'labels', 'labels', 'names and distances next to the boxes', redraw)))
  const playback = h('div', { class: 'playback' },
    h('button', { title: 'previous frame (←)', onclick: () => { stop(); setFrame(state.frame - 1) } }, '◀'),
    playBtn,
    h('button', { title: 'next frame (→)', onclick: () => { stop(); setFrame(state.frame + 1) } }, '▶'),
    speed, slider, frameLabel)

  views = []
  if (state.show.camera && has.rgb) views.push(new CameraView())
  if (state.show.events && has.events) views.push(new EventView())
  if (state.show.top) views.push(new TopView())
  if (state.show.depth && has.depth) views.push(new DepthView())
  if (state.show.map) views.push(new MapView())
  const info = state.show.info ? new InfoView() : null
  if (info) views.push(info)

  content.replaceChildren(head, toolbar, playback, h('div', { class: 'timeline' }, timeline),
    h('div', { class: 'views' }, views.filter((v) => !(v instanceof InfoView)).map((v) => v.el)),
    info ? h('div', { class: 'info' }, info.el, metaCard()) : null)
  for (const v of views) v.mount?.()

  renderScene.ui = { slider, frameLabel, playBtn, timeline }
  drawTimeline()
  updateViews()
}

function drawTimeline() {
  const ui = renderScene.ui
  if (!ui) return
  const seg = state.seg
  const c = ui.timeline
  c.width = c.clientWidth * devicePixelRatio || 600
  c.height = 14 * devicePixelRatio
  const ctx = c.getContext('2d')
  const n = seg.n_frames
  const W = c.width, H = c.height
  const w = W / Math.max(1, n)
  ctx.fillStyle = '#222'
  ctx.fillRect(0, 0, W, H)
  const evMax = Math.max(1, ...seg.frames.events.map((e) => e.count))
  for (let k = 0; k < n; k++) {
    const e = seg.frames.events[k]
    if (e?.count) { ctx.fillStyle = '#b4521f'; const bh = (e.count / evMax) * H * 0.6; ctx.fillRect(k * w, H - bh, Math.max(1, w - 1), bh) }
    const visible = (seg.boxes3d[k] ?? []).some((b) => b.in_fov)
    if (visible) { ctx.fillStyle = '#4772b3'; ctx.fillRect(k * w, 0, Math.max(1, w - 1), H * 0.3) }
  }
  ctx.fillStyle = '#ffffff'
  ctx.fillRect(state.frame * w, 0, Math.max(2, w), H)
  c.title = 'blue: an object is inside the camera image · orange: events per frame'
}

function setFrame(k) {
  const n = state.seg?.n_frames ?? 0
  if (!n) return
  state.frame = ((k % n) + n) % n
  updateViews()
  history.replaceState(null, '', `#${esc(state.segId)}@${state.frame}`)
}

function updateViews() {
  const ui = renderScene.ui
  const seg = state.seg
  if (!ui || !seg) return
  ui.slider.value = state.frame
  ui.frameLabel.textContent = `${state.frame} / ${seg.n_frames - 1} · ${fmt(seg.frames.t[state.frame], 2)} s`
  drawTimeline()
  for (const v of views) v.update(state.frame)
}

// ---- playback ---------------------------------------------------------------------------

let last = 0
function play() {
  state.playing = true
  renderScene.ui && (renderScene.ui.playBtn.textContent = '❚❚ pause', renderScene.ui.playBtn.classList.add('active'))
  last = performance.now()
  clearTimeout(timer)
  tick()
}
function stop() {
  state.playing = false
  clearTimeout(timer)
  if (renderScene.ui) { renderScene.ui.playBtn.textContent = '▶ play'; renderScene.ui.playBtn.classList.remove('active') }
}
// a timer rather than requestAnimationFrame: the frame period (50 ms at 20 fps) is coarse, and
// rAF is throttled in background or embedded panes. The next frame waits for every view to load.
let timer = 0
function tick() {
  if (!state.playing || !state.seg) return
  const t = state.seg.frames.t
  const k = state.frame
  const dt = (t[k + 1] ?? t[k] + 1 / state.seg.fps) - t[k] || 0.05
  const now = performance.now()
  const wait = (dt * 1000) / state.speed - (now - last)
  if (wait <= 0 && !views.some((v) => v.pending)) {
    last = now
    setFrame(k + 1)
    prefetch(state.frame + 1)
    prefetch(state.frame + 2)
  }
  timer = setTimeout(tick, Math.max(5, Math.min(wait, 50)))
}
function prefetch(k) {
  const seg = state.seg
  if (!seg || k >= seg.n_frames) return
  const id = state.segId
  const quiet = (p) => p.catch(() => {})
  if (state.show.camera && seg.has.rgb) quiet(fetchAsset(api.url('image', { id, frame: k }), 'image'))
  if ((state.show.top || state.overlay.lidar) && seg.has.lidar) quiet(fetchAsset(api.url('lidar', { id, frame: k }), 'f32'))
}

// ---- scene list and gallery ------------------------------------------------------------

function filtered() {
  const q = state.query.toLowerCase()
  return (state.dataset?.segments ?? []).filter((s) => {
    if (state.split && s.split !== state.split) return false
    if (!q) return true
    return [s.segment, s.scene, s.dir, s.location?.name, s.datetime?.local, ...(s.objects ?? [])].some((v) => String(v ?? '').toLowerCase().includes(q))
  })
}

function sensorBadges(s) {
  if (!s.ok) return [h('span', { class: 'badge error', title: s.error }, 'unreadable')]
  return [
    s.split ? h('span', { class: 'badge ' + s.split }, s.split) : null,
    s.has?.rgb ? h('span', { class: 'badge rgb' }, 'rgb') : null,
    s.has?.events ? h('span', { class: 'badge event' }, 'event') : null,
    s.has?.lidar ? h('span', { class: 'badge lidar' }, 'lidar') : null,
    s.has?.depth ? h('span', { class: 'badge depth' }, 'depth') : null,
    s.has?.masks ? h('span', { class: 'badge mask' }, 'mask') : null,
  ]
}
const subline = (s) => (s.ok ? `${s.location?.name ?? ''} · ${s.datetime?.local?.slice(0, 16).replace('T', ' ') ?? ''} · ${s.n_frames} fr` : s.error)
const thumb = (s, w) => (s.ok && s.has?.rgb ? h('img', { src: api.url('thumb', { id: s.id, w }), loading: 'lazy', alt: '' }) : h('div', { class: 'noimg' }))

function renderSidebar() {
  const side = $('#sidebar')
  const list = filtered()
  if (!state.dataset) { side.replaceChildren(h('div', { class: 'empty' }, 'loading…')); return }
  if (!list.length) { side.replaceChildren(h('div', { class: 'empty' }, state.dataset.segments.length ? 'no scene matches the filter' : 'no segment-*.h5 found under the dataset root')); return }
  const groups = new Map()
  for (const s of list) {
    if (!groups.has(s.dir)) groups.set(s.dir, [])
    groups.get(s.dir).push(s)
  }
  const names = Object.fromEntries(state.dataset.groups.map((g) => [g.dir, g.name]))
  side.replaceChildren(...[...groups].map(([dir, items]) => h('div', {},
    groups.size > 1 || dir !== '.' ? h('div', { class: 'group-head' }, names[dir] ?? dir, h('span', { class: 'muted small' }, `${items.length} · ${dir}`)) : null,
    items.map((s) => h('div', { class: 'scene-item' + (s.id === state.segId ? ' active' : ''), title: s.id, onclick: () => { location.hash = esc(s.id) } },
      thumb(s, 160),
      h('div', { style: 'min-width:0' }, h('div', { class: 'name' }, s.segment.replace(/^segment-/, '')), h('div', { class: 'sub' }, subline(s)), h('div', { class: 'badges' }, sensorBadges(s))))))))
}

function renderGallery() {
  stop()
  state.segId = null
  state.seg = null
  views = []
  renderScene.ui = null
  renderSidebar()
  const list = filtered()
  const ds = state.dataset
  const frames = list.reduce((s, x) => s + (x.n_frames ?? 0), 0)
  const secs = list.reduce((s, x) => s + (x.duration_s ?? 0), 0)
  const places = new Set(list.map((s) => s.location?.name).filter(Boolean))
  $('#content').replaceChildren(
    h('div', { class: 'summary-line' }, `${list.length} scene${list.length === 1 ? '' : 's'} · ${frames.toLocaleString()} dataset frames · ${fmt(secs, 1)} s simulated · ${places.size} location${places.size === 1 ? '' : 's'} (${[...places].join(', ')})`),
    list.length
      ? h('div', { class: 'gallery' }, list.map((s) => h('div', { class: 'card', title: s.id, onclick: () => { location.hash = esc(s.id) } },
        thumb(s, 480),
        h('div', { class: 'body' }, h('div', { class: 'name' }, s.segment.replace(/^segment-/, '')), h('div', { class: 'muted small' }, subline(s)),
          h('div', { class: 'muted small' }, s.ok ? `${(s.objects ?? []).join(', ')} · sun ${fmt(s.sun?.elevation_deg, 0)}° up` : ''),
          h('div', { class: 'badges' }, sensorBadges(s))))))
      : h('div', { class: 'empty' }, ds.segments.length ? 'no scene matches the filter' : `no segment-*.h5 found under ${ds.root}`))
}

// ---- routing ----------------------------------------------------------------------------

function route() {
  const raw = location.hash.slice(1)
  if (!raw) { renderGallery(); return }
  const at = raw.lastIndexOf('@')
  const id = decodeURIComponent(at >= 0 ? raw.slice(0, at) : raw)
  const frame = at >= 0 ? Number(raw.slice(at + 1)) || 0 : 0
  if (id === state.segId && state.seg) { stop(); setFrame(frame); return }
  openScene(id, frame)
}

async function loadDataset() {
  try {
    state.dataset = await api.dataset()
  } catch (e) {
    $('#content').replaceChildren(h('div', { class: 'empty bad' }, `could not read the dataset: ${e.message}`))
    return
  }
  const ds = state.dataset
  $('#root').textContent = `${ds.root} · ${ds.segments.length} segment${ds.segments.length === 1 ? '' : 's'}`
  document.title = `${ds.name} · Dataset Viewer`
  renderSidebar()
}

$('#search').addEventListener('input', (e) => { state.query = e.target.value; renderSidebar(); if (!state.segId) renderGallery() })
$('#split-filter').addEventListener('change', (e) => { state.split = e.target.value; renderSidebar(); if (!state.segId) renderGallery() })
$('#reload').addEventListener('click', async () => { cache.clear(); await loadDataset(); if (state.segId) openScene(state.segId, state.frame); else renderGallery() })
window.addEventListener('hashchange', route)
window.addEventListener('resize', () => drawTimeline())
window.addEventListener('keydown', (e) => {
  if (!state.seg || (e.target instanceof Element && e.target.matches('input[type=search], select'))) return
  if (e.key === 'ArrowRight') { stop(); setFrame(state.frame + 1); e.preventDefault() }
  else if (e.key === 'ArrowLeft') { stop(); setFrame(state.frame - 1); e.preventDefault() }
  else if (e.key === ' ') { state.playing ? stop() : play(); e.preventDefault() }
})

await loadDataset()
route()
