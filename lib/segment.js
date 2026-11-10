// Read one exported segment (`segment-*.h5`, the linasim Ev-Waymo HDF5 layout of
// 08_simulator/docs/DATASET.md section 7) with h5wasm: a JSON summary for the page, and
// per-frame images, event frames, masks, depth maps and LiDAR points.
import h5wasm from 'h5wasm/node'
import { downscale, encodePng } from './png.js'

export const ready = h5wasm.ready

const OBJECT_RGB = [[255, 90, 90], [255, 179, 71], [127, 212, 255], [197, 138, 255], [138, 255, 154]]

export class SegmentError extends Error {}

/** Plain numbers from any typed array (BigInt64 included) or array. */
function nums(v) {
  if (v == null) return []
  if (typeof v === 'number' || typeof v === 'bigint') return [Number(v)]
  if (v instanceof BigInt64Array || v instanceof BigUint64Array) return Array.from(v, Number)
  if (ArrayBuffer.isView(v)) return Array.from(v)
  return Array.from(v, (x) => (typeof x === 'bigint' ? Number(x) : typeof x === 'boolean' ? Number(x) : x))
}

function rows(flat, width) {
  const out = []
  for (let i = 0; i + width <= flat.length; i += width) out.push(flat.slice(i, i + width))
  return out
}

function round(v, d = 4) {
  const f = 10 ** d
  return Math.round(v * f) / f
}

export class Segment {
  constructor(path) {
    this.path = path
    this.file = new h5wasm.File(path, 'r')
    this._summary = null
    this._eventIndex = null
  }

  close() {
    try { this.file.close() } catch { /* already closed */ }
  }

  has(path) {
    try { return this.file.get(path) != null } catch { return false }
  }

  shape(path) {
    const d = this.file.get(path)
    return d ? d.shape : null
  }

  /** The whole dataset as plain values; [] for a missing or empty one. */
  read(path) {
    const d = this.file.get(path)
    if (!d || (d.shape && d.shape.some((s) => s === 0))) return []
    return d.value
  }

  slice(path, ranges) {
    const d = this.file.get(path)
    if (!d) throw new SegmentError(`${path} missing`)
    return d.slice(ranges)
  }

  attr(name, fallback = null) {
    const a = this.file.attrs[name]
    if (!a) return fallback
    const v = a.value
    if (ArrayBuffer.isView(v)) return nums(v)
    return typeof v === 'bigint' ? Number(v) : v
  }

  get nFrames() {
    return this.shape('timestamps/t')?.[0] ?? 0
  }

  checkFrame(k) {
    if (!Number.isInteger(k) || k < 0 || k >= this.nFrames) throw new SegmentError(`frame ${k} out of range`)
  }

  // ---- summary --------------------------------------------------------------------

  /** Small listing entry: identity, place, time, sizes. */
  listing() {
    const s = this.summary()
    return {
      segment: s.segment, scene: s.scene, split: s.split, location: s.location, datetime: s.datetime,
      sun: s.sun, fps: s.fps, n_frames: s.n_frames, duration_s: s.duration_s, objects: Object.keys(s.objects),
      sensors: s.sensors, image_shape: s.image_shape, has: s.has,
    }
  }

  summary() {
    if (this._summary) return this._summary
    let meta = {}
    try { meta = JSON.parse(String(this.read('meta/segment_meta_json'))) } catch { meta = {} }
    let objects = {}
    try { objects = JSON.parse(this.attr('objects_json') || '{}') } catch { objects = meta.objects ?? {} }
    const n = this.nFrames
    const run = meta.run ?? {}

    // 3D boxes per frame and the paths (box centres in the scene frame) per object
    const b3 = 'annotations/boxes3d'
    const boxes3d = Array.from({ length: n }, () => [])
    const paths = {}
    if (this.has(`${b3}/box`) && this.shape(`${b3}/box`)[0] > 0) {
      const frame = nums(this.read(`${b3}/frame`))
      const names = nums(this.read(`${b3}/obj_name`)).map(String)
      const cls = nums(this.read(`${b3}/name`)).map(String)
      const box = rows(nums(this.read(`${b3}/box`)), 9)
      const glob = rows(nums(this.read(`${b3}/box_global`)), 9)
      const npts = this.has(`${b3}/num_points`) ? nums(this.read(`${b3}/num_points`)) : []
      const inFov = this.has(`${b3}/in_fov`) ? nums(this.read(`${b3}/in_fov`)) : []
      for (let i = 0; i < frame.length; i++) {
        const k = frame[i]
        if (k < 0 || k >= n) continue
        boxes3d[k].push({
          object: names[i], class: cls[i], box: box[i].map((v) => round(v)), global: glob[i].map((v) => round(v)),
          num_points: npts[i] ?? null, in_fov: inFov.length ? Boolean(inFov[i]) : null,
        })
        ;(paths[names[i]] ??= []).push([k, round(glob[i][0], 3), round(glob[i][1], 3), round(glob[i][2], 3)])
        if (!objects[names[i]]) objects[names[i]] = { index: null, class: cls[i] }
      }
    }

    // 2D boxes per frame
    const b2 = 'annotations/boxes2d'
    const boxes2d = Array.from({ length: n }, () => [])
    if (this.has(`${b2}/bbox`) && this.shape(`${b2}/bbox`)[0] > 0) {
      const frame = nums(this.read(`${b2}/frame`))
      const names = nums(this.read(`${b2}/obj_name`)).map(String)
      const bbox = rows(nums(this.read(`${b2}/bbox`)), 4)
      const vis = nums(this.read(`${b2}/visibility`))
      const depth = nums(this.read(`${b2}/depth`))
      const mpix = this.has(`${b2}/mask_pixels`) ? nums(this.read(`${b2}/mask_pixels`)) : []
      const mbox = this.has(`${b2}/mask_bbox`) ? rows(nums(this.read(`${b2}/mask_bbox`)), 4) : []
      for (let i = 0; i < frame.length; i++) {
        if (frame[i] < 0 || frame[i] >= n) continue
        boxes2d[frame[i]].push({
          object: names[i], bbox: bbox[i].map((v) => round(v, 2)), visibility: round(vis[i] ?? 1, 3),
          depth: round(depth[i] ?? 0, 2), mask_bbox: (mpix[i] ?? 0) > 0 && mbox[i] ? mbox[i] : null,
        })
      }
    }

    // events: the [first, last) sub-interval rows of every dataset frame
    const events = this.has('events/raw') ? this.eventIndex() : null
    const lidarNames = this.has('lidar/object_names') ? nums(this.read('lidar/object_names')).map(String) : []
    const mat = (path) => (this.has(path) ? rows(nums(this.read(path)), 16).map((m) => m.map((v) => round(v, 6))) : [])
    const imageShape = this.has('calibration/image_shape') ? nums(this.read('calibration/image_shape')) : null
    const eventShape = this.has('calibration/event_shape') ? nums(this.read('calibration/event_shape')) : null
    const intr = this.has('calibration/image_0_intrinsic') ? nums(this.read('calibration/image_0_intrinsic')) : null
    const lidarOffsets = this.has('lidar/offsets') ? nums(this.read('lidar/offsets')) : []

    const loc = run.location ?? {}
    const dt = run.datetime ?? {}
    this._summary = {
      file: this.path,
      segment: this.attr('segment') ?? meta.segment,
      scene: this.attr('scene') ?? meta.scene,
      split: this.attr('split') ?? meta.split,
      source_run: this.attr('run_dir') ?? meta.source_run,
      linasim_version: this.attr('linasim_version') ?? meta.linasim_version,
      created_at: this.attr('created_at'),
      engine: this.attr('engine'),
      location: {
        name: this.attr('location_name') ?? loc.name, lat: this.attr('location_lat') ?? loc.lat,
        lon: this.attr('location_lon') ?? loc.lon, radius_m: loc.radius_m ?? null,
      },
      datetime: { local: this.attr('datetime_local') ?? dt.local, utc: this.attr('datetime_utc') ?? dt.utc, timezone: this.attr('timezone') ?? dt.timezone },
      sun: { azimuth_deg: this.attr('sun_azimuth_deg'), elevation_deg: this.attr('sun_elevation_deg') },
      fps: this.attr('fps') ?? meta.fps,
      substeps: this.attr('substeps'),
      duration_s: this.attr('duration_s') ?? run.duration_s,
      n_sim_frames: this.attr('n_sim_frames') ?? meta.n_sim_frames,
      n_frames: n,
      sensors: { camera: this.attr('camera') || null, lidar: this.attr('lidar') || null, event: this.attr('event') || null },
      event_window: this.attr('event_window'),
      lidar_fov: this.attr('lidar_fov'),
      class_name: this.attr('class_name'),
      image_shape: imageShape,
      event_shape: eventShape,
      K: intr ? [[intr[0], intr[1], intr[2]], [intr[4], intr[5], intr[6]], [intr[8], intr[9], intr[10]]] : null,
      has: {
        rgb: this.has('images/image_0') && (this.shape('images/image_0')?.[0] ?? 0) > 0,
        events: Boolean(events && events.total > 0),
        lidar: lidarOffsets.length > 1 && lidarOffsets[lidarOffsets.length - 1] > 0,
        masks: this.has('masks'),
        depth: this.has('depth'),
        boxes2d: boxes2d.some((b) => b.length),
      },
      objects,
      frames: {
        t: nums(this.read('timestamps/t')).map((v) => round(v, 6)),
        sim_frame: nums(this.read('timestamps/sim_frame')),
        sub_intervals: this.has('timestamps/sub_intervals') ? nums(this.read('timestamps/sub_intervals')) : [],
        lidar_points: lidarOffsets.slice(1).map((v, i) => v - lidarOffsets[i]),
        events: events ? events.perFrame : [],
      },
      lidar_object_names: lidarNames,
      lidar_points_total: this.has('lidar/points_total') ? rows(nums(this.read('lidar/points_total')), Math.max(1, lidarNames.length)) : [],
      lidar_points_in_fov: this.has('lidar/points_in_fov') ? rows(nums(this.read('lidar/points_in_fov')), Math.max(1, lidarNames.length)) : [],
      poses: mat('poses'),
      extrinsics: mat('calibration/image_0_extrinsic'),
      lidar_world: mat('sensor_poses/lidar_world'),
      camera_world: mat('sensor_poses/camera_world'),
      boxes3d,
      boxes2d,
      paths,
      meta,
    }
    return this._summary
  }

  eventIndex() {
    if (this._eventIndex) return this._eventIndex
    const n = this.nFrames
    const offsets = nums(this.read('events/offsets'))
    const fi = rows(nums(this.read('events/frame_index')), 2)
    const intervals = this.has('events/intervals_us') ? rows(nums(this.read('events/intervals_us')), 2) : []
    const perFrame = Array.from({ length: n }, () => ({ first: -1, last: -1, count: 0, subs: 0 }))
    fi.forEach(([k], i) => {
      if (k < 0 || k >= n) return
      const e = perFrame[k]
      if (e.first < 0) e.first = i
      e.last = i + 1
      e.subs++
      e.count += offsets[i + 1] - offsets[i]
    })
    this._eventIndex = { offsets, intervals, perFrame, total: offsets.length ? offsets[offsets.length - 1] : 0 }
    return this._eventIndex
  }

  // ---- frames ---------------------------------------------------------------------

  imagePng(k, maxWidth) {
    this.checkFrame(k)
    const [, h, w, c] = this.shape('images/image_0')
    const px = this.slice('images/image_0', [[k, k + 1]])
    const d = downscale(px, w, h, c, maxWidth)
    return encodePng(d.pixels, d.width, d.height, c)
  }

  /** Small preview of the middle frame. */
  thumbnailPng(maxWidth = 320) {
    const n = this.nFrames
    if (!this.has('images/image_0') || !n) return null
    return this.imagePng(Math.floor(n / 2), maxWidth)
  }

  /** Events of dataset frames k .. k+frames-1 (or only sub-interval `sub` of frame k, 1-based)
   *  accumulated into an image: ON white-blue, OFF red, on grey. */
  eventPng(k, { frames = 1, sub = 0 } = {}) {
    this.checkFrame(k)
    const idx = this.eventIndex()
    const [h, w] = this.summary().event_shape ?? this.summary().image_shape ?? [360, 640]
    let i0, i1
    if (sub > 0) {
      const e = idx.perFrame[k]
      if (e.first < 0 || sub > e.subs) throw new SegmentError(`frame ${k} has no sub-interval ${sub}`)
      i0 = e.first + sub - 1
      i1 = i0 + 1
    } else {
      const ks = idx.perFrame.slice(k, k + Math.max(1, frames)).filter((e) => e.first >= 0)
      i0 = ks.length ? ks[0].first : 0
      i1 = ks.length ? ks[ks.length - 1].last : 0
    }
    const img = new Uint8Array(w * h * 3).fill(90)
    const r0 = idx.offsets[i0] ?? 0
    const r1 = idx.offsets[i1] ?? 0
    if (r1 > r0) {
      const raw = this.slice('events/raw', [[r0, r1]])
      const on = new Int32Array(w * h)
      const off = new Int32Array(w * h)
      for (let i = 0; i < raw.length; i += 4) {
        const x = Number(raw[i]), y = Number(raw[i + 1]), p = Number(raw[i + 2])
        if (x < 0 || y < 0 || x >= w || y >= h) continue
        if (p > 0) on[y * w + x]++
        else off[y * w + x]++
      }
      for (let i = 0; i < w * h; i++) {
        if (on[i] === 0 && off[i] === 0) continue
        const [r, g, b] = on[i] > off[i] ? [230, 240, 255] : off[i] > on[i] ? [230, 70, 60] : [200, 200, 200]
        img[i * 3] = r
        img[i * 3 + 1] = g
        img[i * 3 + 2] = b
      }
    }
    return { png: encodePng(img, w, h, 3), count: r1 - r0 }
  }

  /** Object-index mask as a transparent RGBA overlay in the object colours. */
  maskPng(k, maxWidth) {
    this.checkFrame(k)
    const [, h, w] = this.shape('masks')
    const m = this.slice('masks', [[k, k + 1]])
    const rgba = new Uint8Array(w * h * 4)
    for (let i = 0; i < w * h; i++) {
      const v = m[i]
      if (!v) continue
      const c = OBJECT_RGB[(v - 1) % OBJECT_RGB.length]
      rgba[i * 4] = c[0]
      rgba[i * 4 + 1] = c[1]
      rgba[i * 4 + 2] = c[2]
      rgba[i * 4 + 3] = 150
    }
    const d = downscale(rgba, w, h, 4, maxWidth)
    return encodePng(d.pixels, d.width, d.height, 4)
  }

  /** z-depth through a perceptual ramp on a log scale; black where there is no surface. */
  depthPng(k, maxWidth) {
    this.checkFrame(k)
    const [, h, w] = this.shape('depth')
    const z = this.slice('depth', [[k, k + 1]])
    let lo = Infinity, hi = 0
    for (let i = 0; i < z.length; i++) if (z[i] > 0) { lo = Math.min(lo, z[i]); hi = Math.max(hi, z[i]) }
    const out = new Uint8Array(w * h * 3)
    const l0 = Math.log(Math.max(lo, 0.1)), l1 = Math.log(Math.max(hi, lo * 1.01, 0.2))
    for (let i = 0; i < w * h; i++) {
      if (!(z[i] > 0)) continue
      const t = 1 - (Math.log(z[i]) - l0) / (l1 - l0)
      const [r, g, b] = ramp(t)
      out[i * 3] = r
      out[i * 3 + 1] = g
      out[i * 3 + 2] = b
    }
    const d = downscale(out, w, h, 3, maxWidth)
    return { png: encodePng(d.pixels, d.width, d.height, 3), min: lo, max: hi }
  }

  /** float32 (N, 6) points of frame k in the vehicle frame, as raw little-endian bytes. */
  lidarBytes(k) {
    this.checkFrame(k)
    const off = this.summary().frames.lidar_points
    const offsets = nums(this.read('lidar/offsets'))
    if (!off[k]) return Buffer.alloc(0)
    const pts = this.slice('lidar/points', [[offsets[k], offsets[k + 1]]])
    return Buffer.from(pts.buffer, pts.byteOffset, pts.byteLength)
  }
}

function ramp(v) {
  // blue -> teal -> yellow, the preview's ramp
  const t = Math.max(0, Math.min(1, v))
  return [
    Math.round(t < 0.5 ? 70 + t * 2 * 30 : 100 + (t - 0.5) * 2 * 155),
    Math.round(t < 0.5 ? 130 + t * 2 * 80 : 210 + (t - 0.5) * 2 * 30),
    Math.round(t < 0.5 ? 230 - t * 2 * 70 : 160 - (t - 0.5) * 2 * 120),
  ]
}
