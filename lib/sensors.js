// Every sensor of a segment's scene with its angular coverage and its pose, for the coverage
// cones on the map and in the top view.
//
// The segment file records only the exported camera and LiDAR (their Blender world matrices per
// frame, the camera's K). The rest comes from the assembled dataset's `dataset.json` next to the
// segment (the scene config of every item: payloads, sensors, presets, mounts) resolved against
// the sensor presets (lib/presets.json, or a simulator catalog given with --catalog), with the
// same rules as the web app (09_simulator_config/frontend/src/geometry.ts: cameraFov, lidarFov,
// radarFov). Poses, best first:
//   recorded  the segment's own matrices (camera_world, lidar_world)
//   relative  another sensor of a payload with a recorded sensor: recorded · mount⁻¹ · mount'
//   config    a static payload's configured position and rotation (T · Rz·Ry·Rx · mount)
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const DEG = Math.PI / 180
const HERE = path.dirname(fileURLToPath(import.meta.url))

// ---- presets ------------------------------------------------------------------------------

/** name -> the fields the cones need, from a catalog ({catalog: {sensor_presets}} or {sensor_presets}). */
export function presetsFromCatalog(catalog) {
  const list = catalog?.catalog?.sensor_presets ?? catalog?.sensor_presets ?? []
  const pick = (o, keys) => (o ? Object.fromEntries(keys.filter((k) => o[k] !== undefined).map((k) => [k, o[k]])) : undefined)
  const out = {}
  for (const p of list) {
    out[p.name] = {
      type: p.type,
      ...pick(p, ['resolution', 'focal_length_mm', 'sensor_width_mm', 'clip_start_m', 'clip_end_m']),
      lidar: pick(p.lidar, ['backend', 'pattern', 'fov_h_deg', 'fov_v_deg', 'range_m', 'min_range_m']),
      pattern_info: pick(p.pattern_info, ['name', 'fov_h_deg', 'fov_v_deg', 'az_min_deg', 'az_max_deg', 'el_min_deg', 'el_max_deg']),
      radar: pick(p.radar, ['schedule', 'fov_az_deg', 'fov_el_deg', 'el_center_deg', 'elevation_beams', 'beamwidth_el_deg', 'min_range_m', 'max_range_m']),
    }
  }
  return out
}

export function loadPresets(catalogPath) {
  let presets = {}
  try { presets = JSON.parse(fs.readFileSync(path.join(HERE, 'presets.json'), 'utf8')) } catch { /* none bundled */ }
  if (catalogPath) presets = { ...presets, ...presetsFromCatalog(JSON.parse(fs.readFileSync(catalogPath, 'utf8'))) }
  return presets
}

// ---- coverage of one sensor entry -----------------------------------------------------------

const r1 = (v) => (v == null ? null : Math.round(v * 10) / 10)

/** { type, hfov, vfov, az: [min, max], el: [min, max], range_m, min_range_m, all_round, detail } */
export function coverage(entry, preset) {
  const type = entry.type ?? preset?.type ?? null
  if (type === 'rgb' || type === 'event') {
    const focal = entry.focal_length_mm ?? preset?.focal_length_mm
    const width = entry.sensor_width_mm ?? preset?.sensor_width_mm
    const res = entry.resolution ?? preset?.resolution
    if (!focal || !width) return { type, known: false }
    const hfov = (2 * Math.atan(width / (2 * focal))) / DEG
    const vfov = res ? (2 * Math.atan((width * res[1]) / res[0] / (2 * focal))) / DEG : null
    return {
      type, known: true, hfov: r1(hfov), vfov: r1(vfov), az: [-hfov / 2, hfov / 2], el: vfov ? [-vfov / 2, vfov / 2] : null,
      range_m: entry.clip_end_m ?? preset?.clip_end_m ?? null, min_range_m: null, all_round: false,
      detail: `${focal} mm on ${width} mm${res ? `, ${res[0]}×${res[1]}` : ''}`,
    }
  }
  if (type === 'lidar') {
    const pl = preset?.lidar ?? {}
    const el = entry.lidar ?? {}
    const backend = String(el.backend ?? pl.backend ?? 'blainder') === 'pattern' ? 'pattern' : 'blainder'
    const range = Number(el.range_m ?? pl.range_m ?? 100)
    const minRange = Number(el.min_range_m ?? pl.min_range_m ?? 0)
    if (backend === 'pattern') {
      const info = preset?.pattern_info
      if (!info?.fov_h_deg) return { type, known: false, range_m: range, detail: 'pattern without pattern_info' }
      const az = info.az_min_deg != null ? [info.az_min_deg, info.az_max_deg] : [-info.fov_h_deg / 2, info.fov_h_deg / 2]
      const elw = info.el_min_deg != null ? [info.el_min_deg, info.el_max_deg] : [-info.fov_v_deg / 2, info.fov_v_deg / 2]
      return {
        type, known: true, hfov: r1(info.fov_h_deg), vfov: r1(info.fov_v_deg), az, el: elw, range_m: range, min_range_m: minRange,
        all_round: info.fov_h_deg >= 359.9, detail: `pattern ${info.name ?? ''}`.trim(),
      }
    }
    const fh = Number(el.fov_h_deg ?? pl.fov_h_deg ?? 360), fv = Number(el.fov_v_deg ?? pl.fov_v_deg ?? 30)
    return {
      type, known: true, hfov: r1(fh), vfov: r1(fv), az: [-fh / 2, fh / 2], el: [-fv / 2, fv / 2], range_m: range, min_range_m: minRange,
      all_round: fh >= 359.9, detail: 'rotating (BlAInder)',
    }
  }
  if (type === 'radar') {
    const pr = preset?.radar ?? {}
    const er = entry.radar ?? {}
    const get = (k, d) => Number(er[k] ?? pr[k] ?? d)
    const schedule = String(er.schedule ?? pr.schedule ?? 'rotating') === 'esa' ? 'esa' : 'rotating'
    const range = get('max_range_m', 5000), minRange = get('min_range_m', 20)
    const elCenter = get('el_center_deg', 0), bwEl = get('beamwidth_el_deg', 20)
    if (schedule === 'esa') {
      const fa = get('fov_az_deg', 120), fe = get('fov_el_deg', 80)
      return {
        type, known: true, hfov: r1(fa), vfov: r1(fe), az: [-fa / 2, fa / 2], el: [elCenter - fe / 2, elCenter + fe / 2],
        range_m: range, min_range_m: minRange, all_round: false, detail: 'scanned array',
      }
    }
    const beams = er.elevation_beams ?? pr.elevation_beams
    const elw = beams?.length ? [Math.min(...beams) - bwEl / 2, Math.max(...beams) + bwEl / 2] : [elCenter - bwEl / 2, elCenter + bwEl / 2]
    return {
      type, known: true, hfov: 360, vfov: r1(elw[1] - elw[0]), az: [-180, 180], el: elw, range_m: range, min_range_m: minRange,
      all_round: true, detail: 'rotating',
    }
  }
  return { type, known: false }
}

// ---- matrices (row-major 4x4 as 16 numbers) -----------------------------------------------

function eulerXyz([x, y, z]) {
  const cx = Math.cos(x * DEG), sx = Math.sin(x * DEG), cy = Math.cos(y * DEG), sy = Math.sin(y * DEG), cz = Math.cos(z * DEG), sz = Math.sin(z * DEG)
  return [ // Rz @ Ry @ Rx (Blender XYZ Euler)
    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
    [-sy, cy * sx, cy * cx],
  ]
}

function poseMatrix(position = [0, 0, 0], rotation = [0, 0, 0]) {
  const r = eulerXyz(rotation)
  return [r[0][0], r[0][1], r[0][2], position[0], r[1][0], r[1][1], r[1][2], position[1], r[2][0], r[2][1], r[2][2], position[2], 0, 0, 0, 1]
}
const mountMatrix = (m) => poseMatrix(m?.position ?? [0, 0, 0], m?.rotation ?? [0, 0, 0])

function mul(a, b) {
  const out = new Array(16).fill(0)
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) for (let k = 0; k < 4; k++) out[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j]
  return out
}

function invRigid(m) {
  const R = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]]
  const t = [m[3], m[7], m[11]]
  const out = new Array(16).fill(0)
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) out[i * 4 + j] = R[j][i]
    out[i * 4 + 3] = -(R[0][i] * t[0] + R[1][i] * t[1] + R[2][i] * t[2])
  }
  out[15] = 1
  return out
}

const round6 = (m) => m.map((v) => Math.round(v * 1e6) / 1e6)

// ---- the scene config of a segment ---------------------------------------------------------

/** The scene config of `segmentName` from the dataset.json beside the segment file, or null. */
export function sceneConfig(segPath, segmentName) {
  let ds
  try { ds = JSON.parse(fs.readFileSync(path.join(path.dirname(segPath), 'dataset.json'), 'utf8')) } catch { return null }
  const bare = String(segmentName ?? '').replace(/^segment-/, '')
  const seg = (ds.segments ?? []).find((s) => s.segment === segmentName)
  const item = seg?.item ?? (ds.items ?? []).find((i) => i.name === bare)?.id
  return item ? ds.scenes?.[item] ?? null : null
}

/** Every sensor of the segment's scene with coverage and pose (see the header). */
export function sceneSensors(segPath, summary, presets) {
  const recorded = { [summary.sensors.camera]: summary.camera_world, [summary.sensors.lidar]: summary.lidar_world }
  const scene = sceneConfig(segPath, summary.segment)
  const out = []
  if (scene) {
    for (const pl of scene.payloads ?? []) {
      const sensors = pl.sensors ?? []
      // a recorded sensor of this payload, to place the others relative to it
      const ref = sensors.map((s) => ({ s, frames: recorded[`${pl.name}.${s.name}`] })).find((r) => r.frames?.length)
      for (const s of sensors) {
        const key = `${pl.name}.${s.name}`
        const preset = s.preset ? presets[s.preset] : undefined
        const cov = coverage(s, preset)
        const entry = { key, payload: pl.name, name: s.name, preset: s.preset ?? null, attach_to: pl.attach_to ?? null, ...cov }
        if (!cov.type && !preset) entry.detail = `preset ${s.preset} unknown to the viewer (update lib/presets.json or pass --catalog)`
        if (recorded[key]?.length) {
          entry.pose = 'recorded'
          entry.worlds = recorded[key]
        } else if (ref) {
          const rel = mul(invRigid(mountMatrix(ref.s.mount)), mountMatrix(s.mount))
          entry.pose = 'relative'
          entry.worlds = ref.frames.map((m) => round6(mul(m, rel)))
        } else if (pl.trajectory && !pl.attach_to) {
          entry.pose = 'config'
          entry.world = round6(mul(poseMatrix(pl.trajectory.position, pl.trajectory.rotation), mountMatrix(s.mount)))
        } else {
          continue // on a drone without a recorded sensor: no pose to draw
        }
        out.push(entry)
      }
    }
  }
  // without a scene config (or for a recorded sensor it doesn't list): what the segment itself says
  const have = new Set(out.map((s) => s.key))
  const camKey = summary.sensors.camera
  if (camKey && !have.has(camKey) && summary.camera_world.length) {
    const [H, W] = summary.image_shape ?? [0, 0]
    const f = summary.K?.[0]?.[0]
    const hfov = f && W ? (2 * Math.atan(W / 2 / f)) / DEG : null
    const vfov = f && H ? (2 * Math.atan(H / 2 / f)) / DEG : null
    out.push({
      key: camKey, payload: camKey.split('.')[0], name: camKey.split('.').slice(1).join('.'), type: 'rgb', known: Boolean(hfov),
      hfov: r1(hfov), vfov: r1(vfov), az: hfov ? [-hfov / 2, hfov / 2] : null, el: vfov ? [-vfov / 2, vfov / 2] : null,
      range_m: null, all_round: false, detail: `from K, ${W}×${H}`, pose: 'recorded', worlds: summary.camera_world,
    })
  }
  const lidarKey = summary.sensors.lidar
  if (lidarKey && !have.has(lidarKey) && summary.lidar_world.length) {
    out.push({
      key: lidarKey, payload: lidarKey.split('.')[0], name: lidarKey.split('.').slice(1).join('.'), type: 'lidar', known: false,
      detail: 'field of view unknown (no dataset.json beside the segment)', pose: 'recorded', worlds: summary.lidar_world,
    })
  }
  return { source: scene ? 'dataset.json' : 'segment only', sensors: out }
}
