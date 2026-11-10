#!/usr/bin/env node
// linasim dataset viewer: `node server.js <dataset_root> [--port 8700] [--host 127.0.0.1] [--token T]`
// Serves the page in public/ and a small JSON/PNG API over the segment-*.h5 files below the root.
import crypto from 'node:crypto'
import fs from 'node:fs'
import http from 'node:http'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import zlib from 'node:zlib'
import { Dataset } from './lib/dataset.js'
import { SegmentError, ready } from './lib/segment.js'
import { loadPresets, sceneSensors } from './lib/sensors.js'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const PUBLIC = path.join(HERE, 'public')

function usage(code = 0) {
  console.log(`usage: node server.js <dataset_root> [options]

  <dataset_root>       folder with segment-*.h5 files (searched up to 4 levels deep)

options:
  --port N             port to listen on (default 8700, env VIEWER_PORT)
  --host H             address to bind (default 127.0.0.1, env VIEWER_HOST);
                       0.0.0.0 to reach it from other machines (e.g. a cluster node)
  --token T            access token (env VIEWER_TOKEN); generated automatically when the
                       host is not a loopback address, "none" disables it
  --max-width N        downscale camera images wider than N pixels (default 1280, 0 = never)
  --catalog FILE       simulator catalog.json (e.g. 09_simulator_config/data/catalog.json) with
                       the sensor presets for the coverage cones; default lib/presets.json`)
  process.exit(code)
}

function parseArgs(argv) {
  const opts = {
    root: null, port: Number(process.env.VIEWER_PORT ?? 8700), host: process.env.VIEWER_HOST ?? '127.0.0.1',
    token: process.env.VIEWER_TOKEN ?? null, maxWidth: 1280, catalog: process.env.VIEWER_CATALOG ?? null,
  }
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i]
    const next = () => { if (i + 1 >= argv.length) usage(2); return argv[++i] }
    if (a === '-h' || a === '--help') usage(0)
    else if (a === '--port' || a === '-p') opts.port = Number(next())
    else if (a === '--host') opts.host = next()
    else if (a === '--token') opts.token = next()
    else if (a === '--max-width') opts.maxWidth = Number(next())
    else if (a === '--catalog') opts.catalog = next()
    else if (a.startsWith('-')) { console.error(`unknown option ${a}`); usage(2) }
    else if (!opts.root) opts.root = a
    else usage(2)
  }
  if (!opts.root) usage(2)
  return opts
}

const opts = parseArgs(process.argv.slice(2))
if (!fs.existsSync(opts.root) || !fs.statSync(opts.root).isDirectory()) {
  console.error(`error: ${opts.root} is not a directory`)
  process.exit(1)
}
const loopback = ['127.0.0.1', 'localhost', '::1'].includes(opts.host)
if (opts.token === 'none') opts.token = null
else if (!opts.token && !loopback) opts.token = crypto.randomBytes(12).toString('hex')

await ready
const dataset = new Dataset(opts.root)
const presets = loadPresets(opts.catalog)

// ---- helpers ----------------------------------------------------------------------

const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png' }

function send(req, res, status, body, type, extra = {}) {
  const headers = { 'Content-Type': type, ...extra }
  const gzip = typeof body !== 'undefined' && body.length > 1024 && !type.startsWith('image/') && /\bgzip\b/.test(req.headers['accept-encoding'] ?? '')
  if (gzip) { body = zlib.gzipSync(body, { level: 4 }); headers['Content-Encoding'] = 'gzip' }
  res.writeHead(status, headers)
  res.end(body)
}

const json = (req, res, obj, status = 200) => send(req, res, status, Buffer.from(JSON.stringify(obj)), 'application/json')
const png = (req, res, buf, extra = {}) => send(req, res, 200, buf, 'image/png', { 'Cache-Control': 'private, max-age=600', ...extra })

function authorised(req, url) {
  if (!opts.token) return { ok: true }
  if (url.searchParams.get('token') === opts.token) return { ok: true, setCookie: true }
  const cookies = Object.fromEntries((req.headers.cookie ?? '').split(';').map((c) => c.trim().split('=')).filter((p) => p.length === 2))
  return { ok: cookies.viewer_token === opts.token }
}

function int(v, fallback) {
  const n = Number(v)
  return Number.isFinite(n) ? Math.trunc(n) : fallback
}

// ---- routes -----------------------------------------------------------------------

function api(req, res, url) {
  const p = url.pathname
  const q = url.searchParams
  if (p === '/api/dataset') return json(req, res, dataset.overview())
  const seg = () => dataset.segment(q.get('id'))
  const k = () => int(q.get('frame'), 0)
  const maxWidth = q.has('max_width') ? int(q.get('max_width'), opts.maxWidth) : opts.maxWidth
  switch (p) {
    case '/api/segment': {
      const s = seg()
      s._coverage ??= sceneSensors(s.path, s.summary(), presets)
      return json(req, res, { ...s.summary(), coverage: s._coverage })
    }
    case '/api/thumb': {
      const buf = seg().thumbnailPng(int(q.get('w'), 320))
      return buf ? png(req, res, buf) : send(req, res, 404, Buffer.from('no image'), 'text/plain')
    }
    case '/api/image': return png(req, res, seg().imagePng(k(), maxWidth))
    case '/api/events': {
      const out = seg().eventPng(k(), { frames: int(q.get('frames'), 1), sub: int(q.get('sub'), 0) })
      return png(req, res, out.png, { 'X-Event-Count': String(out.count) })
    }
    case '/api/mask': return png(req, res, seg().maskPng(k(), maxWidth))
    case '/api/depth': {
      const out = seg().depthPng(k(), maxWidth)
      return png(req, res, out.png, { 'X-Depth-Range': `${out.min} ${out.max}` })
    }
    case '/api/lidar': return send(req, res, 200, seg().lidarBytes(k()), 'application/octet-stream', { 'Cache-Control': 'private, max-age=600' })
    default: return json(req, res, { error: `no route ${p}` }, 404)
  }
}

function serveStatic(req, res, url) {
  const rel = url.pathname === '/' ? 'index.html' : decodeURIComponent(url.pathname).replace(/^\/+/, '')
  const abs = path.resolve(PUBLIC, rel)
  if (!abs.startsWith(PUBLIC + path.sep) || !fs.existsSync(abs) || !fs.statSync(abs).isFile()) {
    return send(req, res, 404, Buffer.from('not found'), 'text/plain')
  }
  send(req, res, 200, fs.readFileSync(abs), MIME[path.extname(abs)] ?? 'application/octet-stream', { 'Cache-Control': 'no-cache' })
}

const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://x')
  const t0 = Date.now()
  try {
    const auth = authorised(req, url)
    if (!auth.ok) return send(req, res, 401, Buffer.from('Open the URL with ?token=… printed by the server.'), 'text/plain')
    if (auth.setCookie) {
      url.searchParams.delete('token')
      res.writeHead(302, { 'Set-Cookie': `viewer_token=${opts.token}; HttpOnly; SameSite=Strict; Path=/`, Location: url.pathname + url.search })
      return res.end()
    }
    if (req.method !== 'GET') return send(req, res, 405, Buffer.from('GET only'), 'text/plain')
    if (url.pathname.startsWith('/api/')) api(req, res, url)
    else serveStatic(req, res, url)
  } catch (err) {
    const status = err.status ?? (err instanceof SegmentError ? 404 : 500)
    if (status === 500) console.error(err)
    if (!res.headersSent) json(req, res, { error: String(err.message ?? err) }, status)
  } finally {
    if (process.env.VIEWER_LOG) console.log(`${req.method} ${req.url} ${res.statusCode} ${Date.now() - t0} ms`)
  }
})

server.listen(opts.port, opts.host, () => {
  const n = dataset.scan().length
  const suffix = opts.token ? `/?token=${opts.token}` : '/'
  console.log(`linasim dataset viewer: ${dataset.root} (${n} segment${n === 1 ? '' : 's'})`)
  if (loopback) {
    console.log(`open http://localhost:${opts.port}${suffix}`)
  } else {
    const host = os.hostname()
    console.log(`listening on ${opts.host}:${opts.port} on ${host}`)
    console.log(`open http://${host}:${opts.port}${suffix}`)
    console.log(`or tunnel from your machine:  ssh -N -L ${opts.port}:${host}:${opts.port} ${process.env.VIEWER_SSH_HOST || '<login-node>'}`)
    console.log(`and open http://localhost:${opts.port}${suffix}`)
  }
})
