// Find the segments under a dataset root and keep a few of them open.
import fs from 'node:fs'
import path from 'node:path'
import { Segment } from './segment.js'

const MAX_DEPTH = 4
const MAX_OPEN = 12

export class Dataset {
  constructor(root) {
    this.root = path.resolve(root)
    this.open = new Map() // abs path -> { seg, mtimeMs, size }
    this.listings = new Map() // abs path -> { key, listing } (survives closing the file)
  }

  /** Every `segment-*.h5` below the root, as { id (relative path), abs, dir, size, mtimeMs }. */
  scan() {
    const out = []
    const walk = (dir, depth) => {
      let entries
      try { entries = fs.readdirSync(dir, { withFileTypes: true }) } catch { return }
      for (const e of entries) {
        if (e.name.startsWith('.') || e.name === 'node_modules') continue
        const abs = path.join(dir, e.name)
        let st
        try { st = fs.statSync(abs) } catch { continue } // follows symlinks; skips broken ones
        if (st.isDirectory() && depth < MAX_DEPTH) walk(abs, depth + 1)
        else if (st.isFile() && e.name.endsWith('.h5') && e.name.startsWith('segment-')) {
          out.push({ id: path.relative(this.root, abs), abs, dir: path.relative(this.root, dir) || '.', size: st.size, mtimeMs: st.mtimeMs })
        }
      }
    }
    walk(this.root, 0)
    out.sort((a, b) => a.id.localeCompare(b.id))
    return out
  }

  /** dataset.json / train.txt / val.txt of a directory below the root. */
  dirInfo(rel) {
    const dir = path.join(this.root, rel)
    const info = { dir: rel, name: rel === '.' ? path.basename(this.root) : rel, splits: {} }
    try {
      const ds = JSON.parse(fs.readFileSync(path.join(dir, 'dataset.json'), 'utf8'))
      if (ds.name) info.name = ds.name
      info.created_at = ds.created_at ?? null
      info.options = ds.options ?? null
    } catch { /* not an assembled dataset */ }
    for (const split of ['train', 'val', 'test']) {
      try {
        for (const line of fs.readFileSync(path.join(dir, `${split}.txt`), 'utf8').split(/\r?\n/)) {
          if (line.trim()) info.splits[line.trim()] = split
        }
      } catch { /* no such split file */ }
    }
    return info
  }

  resolve(id) {
    const abs = path.resolve(this.root, String(id ?? ''))
    if (!abs.startsWith(this.root + path.sep) || !abs.endsWith('.h5') || !path.basename(abs).startsWith('segment-')) {
      throw Object.assign(new Error('bad segment id'), { status: 400 })
    }
    if (!fs.existsSync(abs)) throw Object.assign(new Error(`no segment ${id}`), { status: 404 })
    return abs
  }

  segment(id) {
    const abs = this.resolve(id)
    const st = fs.statSync(abs)
    const hit = this.open.get(abs)
    if (hit && hit.mtimeMs === st.mtimeMs && hit.size === st.size) {
      this.open.delete(abs)
      this.open.set(abs, hit) // most recently used last
      return hit.seg
    }
    if (hit) { hit.seg.close(); this.open.delete(abs) }
    const seg = new Segment(abs)
    this.open.set(abs, { seg, mtimeMs: st.mtimeMs, size: st.size })
    while (this.open.size > MAX_OPEN) {
      const [oldest, entry] = this.open.entries().next().value
      entry.seg.close()
      this.open.delete(oldest)
    }
    return seg
  }

  listing(entry) {
    const key = `${entry.mtimeMs}:${entry.size}`
    const cached = this.listings.get(entry.abs)
    if (cached && cached.key === key) return cached.listing
    let listing
    try {
      listing = { ...this.segment(entry.id).listing(), ok: true }
    } catch (err) {
      listing = { segment: path.basename(entry.id, '.h5'), ok: false, error: String(err.message ?? err) }
    }
    this.listings.set(entry.abs, { key, listing })
    return listing
  }

  /** The whole dataset for the scene list. */
  overview() {
    const entries = this.scan()
    const dirs = new Map()
    for (const e of entries) if (!dirs.has(e.dir)) dirs.set(e.dir, this.dirInfo(e.dir))
    const segments = entries.map((e) => {
      const l = this.listing(e)
      const split = dirs.get(e.dir).splits[l.segment] ?? l.split ?? null
      return { id: e.id, dir: e.dir, size: e.size, mtime: new Date(e.mtimeMs).toISOString(), ...l, split }
    })
    return { root: this.root, name: path.basename(this.root), groups: [...dirs.values()], segments }
  }
}
