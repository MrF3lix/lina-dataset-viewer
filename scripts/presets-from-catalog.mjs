#!/usr/bin/env node
// Regenerate lib/presets.json (the sensor presets' optics and coverage) from the simulator
// catalog that the GPU worker publishes to the web app:
//
//   node scripts/presets-from-catalog.mjs ../09_simulator_config/data/catalog.json
//
// Only what the coverage cones need is kept: camera optics, LiDAR / radar angular window and
// range. The server can also read a catalog at start-up (`--catalog`), which wins over this file.
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { presetsFromCatalog } from '../lib/sensors.js'

const src = process.argv[2]
if (!src) {
  console.error('usage: node scripts/presets-from-catalog.mjs <catalog.json>')
  process.exit(2)
}
const out = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'lib', 'presets.json')
const presets = presetsFromCatalog(JSON.parse(fs.readFileSync(src, 'utf8')))
fs.writeFileSync(out, JSON.stringify(presets, null, 1) + '\n')
console.log(`wrote ${Object.keys(presets).length} presets to ${out}: ${Object.keys(presets).join(', ')}`)
