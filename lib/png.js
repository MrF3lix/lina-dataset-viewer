// Minimal PNG encoder (8-bit RGB or RGBA, no filtering) on top of node:zlib, plus a box
// downscale, so the server needs no native image library.
import zlib from 'node:zlib'

const CRC_TABLE = (() => {
  const t = new Uint32Array(256)
  for (let n = 0; n < 256; n++) {
    let c = n
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1
    t[n] = c >>> 0
  }
  return t
})()

function crc32(buf) {
  let c = 0xffffffff
  for (let i = 0; i < buf.length; i++) c = CRC_TABLE[(c ^ buf[i]) & 0xff] ^ (c >>> 8)
  return (c ^ 0xffffffff) >>> 0
}

function chunk(type, data) {
  const len = Buffer.alloc(4)
  len.writeUInt32BE(data.length)
  const td = Buffer.concat([Buffer.from(type, 'ascii'), data])
  const crc = Buffer.alloc(4)
  crc.writeUInt32BE(crc32(td))
  return Buffer.concat([len, td, crc])
}

/** Encode `pixels` (Uint8Array, row-major, `channels` 3 or 4 per pixel) as a PNG buffer. */
export function encodePng(pixels, width, height, channels = 3, level = 1) {
  const stride = width * channels
  const raw = Buffer.alloc((stride + 1) * height)
  for (let y = 0; y < height; y++) {
    raw[y * (stride + 1)] = 0 // filter: none
    Buffer.from(pixels.buffer, pixels.byteOffset + y * stride, stride).copy(raw, y * (stride + 1) + 1)
  }
  const ihdr = Buffer.alloc(13)
  ihdr.writeUInt32BE(width, 0)
  ihdr.writeUInt32BE(height, 4)
  ihdr[8] = 8
  ihdr[9] = channels === 4 ? 6 : 2
  ihdr[10] = 0
  ihdr[11] = 0
  ihdr[12] = 0
  return Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
    chunk('IHDR', ihdr),
    chunk('IDAT', zlib.deflateSync(raw, { level })),
    chunk('IEND', Buffer.alloc(0)),
  ])
}

/** Integer box-filter downscale so that the width is at most `maxWidth`. */
export function downscale(pixels, width, height, channels, maxWidth) {
  if (!maxWidth || width <= maxWidth) return { pixels, width, height }
  const f = Math.ceil(width / maxWidth)
  const w = Math.floor(width / f)
  const h = Math.floor(height / f)
  const out = new Uint8Array(w * h * channels)
  const n = f * f
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      for (let c = 0; c < channels; c++) {
        let s = 0
        for (let dy = 0; dy < f; dy++) {
          const row = ((y * f + dy) * width + x * f) * channels + c
          for (let dx = 0; dx < f; dx++) s += pixels[row + dx * channels]
        }
        out[(y * w + x) * channels + c] = (s / n + 0.5) | 0
      }
    }
  }
  return { pixels: out, width: w, height: h }
}
