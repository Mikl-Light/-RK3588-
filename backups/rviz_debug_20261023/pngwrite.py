#!/usr/bin/env python3
"""纯 Python PNG 写出（无第三方依赖），用于裁剪/放大截图做目视检查。"""
import zlib, struct


def write_png(path, w, h, rgb):
    """rgb: bytes, len = w*h*3"""
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw += rgb[y * w * 3:(y + 1) * w * 3]

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xFFFFFFFF)

    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) +
                           chunk(b'IDAT', zlib.compress(bytes(raw), 9)) + chunk(b'IEND', b''))


def crop_scale(im, x0, y0, x1, y1, k=3):
    w, h = x1 - x0, y1 - y0
    out = bytearray(w * k * h * k * 3)
    for yy in range(h * k):
        sy = y0 + yy // k
        row = yy * w * k * 3
        for xx in range(w * k):
            r, g, b = im.px(x0 + xx // k, sy)
            i = row + xx * 3
            out[i] = r; out[i + 1] = g; out[i + 2] = b
    return w * k, h * k, bytes(out)
