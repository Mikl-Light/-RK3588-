#!/usr/bin/env python3
"""纯 Python PNG 读取（无第三方依赖）。仅支持 8bit RGB/RGBA/灰度、非隔行。
用途：在没有 pillow 的环境里做图像统计。"""
import zlib, struct


def read_png(path):
    d = open(path, 'rb').read()
    assert d[:8] == b'\x89PNG\r\n\x1a\n', 'not png'
    pos = 8
    idat = b''
    w = h = bit = ct = None
    while pos < len(d):
        ln = struct.unpack('>I', d[pos:pos + 4])[0]
        typ = d[pos + 4:pos + 8]
        body = d[pos + 8:pos + 8 + ln]
        if typ == b'IHDR':
            w, h, bit, ct, comp, filt, inter = struct.unpack('>IIBBBBB', body)
            assert bit == 8 and inter == 0, f'unsupported bit={bit} interlace={inter}'
        elif typ == b'IDAT':
            idat += body
        elif typ == b'IEND':
            break
        pos += 12 + ln
    nch = {0: 1, 2: 3, 4: 2, 6: 4}[ct]
    raw = zlib.decompress(idat)
    stride = w * nch
    out = bytearray(h * stride)
    prev = bytearray(stride)
    p = 0
    for y in range(h):
        f = raw[p]; p += 1
        line = bytearray(raw[p:p + stride]); p += stride
        if f == 1:
            for i in range(nch, stride):
                line[i] = (line[i] + line[i - nch]) & 0xFF
        elif f == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif f == 3:
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif f == 4:
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                b = prev[i]
                c = prev[i - nch] if i >= nch else 0
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 0xFF
        out[y * stride:(y + 1) * stride] = line
        prev = line
    return w, h, nch, bytes(out)


class Img:
    def __init__(self, path):
        self.w, self.h, self.nch, self.buf = read_png(path)

    def px(self, x, y):
        i = (y * self.w + x) * self.nch
        b = self.buf
        if self.nch >= 3:
            return b[i], b[i + 1], b[i + 2]
        return b[i], b[i], b[i]

    def lum(self, x, y):
        r, g, bl = self.px(x, y)
        return (r * 299 + g * 587 + bl * 114) // 1000
