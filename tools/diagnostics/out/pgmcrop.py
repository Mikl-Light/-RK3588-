"""裁剪 PGM 指定窗口并放大成 PNG，用于检查墙壁重影。纯标准库。"""
import sys, zlib, struct


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # 解析 P5 头
    idx = 0
    fields = []
    while len(fields) < 4:
        # 跳过空白与注释
        while idx < len(data) and data[idx:idx+1].isspace():
            idx += 1
        if data[idx:idx+1] == b'#':
            while idx < len(data) and data[idx:idx+1] != b'\n':
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx+1].isspace():
            idx += 1
        fields.append(data[start:idx])
    idx += 1  # 单个空白分隔符
    assert fields[0] == b'P5', fields[0]
    w, h, maxv = int(fields[1]), int(fields[2]), int(fields[3])
    pix = data[idx:idx + w * h]
    return w, h, pix


def color_of(v):
    if v >= 250:
        return (255, 255, 255)
    if v <= 20:
        return (200, 30, 40)      # 占据用红色，方便看墙
    return (235, 235, 238)


def write_png(path, W, H, rows):
    raw = b''.join(b'\x00' + r for r in rows)
    comp = zlib.compress(raw, 9)

    def chunk(typ, data):
        c = struct.pack('>I', len(data)) + typ + data
        return c + struct.pack('>I', zlib.crc32(typ + data) & 0xFFFFFFFF)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', W, H, 8, 2, 0, 0, 0)))
        f.write(chunk(b'IDAT', comp))
        f.write(chunk(b'IEND', b''))


def main():
    src = sys.argv[1]
    x0, y0, cw, ch = (int(v) for v in sys.argv[2:6])
    scale = int(sys.argv[6]) if len(sys.argv) > 6 else 10
    dst = sys.argv[7] if len(sys.argv) > 7 else 'crop.png'

    w, h, pix = read_pgm(src)
    x1, y1 = min(x0 + cw, w), min(y0 + ch, h)
    x0, y0 = max(0, x0), max(0, y0)

    W, H = (x1 - x0) * scale, (y1 - y0) * scale
    rows = []
    for yy in range(H):
        gy = y0 + yy // scale
        row = bytearray()
        for xx in range(W):
            gx = x0 + xx // scale
            v = pix[gy * w + gx]
            r, g, b = color_of(v)
            row += bytes((r, g, b))
        rows.append(bytes(row))
    write_png(dst, W, H, rows)
    print('裁剪 原图%dx%d 窗口(%d,%d)-(%d,%d) -> %dx%d  %s' % (w, h, x0, y0, x1, y1, W, H, dst))


main()
