"""PGM(P5) -> 放大后的 PNG，零第三方依赖（只用标准库 zlib/struct）。

nav2 map_saver 输出的 PGM 是 ROS 惯例配色：
  0   = 占据（障碍）
  205 = 未知
  254 = 自由
直接看原图太小（121x81），放大后叠加坐标刻度更便于判读。
"""

import struct
import sys
import zlib


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()

    # 头部：P5 <w> <h> <maxval>\n，允许任意空白分隔与 # 注释
    tokens = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    i += 1  # 跳过分隔符

    assert tokens[0] == b'P5', '只支持二进制 PGM(P5)'
    w, h = int(tokens[1]), int(tokens[2])
    maxval = int(tokens[3])
    assert maxval < 256, '只支持 8bit PGM'

    pix = data[i:i + w * h]
    assert len(pix) == w * h, '像素数据长度不符：%d != %d' % (len(pix), w * h)
    return w, h, pix


def write_png(path, w, h, rgb_rows):
    """rgb_rows: 每行一个 bytes（长度 = w*3）"""
    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload
                + struct.pack('>I', zlib.crc32(tag + payload) & 0xFFFFFFFF))

    raw = b''.join(b'\x00' + row for row in rgb_rows)   # filter type 0
    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9))
           + chunk(b'IEND', b''))
    with open(path, 'wb') as f:
        f.write(png)


def color_of(v):
    """ROS map 三色 -> RGB。顺手把占据画成醒目的深红，便于与未知区分。"""
    if v >= 250:
        return (255, 255, 255)        # 自由 - 白
    if v <= 20:
        return (25, 25, 30)           # 占据 - 近黑
    return (150, 150, 155)            # 未知 - 灰


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'map_walk.pgm'
    scale = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    dst = sys.argv[3] if len(sys.argv) > 3 else 'map_walk_big.png'

    w, h, pix = read_pgm(src)

    n_occ = sum(1 for v in pix if v <= 20)
    n_free = sum(1 for v in pix if v >= 250)
    n_unk = w * h - n_occ - n_free
    print('尺寸 %dx%d  占据 %d (%.1f%%)  自由 %d (%.1f%%)  未知 %d (%.1f%%)'
          % (w, h, n_occ, 100.0 * n_occ / (w * h),
             n_free, 100.0 * n_free / (w * h),
             n_unk, 100.0 * n_unk / (w * h)))

    # 预留左边和下边的刻度带
    pad_l, pad_b = 46, 34
    W, H = w * scale + pad_l, h * scale + pad_b
    bg = (245, 246, 248)

    rows = []
    for y in range(H):
        row = bytearray()
        for x in range(W):
            gx = (x - pad_l) // scale
            gy = (y) // scale
            if x < pad_l or y >= h * scale:
                row += bytes(bg)
            else:
                row += bytes(color_of(pix[gy * w + gx]))
        rows.append(bytes(row))

    # 每 5 个格子（=25cm）画一条淡网格线
    for gx in range(0, w + 1, 5):
        x = pad_l + gx * scale
        if x >= W:
            continue
        for y in range(0, h * scale):
            r = bytearray(rows[y])
            r[x * 3:x * 3 + 3] = b'\xd0\xd4\xda'
            rows[y] = bytes(r)
    for gy in range(0, h + 1, 5):
        y = gy * scale
        if y >= h * scale:
            continue
        r = bytearray(rows[y])
        for x in range(pad_l, W):
            r[x * 3:x * 3 + 3] = b'\xd0\xd4\xda'
        rows[y] = bytes(r)

    write_png(dst, W, H, rows)
    print('已写出 %s  (%dx%d, 放大 %dx, 网格 5 格 = %.2fm)'
          % (dst, W, H, scale, 5 * 0.05))


if __name__ == '__main__':
    main()
