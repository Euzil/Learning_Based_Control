#!/usr/bin/python3

from skin_real.scn.core import mask
from skin_real.scn.sc.pkt.tools import SC_ID_ALL,set_id


def led_rgb(r : int, g : int, b :int, id : int = SC_ID_ALL) -> bytes:
    m7 = mask(7)
    m1 = mask(1)

    pkt = bytearray([
        0xCA,
        0x00,0x00,
        (r >> 1) & m7, r & m1,
        (g >> 1) & m7, g & m1,
        (b >> 1) & m7, b & m1,
        0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0xAA
    ])
    set_id(pkt,id)
    return bytes(pkt)


def led_rgb_val(val : int, id : int = SC_ID_ALL) -> bytes:
    m8 = mask(8)
    r = (val >> 16) & m8
    g = (val >> 8) & m8
    b = (val >> 0) & m8
    return led_rgb(r,g,b,id)

