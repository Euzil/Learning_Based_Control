#!/usr/bin/python3

from skin_real.scn.core import mask
from skin_real.scn.sc.pkt.tools import SC_ID_ALL,set_id


ACC_RANGE_2G = 0
ACC_RANGE_4G = 1
ACC_RANGE_8G = 2
ACC_RANGE_16G = 3


def acc_range(range: int, id : int = SC_ID_ALL) -> bytes:
    m7 = mask(7)
    
    pkt = bytearray([
        0xEB,
        0x00,0x00,
        0x01,
        range & m7,
        0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0xAA
    ])
    set_id(pkt,id)
    return bytes(pkt)
