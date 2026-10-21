#!/usr/bin/python3



from enum import Enum

from skin_real.scn.core import mask
from skin_real.scn.sc.pkt.tools import SC_ID_ALL,set_id


class UdrMode(Enum):
    UDR_0HZ = 0
    UDR_63HZ = 1
    UDR_125HZ = 2
    UDR_250HZ = 3
    UDR_1HZ  = 16
    UDR_2HZ  = 17
    UDR_5HZ  = 18
    UDR_10HZ = 19
    UDR_20HZ = 20
    UDR_50HZ = 21


def udr(mode : UdrMode, id : int = SC_ID_ALL) -> bytes:
    pkt = bytearray([
        0xC8,
        0x00,0x00,
        mode.value & mask(7), \
        0x00,0x00,0x00,0x00, \
        0x00,0x00,0x00,0x00, \
        0x00,0x00,0x00,0x00, \
        0x00,0x00,0x00,0xAA
    ])
    set_id(pkt,id)
    return bytes(pkt)


