#!/usr/bin/python3


from skin_real.scn.core import mask


SC_ID_ALL = mask(14)


def set_id(pkt, id : int = 0, ind : int = 1) -> bytes:
    m7 = mask(7)
    m14 = mask(14)
    id &= m14
    data = pkt
    if isinstance(pkt,bytes):
        data = bytearray(pkt)

    data[ind] = (id >> 7) & m7
    data[ind+1] = id & m7

    if isinstance(pkt,bytes):
        return bytes(data)

    return data


def get_id(pkt, ind : int = 1) -> int:
    m7 = mask(7)
    id = pkt[ind+1] & m7
    id |= (pkt[ind] & m7) << 7 
    return id




def dummy_pkt(id : int = SC_ID_ALL) -> bytes:
    pkt = bytearray([
        0xFF,
        0x00,0x00,
        0x00,
        0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0xAA
    ])
    set_id(pkt,id)
    return bytes(pkt)