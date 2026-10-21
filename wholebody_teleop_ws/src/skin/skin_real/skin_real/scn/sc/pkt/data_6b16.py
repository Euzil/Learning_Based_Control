#!/usr/bin/python3

"""
Packets with a standard 6 x 2 byte payload layout (6b16).

Bit encoding scheme: most significant bits first.

Uses Big-endian format for 16 bit words encoded in the standard layout
12b8. That is, the higher byte of the 16 bit word is repesented
by the first byte, the lower byte by the second byte.

Word layout:

W0 : D0<7:0> | D1<7:0>


Packet layout:

B0 :    <Header>
B1 :    0 | SC_ID<13:7>
B2 :    0 | SC_ID<6:0>
B3 :    0 | SUB1_ID<6:0>
B4 :    0 | SUB2_ID<6:0> 
B5 :    0 | <START: 7 bit encoded data>
...
B18 :   0 | <END: 7 bit encoded data>
B19 :   0xAA

"""

import struct

from typing import Callable, List, Dict, Union

from skin_real.scn.core import mask


def get_v0(pkt : bytes) -> int:
    v = 0
    m7 = mask(7)
    m2 = mask(2)
    v |= (pkt[5] << 9) & (m7 << 9);     # 7 bits
    v |= (pkt[6] << 2) & (m7 << 2);     # 7 bits
    v |= (pkt[7] >> 5) & m2;            # 2 bits
    return v


def get_v1(pkt : bytes) -> int:
    v = 0
    m7 = mask(7)
    m5 = mask(5)
    m4 = mask(4)
    v |= (pkt[7] << 11) & (m5 << 11);  # 5 bits
    v |= (pkt[8] << 4)  & (m7 << 4);   # 7 bits
    v |= (pkt[9] >> 3)  & m4;          # 4 bits
    return v


def get_v2(pkt : bytes) -> int:
    v = 0
    m7 = mask(7)
    m6 = mask(6)
    m3 = mask(3)
    v |= (pkt[9] << 13) & (m3 << 13);  # 3 bits
    v |= (pkt[10] << 6) & (m7 << 6);   # 7 bits
    v |= (pkt[11] >> 1) & m6;          # 6 bits
    return v


def get_v3(pkt : bytes) -> int:
    v = 0
    m7 = mask(7)
    m1 = mask(1)
    v |= (pkt[11] << 15) & (m1 << 15);  # 1 bits
    v |= (pkt[12] << 8)  & (m7 << 8);   # 7 bits
    v |= (pkt[13] << 1)  & (m7 << 1);   # 7 bits
    v |= (pkt[14] >> 6)  & m1;          # 1 bits
    return v


def get_v4(pkt : bytes) -> int:
    v = 0
    m7 = mask(7)
    m6 = mask(6)
    m3 = mask(3)
    v |= (pkt[14] << 10) & (m6 << 10);  # 6 bits
    v |= (pkt[15] << 3)  & (m7 << 3);   # 7 bits
    v |= (pkt[16] >> 4)  & m3;          # 3 bits
    return v


def get_v5(pkt : bytes) -> int:
    v = 0
    m7 = mask(7)
    m5 = mask(5)
    m4 = mask(4)
    v |= (pkt[16] << 12) & (m4 << 12);  # 4 bits
    v |= (pkt[17] << 5)  & (m7 << 5);   # 7 bits
    v |= (pkt[18] >> 2)  & m5;          # 5 bits
    return v


GET_VALUE_FUNCS : List[Callable[[bytes],int]] = [
        get_v0,
        get_v1,
        get_v2,
        get_v3,
        get_v4,
        get_v5,
    ]


def get_value(pkt : bytes, ind : int) -> int :
    if ind > 5 or ind < 0:
        raise ValueError()
    return GET_VALUE_FUNCS[ind](pkt)



def get_values(pkt : bytes) -> List[int]:
    vals = []
    vals.append(get_v0(pkt))
    vals.append(get_v1(pkt))
    vals.append(get_v2(pkt))
    vals.append(get_v3(pkt))
    vals.append(get_v4(pkt))
    vals.append(get_v5(pkt))
    return vals


def get_value_uint32(pkt : bytes, ind : int) -> int:
    if ind > 2 or ind < 0:
        raise ValueError()
    v0 = GET_VALUE_FUNCS[ind*2](pkt)
    v1 = GET_VALUE_FUNCS[ind*2+1](pkt)
    return (v1 << 16) | v0



def get_values_uint32x3(pkt : bytes) -> List[int]:
    vals = []
    vals.append(get_value_uint32(pkt,0))
    vals.append(get_value_uint32(pkt,1))
    vals.append(get_value_uint32(pkt,2))
    return vals


def get_data(pkt : bytes) -> bytes:
    vals = get_values(pkt)
    # big endian, unsigned 16 bit values
    return struct.pack(f">{len(vals)}H",*vals)


# Still only using 8 bit shift operations / assignment operation
#   use 16 bit opeartions to speed up in an 16 bit architecture
#   => not possible because of 7 bit encoding
#   BUT: less shift operations since 16 bit shifts are possible
def set_values(pkt : Union[bytes,bytearray], values : List[int]) -> bytes:
    pkt_data = pkt
    if isinstance(pkt,bytes):
        pkt_data = bytearray(pkt)
    
    d1 = values[0]
    d2 = values[1]
    d3 = values[2]
    d4 = values[3]
    d5 = values[4]
    d6 = values[5]

    m7 = mask(7)
    m6 = mask(6)
    m5 = mask(5)
    m4 = mask(4)
    m3 = mask(3)
    m2 = mask(2)
    m1 = mask(1)

    pkt_data[5]  = (d1 >> 9) & m7
    pkt_data[6]  = (d1 >> 2) & m7
    pkt_data[7]  = ((d1 & m2) << 5) | ((d2 >> 11) & m5)
    pkt_data[8]  = (d2 >> 4) & m7
    pkt_data[9]  = ((d2 & m4) << 3) | ((d3 >> 13) & m3)
    pkt_data[10] = (d3 >> 6) & m7
    pkt_data[11] = ((d3 & m6) << 1) | ((d4 >> 15) & m1)
    pkt_data[12] = (d4 >> 8) & m7
    pkt_data[13] = (d4 >> 1) & m7
    pkt_data[14] = ((d4 & m1) << 6) | ((d5 >> 10) & m6)
    pkt_data[15] = (d5 >> 3) & m7
    pkt_data[16] = ((d5 & m3) << 4) | ((d6 >> 12) & m4)
    pkt_data[17] = (d6 >> 5) & m7
    pkt_data[18] = ((d6 & m5) << 2)

    return bytes(pkt_data)


def set_data(pkt : Union[bytes,bytearray], data : bytes) -> bytes:
    # big endian, unsigned 16 bit values
    vals = struct.unpack(f">{len(data) // 2}H",data)
    return set_values(pkt,vals)


def set_values_uint32x3(pkt : Union[bytes,bytearray], values : List[int]) -> bytes:
    u16 = [0]*6
    m16 = mask(16)
    for i in range(3):
        v = values[i]
        v0 = v & m16
        v1 = (v >> 16) & m16
        u16[i*2] = v0
        u16[i*2+1] = v1
    # print(u16)
    return set_values(pkt,u16)