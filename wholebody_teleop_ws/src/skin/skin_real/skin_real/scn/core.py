#!/usr/bin/python3


import ipaddress
import math
import socket
import time

from typing import List, Dict, Tuple



def mask(width : int ) -> int:
    """ Mask to select the specified number of LSB bits. """
    return (1 << width) - 1

def mask_to_indices(mask : int, n_bits = 32) -> List[int]:
    inds = []
    for i in range(n_bits):
        if mask & (1 << i) != 0:
            inds.append(i)
    return inds


def intx_to_int(value,width) :
    mask = 0xFFFFFFFF >> (32-width)
    value &= mask

    if(value & (1 << (width-1))):
        value |= ~mask
        return value

    return value


def print_hex_block(d : bytes):
    for i in range(len(d)):
        print(f"{d[i]:02x} ",end="")
        if ((i+1) % 16) == 0 and i+1 < len(d):
            print("")
    print("")


def color_rgb_to_val(r : int, g : int, b : int) -> int:
    m8 = mask(8)
    val = 0x00000000
    val |= (r & m8) << 16
    val |= (g & m8) << 8
    val |= b & m8
    return val

def color_val_to_rgb(val : int) -> Tuple[int,int,int]:
    m8 = mask(8)
    r = (val >> 16) & m8
    g = (val >> 8) & m8
    b = val & m8
    return (r,g,b)