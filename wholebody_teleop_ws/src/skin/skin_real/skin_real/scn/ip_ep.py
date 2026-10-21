#!/usr/bin/python3


import ipaddress
import math
import socket
import time

from typing import List, Dict, Tuple


def from_str(ip_ep : str) -> Tuple[str,int]:
    parts = ip_ep.split(":")
    if len(parts) != 2:
        return None
    ip = ipaddress.IPv4Address(parts[0])
    port = int(parts[1])
    if port < 0 or (port > ((1<<16)-1)):
        return None
    return (str(ip),port)


def to_str(ip_ep : Tuple[str,int]):
    return f"{ip_ep[0]}:{ip_ep[1]}"