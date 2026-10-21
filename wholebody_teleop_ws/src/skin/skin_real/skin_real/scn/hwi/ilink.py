#!/usr/bin/python3

from abc import ABC, abstractmethod
from skin_real.scn.hwi.ilink_base import ILinkBase
from skin_real.scn.hwi.reader import Reader

class ILink(ILinkBase):
    @abstractmethod
    def reader(self) -> Reader:
        pass