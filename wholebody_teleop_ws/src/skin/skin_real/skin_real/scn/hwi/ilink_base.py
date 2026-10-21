#!/usr/bin/python3

from abc import ABC, abstractmethod


class ILinkBase(ABC):

    @abstractmethod
    def isOpened(self) -> bool:
        pass

    @abstractmethod
    def isClosed(self) -> bool:
        pass

    @abstractmethod
    def read(self) -> bytes:
        pass

    @abstractmethod
    def write(self, data : bytes) -> bool:
        pass