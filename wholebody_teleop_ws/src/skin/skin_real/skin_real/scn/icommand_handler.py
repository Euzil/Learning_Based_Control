#!/usr/bin/python3

from abc import abstractmethod, ABC

def descr_entry(cmd : str, descr : str, col_width : int) -> str:
    return f"{cmd:<{int(col_width)}}{descr}\n"

class ICommandHandler(ABC):
    # @abc.abstractclassmethod
    @abstractmethod
    def handleCommand(self,cmd : str) -> bool:
        pass

    @abstractmethod
    def commandDescription(self,col_width = 30) -> str:
        pass
   