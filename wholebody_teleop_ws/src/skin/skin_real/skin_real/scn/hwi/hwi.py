#!/usr/bin/python3


import logging
import time
from typing import Tuple

from skin_real.scn.hwi.ctrl.link import CtrlLink
from skin_real.scn.hwi.data.link import DataLink

from skin_real.scn.hwi.ilink import ILink
import skin_real.scn.ctrl.pkt as ctrl_pkt


# Hardware interface
#   simple, no threading
#   only gives access to control and data link
#   open locks device
#   close unlocks device
#   connnect/disconnect
class HardwareInterface:

    @staticmethod
    def DefaultConfig() -> dict:
        return { 
            "name"          : "Default",
            "type"          : "WI2500",
            "ctrl_link"     : CtrlLink.DefaultConfig(),
            "data_link"     : DataLink.DefaultConfig(),
        }

    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self, config : dict):
        self.__config = config
        self.__opened = False
        self.__ctrl_link = CtrlLink(config["ctrl_link"])
        self.__data_link = DataLink(config["data_link"])


    def __del__(self):
        self.logger.debug("Destroy.")
        if self.isOpened():
            self.close()

    def setConfig(self,config : dict):
        self.__config = config
        self.__ctrl_link.setConfig(config["ctrl_link"])
        self.__data_link.setConfig(config["data_link"])

    
    def config(self) -> dict:
        return self.__config

    
    def open(self) -> bool:
        if self.isOpened():
            self.logger.error("Already opened.")
            return False
        
        self.__ctrl_link.open()
        self.__data_link.open()

        self.__opened = True
        return True

    def close(self):
        if not self.isOpened():
            self.logger.error("Already closed.")
            return
        self.__ctrl_link.close()
        self.__data_link.close()
        self.__opened = False


    def isOpened(self) -> bool:
        return self.__opened

    
    def isClosed(self) -> bool:
        return not self.__opened


    def connect(self) ->  bool:
        if not self.isOpened():
            self.logger.error("Device not opened.")
            return False
        self.ctrl().write(ctrl_pkt.LOCK_CMD_PKT)
        self.ctrl().write(ctrl_pkt.START_CMD_PKT)
        return True

    def disconnect(self) ->  bool:
        if not self.isOpened():
            self.logger.error("Device not opened.")
            return False
        self.ctrl().write(ctrl_pkt.STOP_CMD_PKT)
        time.sleep(0.1)
        self.ctrl().write(ctrl_pkt.UNLOCK_CMD_PKT)
        return True


    def ctrl(self) -> ILink:
        return self.__ctrl_link
    
    def data(self) -> ILink:
        return self.__data_link