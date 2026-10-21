#!/usr/bin/python3


import logging

from skin_real.scn.icommand_handler import ICommandHandler,descr_entry
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
import skin_real.scn.ctrl.pkt as ctrl_pkt

class IdControl(ICommandHandler):

    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self,hwi : Hwi):
        self.__hwi = hwi

    # def __del__(self):
    #     pass


    def handleCommand(self,cmd : str) -> bool:
        cmd_parts = cmd.split()
        cmd_len = len(cmd_parts)

        hwi = self.__hwi

        if(cmd == "store ids"):
            pkt = ctrl_pkt.IDS_STORE_CMD_PKT
            hwi.ctrl().write(pkt)
            return True
        
        if(cmd == "clear ids"):
            pkt = ctrl_pkt.IDS_CLEAR_CMD_PKT
            hwi.ctrl().write(pkt)
            return True
        
        return False
    

    def commandDescription(self,col_width : int = 30) -> str:
        descr = str() \
            + descr_entry("store ids",  "Store the current sc IDs to the nv memory.",col_width) \
            + descr_entry("clear ids",  "Clear the sc IDs from the nv memory.",col_width) \
            + descr_entry("",           "  New sc IDs will be found after a restart.",col_width)
        return descr
