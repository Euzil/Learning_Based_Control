#!/usr/bin/python3


import logging

from skin_real.scn.icommand_handler import ICommandHandler,descr_entry
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
import skin_real.scn.ctrl.pkt as ctrl_pkt



class EventsControl(ICommandHandler):

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

        if(cmd == "e on"):
            pkt = ctrl_pkt.E_ON_CMD_PKT
            hwi.ctrl().write(pkt)
            return True
        
        if(cmd == "e off"):
            pkt = ctrl_pkt.E_OFF_CMD_PKT
            hwi.ctrl().write(pkt)
            return True
        
        return False
    

    def commandDescription(self,col_width : int = 30) -> str:
        descr = str() \
            + descr_entry("e <on | off>",     "Enable/Disable event mode.",col_width)
        return descr
