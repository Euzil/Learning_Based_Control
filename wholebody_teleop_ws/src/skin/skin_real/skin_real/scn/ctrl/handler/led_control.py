#!/usr/bin/python3


import logging
from typing import Dict, List, Tuple

from skin_real.scn.core import color_rgb_to_val
from skin_real.scn.icommand_handler import ICommandHandler,descr_entry
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
import skin_real.scn.sc.pkt.led as led

from skin_real.scn.sc.pkt.tools import SC_ID_ALL


ColorValMap = Dict[str,int]

COLOR_VAL_MAP : ColorValMap = {
    "black"     : color_rgb_to_val(0,0,0),
    "red"       : color_rgb_to_val(255,0,0),
    "green"     : color_rgb_to_val(0,255,0),
    "blue"      : color_rgb_to_val(0,0,255),
    "white"     : color_rgb_to_val(255,255,255),
    "yellow"    : color_rgb_to_val(255,255,0),
    "cyan"      : color_rgb_to_val(0,255,255),
    "magenta"   : color_rgb_to_val(255,0,255),
    "grey"      : color_rgb_to_val(127,127,127),
    "orange"    : color_rgb_to_val(255,165,0),
}


class LedControl(ICommandHandler):

    ScColor = Tuple[int,int]
    ScColorList = List[ScColor]

    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self,hwi : Hwi):
        self.__hwi = hwi

    # def __del__(self):
    #     pass


    def set_led_color_val(self,color_val : int, id : int = SC_ID_ALL):
        hwi = self.__hwi
        hwi.data().write(led.led_rgb_val(color_val,id))

    def set_led_color_rgb(self, r : int, g : int, b : int, id : int = SC_ID_ALL):
        hwi = self.__hwi
        hwi.data().write(led.led_rgb(r,g,b,id))


    def handleCommand(self,cmd : str) -> bool:
        cmd_parts = cmd.split()
        cmd_len = len(cmd_parts)

        if(cmd == "ls colors"):
            descr = str()
            for ind,k in enumerate(COLOR_VAL_MAP.keys()):
                descr += f"{ind+1}: {k}\n"
            print(descr)
            return True
        
        if cmd_len < 1:
            return False

        # get color, always on first place
        color_val = COLOR_VAL_MAP.get(cmd_parts[0])

        if color_val is None:
            return False
        
        # only color, no ids
        if cmd_len == 1:
            self.set_led_color_val(color_val)
            return True

        if cmd_len < 2:
            return False
        
        sc_color_list : LedControl.ScColorList = []
        # get ids after color command
        for id_str_ind in range(1,cmd_len):
            try:
                id = int(cmd_parts[id_str_ind])
            except:
                continue
            if(id < 1 or id > SC_ID_ALL):
                continue
            sc_color_list.append((id,color_val))
            # self.set_led_color_val(color_val,id)   

        if len(sc_color_list) > 0:
            for sc_color in sc_color_list:
                self.set_led_color_val(sc_color[1],sc_color[0])
            return True  

        return False
    

    def commandDescription(self,col_width : int = 30) -> str:
        descr = str() \
            + descr_entry("ls colors",                  "List all supported color commands.",col_width) \
            + descr_entry("<color> [<ID1> <ID2> ...]",  "Set led color.",col_width) \
            + descr_entry("",                           "  Valid IDs: 1, ..., ID_MAX",col_width)
        return descr
