#!/usr/bin/python3


import logging
import threading
from typing import Callable, List, Tuple, TypedDict

from skin_real.scn.icommand_handler import ICommandHandler,descr_entry
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
from skin_real.scn.core import mask, print_hex_block
import skin_real.scn.ctrl.pkt as ctrl_pkt


class Neighbors(TypedDict):
    node_id:        int           # skin cell ID
    neighbor_ids:   List[int]     # skin cell IDs of neighbor skin cells



class NeighListManager(ICommandHandler):
    ScIds = List[int]
    ScNeighbors = Tuple[int,Tuple[int,int,int,int]]
    ScNeighborsList = List[ScNeighbors]

    Callback = Callable[[ScNeighborsList],None]
    CallbackList = List[Callback]


    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self,hwi : Hwi):
        self.__hwi = hwi
        self.__list : NeighListManager.ScNeighborsList = []
        self.__n_page = 0
        self.__page = 0
        self.__list_ok = True

        self.__cb_list : NeighListManager.CallbackList = []
        self.__mutex = threading.Lock()

        self.__sc_neighs : NeighListManager.ScNeighborsList = []
        self.__sc_ids : NeighListManager.ScIds = []

        hwi.ctrl().reader().add_callback(self.__ctrl_packets_handler)

    # def __del__(self):
    #     pass


    def handleCommand(self,cmd : str) -> bool:
        cmd_parts = cmd.split()
        cmd_len = len(cmd_parts)

        hwi = self.__hwi

        if(cmd == "neighs get"):
            pkt = ctrl_pkt.NEIGH_LIST_GET_CMD_PKT
            hwi.ctrl().write(pkt)
            return True
        
        return False
    

    def commandDescription(self,col_width : int = 30) -> str:
        descr = str() \
            + descr_entry("neighs get",    "Request neighbor list from interface.",col_width)
        return descr


    def add_callback(self, cb : Callback):
        with self.__mutex:
            self.__cb_list.append(cb)


    def __ctrl_packets_handler(self,data : bytes):
        # print(f"NeighListHandler: len = {len(data)}")
        # print_hex_block(data)

        if(not data.startswith(ctrl_pkt.NEIGH_LIST_PAGE_PKT_TOKEN)):
            return

        page = data[8]
        n_el = data[10]

        n_page = self.__n_page

        if page == 0:
            n_page = data[9]
            self.__n_page = n_page
            self.__page = 0
            self.__list.clear()
            self.__list_ok = True
            
        # print(f"neigh list page pkt ({page+1} of {n_page}), n_el = {n_el}")
        
        if not self.__list_ok:
            return

        if self.__page != page:
            print(f"ERROR: neigh list page pkt: expected page {self.__page} but got {page}")
            self.__list_ok = False


        for ind in range(n_el):
            vals = []
            for val_ind in range(5):
                off = 11+ind*10+val_ind*2
                v = data[off]
                v |= data[off+1] << 8
                vals.append(v)
            neighs = (vals[0], tuple(vals[1:]))
            self.__list.append(neighs)

        self.__page += 1

        if page+1 == n_page:
            sc_ids = []
            sc_neighs = []
            for n in self.__list:
                sc_ids.append(n[0])
            sc_ids.sort()
            for sc_id in sc_ids:
                for n in self.__list:
                    if n[0] == sc_id:
                        sc_neighs.append(n)
            # print(f"Got neighbors:")
            # print(sc_neighs)

            with self.__mutex:
                self.__sc_neighs = sc_neighs
                self.__sc_ids = sc_ids

                for cb in self.__cb_list:
                    cb(sc_neighs)