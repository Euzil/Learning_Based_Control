#!/usr/bin/python3


import logging
import threading
from typing import Callable, Dict, List, Tuple, TypedDict

from skin_real.scn.icommand_handler import ICommandHandler,descr_entry
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
from skin_real.scn.core import mask, print_hex_block
import skin_real.scn.sc.pkt.data as pkt_data

# Value layout for Data1200
#   0: prox
#   1: force1
#   2: force2
#   3: force3
#   4: acc_x
#   5: acc_y
#   6: acc_z
#   7: temp



class DataPublisher:
    ScIdList = List[int]
    ScData = Tuple[int,List[float]]
    ScDataList = List[ScData] 

    ScIdMap = Dict[int,int]

    Callback = Callable[[ScData], None]
    CallbackList = List[Callback]


    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self,hwi : Hwi):
        self.__hwi = hwi
        self.__cb_list : DataPublisher.CallbackList = []
        self.__mutex = threading.Lock()
        self.__sc_id_map : DataPublisher.ScIdMap = {}
        self.__sc_ids : DataPublisher.ScIdList = []
        self.__sc_data : DataPublisher.ScDataList = []

        hwi.data().reader().add_callback(self.__data_packets_handler)

    # def __del__(self):
    #     pass


    def reset(self):
        with self.__mutex:
            self.__sc_ids.clear()
            self.__sc_id_map.clear()
            self.__sc_data.clear()

    def add_callback(self, cb : Callback):
        with self.__mutex:
            self.__cb_list.append(cb)


    def sc_id_map(self):
        with self.__mutex:
            return self.__sc_id_map

    def sc_ids(self):
        with self.__mutex:
            return self.__sc_ids
        
    def sc_data(self):
        with self.__mutex:
            return self.__sc_data


    def __update_data_list(self, sc_data : ScData):
        sc_id = sc_data[0]
        # vals = sc_data[1]

        if not sc_id in self.__sc_id_map:
            ind = len(self.__sc_ids)
            self.__sc_id_map[sc_id] = ind
            self.__sc_ids.append(sc_id)
            self.__sc_data.append(sc_data)
        else:
            ind = self.__sc_id_map[sc_id]
            self.__sc_data[ind] = sc_data


    def __data_packets_handler(self,data : bytes):
        if data[0] != 0xFF:
            return
        
        sc_id = pkt_data.get_id(data)
        values = pkt_data.get_data_values(data)
        
        sc_data : DataPublisher.ScData = (sc_id,values) 

        with self.__mutex:
            self.__update_data_list(sc_data)

            for cb in self.__cb_list:
                cb(sc_data)

        # print(f"sc data:")
        # print_hex_block(data)


        pass

