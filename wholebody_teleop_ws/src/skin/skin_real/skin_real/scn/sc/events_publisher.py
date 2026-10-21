#!/usr/bin/python3


import logging
import threading
from typing import Callable, Dict, List, Tuple, TypedDict

from skin_real.scn.icommand_handler import ICommandHandler,descr_entry
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
from skin_real.scn.core import mask, print_hex_block
import skin_real.scn.sc.pkt.events as pkt_events


class EventsPublisher:
    ScEvent = pkt_events.Event
    ScEvents = List[ScEvent]

    Callback = Callable[[ScEvents],None]
    CallbackList = List[Callback]

    EVENT_ID_PROX   = pkt_events.EVENT_ID_PROX
    EVENT_ID_FORCE1 = pkt_events.EVENT_ID_FORCE1
    EVENT_ID_FORCE2 = pkt_events.EVENT_ID_FORCE2
    EVENT_ID_FORCE3 = pkt_events.EVENT_ID_FORCE3
    EVENT_ID_ACCX   = pkt_events.EVENT_ID_ACCX
    EVENT_ID_ACCY   = pkt_events.EVENT_ID_ACCY
    EVENT_ID_ACCZ   = pkt_events.EVENT_ID_ACCZ
    EVENT_ID_TEMP   = pkt_events.EVENT_ID_TEMP

    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self,hwi : Hwi):
        self.__hwi = hwi
        self.__cb_list : EventsPublisher.CallbackList = []
        self.__mutex = threading.Lock()

        hwi.data().reader().add_callback(self.__event_packets_handler)

    # def __del__(self):
    #     pass


    def add_callback(self, cb : Callback):
        with self.__mutex:
            self.__cb_list.append(cb)


    def __event_packets_handler(self,pkt : bytes):
        if pkt[0] != 0xE2:
            return
        
        sc_events = pkt_events.get_events(pkt)
        
        with self.__mutex:

            for cb in self.__cb_list:
                cb(sc_events)

        # print(f"sc events:")
        # print_hex_block(pkt)


        pass

