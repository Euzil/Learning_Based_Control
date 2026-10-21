#!/usr/bin/python3

import json
import logging
import socket
import threading
from typing import Callable, List, Tuple

from skin_real.scn.hwi.ilink_base import ILinkBase

class Reader:
    Callback = Callable[[bytes],None]
    CallbackList = List[Callback]

    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    def __init__(self, link : ILinkBase):
        self.__link = link
        self.__started = False
        self.__thread = None
        self.__cb_list : Reader.CallbackList = []
        self.__mutex = threading.Lock()
        self.__stop_event = threading.Event()

    def __del__(self):
        self.logger.debug("Destroy.")
        if self.__started:
            self.stop()

    def add_callback(self, cb : Callback):
        with self.__mutex:
            self.__cb_list.append(cb)


    def start(self):
        if self.__started:
            self.logger.error("Already started.")
            return
        
        if self.__link is None or not self.__link.isOpened():
            self.logger.error("Start failed: Link not open.")
            return
        
        self.__thread = threading.Thread(target=self.__run)
        self.__thread.start()
        self.__started = True

    def stop(self):
        if not self.__started:
            self.logger.error("Already stopped.")
            return
        self.logger.debug("Stopping Thread.")
        self.__stop_event.set()
        self.__thread.join()
        self.__started = False


    def isStarted(self) -> bool:
        return self.__started

    
    def isStopped(self) -> bool:
        return not self.__started


    def __run(self):
        self.logger.debug("Started Thread.")
        while not self.__stop_event.is_set():
            data = self.__link.read()
            if data is None:
                continue
            with self.__mutex:
                for cb in self.__cb_list:
                    cb(data)

        self.logger.debug("Exit Thread.")
        self.__stop_event.clear()