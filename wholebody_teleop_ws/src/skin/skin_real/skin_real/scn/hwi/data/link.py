#!/usr/bin/python3


import json
import logging
import socket
import time
from typing import Tuple

from skin_real.scn.sc.pkt.tools import dummy_pkt
from skin_real.scn.hwi.reader import Reader
from skin_real.scn.hwi.ilink import ILink
import skin_real.scn.ip_ep as ip_ep


class DataLink(ILink):
    @staticmethod
    def DefaultConfig() -> dict:
        return { 
            "pc_ip_ep"         : "0.0.0.0:17011",
            "wi_ip_ep"         : "192.168.4.1:17010",
            "read_timeout_ms"  : 200,
        }


    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")


    def __init__(self, config : dict):
        self.__config = config
        self.__opened = False
        self.__sock = None
        self.__reader = Reader(self)
        self.__read_buf : bytes = None


    def __del__(self):
        self.logger.debug("Destroy.")
        if self.isOpened():
            self.close()

    def setConfig(self,config : dict):
        self.__config = config

    
    def config(self) -> dict:
        return self.__config

    
    def open(self) -> bool:
        if self.isOpened():
            self.logger.error("Already opened.")
            return False
        config = self.__config
        self.logger.info("Using data link config:\n%s",json.dumps(config,indent=4))

        pc_ip_ep = config.get("pc_ip_ep")
        wi_ip_ep = config.get("wi_ip_ep")
        read_timeout_ms = config.get("read_timeout_ms",200)

        self.__pc_ip_ep = ip_ep.from_str(pc_ip_ep)
        self.__wi_ip_ep = ip_ep.from_str(wi_ip_ep)

        self.logger.debug(self.__pc_ip_ep)
        self.logger.debug(self.__wi_ip_ep)

        self.logger.info(f"Connecting to data link: {wi_ip_ep}")
       

       
        self.__sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.__sock.bind(self.__pc_ip_ep)
        self.__sock.settimeout(read_timeout_ms/1e3)

        pkt = dummy_pkt()
        self.__sock.sendto(pkt, self.__wi_ip_ep)
       
        self.__opened = True
        return True

    def close(self):
        if not self.isOpened():
            self.logger.error("Already closed.")
            return
        
        if self.__reader.isStarted():
            self.__reader.stop()
        
        self.logger.debug("Close UDP socket...")
        self.__sock.close()

        self.__opened = False
        self.logger.info("Closed UDP socket.")


    def isOpened(self) -> bool:
        return self.__opened

    
    def isClosed(self) -> bool:
        return not self.__opened


    def read(self) -> bytes:
        if not self.isOpened():
            self.logger.error("Device not opened.")
            return None

        buf = self.__read_buf
        if buf is None:
            buf = self.__read()
        
        if buf is None or len(buf) < 20:
            self.__read_buf = None
            return None
        
        pkt = buf[0:20]
        self.__read_buf = buf[20:]
        return pkt
    

    def write(self,data : bytes) -> bool:
        if not self.isOpened():
            self.logger.error("Device not opened.")
            return False
        
        self.__sock.sendto(data, self.__wi_ip_ep)
        time.sleep(10e-3)    # do not send data to the sc too fast
        return True

    def reader(self) -> Reader:
        return self.__reader
    

    def __read(self) -> bytes:
        try:
            len_max = 1024
            data, addr = self.__sock.recvfrom(len_max)
        except:
            # print("Timeout.")
            return None

        if(addr != self.__wi_ip_ep):                
            return None
        return data

    # def purgeRx(self):
    #     if not self.isOpened():
    #         self.logger.error("Device not opened.")
    
    #     try:
    #         while self.__sock.recv(1024): pass
    #     except:
    #         pass