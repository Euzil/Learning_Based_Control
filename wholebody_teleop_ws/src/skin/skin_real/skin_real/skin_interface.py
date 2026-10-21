#!/usr/bin/python3

import logging
from typing import List, Callable
from time import sleep
import subprocess

from skin_real.scn.ctrl.handler import NeighListManager,UdrControl,CfControl,IdControl,SensControl,LedControl
from skin_real.scn.sc.pkt.acc import acc_range, ACC_RANGE_2G, ACC_RANGE_4G
from skin_real.scn.icommand_handler import ICommandHandler
from skin_real.scn.hwi.hwi import HardwareInterface as Hwi
from skin_real.scn.sc.data_publisher import DataPublisher
from skin_real.scn.icommand_handler import ICommandHandler


class SkinInterface:
    """SkinInterface Class
    Start, Stop, Cmd an Interface.
    """
    
    Callback = Callable[[DataPublisher.ScData], None]
        
    @property
    def logger(self):
        return logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    def __init__(self, ip_addr = "192.168.0.236", port="17000"):
        self._is_connnected = False
    
        self._hwi_conf = Hwi.DefaultConfig()
        self._ip_addr=ip_addr
        self._port_ctrl = port
        self._port_data = str(int(port) + 10)

        self._hwi_conf["ctrl_link"]["wi_ip_ep"] = f"{self._ip_addr}:{self._port_ctrl}"
        self._hwi_conf["ctrl_link"]["pc_ip_ep"] = f"0.0.0.0:{str(int(self._port_ctrl) + 1)}"
        self._hwi_conf["data_link"]["wi_ip_ep"] = f"{self._ip_addr}:{self._port_data}"
        self._hwi_conf["data_link"]["pc_ip_ep"] = f"0.0.0.0:{str(int(self._port_data) + 1)}"

        self._hwi = Hwi(self._hwi_conf)
        self._hwi.open()
        self._hwi.ctrl().reader().start()
        self._hwi.data().reader().start()
        self._ready = False

        self._data_pub = DataPublisher(self._hwi)
        self._data_pub.add_callback(self._data_callback)

        self._handlers : List[ICommandHandler] = []
        self._led_ctrl = LedControl(self._hwi)
        
        self._handlers += [IdControl(self._hwi)]
        self._handlers += [SensControl(self._hwi)]
        self._handlers += [CfControl(self._hwi)]
        self._handlers += [UdrControl(self._hwi)]
        self._handlers += [self._led_ctrl]

        self._udrctrl = UdrControl(self._hwi)
        self._cfctrl = CfControl(self._hwi)
        self._sensctrl = SensControl(self._hwi)
        
        # function to call if new data is available
        self._data_callbacks = []
        
    def is_connected(self):
        return self._is_connnected
        
    def add_data_callback(self, callback: Callback):
        self._data_callbacks.append(callback)
        
    def connect(self): 
        # check connection
        if not self._ping_device():
            raise ConnectionError(f"Interface at {self._ip_addr}:{self._port_ctrl} is unreachable.")
        
        # connect interface
        self._data_pub.reset()
        self._hwi.connect()
        sleep(8)
        
        # set frequency
        cmd = "udr 63"
        self._udrctrl.handleCommand(cmd)
        sleep(3)

        # enable led
        cmd = "cf on"
        self._cfctrl.handleCommand(cmd)
        sleep(1)
        
        # set the acc limits to 4g
        self._hwi.ctrl().write(acc_range(ACC_RANGE_4G))
        sleep(1)
        
        # connected
        self._is_connnected = True

    def disconnect(self):
        self._hwi.disconnect()
        self._hwi.close()
        del self._hwi

    def command(self, cmd:str):
        if self.is_connected:
            self._sensctrl.handleCommand(cmd)
        return True

    def _data_callback(self, data: DataPublisher.ScData):
        # call all registered callbacks
        for fnc in self._data_callbacks:
            fnc(data)
            
    def _ping_device(self, timeout_sec=1) -> bool:
        """ Pings the IP address
        """        
        # Command: ping -c 1 -W 1 <ip_addr> (Ping exactly once, timeout after 1 second)
        command = ['ping', '-c', '1', '-W', str(timeout_sec), self._ip_addr]
        
        try:
            # DEVNULL keeps your ROS terminal logs completely clean of ping output
            result = subprocess.run(
                command, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            return result.returncode == 0
        except Exception as e:
            self.logger.error(f"Ping execution failed: {e}")
            return False