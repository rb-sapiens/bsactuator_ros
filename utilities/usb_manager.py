#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2023 RoboSapiens Inc.

import os
import re
import subprocess

# USBの操作を行うクラス
class UsbManager:

    def __init__(self, sudopass, usb_name):
        self.sudopass = sudopass
        self.usb_name = usb_name
        self.usb_id = "" # e.g. 1-4.4
        self.usb_path = "" # e.g. /dev/ttyACM0

    def get_usb_props(self):
        # getting usb ID(e.g. 1-4.4)
        usb_str = os.popen('journalctl -o short-precise -k | sed -e "/Manufacturer/!d" -e "/'+self.usb_name+'/!d" | tail -1').read()
        try:
            self.usb_id = usb_str.split(": Manufacturer")[0].split(" ")[-1]
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("No USB devices found.")
        
        # getting usb path(e.g. /dev/ttyACM0)
        usb_str = os.popen('journalctl -o short-precise -k | sed -e "/tty/!d" -e "/'+self.usb_id+'/!d" | tail -1').read()
        usb_strarr = re.split(r':| ',usb_str)
        self.usb_path = ""
        for elm in usb_strarr:
            if "tty" in elm:
                self.usb_path = "/dev/" + elm.replace(" ","")

    def plug_unplug(self):
        # unplug USB
        process = subprocess.Popen('echo "'+self.sudopass+'" | sudo -S sh -c "echo  ' + self.usb_id + ' > /sys/bus/usb/drivers/usb/unbind"', shell=True, stdout=subprocess.PIPE)
        process.wait()

        # plug USB
        process = subprocess.Popen('echo "'+self.sudopass+'" | sudo -S sh -c "echo  ' + self.usb_id + ' > /sys/bus/usb/drivers/usb/bind"', shell=True, stdout=subprocess.PIPE)
        process.wait()

    def add_permission(self):
        # set chmod 777
        process = subprocess.Popen('echo "'+self.sudopass+'" | sudo -S chmod 777 ' + self.usb_path, shell=True, stdout=subprocess.PIPE)
        process.wait()