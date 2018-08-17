#!/usr/bin/env python

from mbientlab.warble import *
from time import sleep
import struct

import rospy
from rospy import Subscriber
from neopixel_ble.msg import NeoPixelConfig, NeoPixelColor

GATT_SERVICE   = "427c0000-97c7-413b-8de7-490917d220aa"
CONFIG_CHR     = "1001"
COLOR_CHR      = "1002"


class NeoPixelNode:
    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 25)
        self.name = rospy.get_param('~name', rospy.get_name().split('/')[-1])

        self.address = rospy.get_param('~address')

        self.sub_config = Subscriber('~config', NeoPixelConfig, self.config_cb)
        self.sub_color = Subscriber('~color', NeoPixelColor, self.color_cb)

        self.gatt = Gatt(self.address)
        self.gatt.on_disconnect(self.on_disconnect)
        self.gatt.connect_async(self.on_connect)

    def on_connect(self, err):
        rospy.loginfo('Connected to %s', self.address)
        if not self.gatt.service_exists(GATT_SERVICE):
            rospy.logerr('Could not find LED service')
            self.gatt.disconnect()
            return
        else:
            rospy.info('Found LED service')

        self.config_chr = self.gatt.find_characteristic(CONFIG_CHR)
        print 'Checkpoint 1'
        if self.config_chr is None:
            rospy.logerr('Could not find config characteristic')

        if self.color_chr is None:
            rospy.logerr('Could not find color characteristic')

        self.color_chr = self.gatt.find_characteristic(COLOR_CHR)

        color = {}
        color.index = 0xff
        color.red   = 0x00
        color.green = 0x10
        color.blue  = 0x10

        if self.config_chr and self.color_chr:
            rospy.loginfo('Setting pixels')
            self.color_chr.write_value_async(struct.pack('>8h', 0xff, 0x00, 0x10, 0x10))

    def on_disconnect(self, err):
        if self.config_chr and self.color_chr:
            rospy.loginfo('Clearing pixels')
            self.color_chr.write_value_async(struct.pack('>8h', 0xff, 0x00, 0x00, 0x00))

        rospy.loginfo('Disconnected from %s', self.address)

    def config_cb(self, msg):
        pass

    def color_cb(self, msg):
        pass

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                loop_rate.sleep()

            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn('Saw a negative time change, resseting.')


if __name__ == '__main__':
    rospy.init_node('neopixel_ble', anonymous = False)

    pixel = NeoPixelNode()
    pixel.run()

# def scan_result_printer(result):
#     print("uuid: %s" % result.uuid)
#     print("name: %s" % result.name)
#     print("rssi: %ddBm" % result.rssi)

#     print("neopixel service? %d" % result.has_service_uuid())

#     data = result.get_manufacturer_data(0x626d)
#     print("======")

# BleScanner.set_handler(scan_result_printer)

# print("-- active scan --")
# BleScanner.start()
# sleep(5.0)
# BleScanner.stop()