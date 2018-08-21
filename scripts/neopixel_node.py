#!/usr/bin/env python

from mbientlab.warble import *
from time import sleep
import struct

import rospy
from rospy import Subscriber
from neopixel_ble.msg import NeoPixelConfig, NeoPixelColor
from sensor_msgs.msg import BatteryState

GATT_SERVICE   = "427c0000-97c7-413b-8de7-490917d220aa"
CONFIG_CHR     = "427c1001-97c7-413b-8de7-490917d220aa"
COLOR_CHR      = "427c1002-97c7-413b-8de7-490917d220aa"

BAT_CHR        = "2a19"


class NeoPixelNode:
    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 25)
        self.name = rospy.get_param('~name', rospy.get_name().split('/')[-1])

        self.address = rospy.get_param('~address')

        self.sub_config = Subscriber('~config', NeoPixelConfig, self.config_cb)
        self.sub_color = Subscriber('~color', NeoPixelColor, self.color_cb)
        self.pub_battery = rospy.Publisher('~battery', BatteryState, queue_size = 10, latch=True)

        self.gatt = Gatt(self.address)
        self.gatt.on_disconnect(self.on_disconnect)
        self.gatt.connect_async(self.on_connect)

        self.config_chr = None
        self.color_chr = None

        rospy.on_shutdown(lambda: self.on_disconnect(None))

    def on_connect(self, err):
        rospy.loginfo('Connected to %s', self.address)
        if not self.gatt.service_exists(GATT_SERVICE):
            rospy.logerr('Could not find LED service')
            self.gatt.disconnect()
            return
        else:
            rospy.loginfo('Found LED service')

        self.bat_chr = self.gatt.find_characteristic(BAT_CHR)
        self.config_chr = self.gatt.find_characteristic(CONFIG_CHR)
        self.color_chr = self.gatt.find_characteristic(COLOR_CHR)

        if self.bat_chr:
            self.bat_chr.on_notification_received(self.on_battery)
            self.bat_chr.enable_notifications_async(lambda x: None)
            self.bat_chr.read_value_async(lambda x, err: self.on_battery(x))
        else:
            rospy.logwarn('Could not find battery characteristic')

        if self.config_chr is None:
            rospy.logerr('Could not find config characteristic')
        else:
            def read_done(val, err):
                rospy.loginfo('Current config: {length: %d, pin: %d, type: 0x%04x}',
                        val[0], val[1], struct.unpack('<1H', struct.pack('>2B', *val[2:4]))[0]
                )

            self.config_chr.read_value_async(read_done)


        if self.color_chr is None:
            rospy.logerr('Could not find color characteristic')

        if self.config_chr and self.color_chr:
            rospy.loginfo('Setting pixels')
            self.setPixels(0, 16, 16)

    def setPixels(self, r, g, b, index = NeoPixelColor.NEO_ALL_PIXELS):
        if self.color_chr:
            self.color_chr.write_async([index, r, g, b], lambda x: None)

    def setConfig(self, length, freq, order, pin):
        if self.config_chr:
            new_cfg = [length, pin, order, freq >> 8]

            def write_done(err):
                def read_done(val, err):
                    if val == new_cfg:
                        rospy.loginfo('Configuration changed successfully')

                self.config_chr.read_value_async(read_done)

            self.config_chr.write_async(new_cfg, write_done)

    def on_disconnect(self, err):
        if self.config_chr and self.color_chr:
            rospy.loginfo('Clearing pixels')
            self.setPixels(0, 0, 0)

        rospy.loginfo('Disconnected from %s', self.address)

    def on_battery(self, val):
        stamp = rospy.Time.now()

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = float('nan')
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = val[0] / 100.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        #msg.cell_voltage
        msg.location = 'inside'
        msg.serial_number = 'n/a'

        self.pub_battery.publish(msg)

    def config_cb(self, msg):
        if self.config_chr:
            self.setConfig(msg.stripe_length, msg.pixel_freq, msg.color_order, msg.pin_number)

    def color_cb(self, msg):
        if self.color_chr:
            r = self.clamp(int(msg.color.r * 255.0))
            g = self.clamp(int(msg.color.g * 255.0))
            b = self.clamp(int(msg.color.b * 255.0))
            self.setPixels(r, g, b, msg.index)

    def clamp(self, val, min = 0, max = 255):
        val = 0 if val < 0 else val
        val = 255 if val > 255 else val
        return val

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
