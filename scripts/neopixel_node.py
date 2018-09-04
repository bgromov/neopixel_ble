#!/usr/bin/env python

from bluepy import btle

from time import sleep
import struct
from threading import Thread

import rospy
from rospy import Subscriber
from std_msgs.msg import ColorRGBA
from neopixel_ble.msg import NeoPixelConfig, NeoPixelColor
from sensor_msgs.msg import BatteryState

GATT_SERVICE   = "427c0000-97c7-413b-8de7-490917d220aa"
CONFIG_CHR     = "427c1001-97c7-413b-8de7-490917d220aa"
COLOR_CHR      = "427c1002-97c7-413b-8de7-490917d220aa"

BAT_CHR        = "2a19"
CCCD_UUID      = "2902"

class BleDelegate(btle.DefaultDelegate):
    def __init__(self):
        btle.DefaultDelegate.__init__(self)
        self.cb_map = {}

    def enableNotifications(self, char, cb):
        if char and cb:
            self.cb_map[char.getHandle()] = cb

            desc = char.getDescriptors(forUUID=CCCD_UUID)[0]
            desc.write('\x01\x00')

    def disableNotifications(self, char):
        if char:
            self.cb_map[char.getHandle()] = None

            desc = char.getDescriptors(forUUID=CCCD_UUID)[0]
            desc.write('\x00\x00')

    def handleNotification(self, cHandle, data):
        if cHandle in self.cb_map:
            if self.cb_map[cHandle]:
                self.cb_map[cHandle](map(ord, data))

class NeoPixelNode:
    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 25)
        self.name = rospy.get_param('~name', rospy.get_name().split('/')[-1])

        self.address = rospy.get_param('~address')
        cc = rospy.get_param('~default_color', {'index': 255, 'color': {'r': 0.1, 'g': 0.0, 'b': 0.0, 'a': 1.0}})
        self.default_color = NeoPixelColor(index=cc['index'], color=ColorRGBA(**cc['color']))

        self.sub_config = Subscriber('~config', NeoPixelConfig, self.config_cb)
        self.sub_color = Subscriber('~color', NeoPixelColor, self.color_cb)
        self.pub_battery = rospy.Publisher('~battery', BatteryState, queue_size = 10, latch=True)

        self.config_chr = None
        self.color_chr = None

        try:
            self.gatt = btle.Peripheral(self.address, addrType='random')
            self.gatt.withDelegate(BleDelegate())
        except Exception as e:
            rospy.logerr(e.message)
            self.on_connect(e.message)

        self.on_connect(None)
        rospy.on_shutdown(lambda: self.on_disconnect(None))

    def on_connect(self, err):
        rospy.loginfo('Connected to %s', self.address)
        try:
            self.gatt.getServiceByUUID(GATT_SERVICE)
            rospy.loginfo('Found LED service')
        except btle.BTLEException as e:
            if e.code == btle.BTLEException.GATT_ERROR:
                rospy.logerr('Could not find LED service')
                self.gatt.disconnect()
                return
            raise e

        bat_chr = self.gatt.getCharacteristics(uuid=BAT_CHR)
        self.bat_chr = bat_chr[0] if bat_chr else None

        config_chr = self.gatt.getCharacteristics(uuid=CONFIG_CHR)
        self.config_chr = config_chr[0] if config_chr else None
        color_chr = self.gatt.getCharacteristics(uuid=COLOR_CHR)
        self.color_chr = color_chr[0] if color_chr else None

        if self.bat_chr:
            self.gatt.delegate.enableNotifications(self.bat_chr, self.on_battery)

            val = map(ord, self.bat_chr.read())
            self.on_battery(val)

        else:
            rospy.logwarn('Could not find battery characteristic')

        if not self.config_chr:
            rospy.logerr('Could not find config characteristic')
        else:
            def read_done(val, err):
                rospy.loginfo('Current config: {length: %d, pin: %d, type: 0x%04x}',
                        val[0], val[1], struct.unpack('<1H', struct.pack('>2B', *val[2:4]))[0]
                )

            read_done(map(ord, self.config_chr.read()), None)

        if not self.color_chr:
            rospy.logerr('Could not find color characteristic')

        if self.config_chr and self.color_chr:
            self.color_cb(self.default_color)

    def setPixels(self, r, g, b, index = NeoPixelColor.NEO_ALL_PIXELS, withResponse=False):
        if self.color_chr:
            self.color_chr.write(struct.pack('>4B', index, r, g, b), withResponse=withResponse)

    def setConfig(self, length, freq, order, pin):
        if self.config_chr:
            new_cfg = [length, pin, order, freq >> 8]

            def write_done(err):
                def read_done(val, err):
                    if val == new_cfg:
                        rospy.loginfo('Configuration changed successfully')

                read_done(map(ord, self.config_chr.read()), None)

            self.config_chr.write(bytearray(new_cfg))
            write_done(None)

    def on_disconnect(self, err):
        if self.bat_chr:
            self.gatt.delegate.disableNotifications(self.bat_chr)

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
            # rospy.loginfo('Setting color')
            r = self.clamp(int(msg.color.r * msg.color.a * 255.0))
            g = self.clamp(int(msg.color.g * msg.color.a * 255.0))
            b = self.clamp(int(msg.color.b * msg.color.a * 255.0))
            self.setPixels(r, g, b, msg.index)

    def clamp(self, val, min = 0, max = 255):
        val = 0 if val < 0 else val
        val = 255 if val > 255 else val
        return val

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        def run_thread():
            while rospy.is_shutdown():
                self.gatt.waitForNotifications(0.1)

        t = Thread(name='btle', target=run_thread)
        t.start()

        while not rospy.is_shutdown():
            try:
                loop_rate.sleep()

            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn('Saw a negative time change, resseting.')

        t.join()

if __name__ == '__main__':
    rospy.init_node('neopixel_ble', anonymous = False)

    pixel = NeoPixelNode()
    pixel.run()
