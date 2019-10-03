#!/usr/bin/env python

# Copyright 2015-2019 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy
import json
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import urllib2

class VelodyneMonitor():
    def __init__(self):
        self._ip = rospy.get_param('~ip_address', '192.168.1.201')
        self._v_in_warn = rospy.get_param('~v_in_warn', 11.0)
        self._v_in_error = rospy.get_param('~v_in_error', 9.0)
        self._temp_cold_warn = rospy.get_param('~temp_cold_warn', -5.0)
        self._temp_cold_error = rospy.get_param('~temp_cold_error', -10.0)
        self._temp_heat_warn = rospy.get_param('~temp_heat_warn', 70.0)
        self._temp_heat_error = rospy.get_param('~temp_heat_error', 90.0)
        self._pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.name = rospy.get_namespace().strip('/')
        if self.name == '':
            self.name = 'velodyne'
    def check_connection(self):
        try:
            info_url = 'http://' + self._ip + '/cgi/info.json'
            diag_url = 'http://' + self._ip + '/cgi/diag.json'
            self._info_data = json.load(urllib2.urlopen(info_url, timeout=0.3))
            self._diag_data = json.load(urllib2.urlopen(diag_url, timeout=0.3))
            return True
        except urllib2.URLError as err:
            print err
            return False
        except Exception as e:
            print err
            return False

    def judge_risk_level(self, top_temp, bot_temp, i_out, v_in, stat):
        stat.level = DiagnosticStatus.OK
        stat.message = ''
        if v_in < self._v_in_error:
            stat.level = DiagnosticStatus.ERROR
            stat.message += 'V_in is too low: ' + str(v_in) + 'V  '
        elif v_in < self._v_in_warn:
            if stat.level < DiagnosticStatus.WARN:
                stat.level = DiagnosticStatus.WARN
            stat.message += 'V_in is low: ' + str(v_in) + 'V  '
        if top_temp < self._temp_cold_error:
            stat.level = DiagnosticStatus.ERROR
            stat.message += 'TopTemp is too cold: ' + str(top_temp) + '*C  '
        elif top_temp < self._temp_cold_warn:
            if stat.level < DiagnosticStatus.WARN:
                stat.level = DiagnosticStatus.WARN
            stat.message += 'TopTemp is cold: ' + str(top_temp) + '*C  '
        elif top_temp > self._temp_heat_error:
            stat.level = DiagnosticStatus.ERROR
            stat.message += 'TopTemp is too heat: ' + str(top_temp) + '*C  '
        elif top_temp > self._temp_heat_warn:
            if stat.level < DiagnosticStatus.WARN:
                stat.level = DiagnosticStatus.WARN
            stat.message += 'TopTemp is heat: ' + str(top_temp) + '*C  '
        if bot_temp < self._temp_cold_error:
            stat.level = DiagnosticStatus.ERROR
            stat.message += 'BottomTemp is too cold: ' + str(bot_temp) + '*C  '
        elif bot_temp < self._temp_cold_warn:
            if stat.level < DiagnosticStatus.WARN:
                stat.level = DiagnosticStatus.WARN
            stat.message += 'BottomTemp is cold: ' + str(bot_temp) + '*C  '
        elif bot_temp > self._temp_heat_error:
            stat.level = DiagnosticStatus.ERROR
            stat.message += 'BottomTemp is too heat: ' + str(bot_temp) + '*C  '
        elif bot_temp > self._temp_heat_warn:
            if stat.level < DiagnosticStatus.WARN:
                stat.level = DiagnosticStatus.WARN
            stat.message += 'BottomTemp is heat: ' + str(bot_temp) + '*C  '
        if stat.level == DiagnosticStatus.OK:
            stat.message = str(round(max(top_temp, bot_temp),1)) + '*C  '

    def convertTemp(self, temp):
        return numpy.sqrt(2.1962e6 + (1.8639 - float(temp) * 5.0 / 4096) / 3.88e-6) - 1481.96

    def convertVolt(self, volt):
        return 11.0 * float(volt) * 5.0 / 4096

    def convertAmp(self, amp):
        return 10.0 * (float(amp) * 5.0 / 4096 - 2.5)

    def publish_status(self):
        stat = DiagnosticStatus()
        stat.name = self.name
        is_connect_ok = self.check_connection()
        if is_connect_ok:
          stat.hardware_id = str(self._info_data['serial'])
          model = str(self._info_data['model'])
          top_temp = self.convertTemp(self._diag_data['volt_temp']['top']['lm20_temp'])
          bot_temp = self.convertTemp(self._diag_data['volt_temp']['bot']['lm20_temp'])
          i_out = self.convertAmp(self._diag_data['volt_temp']['bot']['i_out'])
          v_in = self.convertVolt(self._diag_data['volt_temp']['bot']['pwr_v_in'])
          stat.values = [ KeyValue(key = 'Model', value = str(model)),
                          KeyValue(key = 'TopTemp[DegC]', value = str(round(top_temp, 3))),
                          KeyValue(key = 'BottomTemp[DegC]', value = str(round(bot_temp, 3))),
                          KeyValue(key = 'Iout[V]', value = str(round(i_out, 3))),
                          KeyValue(key = 'Vin[V]', value = str(round(v_in, 3))) ]
          self.judge_risk_level(top_temp, bot_temp, i_out, v_in, stat)
        else:
          stat.level = DiagnosticStatus.ERROR
          stat.message = 'Connection Lost ' + self._ip 

        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status.append(stat)
        self._pub.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('velodyne_monitor')
        rate = rospy.Rate(2)
        monitor = VelodyneMonitor()

        while not rospy.is_shutdown():
            monitor.publish_status()
            rate.sleep()
    except rospy.ROSInterruptException: pass

