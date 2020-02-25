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
from velodyne_monitor import VelodyneMonitor


if __name__ == '__main__':
    try:
        rospy.init_node('scheduled_velodyne_monitor')
        monitoring_interval_sec = rospy.get_param('~monitoring_interval_sec', 1)
        ip_addresses = rospy.get_param('~ip_addresses', ['192.168.1.201'])
        names = rospy.get_param('~names', ip_addresses)

        rate = rospy.Rate(monitoring_interval_sec)

        monitors = {}
        for i, ip in enumerate(ip_addresses):
            rospy.set_param('~ip_address', ip)
            monitors[i] = VelodyneMonitor()
            monitors[i].name = names[i]

        monitor_index = 0
        while not rospy.is_shutdown():
            monitors[monitor_index].publish_status()
            monitor_index = monitor_index + 1 if monitor_index + 1 < len(monitors) else 0
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
