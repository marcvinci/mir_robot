#!/usr/bin/env python3

# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Günther

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class rep117_filter(Node):
    def __init__(self):
        super().__init__('rep117_filter')

        pub = self.create_publisher(
            msg_type=LaserScan,
            topic='scan_filtered',
            qos_profile=qos_profile_sensor_data
        )

        def callback(msg):
            """
            Convert laser scans to REP 117 standard.

            See http://www.ros.org/reps/rep-0117.html
            """
            ranges_out = []
            for dist in msg.ranges:
                if dist < msg.range_min:
                    # assume "reading too close to measure",
                    # although it could also be "reading invalid" (nan)
                    ranges_out.append(float("-inf"))

                elif dist > msg.range_max:
                    # assume "reading of no return (outside sensor range)",
                    # although it could also be "reading invalid" (nan)
                    ranges_out.append(float("inf"))
                else:
                    ranges_out.append(dist)

            msg.ranges = ranges_out
            pub.publish(msg)

        self.create_subscription(
            msg_type=LaserScan,
            topic="scan",
            callback=callback,
            qos_profile=qos_profile_sensor_data
        )
    

def main(args=None):
    rclpy.init(args=args)
    node = rep117_filter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()