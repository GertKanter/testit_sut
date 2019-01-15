#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019 Gert Kanter.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Gert Kanter

import rospy
import std_srvs.srv
import re
import os
import psutil

class TestItSut:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "service")
        if self.mode == "service":
            # Service mode
            rospy.loginfo("TestIt SUT in SERVICE mode")
            self.flush_service = rospy.Service("/testit/flush_coverage", std_srvs.srv.Trigger, self.handle_flush)
        else:
            # Topic mode
            #TODO add support for topic trigger, needed for multi-computer node configuration
            rospy.loginfo("TestIt SUT in TOPIC mode")
            pass
        self.node_workspace = rospy.get_param("~node_workspace", "catkin_ws")
        

    def handle_flush(self, req):
        rospy.logdebug("Coverage results requested")
        message = "coverage message"
        success = self.flush()
        return std_srvs.srv.TriggerResponse(success, message)

    def flush(self):
        rospy.loginfo("Flushing...")
        pids = psutil.pids()
        for pid in pids:
            p = psutil.Process(pid)
            try:
                rospy.loginfo("pid " + str(p.pid) + "  cmd " + str(p.cmdline()) + "  cwd " + str(p.cwd()))
            except:
                pass
        return True

if __name__ == "__main__":
    rospy.init_node('testit_sut', anonymous=True)
    testit_sut = TestItSut()
    rospy.loginfo("TestIt SUT services started...")
    rospy.spin()
    rospy.loginfo("Shut down everything!")
