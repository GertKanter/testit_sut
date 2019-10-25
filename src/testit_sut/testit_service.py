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
import re
import os
import psutil
import signal
import testit_msgs.srv
import testit_msgs.msg
import std_msgs.msg

class TestItSut(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "srv")
        if self.mode == "srv":
            # Service mode
            rospy.loginfo("TestIt SUT in SERVICE mode")
            self.flush_service = rospy.Service("/testit/flush_coverage", testit_msgs.srv.Coverage, self.handle_flush_service)
        else:
            # Topic mode
            rospy.loginfo("TestIt SUT in TOPIC mode")
            self.flush_subscriber = rospy.Subscriber("/testit/flush_coverage", std_msgs.msg.UInt32, self.handle_flush_topic)
            self.flush_publisher = rospy.Publisher("/testit/flush_data", testit_msgs.msg.FlushData, queue_size=10)
        self.node_workspace = rospy.get_param("~node_workspace", "")
        self.coverage_directories = rospy.get_param("~coverage_directories", "")
        self.host = rospy.get_param("~host", "")


    @property
    def host(self):
        if self._host is None or len(self._host) == 0:
            rospy.logwarn("SUT host identifier is not defined!")
            return ""
        else:
            return self._host

    @host.setter
    def host(self, value):
        if value is not None and type(value) == str:
            self._node_workspace = value
        else:
            raise ValueError("host value must be string!")
        
    @property
    def node_workspace(self):
        if self._node_workspace is None or len(self._node_workspace) == 0:
            rospy.logwarn("Catkin workspace for tested packages is not defined (parameter 'node_workspace', this should be a string e.g., '/catkin_ws')")
            return ""
        else:
            return self._node_workspace

    @node_workspace.setter
    def node_workspace(self, value):
        if value is not None and type(value) == str:
            self._node_workspace = value
        else:
            raise ValueError("node_workspace value must be string!")

    @property
    def coverage_directories(self):
        if self._coverage_directories is None or len(self._coverage_directories) == 0:
            rospy.logwarn("Coverage recording log file directories are not defined (parameter 'coverage_directories', this should be a semicolon-separated string e.g., '/catkin_ws/build;/root/.ros')")
            return []
        else:
            return self._coverage_directories

    @coverage_directories.setter
    def coverage_directories(self, value):
        if value is not None and type(value) == str:
            self._coverage_directories = value.split(";")
        else:
            raise ValueError("coverage_directories value must be string!")

    def get_coverage(self):
        file_coverages = []
        success = self.flush()
	if self.coverage is not None:
            for file_coverage in self.coverage.keys():
                coverage = testit_msgs.msg.FileCoverage()
                coverage.filename = file_coverage
                coverage.lines = self.coverage[file_coverage]
                file_coverages.append(coverage)
        return file_coverages

    def handle_flush_topic(self, data):
        # Received request to send coverage data - send it via topic
        message = testit_msgs.msg.FlushData()
        message.host_id = self.host
        message.seq = data.data
        message.coverage = self.get_coverage()
        self.flush_publisher.publish(message)

    def handle_flush_service(self, req):
        rospy.logdebug("Coverage results requested")
        result = True
        return testit_msgs.srv.CoverageResponse(result, self.get_coverage())

    def process_coverage(self, filename):
        rospy.loginfo("process_coverage(" + str(filename) + ")")
        header = "!coverage.py: This is a private format, don't read it directly!"
        data = []
        try:
            while True:
                with open(filename) as f:
                    data = f.readlines()
                    replaced = data[0].replace(header, '')
                    lines = eval(replaced)
                    return lines['lines']
        except Exception as e:
            rospy.logerr(e)
            rospy.sleep(0.05)
        return {}

    def flush(self):
        rospy.loginfo("Flushing...")
        if self.node_workspace is not None:
            # Remove *.gcda and .coverage files

            # Send SIGUSR1 to packages under test
            pids = psutil.pids()
            for pid in pids:
                p = psutil.Process(pid)
                try:
                    cmdline = " ".join(p.cmdline())
                    if cmdline.find(" " + self.node_workspace) >= 0 and cmdline.find("/opt/ros/") == -1 and not cmdline.startswith("/bin/bash"):
                        if cmdline.find("testit_sut") == -1:
                            # Don't send SIGUSR1 to self
                            #rospy.loginfo("Sending SIGUSR1 to " + str(p.pid) + "(" + str(cmdline) + ")")
                            os.kill(p.pid, signal.SIGUSR1)
                except psutil.AccessDenied:
                    # Some processes might be inaccessible
                    pass
            # Process all *.gcda and .coverage files
            self.coverage = {}
            for coverage_directory in self.coverage_directories:
                rospy.loginfo("Looking into " + coverage_directory)
                for directory, dirnames, filenames in os.walk(coverage_directory):
                    for filename in filenames:
                        if filename == ".coverage":
                            self.coverage.update(self.process_coverage(str(directory) + "/" +  filename))
            return True
        return False

if __name__ == "__main__":
    rospy.init_node('testit_sut', anonymous=True)
    testit_sut = TestItSut()
    rospy.loginfo("TestIt SUT services started...")
    rospy.spin()
    rospy.loginfo("Shut down everything!")
