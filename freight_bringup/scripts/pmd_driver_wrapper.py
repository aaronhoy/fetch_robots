#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Authors: Eric Relson, Aaron Hoy

# ROS
import rospy
from sensor_msgs.msg import CameraInfo

# Process launching, tracking, etc
import subprocess
from time import sleep
import signal
from argparse import ArgumentParser

# All this stuff so we can send emails...
from apiclient import discovery, errors
import oauth2client
from oauth2client import client
from oauth2client import tools
import os
from email.mime.text import MIMEText
import socket
import base64
import httplib2

# If modifying these scopes, delete your previously saved credentials
# at ~/.credentials/gmail-python-quickstart.json
SCOPES = 'https://www.googleapis.com/auth/gmail.send'
APPLICATION_NAME = 'PMD Status Emailer'

def get_credentials():
    """Gets valid user credentials from storage.

    If nothing has been stored, or if the stored credentials are invalid,
    the OAuth2 flow is completed to obtain the new credentials.

    Returns:
        Credentials, the obtained credential.
    """
    home_dir = "/home/fetch" # os.path.expanduser('~')
    credential_dir = os.path.join(home_dir, '.credentials')
    client_secret_file = os.path.join(credential_dir, 'gmail_client_secrets.json')
    if not os.path.exists(credential_dir):
        os.makedirs(credential_dir)
    credential_path = os.path.join(credential_dir,
                                   'gmail-python-pmd-emailer.json')

    store = oauth2client.file.Storage(credential_path)
    credentials = store.get()
    if not credentials or credentials.invalid:
        flow = client.flow_from_clientsecrets(client_secret_file, SCOPES)
        flow.user_agent = APPLICATION_NAME
        # Use --noauth_local_server to do gmail authentication
        flags = ArgumentParser(parents=[tools.argparser]).parse_args(["--noauth_local_webserver"])
        credentials = tools.run_flow(flow, store, flags)
        rospy.loginfo('Storing credentials to ' + credential_path)
    # else: got credentials from file
    return credentials


# A class that runs and monitors the PMD driver, restarting it when it appears
# to have hung.
class PmdWrapper(object):
    def __init__(self, launch_cmd, monitor_topic, recovery_timeout,
                 usb_rebinder, skip_simple_recovery, email_address):
        # Some configurable params
        # What command to run to actually launch the driver
        self.launch_args = launch_cmd.split(" ")
        # Which topic to listen to to check if the camera has died
        self.topic_name = monitor_topic
        # How long to allow dropouts before recovering
        self.recovery_timeout = recovery_timeout
        # Path to the USB re-bind executable
        self.rebind_exec = usb_rebinder
        # Whether to skip simple recovery (used if simple recovery is done
        # inside driver, recommended to increase recovery timeout in this
        # case also)
        self.skip_simple_recovery = skip_simple_recovery
        # Email address to mail to when recovery is used
        self.email_address = email_address

        # Subscriber to monitor the camera status
        self.camera_sub = rospy.Subscriber(self.topic_name,
                                           CameraInfo,
                                           self.camera_callback)
        # Process of the camera driver itself
        self.launch_proc = None

        # State to keep track of which recoveries we have tried already
        self.tried_reset = False
        self.tried_rebind = False

    def __del__(self):
        # Kill the driver before going down
        if self.launch_proc:
            self.stop_driver()

    def camera_callback(self, data):
        # If we got some data from the camera, it's up, so update our stamp
        # and reset recovery state
        self.tried_reset = False
        self.tried_rebind = False
        self.time_last_pub = data.header.stamp

    def start_driver(self):
        # Simply Popen the driver launch
        self.launch_proc = subprocess.Popen(self.launch_args)

    def stop_driver(self):
        rospy.logwarn("Going to stop driver")
        # Kill the process, wait for it to die, and delete the Popen object
        self.launch_proc.send_signal(subprocess.signal.SIGINT)
        self.launch_proc.wait()
        self.launch_proc = None

    def rebind_usb(self):
        # Call the USB rebind script with sudo and block until it completes
        subprocess.call(["sudo", self.rebind_exec])

    def recover(self):
        # If we haven't tried just resetting the driver, do that
        if not self.tried_reset and not self.skip_simple_recovery:
            msg = "Camera has stopped responding, attempting driver reset."
            rospy.logwarn(msg)
            if self.email_address:
                self.email_msg(msg)

            # Kill the driver, wait a bit, then start it back up
            self.stop_driver()
            sleep(0.1)
            rospy.logwarn("Driver killed, starting driver back up...")
            self.start_driver()
            rospy.logwarn("Driver restart complete.")

            # Keep track of the fact that we tried this
            self.tried_reset = True

        # Otherwise, if we haven't tried to rebind, do that
        elif not self.tried_rebind:
            msg = "Camera still not responding after driver reset. Trying USB " \
                  "re-bind."
            rospy.logwarn(msg)
            if self.email_address:
                self.email_msg(msg)

            # Kill the driver, unbind USB, re-bind USB, re-start the driver
            self.stop_driver()
            sleep(0.1)
            rospy.logwarn("Driver killed, rebinding usb...")
            self.rebind_usb()
            sleep(1)
            rospy.logwarn("Usb rebind complete, startind driver back up...")
            self.start_driver()
            rospy.logwarn("USB rebind + driver restart complete.")

            # Keep track of the fact that we tried this
            self.tried_rebind = True
        else:
            # If we get here, we're out of stuff to try so just wait for a
            # person to come and investigate
            msg = "Failed to recover after USB re-bind. Please come check on " \
                           "me!!!!"
            rospy.logwarn(msg)
            if self.email_address:
                self.email_msg(msg)
            rospy.spin()

    def run(self):
        # Start the driver for the first time
        sleep(1)
        self.start_driver()

        # Give it some time to come up and then start monitoring it
        sleep(1)
        rospy.logwarn("Waiting a while before starting driver monitor")
        sleep(5 + self.recovery_timeout)

        if self.email_address:
            # Setup gmail auths after the network and stuff is up
            credentials = get_credentials()
            http = credentials.authorize(httplib2.Http())
            self.service = discovery.build('gmail', 'v1', http=http)

        # Record the time right before starting the monitor so we dont false
        # positive on a stale one
        self.time_last_pub = rospy.Time.now()
        rospy.logwarn("Starting driver monitor...")

        # Poll at 1Hz and make sure we have data that's less than 3 seconds old
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            time = rospy.Time.now()
            diff = time - self.time_last_pub
            if (diff.to_sec() > self.recovery_timeout):
                rospy.logwarn("No sensor data published for " + str(diff.to_sec()) + "seconds.")

                # Attempt recovery, wait for initialization, then update
                # the stamp so we can continue monitoring
                self.recover()
                sleep(2)
                self.time_last_pub = rospy.Time.now()

            rate.sleep()

    def email_msg(self, message_text):
        # Send the actual email
        message = MIMEText(message_text)
        message['to'] = self.email_address
        message['from'] = "me" # "me" uses owner of credentials being used
        message['subject'] = "PMD Wrapper Status: {0}".format(socket.gethostname())
        message_raw = {'raw': base64.urlsafe_b64encode(message.as_string())}

        try:
            result = self.service.users().messages().send(userId="me",
                                                           body=message_raw
                                                          ).execute()
            rospy.loginfo("Email Message ID: {0}".format(result['id']))
        except errors.HttpError as e:
            rospy.logerr("HttpError in pmd_status_emailer.PMDStatusEmailer.email_msgs()")
            rospy.logerr("{}".format(e._get_reason()))
        except Exception as e:
            rospy.logerr("Error in pmd_status_emailer.PMDStatusEmailer.email_msgs()")
            rospy.logerr(e)


if __name__ == "__main__":

    parser = ArgumentParser(description="Launch and monitor the PMD driver")
    parser.add_argument("--driver_launch_cmd",
                        default = "roslaunch freight_bringup base_camera_pmd.launch.xml",
                        help = "The actually command used to launch the PMD driver.")
    parser.add_argument("--monitor_topic",
                        default = "/base_camera/depth_downsample/camera_info",
                        help = "The ROS topic to monitor to tell when the "
                               "driver dies.")
    parser.add_argument("--recovery_timeout",
                        type=float,
                        default = 3.0,
                        help = "Time (seconds) to wait after the driver stops "
                               "publishing before performing a recovery.")
    parser.add_argument("--usb_rebinder",
                        help = "Script used to re-bind the USB.")
    parser.add_argument("--skip_simple_recovery",
                        action = "store_true",
                        help = "Skip the simple recovery which just stops and "
                               "re-starts the driver.")
    parser.add_argument("--email",
                        help = "The email address for recovery notifications.")

    args = parser.parse_args()

    rospy.init_node("pmd_wrapper")
    pmd_wrapper = PmdWrapper(args.driver_launch_cmd,
                             args.monitor_topic,
                             args.recovery_timeout,
                             args.usb_rebinder,
                             args.skip_simple_recovery,
                             args.email)
    pmd_wrapper.run()

