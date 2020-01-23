#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys
import cv2
import numpy
import math
from enum import Enum
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import ntcore

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        ntcore.constants.NT_NOTIFY_IMMEDIATE |
        ntcore.constants.NT_NOTIFY_NEW |
        ntcore.constants.NT_NOTIFY_UPDATE)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    # loop forever
    while True:
        time.sleep(10)
        def __init__(self):
        """initializes all values to presets or None if need to be set
        """
         self.__hsv_threshold_hue = [59.89208633093525, 100.13651877133107]
         self.__hsv_threshold_saturation = [139.88309352517987, 255.0]
         self.__hsv_threshold_value = [162.81474820143885, 255.0]

         self.hsv_threshold_output = None

         self.__find_contours_input = self.hsv_threshold_output
         self.__find_contours_external_only = False

         self.find_contours_output = None

         self.__filter_contours_contours = self.find_contours_output
         self.__filter_contours_min_area = 16.0
         self.__filter_contours_min_perimeter = 0
         self.__filter_contours_min_width = 0
         self.__filter_contours_max_width = 1000.0
         self.__filter_contours_min_height = 0
         self.__filter_contours_max_height = 1000
         self.__filter_contours_solidity = [0.0, 33.44709897610921]
         self.__filter_contours_max_vertices = 1000000.0
         self.__filter_contours_min_vertices = 10.0
         self.__filter_contours_min_ratio = 0
         self.__filter_contours_max_ratio = 1000

         self.filter_contours_output = None


        def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
            # Step HSV_Threshold0:
            self.__hsv_threshold_input = source0
            (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

            # Step Find_Contours0:
            self.__find_contours_input = self.hsv_threshold_output
            (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

            # Step Filter_Contours0:
            self.__filter_contours_contours = self.find_contours_output
            (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours, self.__filter_contours_min_area, self.__filter_contours_min_perimeter, self.__filter_contours_min_width, self.__filter_contours_max_width, self.__filter_contours_min_height, self.__filter_contours_max_height, self.__filter_contours_solidity, self.__filter_contours_max_vertices, self.__filter_contours_min_vertices, self.__filter_contours_min_ratio, self.__filter_contours_max_ratio)


        @staticmethod
        def __hsv_threshold(input, hue, sat, val):
            """Segment an image based on hue, saturation, and value ranges.
            Args:
                input: A BGR numpy.ndarray.
                hue: A list of two numbers the are the min and max hue.
                sat: A list of two numbers the are the min and max saturation.
                lum: A list of two numbers the are the min and max value.
            Returns:
                A black and white numpy.ndarray.
            """
            out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
            return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

        @staticmethod
        def __find_contours(input, external_only):
            """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
            Args:
                input: A numpy.ndarray.
                external_only: A boolean. If true only external contours are found.
            Return:
                A list of numpy.ndarray where each one represents a contour.
            """
            if(external_only):
                mode = cv2.RETR_EXTERNAL
            else:
                mode = cv2.RETR_LIST
            method = cv2.CHAIN_APPROX_SIMPLE
            im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
            return contours

        @staticmethod
        def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                            min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                            min_ratio, max_ratio):
            """Filters out contours that do not meet certain criteria.
            Args:
                input_contours: Contours as a list of numpy.ndarray.
                min_area: The minimum area of a contour that will be kept.
                min_perimeter: The minimum perimeter of a contour that will be kept.
                min_width: Minimum width of a contour.
                max_width: MaxWidth maximum width.
                min_height: Minimum height.
                max_height: Maximimum height.
                solidity: The minimum and maximum solidity of a contour.
                min_vertex_count: Minimum vertex Count of the contours.
                max_vertex_count: Maximum vertex Count.
                min_ratio: Minimum ratio of width to height.
                max_ratio: Maximum ratio of width to height.
            Returns:
                Contours as a list of numpy.ndarray.
            """
            output = []
            for contour in input_contours:
                x,y,w,h = cv2.boundingRect(contour)
                if (w < min_width or w > max_width):
                    continue
                if (h < min_height or h > max_height):
                    continue
                area = cv2.contourArea(contour)
                if (area < min_area):
                    continue
                if (cv2.arcLength(contour, True) < min_perimeter):
                    continue
             hull = cv2.convexHull(contour)
                solid = 100 * area / cv2.contourArea(hull)
                if (solid < solidity[0] or solid > solidity[1]):
                    continue
                if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                    continue
                ratio = (float)(w) / h
                if (ratio < min_ratio or ratio > max_ratio):
                    continue
                output.append(contour)
            return output d x y
            d = int((3.25 * 640)/(2 * w * 30.5))
            print(d, x, y)
        
