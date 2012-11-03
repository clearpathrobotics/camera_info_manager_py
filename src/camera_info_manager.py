# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
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
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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

"""
.. module:: camera_info_manager

Python camera_info_manager interface, providing `CameraInfo` support
for drivers written in Python. This is similar to the C++
camera_info_manager package, but not identical.

.. _`sensor_msgs/CameraInfo`: http://ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html
.. _`sensor_msgs/SetCameraInfo`: http://ros.org/doc/api/sensor_msgs/html/srv/SetCameraInfo.html

"""

PKG='camera_info_manager_py'
import roslib; roslib.load_manifest(PKG)
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

import yaml


class CameraInfoManager():
    """
    :class:`CameraInfoManager` provides ROS CameraInfo support for
    Python camera drivers. It handles the `sensor_msgs/SetCameraInfo`_
    service requests, saving and restoring `sensor_msgs/CameraInfo`_
    data.

    :param cname: camera name.
    :param url: Uniform Resource Locator for camera calibration data.
 
    .. describe:: str(wu_point)
 
       :returns: String representation of :class:`CameraInfoManager` object.

    ROS Service
    -----------

    - set_camera_info (`sensor_msgs/SetCameraInfo`_) stores
                      calibration information

    Typically, these service requests are made by a calibration
    package, such as:

    - http://www.ros.org/wiki/camera_calibration

    The calling node *must* invoke rospy.spin() in some thread, so
    CameraInfoManager can handle arriving service requests.

    @par Camera Name

    The device driver sets a camera name via the
    CameraInfoManager::CameraInfoManager constructor or the
    setCameraName() method.  This name is written when CameraInfo is
    saved, and checked when data are loaded, with a warning logged if
    the name read does not match.

    Syntax: a camera name contains any combination of alphabetic,
            numeric and '_' characters.  Case is significant.

    Camera drivers may use any syntactically valid name they please.
    Where possible, it is best for the name to be unique to the
    device, such as a GUID, or the make, model and serial number.  Any
    parameters that affect calibration, such as resolution, focus,
    zoom, etc., may also be included in the name, uniquely identifying
    each CameraInfo file.

    Beginning with Electric Emys, the camera name can be resolved as
    part of the URL, allowing direct access to device-specific
    calibration information.

    @par Uniform Resource Locator

    The location for getting and saving calibration data is expressed
    by Uniform Resource Locator.  The driver defines a URL via the
    CameraInfoManager::CameraInfoManager constructor or the
    loadCameraInfo() method.  Many drivers provide a @c
    ~camera_info_url parameter so users may customize this URL, but
    that is handled outside this class.

    Typically, cameras store calibration information in a file, which
    can be in any format supported by @c camera_calibration_parsers.
    Currently, that includes YAML and Videre INI files, identified by
    their .yaml or .ini extensions as shown in the examples.  These
    file formats are described here:

    - http://www.ros.org/wiki/camera_calibration_parsers#File_formats

    Example URL syntax:

    - file:///full/path/to/local/file.yaml
    - file:///full/path/to/videre/file.ini
    - package://camera_info_manager/tests/test_calibration.yaml
    - package://ros_package_name/calibrations/camera3.yaml

    The @c file: URL specifies a full path name in the local system.
    The @c package: URL is handled the same as @c file:, except the
    path name is resolved relative to the location of the named ROS
    package, which @em must be reachable via @c $ROS_PACKAGE_PATH.

    Beginning with Electric Emys, the URL may contain substitution
    variables delimited by <tt>${...}</tt>, including:

    - @c ${NAME} resolved to the current camera name defined by the
                 device driver.
    - @c ${ROS_HOME} resolved to the @c $ROS_HOME environment variable
                     if defined, <tt>~/.ros</tt> if not.

    Resolution is done in a single pass through the URL string.
    Variable values containing substitutable strings are not resolved
    recursively.  Unrecognized variable names are treated literally
    with no substitution, but an error is logged.

    Examples with variable substitution:

    - package://my_cameras/calibrations/${NAME}.yaml
    - file://${ROS_HOME}/camera_info/left_front_camera.yaml

    In C-turtle and Diamondback, if the URL was empty, no calibration
    data were loaded, and any data provided via `set_camera_info`
    would be stored in:

    - file:///tmp/calibration_${NAME}.yaml

    Beginning in Electric, the default URL changed to:

    - file://${ROS_HOME}/camera_info/${NAME}.yaml.

    If that file exists, its contents are used. Any new calibration
    will be stored there, missing parent directories being created if
    necessary and possible.

    @par Loading Calibration Data

    Prior to Fuerte, calibration information was loaded in the
    constructor, and again each time the URL or camera name was
    updated. This frequently caused logging of confusing and
    misleading error messages.

    Beginning in Fuerte, camera_info_manager loads nothing until the
    @c loadCameraInfo(), @c isCalibrated() or @c getCameraInfo()
    method is called. That suppresses bogus error messages, but allows
    (valid) load errors to occur during the first @c getCameraInfo(),
    or @c isCalibrated(). To avoid that, do an explicit @c
    loadCameraInfo() first.

    """

    def __init__(self, cname='axis_camera', url=''):
        """Constructor.
        """
        self.cname = cname
        self.url = url
        self.loaded_cam_info = False

        # :todo: advertise set_camera_info service

    def __str__(self):
        """:returns: String representation of :class:`CameraInfoManager` """
        return '[' + self.cname + ']' + str(self.utm)

    def setCameraName(self, cname):
        """ Set a new camera name.

        :param cname: camera name to use for saving calibration data

        :returns: True if new name has valid syntax; valid names
                  contain only alphabetic, numeric, or '_' characters.

        :post: camera name updated, if valid; since it may affect the
               URL, cam_info will be reloaded before being used again.

        """
        # validate name
        if cname == '':
            return False        # name may not be empty
        for ch in cname:
            if not ch.isalnum() and ch != '_':
                return False    # invalid character

        # name is valid, use it
        self.cname = cname
        self.loaded_cam_info = False
        return True


def genCameraName(from_string):
    """ Generate a valid camera name.

    Valid names contain only alphabetic, numeric, or '_'
    characters. All invalid characters in from_string are replaced
    by an '_'.

    :param from_string: string from which to base camera name.

    :returns: a valid camera name based on from_string.

    """
    if from_string == '':
        return '_'          # name may not be empty

    retval = ''
    for i in range(len(from_string)):
        if not from_string[i].isalnum() and from_string[i] != '_':
            retval += '_'
        else:
            retval += from_string[i]
    return retval

def resolveURL(url, cname):
    """ Resolve Uniform Resource Locator string.

    :param url: URL to resolve, which may include `${...}`
                substitution variables.
    :param cname: camera name to use for name resolution.

    :returns: a copy of the URL with any variable information resolved.

    """
    return url                  # test scaffolding with no resolution
