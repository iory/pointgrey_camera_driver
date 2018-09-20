/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie
Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution
Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*-*-C++-*-*/
/**
   @file PointGreyCamera.h
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#ifndef _POINTGREYCAMERA_SP_H_
#define _POINTGREYCAMERA_SP_H_

#include <sensor_msgs/Image.h>            // ROS message header for Image
#include <sensor_msgs/image_encodings.h>  // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>
#include <pointgrey_camera_driver/camera_exceptions.h>

#include <chrono>
#include <sstream>

// Header generated by dynamic_reconfigure
#include <pointgrey_camera_driver/PointGreyConfig.h>

// Spinnaker SDK from Point Grey
#include "Spinnaker.h"

class PointGreyCameraSP
{
public:
  PointGreyCameraSP();
  ~PointGreyCameraSP();

  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function handles a reference of a camera_library::CameraConfig object and
  * configures the camera as close to the given values as possible.  As a function for
  * dynamic_reconfigure, values that are not valid are changed by the driver and can
  * be inspected after this function ends.
  * This function will stop and restart the camera when called on a SensorLevels::RECONFIGURE_STOP level.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver
  * is currently using.
  * \param level  Reconfiguration level. See constants below for details.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool setNewConfiguration(const int& camera_id, pointgrey_camera_driver::PointGreyConfig& config,
                           const uint32_t& level);

  /** Parameters that need a sensor to be stopped completely when changed. */
  static const uint8_t LEVEL_RECONFIGURE_CLOSE = 3;

  /** Parameters that need a sensor to stop streaming when changed. */
  static const uint8_t LEVEL_RECONFIGURE_STOP = 1;

  /** Parameters that can be changed while a sensor is streaming. */
  static const uint8_t LEVEL_RECONFIGURE_RUNNING = 0;

  /*!
  * \brief Function that connects to a specified camera.
  *
  * Will connect to the camera specified in the setDesiredCamera(std::string id) call.  If setDesiredCamera is not
  * called first
  * this will connect to the first camera.  Connecting to the first camera is not recommended for multi-camera or
  * production systems.
  * This function must be called before setNewConfiguration() or start()!
  */
  void connect(const int& camera_id);

  /*!
  * \brief Disconnects from the camera.
  *
  * Disconnects the camera and frees it.
  */
  void disconnect();

  /*!
  * \brief check connection with the camera
  *
  */
  bool isConnected();

  /*!
  * \brief Starts the camera loading data into its buffer.
  *
  * This function will start the camera capturing images and loading them into the buffer.  To retrieve images,
  * grabImage must be called.
  */
  void start();

  /*!
  * \brief Stops the camera loading data into its buffer.
  *
  * This function will stop the camera capturing images and loading them into the buffer.
  *
  * \return Returns true if the camera was started when called.  Useful for knowing if the camera needs restarted in
  * certain instances.
  */
  bool stop();

  /*!
  * \brief Loads the raw data from the cameras buffer.
  *
  * This function will load the raw data from the buffer and place it into a sensor_msgs::Image.
  * \param image sensor_msgs::Image that will be filled with the image currently in the buffer.
  * \param frame_id The name of the optical frame of the camera.
  */
  void grabImage(sensor_msgs::Image& image, const std::string& frame_id);

  void grabStereoImage(sensor_msgs::Image& image, const std::string& frame_id, sensor_msgs::Image& second_image,
                       const std::string& second_frame_id);

  /*!
  * \brief Will set grabImage timeout for the camera.
  *
  * This function will set the time required for grabCamera to throw a timeout exception.  Must be called after
  * connect().
  * \param timeout The desired timeout value (in seconds)
  *
  */
  void setTimeout(const double& timeout);

  /*!
  * \brief Used to set the serial number for the camera you wish to connect to.
  *
  * Sets the desired serial number.  If this value is not set, the driver will try to connect to the first camera on the
  * bus.
  * This function should be called before connect().
  * \param id serial number for the camera.  Should be something like 10491081.
  */
  void setDesiredCamera(const uint32_t& id);

  /*!
  * \brief Set parameters relative to GigE cameras.
  *
  * \param auto_packet_size Flag stating if packet size should be automatically determined or not.
  * \param packet_size The packet size value to use if auto_packet_size is false.
  */
  void setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay)
  {
  }

  std::vector<uint32_t> getAttachedCameras();

  /*!
  * \brief Gets the current operating temperature.
  *
  * Gets the camera's current reported operating temperature.
  *
  * \return The reported temperature in Celsius.
  */
  float getCameraTemperature();

  void setGain(double& gain);

  void setBRWhiteBalance(bool auto_white_balance, uint16_t& blue, uint16_t& red);

  uint getGain();

  uint getShutter();

  uint getBrightness();

  uint getExposure();

  uint getWhiteBalance();

  uint getROIPosition();

  void setTime(ros::Time& tm);

private:
  uint32_t serial_;  ///< A variable to hold the serial number of the desired camera.
  Spinnaker::CameraPtr cam_ptr_;
  Spinnaker::SystemPtr system_;
  Spinnaker::CameraList cam_list_;
  //
  boost::mutex mutex_;  ///< A mutex to make sure that we don't try to grabImages while reconfiguring or vice versa.
                        /// Implemented with boost::mutex::scoped_lock.
  volatile bool captureRunning_;  ///< A status boolean that checks if the camera has been started and is loading images
                                  /// into its buffer.ù

  /// If true, camera is currently running in color mode, otherwise camera is running in mono mode
  bool isColor_;

  // For GigE cameras:
  /// If true, GigE packet size is automatically determined, otherwise packet_size_ is used:
  bool auto_packet_size_;
  /// GigE packet size:
  unsigned int packet_size_;
  /// GigE packet delay:
  unsigned int packet_delay_;

  ros::Time last_tm_;
  double time_delay_;

  std::chrono::time_point<std::chrono::system_clock> now_, last_;
  int count_test_ = 0;
  double sum_test_ = 0.0;

  // bool setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, double &value);
  void setFrameRate(double& value);
  void setExposure(bool& autoset, double& value);
  void setGain(bool& autoset, double& value);
  void setExternalTrigger(bool& enable, std::string& trigger_source, int& trigger_polarity);

#if 0
  /*!
  * \brief Changes the video mode of the connected camera.
  *
  * This function will change the camera to the desired videomode and allow up the maximum framerate for that mode.
  * \param videoMode string of desired video mode, will be changed if unsupported.
  */
  void setVideoMode(FlyCapture2::VideoMode &videoMode);

  /*!
  * \brief Changes the camera into Format7 mode with the associated parameters.
  *
  * This function will stop the camera, change the video mode into Format 7, and then restart the camera.
  * \param fmt7Mode Flycapture2::Mode, desired Format 7 mode.
  * \param fmt7PixFmt FlyCapture2::PixelFormat, desired Format 7 pixel format.
  * \param roi_width width of the region of interest for Format 7 in pixels, will be changed if unsupported.  '0' is full width.
  * \param roi_height height of the region of interest for Format 7 in pixels, will be changed if unsupported. '0' is full height
  * \param roi_offset_x offset in pixels from the left side of the image for the region of interest for Format 7, will be changed if unsupported.
  * \param roi_offset_y offset in pixels from the top of the image for the region of interest for Format 7, will be changed if unsupported.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool setFormat7(FlyCapture2::Mode &fmt7Mode, FlyCapture2::PixelFormat &fmt7PixFmt, uint16_t &roi_width, uint16_t &roi_height, uint16_t &roi_offset_x, uint16_t &roi_offset_y);

  /*!
  * \brief Converts the dynamic_reconfigure string type into a FlyCapture2::VideoMode.
  *
  * This function will convert the string input from dynamic_reconfigure into the proper datatype for use with FlyCapture enum.
  * \param vmode input video mode, will be changed if unsupported.
  * \param vmode_out FlyCapture2::VideoMode, will be changed to either the corresponding type as vmode, or to the most compatible type.
  * \param fmt7Mode will be set to the appropriate FlyCapture2::Mode if vmode is a format 7 mode.upacket, upacket,
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool getVideoModeFromString(std::string &vmode, FlyCapture2::VideoMode &vmode_out, FlyCapture2::Mode &fmt7Mode);

  /*!
  * \brief Converts the dynamic_reconfigure string type into a FlyCapture2::PixelFormat
  *
  * This function will convert the string input from dynamic_reconfigure into the proper datatype for use with FlyCapture enum.
  * \param fmt7Mode input video mode, needed to know which PixelFormats are valid.
  * \param sformat dynamic_reconfigure Format 7 color coding, will be changed if unsupported.
  * \param fmt7PixFmt FlyCapture2::PixelFormat, will be set to the appropriate output type.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool getFormat7PixelFormatFromString(std::string &sformat, FlyCapture2::PixelFormat &fmt7PixFmt);

  bool setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet,  unsigned int &valueA,  unsigned int &valueB);

  /*!
  * \brief Generic wrapper for setting properties in FlyCapture2
  *
  * This function will set the appropriate type and if desired, will allow the camera to change its own value.  If value is outside the range of min and max,
  * it will be set to either extreme.
  * \param type FlyCapture2::PropertyType to set.  Examples: FlyCapture2::GAIN FlyCapture2::SHUTTER FlyCapture2::BRIGHTNESS
  * \param autoSet whether or not to allow the camera to automatically adjust values.  Ex: auto exposure, auto shutter.  Not supported for all types.
  * \param value Desired absolute value to be set in appropriate units.  Will be changed if this value is not supported.
  * \param min Absolute mininum value that this type can be set to.
  * \param max Absolute maximum value that this type can be set to.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, double &value);

  /*!
  * \brief Sets the white balance property
  *
  * This function will set the white balance for the camera..  If value is outside the range of min and max,
  * it will be set to either extreme.
  * \param blue Value for the blue white balance setting.
  * \param red Value for the red white balance setting.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool setWhiteBalance(bool& auto_white_balance, uint16_t &blue, uint16_t &red);

  /*!
  * \brief Gets the current frame rate.
  *
  * Gets the camera's current reported frame rate.
  *
  * \return The reported frame rate.
  */
  float getCameraFrameRate();

  /*!
  * \brief Will set the external triggering of the camera.
  *
  * This function will enable external triggering of the camera and set the desired source, parameter, and delay.
  * \param enable Whether or not to use external triggering.
  * \param mode The desired external triggering mode.
  * \param source The desired external triggering source.
  * \param parameter The parameter currently only used by trigger mode 3 (skip N frames, where parameter is N).
  * \param delay The delay in seconds to wait after being triggered.
  * \param polarityHigh Whether the polarity of the triggering signal is high.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool setExternalTrigger(bool &enable, std::string &mode, std::string &source, int32_t &parameter, double &delay, bool &polarityHigh);

  /*!
  * \brief Will set the external strobe of the camera.
  *
  * This function will enable external strobing of the camera on the specifed pin and set the desired duration, and delay.
  * Note that unlike the trigger, multiple output strobes on different pins are quite possible; each output can be enable
  * and disabled separately.
  *
  * \param enable Whether or not to use enable strobing on the give pin.
  * \param dest The pin to modify.
  * \param delay The delay in milliseconds to wait after image capture.
  * \param duration The length in milliseconds to hold the strobe.
  * \param polarityHigh Whether the polarity of the strobe signal is high.
  *
  * \return Returns true when the configuration could be applied without modification.
  */
  bool setExternalStrobe(bool &enable, const std::string &dest, double &duration, double &delay, bool &polarityHigh);

  /*!
  * \brief Will autoconfigure the packet size of the GigECamera with the given GUID.
  *
  * Note that this is expected only to work for GigE cameras, and only if the camera
  * is not connected.
  *
  * \param guid the camera to autoconfigure
  */
  void setupGigEPacketSize(FlyCapture2::PGRGuid & guid);

  /*!
  * \brief Will configure the packet size of the GigECamera with the given GUID to a given value.
  *
  * Note that this is expected only to work for GigE cameras, and only if the camera
  * is not connected.
  *
  * \param guid the camera to autoconfigure
  * \param packet_size The packet size value to use.
  */
  void setupGigEPacketSize(FlyCapture2::PGRGuid & guid, unsigned int packet_size);

  /*!
  * \brief Will configure the packet delay of the GigECamera with the given GUID to a given value.
  *
  * Note that this is expected only to work for GigE cameras, and only if the camera
  * is not connected.
  *
  * \param guid the camera to autoconfigure
  * \param packet_delay The packet delay value to use.
  */
  void setupGigEPacketDelay(FlyCapture2::PGRGuid & guid, unsigned int packet_delay);


public:
  /*!
  * \brief Handles errors returned by FlyCapture2.
  *
  * Checks the status of a FlyCapture2::Error and if there is an error, will throw a runtime_error
  * \param prefix Message that will prefix the obscure FlyCapture2 error and provide context on the problem.
  * \param error FlyCapture2::Error that is returned from many FlyCapture functions.
  */
  static void handleError(const std::string &prefix, const FlyCapture2::Error &error);
#endif
};

#endif
