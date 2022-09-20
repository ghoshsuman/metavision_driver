// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "metavision_ros_driver/metavision_wrapper.h"

#include <metavision/hal/facilities/i_device_control.h>
#include <metavision/hal/facilities/i_erc.h>
#include <metavision/hal/facilities/i_plugin_software_info.h>
#include <metavision/hal/facilities/i_trigger_in.h>

#include <chrono>
#include <set>
#include <thread>

#include "metavision_ros_driver/logging.h"
#include "metavision_ros_driver/ros1_ros2_compatibility.h"

namespace metavision_ros_driver
{
MetavisionWrapper::MetavisionWrapper(const std::string & loggerName)
{
  setLoggerName(loggerName);
  lastPrintTime_ = std::chrono::system_clock::now();
}

MetavisionWrapper::~MetavisionWrapper() { stop(); }

int MetavisionWrapper::getBias(const std::string & name)
{
  const Metavision::Biases biases = cam_.biases();
  Metavision::I_LL_Biases * hw_biases = biases.get_facility();
  const auto pmap = hw_biases->get_all_biases();
  auto it = pmap.find(name);
  if (it == pmap.end()) {
    LOG_ERROR_NAMED("unknown bias parameter: " << name);
    throw(std::runtime_error("bias parameter not found!"));
  }
  return (it->second);
}

int MetavisionWrapper::setBias(const std::string & name, int val)
{
  std::set<std::string> dont_touch_set = {{"bias_diff"}};
  if (dont_touch_set.count(name) != 0) {
    LOG_WARN_NAMED("ignoring change to parameter: " << name);
    return (val);
  }
  Metavision::Biases & biases = cam_.biases();
  Metavision::I_LL_Biases * hw_biases = biases.get_facility();
  const int prev = hw_biases->get(name);
  if (val != prev) {
    if (!hw_biases->set(name, val)) {
      LOG_WARN_NAMED("cannot set parameter " << name << " to " << val);
    }
  }
  const int now = hw_biases->get(name);  // read back what actually took hold
  LOG_INFO_NAMED("changed  " << name << " from " << prev << " to " << val << " adj to: " << now);
  return (now);
}

bool MetavisionWrapper::initialize(bool useMultithreading, const std::string & biasFile)
{
  biasFile_ = biasFile;
  useMultithreading_ = useMultithreading;

  if (!initializeCamera()) {
    LOG_ERROR_NAMED("could not initialize camera!");
    return (false);
  }
  return (true);
}

bool MetavisionWrapper::stop()
{
  bool status = false;
  if (cam_.is_running()) {
    cam_.stop();
    status = true;
  }
  if (rawDataCallbackActive_) {
    cam_.raw_data().remove_callback(rawDataCallbackId_);
  }
  if (statusChangeCallbackActive_) {
    cam_.remove_status_change_callback(statusChangeCallbackId_);
  }

  keepRunning_ = false;
  if (processingThread_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.notify_all();
    }
    processingThread_->join();
    processingThread_.reset();
  }
  if (statsThread_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.notify_all();
    }
    statsThread_->join();
    statsThread_.reset();
  }
  return (status);
}

void MetavisionWrapper::applyROI(const std::vector<int> & roi)
{
  if (!roi.empty()) {
    if (roi.size() % 4 != 0) {
      LOG_ERROR_NAMED("ROI vec must be multiple of 4, but is: " << roi.size());
    } else {
#ifdef CHECK_IF_OUTSIDE_ROI
      x_min_ = std::numeric_limits<uint16_t>::max();
      x_max_ = std::numeric_limits<uint16_t>::min();
      y_min_ = std::numeric_limits<uint16_t>::max();
      y_max_ = std::numeric_limits<uint16_t>::min();
#endif
      std::vector<Metavision::Roi::Rectangle> rects;
      for (size_t i = 0; i < roi.size(); i += 4) {
        Metavision::Roi::Rectangle rect;
        rect.x = roi[i];
        rect.y = roi[i + 1];
        rect.width = roi[i + 2];
        rect.height = roi[i + 3];
        rects.push_back(rect);
#ifdef CHECK_IF_OUTSIDE_ROI
        x_min_ = std::min(static_cast<uint16_t>(rect.x), x_min_);
        x_max_ = std::max(static_cast<uint16_t>(rect.x + rect.width), x_max_);
        y_min_ = std::min(static_cast<uint16_t>(rect.y), y_min_);
        y_max_ = std::max(static_cast<uint16_t>(rect.y + rect.height), y_max_);
#endif
      }
      cam_.roi().set(rects);
    }
  } else {
#ifdef CHECK_IF_OUTSIDE_ROI
    x_min_ = 0;
    x_max_ = std::numeric_limits<uint16_t>::max();
    y_min_ = 0;
    y_max_ = std::numeric_limits<uint16_t>::max();
#endif
  }
}

void MetavisionWrapper::applySyncMode(const std::string & mode)
{
  Metavision::I_DeviceControl * control =
    cam_.get_device().get_facility<Metavision::I_DeviceControl>();
  if (!control) {  // happens when playing from file
    if (mode != "standalone") {
      LOG_WARN_NAMED("cannot set sync mode to: " << mode);
    }
    return;
  }

  if (mode == "standalone") {
    if (control->get_mode() != Metavision::I_DeviceControl::SyncMode::STANDALONE) {
      control->set_mode_standalone();
    }
  } else if (mode == "primary") {
    control->set_mode_master();
  } else if (mode == "secondary") {
    control->set_mode_slave();
  } else {
    LOG_ERROR_NAMED("INVALID SYNC MODE: " << mode);
    throw std::runtime_error("invalid sync mode!");
  }
}

void MetavisionWrapper::configureExternalTriggers(
  const std::string & mode_in, const std::string & mode_out, const int period,
  const double duty_cycle)
{
  if (mode_out == "enabled") {
    Metavision::I_TriggerOut * i_trigger_out =
      cam_.get_device().get_facility<Metavision::I_TriggerOut>();
    if (i_trigger_out) {
      i_trigger_out->set_period(period);  // in usec
      i_trigger_out->set_duty_cycle(duty_cycle);
      i_trigger_out->enable();
      LOG_INFO_NAMED("Enabled trigger output");
    } else {
      LOG_ERROR_NAMED("Failed enabling trigger output");
    }
  }

  if (mode_in == "external" || mode_in == "loopback") {
    Metavision::I_TriggerIn * i_trigger_in =
      cam_.get_device().get_facility<Metavision::I_TriggerIn>();

    if (i_trigger_in) {
      int pin = hardwarePinConfig_[softwareInfo_][mode_in];
      i_trigger_in->enable(pin);
      LOG_INFO_NAMED("Enabled trigger input " << mode_in << " on " << pin);
    } else {
      LOG_ERROR_NAMED("Failed enabling trigger input");
    }
  }
}

void MetavisionWrapper::configureEventRateController(
  const std::string & mode, const int events_per_sec)
{
  if (mode == "enabled" || mode == "disabled") {
    Metavision::I_Erc * i_erc = cam_.get_device().get_facility<Metavision::I_Erc>();
    if (i_erc) {
      i_erc->enable(mode == "enabled");
      i_erc->set_cd_event_rate(events_per_sec);
    } else {
      LOG_WARN_NAMED("cannot set event rate control for this camera!");
    }
  }
}

bool MetavisionWrapper::initializeCamera()
{
  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    try {
      if (!fromFile_.empty()) {
        LOG_INFO_NAMED("reading events from file: " << fromFile_);
        cam_ = Metavision::Camera::from_file(fromFile_);
      } else {
        if (!serialNumber_.empty()) {
          cam_ = Metavision::Camera::from_serial(serialNumber_);
        } else {
          cam_ = Metavision::Camera::from_first_available();
        }
      }
      break;  // were able to open the camera, exit the for loop
    } catch (const Metavision::CameraException & e) {
      const std::string src =
        fromFile_.empty() ? (serialNumber_.empty() ? "default" : serialNumber_) : fromFile_;
      LOG_WARN_NAMED(
        "cannot open " << src << " on attempt " << i + 1 << ", retrying " << num_tries - i
                       << " more times");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  try {
    // Record the plugin software information about the camera.
    Metavision::I_PluginSoftwareInfo * psi =
      cam_.get_device().get_facility<Metavision::I_PluginSoftwareInfo>();
    softwareInfo_ = psi->get_plugin_name();
    LOG_INFO_NAMED("Plugin Software Name: " << softwareInfo_);

    if (!biasFile_.empty()) {
      try {
        cam_.biases().set_from_file(biasFile_);
        LOG_INFO_NAMED("using bias file: " << biasFile_);
      } catch (const Metavision::CameraException & e) {
        LOG_WARN_NAMED("reading bias file failed with error: " << e.what());
        LOG_WARN_NAMED("continuing with default biases!");
      }
    } else {
      LOG_INFO_NAMED("no bias file provided, using camera defaults");
    }
    // overwrite serial in case it was not set
    serialNumber_ = cam_.get_camera_configuration().serial_number;
    LOG_INFO_NAMED("camera serial number: " << serialNumber_);
    const auto & g = cam_.geometry();
    width_ = g.width();
    height_ = g.height();
    LOG_INFO_NAMED("sensor geometry: " << width_ << " x " << height_);
    if (fromFile_.empty()) {
      applySyncMode(syncMode_);
      applyROI(roi_);
      configureExternalTriggers(
        triggerInMode_, triggerOutMode_, triggerOutPeriod_, triggerOutDutyCycle_);
      configureEventRateController(ercMode_, ercRate_);
    }
    statusChangeCallbackId_ = cam_.add_status_change_callback(
      std::bind(&MetavisionWrapper::statusChangeCallback, this, ph::_1));
    statusChangeCallbackActive_ = true;
    runtimeErrorCallbackId_ = cam_.add_runtime_error_callback(
      std::bind(&MetavisionWrapper::runtimeErrorCallback, this, ph::_1));
    runtimeErrorCallbackActive_ = true;
    rawDataCallbackId_ = cam_.raw_data().add_callback(std::bind(
      useMultithreading_ ? &MetavisionWrapper::rawDataCallbackMultithreaded
                         : &MetavisionWrapper::rawDataCallback,
      this, ph::_1, ph::_2));
    rawDataCallbackActive_ = true;

  } catch (const Metavision::CameraException & e) {
    LOG_ERROR_NAMED("unexpected sdk error: " << e.what());
    return (false);
  }
  return (true);
}

bool MetavisionWrapper::startCamera(CallbackHandler * h)
{
  try {
    callbackHandler_ = h;
    if (useMultithreading_) {
      processingThread_ = std::make_shared<std::thread>(&MetavisionWrapper::processingThread, this);
    }
    statsThread_ = std::make_shared<std::thread>(&MetavisionWrapper::statsThread, this);
    // this will actually start the camera
    cam_.start();
  } catch (const Metavision::CameraException & e) {
    LOG_ERROR_NAMED("unexpected sdk error: " << e.what());
    return (false);
  }
  return (true);
}

void MetavisionWrapper::runtimeErrorCallback(const Metavision::CameraException & e)
{
  LOG_ERROR_NAMED("camera runtime error occured: " << e.what());
}

void MetavisionWrapper::statusChangeCallback(const Metavision::CameraStatus & s)
{
  LOG_INFO_NAMED("camera " << (s == Metavision::CameraStatus::STARTED ? "started." : "stopped."));
}

bool MetavisionWrapper::saveBiases()
{
  if (biasFile_.empty()) {
    LOG_WARN_NAMED("no bias file specified at startup, no biases saved!");
    return (false);
  } else {
    try {
      cam_.biases().save_to_file(biasFile_);
      LOG_INFO_NAMED("biases written to file: " << biasFile_);
    } catch (const Metavision::CameraException & e) {
      LOG_WARN_NAMED("failed to write bias file: " << e.what());
      return (false);
    }
  }
  return (true);
}

void MetavisionWrapper::rawDataCallback(const uint8_t * data, size_t size)
{
  if (size != 0) {
    const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    callbackHandler_->rawDataCallback(t, data, data + size);
    {
      std::unique_lock<std::mutex> lock(statsMutex_);
      stats_.msgsRecv++;
      stats_.bytesRecv += size;
    }
  }
}

void MetavisionWrapper::rawDataCallbackMultithreaded(const uint8_t * data, size_t size)
{
  // queue stuff away quickly to prevent events from being
  // dropped at the SDK level
  if (size != 0) {
    const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    {
      void * memblock = malloc(size);
      memcpy(memblock, data, size);
      std::unique_lock<std::mutex> lock(mutex_);
      queue_.push_front(QueueElement(memblock, size, t));
      cv_.notify_all();
    }
    {
      std::unique_lock<std::mutex> lock(statsMutex_);
      stats_.msgsRecv++;
      stats_.bytesRecv += size;
    }
  }
}

void MetavisionWrapper::processingThread()
{
  const std::chrono::microseconds timeout((int64_t)(1000000LL));
  while (GENERIC_ROS_OK() && keepRunning_) {
    QueueElement qe;
    size_t qs = 0;
    {  // critical section, no processing done here
      std::unique_lock<std::mutex> lock(mutex_);
      while (GENERIC_ROS_OK() && keepRunning_ && queue_.empty()) {
        cv_.wait_for(lock, timeout);
      }
      if (!queue_.empty()) {
        qs = queue_.size();
        qe = queue_.back();  // makes copy of element, not the data
        queue_.pop_back();
      }
    }
    if (qe.numBytes != 0) {
      const uint8_t * data = static_cast<const uint8_t *>(qe.start);
      callbackHandler_->rawDataCallback(qe.timeStamp, data, data + qe.numBytes);
      free(const_cast<void *>(qe.start));
      {
        std::unique_lock<std::mutex> lock(statsMutex_);
        stats_.maxQueueSize = std::max(stats_.maxQueueSize, qs);
      }
    }
  }
  LOG_INFO_NAMED("processing thread exited!");
}

void MetavisionWrapper::setExternalTriggerOutMode(
  const std::string & mode, const int period, const double duty_cycle)
{
  triggerOutMode_ = mode;
  triggerOutPeriod_ = period;
  triggerOutDutyCycle_ = duty_cycle;
}

void MetavisionWrapper::statsThread()
{
  while (GENERIC_ROS_OK() && keepRunning_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(statsInterval_ * 1000)));
    printStatistics();
  }
  LOG_INFO_NAMED("statistics thread exited!");
}

void MetavisionWrapper::printStatistics()
{
  Stats stats;
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats = stats_;
    stats_ = Stats();  // reset statistics
  }
  std::chrono::time_point<std::chrono::system_clock> t_now = std::chrono::system_clock::now();
  const double dt = std::chrono::duration<double>(t_now - lastPrintTime_).count();
  lastPrintTime_ = t_now;
  const double invT = dt > 0 ? 1.0 / dt : 0;
  const double recvByteRate = 1e-6 * stats.bytesRecv * invT;

  const int recvMsgRate = static_cast<int>(stats.msgsRecv * invT);
  const int sendMsgRate = static_cast<int>(stats.msgsSent * invT);

#ifndef USING_ROS_1
  if (useMultithreading_) {
    LOG_INFO_NAMED_FMT(
      "bw in: %9.5f MB/s, msgs/s in: %7d, "
      "out: %7d, maxq: %4zu",
      recvByteRate, recvMsgRate, sendMsgRate, stats.maxQueueSize);
  } else {
    LOG_INFO_NAMED_FMT(
      "bw in: %9.5f MB/s, msgs/s in: %7d, "
      "out: %7d",
      recvByteRate, recvMsgRate, sendMsgRate);
  }
#else
  if (useMultithreading_) {
    LOG_INFO_NAMED_FMT(
      "%s: bw in: %9.5f MB/s, msgs/s in: %7d, out: %7d, maxq: %4zu", loggerName_.c_str(),
      recvByteRate, recvMsgRate, sendMsgRate, stats.maxQueueSize);
  } else {
    LOG_INFO_NAMED_FMT(
      "%s: bw in: %9.5f MB/s, msgs/s in: %7d, out: %7d", loggerName_.c_str(), recvByteRate,
      recvMsgRate, sendMsgRate);
  }
#endif
}

}  // namespace metavision_ros_driver
