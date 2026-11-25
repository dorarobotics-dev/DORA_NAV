extern "C"
{
#include "node_api.h"
}

#include "CYdLidar.h"
#include "config_loader.hpp"
#include "scan_contract.hpp"
#include "ydlidar_tmini_plus_config.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef YDLIDAR_TMINI_PLUS_DEFAULT_CONFIG
#define YDLIDAR_TMINI_PLUS_DEFAULT_CONFIG "../modules/drivers/ydlidar_tmini_plus/config/config.yaml"
#endif

namespace
{

template <typename T>
void SetOption(CYdLidar &laser, int property, const T &value, const char *name)
{
  if (!laser.setlidaropt(property, &value, sizeof(T)))
  {
    throw std::runtime_error(std::string("failed to set YDLIDAR option: ") + name);
  }
}

void SetStringOption(CYdLidar &laser, int property, const std::string &value, const char *name)
{
  if (!laser.setlidaropt(property, value.c_str(), static_cast<int>(value.size())))
  {
    throw std::runtime_error(std::string("failed to set YDLIDAR option: ") + name);
  }
}

void ConfigureLaser(CYdLidar &laser, const ydlidar_tmini_plus::Config &config)
{
  SetStringOption(laser, LidarPropSerialPort, config.serial_port, "serial port");
  SetStringOption(laser, LidarPropIgnoreArray, "", "ignore array");

  SetOption(laser, LidarPropSerialBaudrate, config.serial_baud, "baudrate");
  const int lidar_type = TYPE_TRIANGLE;
  const int device_type = YDLIDAR_TYPE_SERIAL;
  const int sample_rate = 4;
  const int abnormal_count = 4;
  const int intensity_bits = 8;
  SetOption(laser, LidarPropLidarType, lidar_type, "lidar type");
  SetOption(laser, LidarPropDeviceType, device_type, "device type");
  SetOption(laser, LidarPropSampleRate, sample_rate, "sample rate");
  SetOption(laser, LidarPropAbnormalCheckCount, abnormal_count, "abnormal check count");
  SetOption(laser, LidarPropIntenstiyBit, intensity_bits, "intensity bits");

  const bool enabled = true;
  const bool disabled = false;
  SetOption(laser, LidarPropFixedResolution, enabled, "fixed resolution");
  SetOption(laser, LidarPropReversion, disabled, "reversion");
  SetOption(laser, LidarPropInverted, disabled, "inverted");
  SetOption(laser, LidarPropAutoReconnect, enabled, "auto reconnect");
  SetOption(laser, LidarPropSingleChannel, disabled, "single channel");
  SetOption(laser, LidarPropIntenstiy, enabled, "intensity");
  SetOption(laser, LidarPropSupportMotorDtrCtrl, disabled, "motor DTR");
  SetOption(laser, LidarPropSupportHeartBeat, disabled, "heartbeat");

  const float maximum_angle = 180.0F;
  const float minimum_angle = -180.0F;
  const float maximum_range = static_cast<float>(config.range_max);
  const float minimum_range = static_cast<float>(config.range_min);
  const float scan_frequency = static_cast<float>(config.scan_frequency_hz);
  SetOption(laser, LidarPropMaxAngle, maximum_angle, "maximum angle");
  SetOption(laser, LidarPropMinAngle, minimum_angle, "minimum angle");
  SetOption(laser, LidarPropMaxRange, maximum_range, "maximum range");
  SetOption(laser, LidarPropMinRange, minimum_range, "minimum range");
  SetOption(laser, LidarPropScanFrequency, scan_frequency, "scan frequency");
  laser.enableGlassNoise(false);
  laser.enableSunNoise(false);
}

void SendJson(void *context, const std::string &output_id, const nlohmann::json &message)
{
  const std::string payload = message.dump();
  const int result = dora_send_output(
    context, const_cast<char *>(output_id.data()), output_id.size(),
    payload.data(), payload.size());
  if (result != 0)
  {
    throw std::runtime_error("failed to send LaserScan: " + std::to_string(result));
  }
}

} // namespace

int main()
{
  void *context = nullptr;
  bool scanning = false;
  CYdLidar laser;
  try
  {
    const ydlidar_tmini_plus::Config config =
      ydlidar_tmini_plus::LoadDriverConfig(YDLIDAR_TMINI_PLUS_DEFAULT_CONFIG);
    const auto mask_windows = ydlidar_tmini_plus::ParseAngleWindows(config.mask_angle_ranges_deg);
    std::cout << "YDLIDAR T-mini Plus native Dora node\n"
              << "serial=" << config.serial_port << " baud=" << config.serial_baud << '\n'
              << "frame_id=" << config.frame_id << " output=" << config.output_id << '\n'
              << "angle=[" << config.angle_min_deg << ", " << config.angle_max_deg
              << "] range=[" << config.range_min << ", " << config.range_max << "]\n";

    ydlidar::os_init();
    ConfigureLaser(laser, config);
    if (!laser.initialize())
    {
      throw std::runtime_error(std::string("YDLIDAR initialize failed: ") + laser.DescribeError());
    }
    if (!laser.turnOn())
    {
      throw std::runtime_error(std::string("YDLIDAR scan start failed: ") + laser.DescribeError());
    }
    scanning = true;

    context = init_dora_context_from_env();
    if (context == nullptr)
    {
      throw std::runtime_error("failed to initialize Dora context");
    }

    int sequence = 0;
    int tick_count = 0;
    int scan_ok_count = 0;
    int scan_fail_count = 0;
    int scan_sent_count = 0;
    while (true)
    {
      void *event = dora_next_event(context);
      if (event == nullptr)
      {
        break;
      }
      const DoraEventType type = read_dora_event_type(event);
      if (type == DoraEventType_Stop)
      {
        free_dora_event(event);
        break;
      }
      if (type == DoraEventType_Input)
      {
        ++tick_count;
        LaserScan scan;
        if (laser.doProcessSimple(scan) && !scan.points.empty())
        {
          // SDK stamp is Unix nanoseconds at acquisition, not at publication.
          using ScanNanoseconds = std::chrono::nanoseconds;
          if (scan.stamp == 0 || scan.stamp > static_cast<std::uint64_t>(
                std::numeric_limits<ScanNanoseconds::rep>::max()))
          {
            ++scan_fail_count;
            std::cerr << "YDLIDAR T-mini Plus invalid SDK scan timestamp: "
                      << scan.stamp << '\n';
            free_dora_event(event);
            continue;
          }
          const std::chrono::time_point<std::chrono::system_clock, ScanNanoseconds>
            acquisition_time{ScanNanoseconds{static_cast<ScanNanoseconds::rep>(scan.stamp)}};
          ++scan_ok_count;
          std::vector<ydlidar_tmini_plus::ScanPoint> points;
          points.reserve(scan.points.size());
          for (const auto &point : scan.points)
          {
            points.push_back({point.angle, point.range, point.intensity});
          }
          ydlidar_tmini_plus::ScanOptions options;
          options.frame_id = config.frame_id;
          options.seq = sequence++;
          options.angle_min_deg = config.angle_min_deg;
          options.angle_max_deg = config.angle_max_deg;
          options.range_min = config.range_min;
          options.range_max = config.range_max;
          options.clockwise = config.clockwise;
          options.mask_windows = mask_windows;
          options.scan_time_s = scan.config.scan_time;
          options.target_count = config.target_point_count;
          options.rotation_speed_hz = scan.scanFreq;
          SendJson(context, config.output_id,
                   ydlidar_tmini_plus::BuildLaserScanJson(points, options, acquisition_time));
          ++scan_sent_count;
        }
        else
        {
          ++scan_fail_count;
          std::cerr << "YDLIDAR T-mini Plus scan read failed: " << laser.DescribeError() << '\n';
        }
        if (tick_count % 100 == 0)
        {
          std::cout << "YDLIDAR T-mini Plus stats: ticks=" << tick_count
                    << " ok=" << scan_ok_count
                    << " fail=" << scan_fail_count
                    << " sent=" << scan_sent_count << '\n';
        }
      }
      free_dora_event(event);
    }
    std::cout << "YDLIDAR T-mini Plus final stats: ticks=" << tick_count
              << " ok=" << scan_ok_count
              << " fail=" << scan_fail_count
              << " sent=" << scan_sent_count << '\n';
    laser.turnOff();
    scanning = false;
    laser.disconnecting();
    free_dora_context(context);
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "fatal: " << error.what() << '\n';
    if (scanning)
    {
      laser.turnOff();
    }
    laser.disconnecting();
    if (context != nullptr)
    {
      free_dora_context(context);
    }
    return 1;
  }
}
