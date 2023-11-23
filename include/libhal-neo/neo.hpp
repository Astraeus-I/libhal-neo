// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include <array>
#include <libhal-neo/nmea_parser.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/streams.hpp>
#include <libhal/functional.hpp>
#include <libhal/serial.hpp>

namespace hal::neo {

class GGA_Sentence : public nmea_parser
{
public:
  GGA_Sentence()
  {
  }
  struct gga_data_t
  {
    bool is_locked = false;
    float time = 0.0f;
    float latitude = 0.0f;
    char latitude_direction = ' ';
    float longitude = 0.0f;
    char longitude_direction = ' ';
    int fix_status = 0;
    int satellites_used = 0;
    float hdop = 0.0f;
    float altitude = 0.0f;
    char altitude_units = ' ';
    float height_of_geoid = 0.0f;
    char height_of_geoid_units = ' ';
    char time_since_last_dgps_update = ' ';
    char dgps_station_id_checksum[10] = { '\0' };
  };
  void parse(std::string_view p_data) override;
  gga_data_t read();
  gga_data_t calculate_lon_lat(const gga_data_t& p_gps_data);
  std::string_view sentence_header() const override;

private:
  gga_data_t m_gga_data;
};

class VTG_Sentence : public nmea_parser
{
public:
  VTG_Sentence()
  {
  }
  struct vtg_data_t
  {
    float true_track_degrees = 0.0f;
    char true_track_degrees_t = ' ';
    float magnetic_track_degrees = 0.0f;
    char magnetic_track_degrees_t = ' ';
    float ground_speed_knots = 0.0f;
    char ground_speed_knots_n = ' ';
    float ground_speed_kph = 0.0f;
    char ground_speed_kph_k = ' ';
    char mode;
  };
  void parse(std::string_view p_data) override;
  std::string_view sentence_header() const override;
  vtg_data_t read();

private:
  vtg_data_t m_vtg_data;
};

class GSA_Sentence : public nmea_parser
{
public:
  GSA_Sentence()
  {
  }
  struct gsa_data_t
  {
    char mode = ' ';
    int fix_type = 0;
    std::array<int, 12> satellite_ids{};  // assuming up to 12 satellites
    float pdop = 0.0f;
    float hdop = 0.0f;
    float vdop = 0.0f;
  };
  void parse(std::string_view p_data) override;
  std::string_view sentence_header() const override;
  gsa_data_t read();

private:
  gsa_data_t m_gsa_data;
};

class GSV_Sentence : public nmea_parser
{
public:
  GSV_Sentence()
  {
  }
  struct satellite_data_t
  {
    int number_of_messages = 0;
    int message_number = 0;
    int satellites_in_view = 0;
    int id = 0;
    int elevation = 0;
    int azimuth = 0;
    int snr = 0;
  };
  void parse(std::string_view p_data) override;
  std::string_view sentence_header() const override;
  satellite_data_t read();

private:
  satellite_data_t m_satellite_data;
};

class RMC_Sentence : public nmea_parser
{
public:
  RMC_Sentence()
  {
  }
  struct rmc_data_t
  {

    int reading_status = 0;

    float time = 0.0f;
    char status = ' ';
    float latitude = 0.0f;
    char latitude_direction = ' ';
    float longitude = 0.0f;
    char longitude_direction = ' ';
    float speed = 0.0f;
    float track_angle = 0.0f;
    int date = 0;
    float magnetic_variation = 0.0f;
    char magnetic_direction = ' ';
  };
  void parse(std::string_view p_data) override;
  std::string_view sentence_header() const override;
  rmc_data_t read();

private:
  rmc_data_t m_rmc_data;
};

class ZDA_Sentence : public nmea_parser
{
public:
  ZDA_Sentence()
  {
  }
  struct zda_data_t
  {
    float time = 0.0f;
    int day = 0;
    int month = 0;
    int year = 0;
  };
  void parse(std::string_view p_data) override;
  std::string_view sentence_header() const override;
  zda_data_t read();

private:
  zda_data_t m_zda_data;
};

class PASHR_Sentence : public nmea_parser
{
public:
  PASHR_Sentence()
  {
  }
  struct pashr_data_t
  {
    float heading = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float heave = 0.0f;
    float yaw = 0.0f;
    float tilt = 0.0f;
    float roll_accuracy = 0.0f;
    float pitch_accuracy = 0.0f;
    float heading_accuracy = 0.0f;
    float heave_accuracy = 0.0f;
    float yaw_accuracy = 0.0f;
    float tilt_accuracy = 0.0f;
  };
  void parse(std::string_view p_data) override;
  std::string_view sentence_header() const override;
  pashr_data_t read();

private:
  pashr_data_t m_pashr_data;
};

class nmea_router
{

public:
  enum class error_handling
  {
    thow_exeption,
    ignore_failures
  };

  [[nodiscard]] static result<nmea_router> create(
    hal::serial& p_console,
    hal::serial& p_serial,
    const std::array<hal::neo::nmea_parser*, 6>& p_parsers = {});
  hal::status parse();

private:
  nmea_router(hal::serial& p_console,
              hal::serial& p_serial,
              const std::array<hal::neo::nmea_parser*, 6>& p_parsers)

    : m_console(&p_console)
    , m_serial(&p_serial)
    , m_parsers(p_parsers)
  {
  }
  hal::result<std::span<const hal::byte>> read_serial();
  hal::result<std::string_view> route(nmea_parser* p_parser,
                                      std::span<const hal::byte> p_data);

  hal::serial* m_console;
  hal::serial* m_serial;
  std::array<hal::neo::nmea_parser*, 6> m_parsers;
  std::array<hal::byte, 1020> m_gps_buffer;
};

}  // namespace hal::neo