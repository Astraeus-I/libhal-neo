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

#include <libhal-neo/nmea_parser.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/streams.hpp>
#include <libhal/functional.hpp>
#include <libhal/serial.hpp>
#include <vector>

namespace hal::neo {

class GGA_Sentence : public nmea_parser
{
public:
  GGA_Sentence();
  struct gga_data_t
  {
    bool is_locked = false;
    float time;
    float latitude;
    char latitude_direction;
    float longitude;
    char longitude_direction;
    int fix_status;
    int satellites_used;
    float hdop;
    float altitude;
    char altitude_units;
    float height_of_geoid;
    char height_of_geoid_units;
    char time_since_last_dgps_update;
    char dgps_station_id_checksum[10];
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  gga_data_t read();
  gga_data_t calculate_lon_lat(const gga_data_t& p_gps_data);
  std::string sentence_header() const override;

private:
  gga_data_t m_gga_data;
};

class VTG_Sentence : public nmea_parser
{
public:
  VTG_Sentence();
  struct vtg_data_t
  {
    float true_track_degrees;
    char true_track_degrees_t;
    float magnetic_track_degrees;
    char magnetic_track_degrees_t;
    float ground_speed_knots;
    char ground_speed_knots_n;
    float ground_speed_kph;
    char ground_speed_kph_k;
    char mode;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  std::string sentence_header() const override;
  vtg_data_t read();

private:
  vtg_data_t m_vtg_data;
};

class GSA_Sentence : public nmea_parser
{
public:
  GSA_Sentence();
  struct gsa_data_t
  {
    char mode;
    int fix_type;
    std::array<int, 12> satellite_ids{};  // assuming up to 12 satellites
    float pdop;
    float hdop;
    float vdop;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  std::string sentence_header() const override;
  gsa_data_t read();

private:
  gsa_data_t m_gsa_data;
};

class GSV_Sentence : public nmea_parser
{
public:
  GSV_Sentence();
  struct satellite_data_t
  {
    int number_of_messages;
    int message_number;
    int satellites_in_view;
    int id;
    int elevation;
    int azimuth;
    int snr;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  std::string sentence_header() const override;
  satellite_data_t read();

private:
  satellite_data_t m_satellite_data;
};

class RMC_Sentence : public nmea_parser
{
public:
  RMC_Sentence();
  struct rmc_data_t
  {

    int reading_status = 0;

    float time;
    char status;
    float latitude;
    char latitude_direction;
    float longitude;
    char longitude_direction;
    float speed;
    float track_angle;
    int date;
    float magnetic_variation;
    char magnetic_direction;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  std::string sentence_header() const override;
  rmc_data_t read();

private:
  rmc_data_t m_rmc_data;
};

class ZDA_Sentence : public nmea_parser
{
public:
  ZDA_Sentence();
  struct zda_data_t
  {
    float time;
    int day;
    int month;
    int year;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  std::string sentence_header() const override;
  zda_data_t read();

private:
  zda_data_t m_zda_data;
};

class PASHR_Sentence : public nmea_parser
{
public:
  PASHR_Sentence();
  struct pashr_data_t
  {
    float heading;
    float pitch;
    float roll;
    float heave;
    float yaw;
    float tilt;
    float roll_accuracy;
    float pitch_accuracy;
    float heading_accuracy;
    float heave_accuracy;
    float yaw_accuracy;
    float tilt_accuracy;
  };
  std::span<const hal::byte> parse(std::span<const hal::byte> p_data) override;
  std::string sentence_header() const override;
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


  [[nodiscard]] static result<nmea_router> create(hal::serial& p_serial, const std::vector<nmea_parser*>& p_parsers = {});
  hal::result<std::span<const hal::byte>> read_serial();
  hal::status route(std::span<const hal::byte> data);

private:
  nmea_router(hal::serial& p_serial, const std::vector<nmea_parser*>& p_parsers)
    : m_serial(&p_serial),
      m_parsers(p_parsers)
  {
  }
  hal::serial* m_serial;
  std::vector<nmea_parser*> m_parsers;
  std::array<hal::byte, 1020> m_gps_buffer;
};

}  // namespace hal::neo