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

#include <string_view>

#include <algorithm>
#include <array>
#include <span>

#include <libhal-neo/neo.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/streams.hpp>
#include <libhal-util/timeout.hpp>
#include <libhal/serial.hpp>

#include "neo-constants.hpp"

namespace hal::neo {

std::string GGA_Sentence::sentence_header() const
{
  return std::string(gga_start_of_line);
}

std::string VTG_Sentence::sentence_header() const
{
  return std::string(vtg_start_of_line);
}

std::string GSA_Sentence::sentence_header() const
{
  return std::string(gsa_start_of_line);
}

std::string GSV_Sentence::sentence_header() const
{
  return std::string(gsv_start_of_line);
}

std::string RMC_Sentence::sentence_header() const
{
  return std::string(rmc_start_of_line);
}

std::string ZDA_Sentence::sentence_header() const
{
  return std::string(zda_start_of_line);
}

std::string PASHR_Sentence::sentence_header() const
{
  return std::string(pashr_start_of_line);
}

void GGA_Sentence::parse(std::string_view p_data)
{

  int ret = sscanf(p_data.data(),
                   GGA_FORMAT,
                   &m_gga_data.time,
                   &m_gga_data.latitude,
                   &m_gga_data.latitude_direction,
                   &m_gga_data.longitude,
                   &m_gga_data.longitude_direction,
                   &m_gga_data.fix_status,
                   &m_gga_data.satellites_used,
                   &m_gga_data.hdop,
                   &m_gga_data.altitude,
                   &m_gga_data.altitude_units,
                   &m_gga_data.height_of_geoid,
                   &m_gga_data.height_of_geoid_units,
                   m_gga_data.dgps_station_id_checksum);

  m_gga_data.is_locked = (ret < 5) ? false : true;
}

GGA_Sentence::gga_data_t GGA_Sentence::calculate_lon_lat(
  const GGA_Sentence::gga_data_t& p_gps_data)
{

  GGA_Sentence::gga_data_t modified_data = p_gps_data;
  char lon_dir = modified_data.longitude_direction;
  char lat_dir = modified_data.latitude_direction;
  float lon = modified_data.longitude;
  float lat = modified_data.latitude;

  if (lon_dir == 'W') {
    lon = -lon;
  }
  if (lat_dir == 'S') {
    lat = -lat;
  }

  float lon_intpart = static_cast<int>(lon / 100);
  float lon_fractpart = lon - (lon_intpart * 100);
  lon = lon_intpart + (lon_fractpart / 60);

  float lat_intpart = static_cast<int>(lat / 100);
  float lat_fractpart = lat - (lat_intpart * 100);
  lat = lat_intpart + (lat_fractpart / 60);

  modified_data.longitude = lon;
  modified_data.latitude = lat;

  return GGA_Sentence::gga_data_t(modified_data);
}

GGA_Sentence::gga_data_t GGA_Sentence::read()
{
  auto data = calculate_lon_lat(m_gga_data);
  return GGA_Sentence::gga_data_t(data);
}

void VTG_Sentence::parse(std::string_view p_data)
{

  sscanf(p_data.data(),
         VTG_FORMAT,
         &m_vtg_data.true_track_degrees,
         &m_vtg_data.true_track_degrees_t,
         &m_vtg_data.magnetic_track_degrees,
         &m_vtg_data.magnetic_track_degrees_t,
         &m_vtg_data.ground_speed_knots,
         &m_vtg_data.ground_speed_knots_n,
         &m_vtg_data.ground_speed_kph,
         &m_vtg_data.ground_speed_kph_k);
}

VTG_Sentence::vtg_data_t VTG_Sentence::read()
{
  return VTG_Sentence::vtg_data_t(m_vtg_data);
}

void GSA_Sentence::parse(std::string_view p_data)
{

  sscanf(p_data.data(),
         GSA_FORMAT,
         &m_gsa_data.mode,
         &m_gsa_data.fix_type,
         &m_gsa_data.satellite_ids[0],
         &m_gsa_data.satellite_ids[1],
         &m_gsa_data.satellite_ids[2],
         &m_gsa_data.satellite_ids[3],
         &m_gsa_data.satellite_ids[4],
         &m_gsa_data.satellite_ids[5],
         &m_gsa_data.satellite_ids[6],
         &m_gsa_data.satellite_ids[7],
         &m_gsa_data.satellite_ids[8],
         &m_gsa_data.satellite_ids[9],
         &m_gsa_data.satellite_ids[10],
         &m_gsa_data.satellite_ids[11],
         &m_gsa_data.pdop,
         &m_gsa_data.hdop,
         &m_gsa_data.vdop);
}

GSA_Sentence::gsa_data_t GSA_Sentence::read()
{
  return GSA_Sentence::gsa_data_t(m_gsa_data);
}

void GSV_Sentence::parse(std::string_view p_data)
{

  sscanf(p_data.data(),
         GSV_FORMAT,
         &m_satellite_data.number_of_messages,
         &m_satellite_data.message_number,
         &m_satellite_data.satellites_in_view,
         &m_satellite_data.id,
         &m_satellite_data.elevation,
         &m_satellite_data.azimuth,
         &m_satellite_data.snr);
}

GSV_Sentence::satellite_data_t GSV_Sentence::read()
{
  return GSV_Sentence::satellite_data_t(m_satellite_data);
}

void RMC_Sentence::parse(std::string_view p_data)
{

  int ret = sscanf(p_data.data(),
                   RMC_FORMAT,
                   &m_rmc_data.time,
                   &m_rmc_data.status,
                   &m_rmc_data.latitude,
                   &m_rmc_data.latitude_direction,
                   &m_rmc_data.longitude,
                   &m_rmc_data.longitude_direction,
                   &m_rmc_data.speed,
                   &m_rmc_data.track_angle,
                   &m_rmc_data.date,
                   &m_rmc_data.magnetic_variation,
                   &m_rmc_data.magnetic_direction);

  m_rmc_data.reading_status = ret;
}

RMC_Sentence::rmc_data_t RMC_Sentence::read()
{
  return RMC_Sentence::rmc_data_t(m_rmc_data);
}

void ZDA_Sentence::parse(std::string_view p_data)
{

  sscanf(p_data.data(),
         ZDA_FORMAT,
         &m_zda_data.time,
         &m_zda_data.day,
         &m_zda_data.month,
         &m_zda_data.year);
}

ZDA_Sentence::zda_data_t ZDA_Sentence::read()
{
  return ZDA_Sentence::zda_data_t(m_zda_data);
}

void PASHR_Sentence::parse(std::string_view p_data)
{

  sscanf(p_data.data(),
         PASHR_FORMAT,
         &m_pashr_data.heading,
         &m_pashr_data.pitch,
         &m_pashr_data.roll,
         &m_pashr_data.heave,
         &m_pashr_data.yaw,
         &m_pashr_data.tilt,
         &m_pashr_data.roll_accuracy,
         &m_pashr_data.pitch_accuracy,
         &m_pashr_data.heading_accuracy,
         &m_pashr_data.heave_accuracy,
         &m_pashr_data.yaw_accuracy,
         &m_pashr_data.tilt_accuracy);
}

PASHR_Sentence::pashr_data_t PASHR_Sentence::read()
{
  return PASHR_Sentence::pashr_data_t(m_pashr_data);
}

result<nmea_router> nmea_router::create(
  hal::serial& p_serial,
  const std::vector<nmea_parser*>& p_parsers)
{
  nmea_router new_nmea_router(p_serial, p_parsers);
  return new_nmea_router;
}

hal::result<std::span<const hal::byte>> nmea_router::read_serial()
{
  auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;
  return bytes_read_array;
}

hal::result<std::string_view> nmea_router::route(
  std::span<const hal::byte> p_data)
{

  using namespace std::literals;

  for (auto* parser : m_parsers) {
    auto start_of_line = parser->sentence_header();
    auto start_of_line_finder = hal::stream_find(hal::as_bytes(start_of_line));
    auto end_of_line_finder = hal::stream_find(hal::as_bytes(end_of_line));

    auto start_of_line_found = p_data | start_of_line_finder;
    auto remaining_data = start_of_line_found | end_of_line_finder;

    std::string_view data(
      reinterpret_cast<const char*>(start_of_line_found.data()),
      remaining_data.data() - start_of_line_found.data());
    parser->parse(data);
    return data;
  }
}

}  // namespace hal::neo