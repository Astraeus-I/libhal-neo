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

result<neo_gps> neo_gps::create(hal::serial& p_serial,
                                std::vector<hal::neo::nmea_parser*>& p_parsers)
{
  neo_gps new_neo(p_serial, p_parsers);
  return new_neo;
}

hal::result<std::span<const hal::byte>> neo_gps::read_serial()
{
  auto bytes_read_array = HAL_CHECK(m_serial->read(m_gps_buffer)).data;
  return bytes_read_array;
}

hal::result<neo_gps::gps_data_t> neo_gps::read()
{
  auto bytes_read_array = HAL_CHECK(read_serial());
  gps_sentences_t sentence_objs;
  gps_data_t gps_data;

  for (auto& parser_ptr : m_parsers) {
    switch (parser_ptr->getType()) {
      case nmea_parser::ParserType::GGA: {
        sentence_objs.gga_sentece.parse(bytes_read_array);
        gps_data.gga_data = sentence_objs.gga_sentece.read();
      } break;
      case nmea_parser::ParserType::GSA: {
        sentence_objs.gsa_sentece.parse(bytes_read_array);
        gps_data.gsa_data = sentence_objs.gsa_sentece.read();
      } break;
      case nmea_parser::ParserType::GSV: {
        sentence_objs.gsv_sentece.parse(bytes_read_array);
        gps_data.gsv_data = sentence_objs.gsv_sentece.read();
      } break;
      case nmea_parser::ParserType::RMC: {
        sentence_objs.rmc_sentece.parse(bytes_read_array);
        gps_data.rmc_data = sentence_objs.rmc_sentece.read();
      } break;
      default:
        sentence_objs = sentence_objs;
        break;
    }
  }

  return result<neo_gps::gps_data_t>(gps_data);
}

neo_gps::nmea_parse_t parse(std::span<nmea_parser*> p_parsers,
                            std::span<const hal::byte> p_data)
{
  bool end_token_found = false;

  if (!p_data.empty()) {
    for (auto& parser : p_parsers) {
      if (parser->state() == nmea_parser::state_t::inactive) {
        p_data = parser->parse(p_data);
      }
    }
  }

  return { p_data, end_token_found };
}

nmea_parser::ParserType GGA_Sentence::getType() const
{
  return nmea_parser::ParserType::GGA;
}

nmea_parser::ParserType GSA_Sentence::getType() const
{
  return nmea_parser::ParserType::GSA;
}

nmea_parser::ParserType GSV_Sentence::getType() const
{
  return nmea_parser::ParserType::GSV;
}

nmea_parser::ParserType RMC_Sentence::getType() const
{
  return nmea_parser::ParserType::RMC;
}

GGA_Sentence::GGA_Sentence()
{
}

GSA_Sentence::GSA_Sentence()
{
}

GSV_Sentence::GSV_Sentence()
{
}

RMC_Sentence::RMC_Sentence()
{
}

nmea_parser::state_t GGA_Sentence::state()
{
  return m_state;
}

nmea_parser::state_t GSA_Sentence::state()
{
  return m_state;
}

nmea_parser::state_t GSV_Sentence::state()
{
  return m_state;
}

nmea_parser::state_t RMC_Sentence::state()
{
  return m_state;
}

void GGA_Sentence::reset()
{
  m_state = state_t::inactive;
}

void GSA_Sentence::reset()
{
  m_state = state_t::inactive;
}

void GSV_Sentence::reset()
{
  m_state = state_t::inactive;
}

void RMC_Sentence::reset()
{
  m_state = state_t::inactive;
}

std::span<const hal::byte> GGA_Sentence::parse(
  std::span<const hal::byte> p_data)
{
  using namespace std::literals;
  m_state = state_t::active;

  auto start_of_line_finder =
    hal::stream_find(hal::as_bytes(gga_start_of_line));
  auto end_of_line_finder = hal::stream_find(hal::as_bytes(end_of_line));

  auto start_of_line_found = p_data | start_of_line_finder;
  auto remaining_data = start_of_line_found | end_of_line_finder;

  std::string_view gga_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    remaining_data.data() - start_of_line_found.data());

  int ret = sscanf(gga_data.data(),
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

  return std::span<const hal::byte>(p_data);
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

std::span<const hal::byte> GSA_Sentence::parse(
  std::span<const hal::byte> p_data)
{
  using namespace std::literals;
  m_state = state_t::active;

  auto start_of_line_finder =
    hal::stream_find(hal::as_bytes(gsa_start_of_line));
  auto end_of_line_finder = hal::stream_find(hal::as_bytes(end_of_line));

  auto start_of_line_found = p_data | start_of_line_finder;
  auto remaining_data = start_of_line_found | end_of_line_finder;

  std::string_view gsa_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    remaining_data.data() - start_of_line_found.data());

  int ret = sscanf(gsa_data.data(),
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

  return std::span<const hal::byte>(p_data);
}

GSA_Sentence::gsa_data_t GSA_Sentence::read()
{
  return GSA_Sentence::gsa_data_t(m_gsa_data);
}

std::span<const hal::byte> GSV_Sentence::parse(
  std::span<const hal::byte> p_data)
{
  using namespace std::literals;
  m_state = state_t::active;

  auto start_of_line_finder =
    hal::stream_find(hal::as_bytes(gsv_start_of_line));
  auto end_of_line_finder = hal::stream_find(hal::as_bytes(end_of_line));

  auto start_of_line_found = p_data | start_of_line_finder;
  auto remaining_data = start_of_line_found | end_of_line_finder;

  std::string_view gsv_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    remaining_data.data() - start_of_line_found.data());

  int ret = sscanf(gsv_data.data(),
                   GSV_FORMAT,
                   &m_satellite_data.number_of_messages,
                   &m_satellite_data.message_number,
                   &m_satellite_data.satellites_in_view,
                   &m_satellite_data.id,
                   &m_satellite_data.elevation,
                   &m_satellite_data.azimuth,
                   &m_satellite_data.snr);

  return std::span<const hal::byte>(p_data);
}

GSV_Sentence::satellite_data_t GSV_Sentence::read()
{
  return GSV_Sentence::satellite_data_t(m_satellite_data);
}

std::span<const hal::byte> RMC_Sentence::parse(
  std::span<const hal::byte> p_data)
{
  using namespace std::literals;
  m_state = state_t::active;

  auto start_of_line_finder =
    hal::stream_find(hal::as_bytes(rmc_start_of_line));
  auto end_of_line_finder = hal::stream_find(hal::as_bytes(end_of_line));

  auto start_of_line_found = p_data | start_of_line_finder;
  auto remaining_data = start_of_line_found | end_of_line_finder;

  std::string_view rmc_data(
    reinterpret_cast<const char*>(start_of_line_found.data()),
    remaining_data.data() - start_of_line_found.data());

  int ret = sscanf(rmc_data.data(),
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

  return std::span<const hal::byte>(p_data);
}

RMC_Sentence::rmc_data_t RMC_Sentence::read()
{
  return RMC_Sentence::rmc_data_t(m_rmc_data);
}

}  // namespace hal::neo