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

#include "../hardware_map.hpp"
#include <libhal-neo/neo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application(hardware_map& p_map)
{
  using namespace std::chrono_literals;
  using namespace std::literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& gps = *p_map.gps;

  hal::neo::GGA_Sentence gga_sentence;
  hal::neo::VTG_Sentence vtg_sentence;
  hal::neo::GSA_Sentence gsa_sentence;
  hal::neo::GSV_Sentence gsv_sentence;
  hal::neo::RMC_Sentence rmc_sentence;
  hal::neo::ZDA_Sentence zda_sentence;

  std::array<hal::neo::nmea_parser*, 6> parsers = {
    &gga_sentence, &vtg_sentence, &gsa_sentence, &rmc_sentence
  };
  

  hal::print(console, "Initializing GPS...\n");
  auto GPS = HAL_CHECK(hal::neo::nmea_router::create(console, gps, parsers));

  hal::print(console, "GPS created! \n");
  hal::print(
    console,
    "***You may need to wait a few minutes before having a full GPS lock***\n");

  while (true) {
    HAL_CHECK(GPS.parse());

    auto GGA = gga_sentence.read();
    auto VTG = vtg_sentence.read();
    auto GSA = gsa_sentence.read();
    auto GSV = gsv_sentence.read();
    auto RMC = rmc_sentence.read();
    auto ZDA = zda_sentence.read();

    hal::print(console,
               "====================================GPS "
               "READ====================================\n");

    hal::print(
      console,
      "\n=================== GPS Coordinate Data (GGA) ===================\n");
    hal::print<128>(console,
                    "Time: %f\nLatitude: %f\nLongitude: %f\nNumber of "
                    "satellites seen: %d\nAltitude: %f meters",
                    GGA.time,
                    GGA.latitude,
                    GGA.longitude,
                    GGA.satellites_used,
                    GGA.altitude);

    hal::print(
      console,
      "\n=================== GPS Status Data (GSA) ===================\n");
    hal::print<128>(console,
                    "Fix type: %d\nFix mode: %c\nPDOP: %f\nHDOP: %f\nVDOP: "
                    "%f\n",
                    GSA.fix_type,
                    GSA.mode,
                    GSA.pdop,
                    GSA.hdop,
                    GSA.vdop);

    hal::print(
      console,
      "\n=================== GPS Velocity Data (VTG) ===================\n");
    hal::print<128>(
      console,
      "True track degrees: %f\nTrue track degrees type: %c\n"
      "Magnetic track degrees: %f\nMagnetic track degrees type: "
      "%c\nGround speed knots: %f\nGround speed knots type: "
      "%c\nGround speed kph: %f\nGround speed kph type: %c\nMode: "
      "%c\n",
      VTG.true_track_degrees,
      VTG.true_track_degrees_t,
      VTG.magnetic_track_degrees,
      VTG.magnetic_track_degrees_t,
      VTG.ground_speed_knots,
      VTG.ground_speed_knots_n,
      VTG.ground_speed_kph,
      VTG.ground_speed_kph_k,
      VTG.mode);

    hal::print(
      console,
      "\n=================== GPS Satellite Data (GSV) ===================\n");
    hal::print<128>(console,
                    "Satellites in view: %d\nElevation: %d\nAzimuth: %d\nSNR: "
                    "%d\n",
                    GSV.satellites_in_view,
                    GSV.elevation,
                    GSV.azimuth,
                    GSV.snr);

    hal::print(
      console,
      "\n=================== GPS Speed Data (RMC) ===================\n");
    hal::print<128>(
      console,
      "Time: %f\nSpeed: %f\nTrack angle: %f\nMagnetic direction: %c\n",
      RMC.time,
      RMC.speed,
      RMC.track_angle,
      RMC.magnetic_direction);

    hal::print<128>(console, "Reading Status RMC: %d\n", RMC.reading_status);

    hal::print(
      console,
      "\n=================== GPS Time Data (ZDA) ===================\n");

    hal::print<128>(console,
                    "Time: %f\nDay: %d\nMonth: %d\nYear: %d\n",
                    ZDA.time,
                    ZDA.day,
                    ZDA.month,
                    ZDA.year);

    hal::print(console,
               "==============================================================="
               "=================\n");

    hal::delay(clock, 1000ms);
  }

  return hal::success();
}