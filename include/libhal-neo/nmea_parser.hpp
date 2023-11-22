

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

#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <span>

namespace hal::neo {

/**
 * @brief Interface for nmea sentences (get better docs later)
 *
 * This interface creates a common interface for nmea sentences to parse
 * data and report if they have completed their sentence parsing.
 *
 */
class nmea_parser
{
public:
  enum class state_t
  {
    // the start of the sentence has not been found yet, or has recently
    // completed a sentence.
    inactive,
    // The start of a sentence has been found and the sentence is partially
    // parsed.
    active,
  };

  virtual ~nmea_parser() = default;

  //   The used sentence identifiers are:
  // * GGA – Global Positioning System Fix Data
  // * VTG – Course over Ground and Ground Speed
  // * GSA – GNSS DOP and Active Satellites
  // * GSV – GNSS Satellites in View
  // * RMC – Recommended Minimum Specific GNSS Data
  // * ZDA – Time and Date
  // * PASHR – Attitude Data

  /**
   * @brief Get the sentence header for this parser.
   *
   * @return std::string - The sentence header unique to this parser.
   */
  virtual std::string_view sentence_header() const = 0;

  /**
   * @brief Parse byte data and find
   *
   * @param p_data - data containing GPS serial data to be parsed by this
   * sentence.
   * @return std::span<hal::byte> - returns the remaining bytes not consumed
   * by this nmea sentence
   */
  virtual void parse(std::string_view p_data) = 0;
};

// void parse(std::span<nmea_parser*> p_parsers, std::span<hal::byte> p_data);
// void parse(std::span<nmea_parser*> p_parsers, hal::serial p_serial);

}  // namespace hal::neo