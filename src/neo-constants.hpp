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

namespace hal::neo {

using namespace std::literals;

constexpr auto gga_start_of_line = "GGA,"sv;
constexpr auto vtg_start_of_line = "VTG,"sv;
constexpr auto gsa_start_of_line = "GSA,"sv;
constexpr auto gsv_start_of_line = "GSV,"sv;
constexpr auto rmc_start_of_line = "RMC,"sv;
constexpr auto zda_start_of_line = "ZDA,"sv;
constexpr auto pashr_start_of_line = "PASHR,"sv;

constexpr auto end_of_line = "\r\n"sv;

const char* GGA_FORMAT = ",%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c,,%s";
// Example:
// $GNGGA,185833.80,4808.7402397,N,01133.9325039,E,5,15,1.1,470.50,M,45.65,M,,*75

const char* VTG_FORMAT = ",%f,%c,%f,%c,%f,%c,%f,%c,%c*";
// Example:
// $GNVTG,112.99,T,109.99,M,0.15,N,0.08,K,A*3B

const char* GSA_FORMAT = ",%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f*";
// Example:
// $GNGSA,2,M,06,12,15,17,19,24,25,32,1.34,0.96,0.93*1D
// $GNGSA,2,M,70,71,79,80,81,82,88,1.34,0.96,0.93*3A

const char* GSV_FORMAT = ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s";
// Example:
// $GPGSV,6,1,10,02,3.6,133.2,26,10,06,11.7,100.7,39,10,10,9.6,281.5,35,10,12,63.1,256.5,46*58
// $GPGSV,6,2,10,15,26.5,186.0,43,10,17,30.5,48.7,42,10,19,43.9,65.3,46,10,24,86.5,103.6,46*5E
// $GPGSV,6,3,10,25,21.6,250.8,43,10,32,21.7,316.0,41,,,,,,,,,,*5E
// $GLGSV,6,4,09,69,7.0,215.9,30,09,70,30.8,267.4,44,09,71,23.0,324.4,46,09,73,13.0,286.8,33*72
// $GLGSV,6,5,09,79,47.8,70.6,43,09,80,54.9,314.5,38,09,81,48.6,86.8,43,09,82,28.4,150.8,46*49
// $GLGSV,6,6,09,88,21.3,28.0,40,,,,,,,,,,,,,,,*4E

const char* RMC_FORMAT = ",%f,%c,%f,%c,%f,%c,%f,%f,%d,%f,%c*";
// Example:
// $GNRMC,185823.40,A,4808.7402374,N,01133.9324760,E,0.00,112.64,130117,3.00,E,A*14

const char* ZDA_FORMAT = ",%f,%d,%d,%d,,*";
// Example:
// $GNZDA,185823.40,13,01,2017,,*7E


const char* PASHR_FORMAT = ",%f,%f,%c,%f,%c,%f,%f,%f,%f,%f,%f,%d*";
// Example:
// $PASHR,190558.56,107.09,T,,-0.16,,,0.067,0.056,2*34



}  // namespace hal::neo