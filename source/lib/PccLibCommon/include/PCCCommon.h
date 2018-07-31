/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef PCCTMC2Common_h
#define PCCTMC2Common_h

#include <limits>
#include <assert.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstring>
#include <unordered_map>
#include <sys/stat.h>
#include <memory>
#include <queue>
#include <algorithm>

#include "PCCConfig.h"

namespace pcc {

  // ******************************************************************* //
  // Version information
  // ******************************************************************* //
  #define TM2_VERSION         TMC2_VERSION_MAJOR "." TMC2_VERSION_MINOR

  // ******************************************************************* //
  // Coding tool configuration
  // ******************************************************************* //
  static const uint8_t PCC_SAVE_POINT_TYPE = 0; // Save point information in reconstructed ply.

  // ******************************************************************* //
  // Common constants
  // ******************************************************************* //
  const uint32_t PCCTMC2ContainerMagicNumber  = 23021981;
  const uint32_t PCCTMC2ContainerVersion      = 1;
  const uint32_t PCC_UNDEFINED_INDEX          = -1;

  enum PCCEndianness  { PCC_BIG_ENDIAN = 0, PCC_LITTLE_ENDIAN = 1 };
  enum ColorTransform { COLOR_TRANSFORM_NONE = 0, COLOR_TRANSFORM_RGB_TO_YCBCR = 1 };
  enum PCCAxis3       { PCC_AXIS3_UNDEFINED = -1, PCC_AXIS3_X = 0, PCC_AXIS3_Y = 1, PCC_AXIS3_Z = 2 };
  enum PointType      { Unset = 0, D0, D1, Smooth };

  // ******************************************************************* //
  // Static functions
  // ******************************************************************* //
  static bool exist(const std::string& sString )
  {
    struct stat buffer;
    return (stat( sString.c_str(), &buffer) == 0);
  }

  static void removeFile( const std::string string ) {
    if( exist( string ) ) {
      if( remove( string.c_str() ) != 0 ) {
        std::cout << "Could not remove the file: " << string << std::endl;
      }
    }
  }

  static std::string removeFileExtension( const std::string string ) {
    size_t pos = string.find_last_of(".");
    return ( pos != std::string::npos && pos + 4 == string.length() ) ?
        string.substr( 0, pos ) : string;
  }

  static std::string addVideoFormat( const std::string string,
                                     const size_t width, const size_t height,
                                     const bool yuv420 = true,
                                     const std::string pixel = "8" ) {
    size_t pos = string.find_last_of(".");
    std::string filename  = string.substr( 0, pos ), extension = string.substr( pos );
    if( extension == ".yuv" || extension == ".rgb" ) {
      std::stringstream result;
      result << filename << "_" <<  width << "x" << height << "_" << pixel
             << "bit_" << ( extension == ".yuv" && yuv420 ? "p420" : "p444" ) << extension;
      return result.str();
    }
    return string;
  }

  static inline PCCEndianness PCCSystemEndianness() {
    uint32_t num = 1;
    return (*(reinterpret_cast<char *>(&num)) == 1) ? PCC_LITTLE_ENDIAN : PCC_BIG_ENDIAN;
  }

  template <typename T>
  static const T PCCEndianSwap(const T u) {
    union {
      T u;
      uint8_t u8[sizeof(T)];
    } source, dest;
    source.u = u;
    for (size_t k = 0; k < sizeof(T); k++) dest.u8[k] = source.u8[sizeof(T) - k - 1];
    return dest.u;
  }
  template <typename T>
  static const T PCCToLittleEndian(const T u) {
    return (PCCSystemEndianness() == PCC_BIG_ENDIAN) ? PCCEndianSwap(u) : u;
  }

  template <typename T>
  static const T PCCFromLittleEndian(const T u) {
    return (PCCSystemEndianness() == PCC_BIG_ENDIAN) ? PCCEndianSwap(u) : u;
  }

  static void PCCDivideRange(const size_t start, const size_t end,
                             const size_t chunckCount, std::vector<size_t> &subRanges) {
    const size_t elementCount = end - start;
    if (elementCount <= chunckCount) {
      subRanges.resize(elementCount + 1);
      for (size_t i = start; i <= end; ++i) {
        subRanges[i - start] = i;
      }
    } else {
      subRanges.resize(chunckCount + 1);
      const double step = static_cast<double>(elementCount) / (chunckCount + 1);
      double pos = static_cast<double>(start);
      for (size_t i = 0; i < chunckCount; ++i) {
        subRanges[i] = static_cast<size_t>(pos);
        pos += step;
      }
      subRanges[chunckCount] = end;
    }
  }

  static uint32_t PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t range) {
    int count = 0;
    if ( range > 0) {
      range -= 1;
      while ( range > 0) {
        count++;
        range >>= 1;
      }
    }
    return count;
  }
  
}

#endif /* PCCTMC2Common_h */
