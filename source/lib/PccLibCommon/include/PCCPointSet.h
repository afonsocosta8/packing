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

#ifndef PCCPointSet_h
#define PCCPointSet_h

#include "PCCCommon.h"
#include "PCCMath.h"

namespace pcc {

class PCCPointSet3 {
 public:
  PCCPointSet3() : withColors_( false ), withReflectances_ ( false ) {
  }
  PCCPointSet3(const PCCPointSet3 &) = default;
  PCCPointSet3 &operator=(const PCCPointSet3 &rhs) = default;
  ~PCCPointSet3() = default;

  PCCPoint3D operator[](const size_t index) const {
    assert(index < positions_.size());
    return positions_[index];
  }
  PCCPoint3D &operator[](const size_t index) {
    assert(index < positions_.size());
    return positions_[index];
  }
  void setPosition(const size_t index, const PCCPoint3D position) {
    assert(index < positions_.size());
    positions_[index] = position;
  }
  PCCColor3B getColor(const size_t index) const {
    assert(index < colors_.size() && withColors_);
    return colors_[index];
  }
  PCCColor3B &getColor(const size_t index) {
    assert(index < colors_.size() && withColors_);
    return colors_[index];
  }
  void setColor(const size_t index, const PCCColor3B color) {
    assert(index < colors_.size() && withColors_);
    colors_[index] = color;
  }
  uint16_t getReflectance(const size_t index) const {
    assert(index < reflectances_.size() && withReflectances_);
    return reflectances_[index];
  }
  uint16_t &getReflectance(const size_t index) {
    assert(index < reflectances_.size() && withReflectances_);
    return reflectances_[index];
  }
  void setReflectance(const size_t index, const uint16_t reflectance) {
    assert(index < reflectances_.size() && withReflectances_);
    reflectances_[index] = reflectance;
  }
  std::vector<PCCPoint3D>& getPositions   () { return positions_; }
  std::vector<PCCColor3B>& getColors      () { return colors_; }
  std::vector<uint16_t>&   getReflectances() { return reflectances_; }
  std::vector<uint8_t>&    getTypes       () { return types_; }


  bool hasReflectances() const { return withReflectances_; }
  void addReflectances() {
    withReflectances_ = true;
    resize(getPointCount());
  }
  void removeReflectances() {
    withReflectances_ = false;
    reflectances_.resize(0);
  }

  uint8_t &getType(const size_t index) {
    assert(index < types_.size());
    return types_[index];
  }
  void setType(const size_t index, const uint8_t type) {
    assert(index < types_.size() );
    types_[index] = type;
  }

  bool hasColors() const { return withColors_; }
  void addColors() {
    withColors_ = true;
    resize(getPointCount());
  }
  void removeColors() {
    withColors_ = false;
    colors_.resize(0);
  }

  bool transfertColors( PCCPointSet3 &target, const int32_t searchRange,
                        const bool losslessTexture = false ) const;

  size_t getPointCount() const { return positions_.size(); }
  void resize(const size_t size) {
    positions_.resize(size);
    if (hasColors()) {
      colors_.resize(size);    }
    if (hasReflectances()) {
      reflectances_.resize(size);
    }
    if( PCC_SAVE_POINT_TYPE ) {
      types_.resize( size );
    }
  }
  void reserve(const size_t size) {
    positions_.reserve(size);
    if (hasColors()) {
      colors_.reserve(size);
    }
    if (hasReflectances()) {
      reflectances_.reserve(size);
    }
    if( PCC_SAVE_POINT_TYPE ) {
      types_.reserve( size );
    }
  }
  void clear() {
    positions_.clear();
    colors_.clear();
    reflectances_.clear();
    if( PCC_SAVE_POINT_TYPE ) {
      types_.clear();
    }
  }
  size_t addPoint(const PCCPoint3D &position) {
    const size_t index = getPointCount();
    resize(index + 1);
    positions_[index] = position;
    return index;
  }
  void swapPoints(const size_t index1, const size_t index2) {
    assert(index1 < getPointCount());
    assert(index2 < getPointCount());
    std::swap((*this)[index1], (*this)[index2]);
    if (hasColors()) {
      std::swap(getColor(index1), getColor(index2));
    }
    if (hasReflectances()) {
      std::swap(getReflectance(index1), getReflectance(index2));
    }
    if( PCC_SAVE_POINT_TYPE ) {
      std::swap( getType( index1 ), getType( index2 ) );
    }
  }
  PCCPoint3D computeCentroid() const;
  PCCBox3D computeBoundingBox() const;

  static bool compareSeparators(char aChar, const char *const sep) {
    int i = 0;
    while (sep[i] != '\0') {
      if (aChar == sep[i]) return false;
      i++;
    }
    return true;
  }
  static inline bool getTokens(const char *str, const char *const sep,
                               std::vector<std::string> &tokens) {
    if (!tokens.empty()) tokens.clear();
    std::string buf = "";
    size_t i = 0;
    size_t length = ::strlen(str);
    while (i < length) {
      if (compareSeparators(str[i], sep)) {
        buf += str[i];
      } else if (buf.length() > 0) {
        tokens.push_back(buf);
        buf = "";
      }
      i++;
    }
    if (!buf.empty()) tokens.push_back(buf);
    return !tokens.empty();
  }
  bool write(const std::string &fileName, const bool asAscii = false ) ;
  bool read(const std::string &fileName) ;
  void convertRGBToYUV() ;
  void convertRGBToYUVClosedLoop() ;
  void convertYUVToRGB() ;

 private:
  std::vector<PCCPoint3D> positions_;
  std::vector<PCCColor3B> colors_;
  std::vector<uint16_t>   reflectances_;
  std::vector<uint8_t>    types_;
  bool                    withColors_;
  bool                    withReflectances_;
};
}

#endif /* PCCPointSet_h */
