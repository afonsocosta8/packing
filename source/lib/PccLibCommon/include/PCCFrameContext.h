
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
#ifndef PCCFrameContext_h
#define PCCFrameContext_h

#include "PCCCommon.h"

#include "PCCPointSet.h"
#include "PCCPatch.h"

namespace pcc {

class PCCFrameContext {
 public:
  PCCFrameContext();
  ~PCCFrameContext();

  size_t&                          getIndex             () { return index_;             }
  size_t&                          getWidth             () { return width_;             }
  size_t&                          getHeight            () { return height_;            }
  size_t&                          getNumMatchedPatches () { return numMatchedPatches_; }
  size_t&                          setNumMatchedPatches () { return numMatchedPatches_; }
  std::vector<PCCVector3<size_t>>& getPointToPixel      () { return pointToPixel_;      }
  std::vector<size_t>&             getBlockToPatch      () { return blockToPatch_;      }
  std::vector<uint32_t>&           getOccupancyMap      () { return occupancyMap_;      }
  std::vector<PCCPatch>&           getPatches           () { return patches_;           }
  PCCMissedPointsPatch&            getMissedPointsPatch () { return missedPointsPatch_; }

 private:
  size_t index_;
  size_t width_;
  size_t height_;
  size_t numMatchedPatches_;
  std::vector<PCCVector3<size_t>> pointToPixel_;
  std::vector<size_t> blockToPatch_;
  std::vector<uint32_t> occupancyMap_;
  std::vector<PCCPatch> patches_;
  PCCMissedPointsPatch missedPointsPatch_;
};

}; //~namespace

#endif /* PCCFrameContext_h */
