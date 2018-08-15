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
#include "PCCCommon.h"
#include "ArithmeticCodec.h"
#include "PCCBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCPatchSegmenter.h"
#include "PCCVideoEncoder.h"
#include "PCCSystem.h"
#include "PCCGroupOfFrames.h"
#include "PCCPointSet.h"
#include "PCCEncoderParameters.h"
#include "PCCKdTree.h"
#include <tbb/tbb.h>

#include "PCCEncoder.h"

using namespace std;
using namespace pcc;

class Buddies {
 public:
  int &getPosU() { return pos_u; }
  int &getPosV() { return pos_v; }
  double &getScore() { return score; }
  PCCPatch &getFather() { return father; }
  PCCPatch &getPatch() { return patch; }

  int getPosU() const { return pos_u; }
  int getPosV() const { return pos_v; }
  double getScore() const { return score; }
  PCCPatch getFather() const { return father; }
  PCCPatch getPatch() const { return patch; }

  friend bool operator<(const Buddies &lhs, const Buddies &rhs) { return lhs.score < rhs.score; }

 private:
  int pos_u;
  int pos_v;
  double score;
  PCCPatch father;
  PCCPatch patch;
};

void EncodeUInt32(const uint32_t value, const uint32_t bitCount,
                  o3dgc::Arithmetic_Codec &arithmeticEncoder, o3dgc::Static_Bit_Model &bModel0) {
  uint32_t valueToEncode = PCCToLittleEndian<uint32_t>(value);
  for (uint32_t i = 0; i < bitCount; ++i) {
    arithmeticEncoder.encode(valueToEncode & 1, bModel0);
    valueToEncode >>= 1;
  }
}

bool orderByFather(const std::vector<Buddies> lhs, const std::vector<Buddies> rhs) {
  return lhs[0].getFather().getIndex() < rhs[0].getFather().getIndex();
}

// Empty Space
bool wasteSpace(const PCCPatch &lhs, const PCCPatch &rhs) {
  return (lhs.getSizeV0() * lhs.getSizeU0() - lhs.getPixelCount()) !=
                 (rhs.getSizeV0() * rhs.getSizeU0() - rhs.getPixelCount())
             ? (lhs.getSizeV0() * lhs.getSizeU0() - lhs.getPixelCount()) >
                   (rhs.getSizeV0() * rhs.getSizeU0() - rhs.getPixelCount())
             : lhs.getIndex() < rhs.getIndex();
}

// Empty Space
bool byIndex(const PCCPatch &lhs, const PCCPatch &rhs) { return lhs.getIndex() < rhs.getIndex(); }

// Effective Area one - Block level
bool effectiveArea(const PCCPatch &lhs, const PCCPatch &rhs) {
  return lhs.getPixelCount() != rhs.getPixelCount() ? lhs.getPixelCount() > rhs.getPixelCount()
                                                    : lhs.getIndex() < rhs.getIndex();
}

// Area one
bool bBArea(const PCCPatch &lhs, const PCCPatch &rhs) {
  return (lhs.getSizeV() * lhs.getSizeU()) != (rhs.getSizeV() * rhs.getSizeU())
             ? (lhs.getSizeV() * lhs.getSizeU()) > (rhs.getSizeV() * rhs.getSizeU())
             : lhs.getIndex() < rhs.getIndex();
}

PCCEncoder::PCCEncoder() {}
PCCEncoder::~PCCEncoder() {}

void PCCEncoder::setParameters(PCCEncoderParameters params) { params_ = params; }

int PCCEncoder::encode(const PCCGroupOfFrames &sources, PCCContext &context,
                       PCCBitstream &bitstream, PCCGroupOfFrames &reconstructs) {
  assert(sources.size() < 256);
  if (sources.size() == 0) {
    return 0;
  }

  if (params_.nbThread_ > 0) {
    tbb::task_scheduler_init init((int)params_.nbThread_);
  }
  reconstructs.resize(sources.size());
  context.resize(sources.size());
  auto &frames = context.getFrames();
  PCCVideoEncoder videoEncoder;
  const size_t pointCount = sources[0].getPointCount();
  std::stringstream path;
  path << removeFileExtension(params_.compressedStreamPath_) << "_GOF" << context.getIndex() << "_";

  generateGeometryVideo(sources, context);
  resizeGeometryVideo(context);
  dilateGeometryVideo(context);
  auto &width = context.getWidth();
  auto &height = context.getHeight();
  width = (uint16_t)frames[0].getWidth();
  height = (uint16_t)frames[0].getHeight();
  compressHeader(context, bitstream);

  const size_t nbyteGeo = params_.losslessGeo_ ? 2 : 1;
  if (!params_.absoluteD1_) {
    // Compress geometryD0
    auto sizeGeometryD0Video = bitstream.size();
    auto &videoGeometry = context.getVideoGeometry();
    // For sequential start comment
    videoEncoder.compress(
        videoGeometry, videoGeometry, 0, path.str() + "geometryD0", (params_.geometryQP_ - 1),
        bitstream, params_.geometryD0Config_, params_.videoEncoderPath_, "", "", "",
        params_.losslessGeo_ && params_.losslessGeo444_, nbyteGeo, params_.keepIntermediateFiles_);
    sizeGeometryD0Video = bitstream.size() - sizeGeometryD0Video;

    std::cout << "geometry D0 video ->" << sizeGeometryD0Video << " B ("
              << (sizeGeometryD0Video * 8.0) / (frames.size() * pointCount) << " bpp)" << std::endl;

    // For sequential end comment
    // Form differential video geometryD1
    auto &videoGeometryD1 = context.getVideoGeometryD1();
    for (size_t f = 0; f < frames.size(); ++f) {
      auto &frame1 = videoGeometryD1.getFrame(f);
      predictGeometryFrame(frames[f], videoGeometry.getFrame(f), frame1);
    }

    // Compress geometryD1
    auto sizeGeometryD1Video = bitstream.size();
    // For sequential start comment
    videoEncoder.compress(
        videoGeometryD1, videoGeometry, 0, path.str() + "geometryD1", params_.geometryQP_,
        bitstream, params_.geometryD1Config_, params_.videoEncoderPath_, "", "", "",
        params_.losslessGeo_ && params_.losslessGeo444_, nbyteGeo, params_.keepIntermediateFiles_);

    sizeGeometryD1Video = bitstream.size() - sizeGeometryD1Video;

    std::cout << "geometry D1 video ->" << sizeGeometryD1Video << " B ("
              << (sizeGeometryD1Video * 8.0) / (frames.size() * pointCount) << " bpp)" << std::endl;
    // For sequential end comment
    auto sizeGeometryVideo = sizeGeometryD0Video + sizeGeometryD1Video;
    std::cout << "geometry video ->" << sizeGeometryVideo << " B ("
              << (sizeGeometryVideo * 8.0) / (2 * frames.size() * pointCount) << " bpp)"
              << std::endl;
  } else {
    // For sequential start comment
    auto sizeGeometryVideo = bitstream.size();
    auto &videoGeometry = context.getVideoGeometry();
    videoEncoder.compress(
        videoGeometry, videoGeometry, 0, path.str() + "geometry", params_.geometryQP_, bitstream,
        params_.geometryConfig_, params_.videoEncoderPath_, "", "", "",
        params_.losslessGeo_ && params_.losslessGeo444_, nbyteGeo, params_.keepIntermediateFiles_);
    sizeGeometryVideo = bitstream.size() - sizeGeometryVideo;
    std::cout << "geometry video ->" << sizeGeometryVideo << " B ("
              << (sizeGeometryVideo * 8.0) / (2 * frames.size() * pointCount) << " bpp)"
              << std::endl;
    // For sequential end comment
  }

  auto sizeOccupancyMap = bitstream.size();
  compressOccupancyMap(context, bitstream);
  sizeOccupancyMap = bitstream.size() - sizeOccupancyMap;
  std::cout << " occupancy map  ->" << sizeOccupancyMap << " B ("
            << (sizeOccupancyMap * 8.0) / (2 * frames.size() * pointCount) << " bpp)" << std::endl;

  GeneratePointCloudParameters generatePointCloudParameters = {
      params_.occupancyResolution_, params_.neighborCountSmoothing_,
      params_.radius2Smoothing_,    params_.radius2BoundaryDetection_,
      params_.thresholdSmoothing_,  params_.losslessGeo_,
      params_.losslessGeo444_,      params_.nbThread_,
      params_.absoluteD1_};
  std::cout << "Estou Aqui1" << std::endl;
  std::cout << "Estou Aqui1" << std::endl;
  generatePointCloud(reconstructs, context, generatePointCloudParameters);
  std::cout << "Estou Aqui2" << std::endl;
  std::cout << "Estou Aqui2" << std::endl;

  if (!params_.noAttributes_) {
    // reconstructs = sources;
    generateTextureVideo(sources, reconstructs, context);

    auto &videoTexture = context.getVideoTexture();
    pcc::PCCVideo<uint8_t, 3> svideoTexture = videoTexture;
    const size_t textureFrameCount = 2 * frames.size();
    assert(textureFrameCount == videoTexture.getFrameCount());
    for (size_t f = 0; f < textureFrameCount; ++f) {
      dilate(frames[f / 2], videoTexture.getFrame(f), svideoTexture.getFrame(f), 1);
    }

    auto sizeTextureVideo = bitstream.size();
    const size_t nbyteTexture = 1;
    videoEncoder.compress(
        videoTexture, svideoTexture, 1, path.str() + "texture", params_.textureQP_, bitstream,
        params_.textureConfig_, params_.videoEncoderPath_, params_.colorSpaceConversionConfig_,
        params_.inverseColorSpaceConversionConfig_, params_.colorSpaceConversionPath_,
        params_.losslessTexture_, nbyteTexture, params_.keepIntermediateFiles_);
    sizeTextureVideo = bitstream.size() - sizeTextureVideo;
    std::cout << "texture video  ->" << sizeTextureVideo << " B ("
              << (sizeTextureVideo * 8.0) / pointCount << " bpp)" << std::endl;
  }
  std::cout << "Im Here\n";
  colorPointCloud(reconstructs, context, params_.noAttributes_ != 0, params_.colorTransform_);
  return 0;
}

void PCCEncoder::printMap(std::vector<bool> img, const size_t sizeU, const size_t sizeV) {
  std::cout << std::endl;
  std::cout << "PrintMap size = " << sizeU << " x " << sizeV << std::endl;
  for (size_t v = 0; v < sizeV; ++v) {
    for (size_t u = 0; u < sizeU; ++u) {
      std::cout << (img[v * sizeU + u] ? 'X' : '.');
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void PCCEncoder::spatialConsistencyPack(PCCFrameContext &frame, PCCFrameContext &prevFrame) {
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  auto &patches = frame.getPatches();

  auto &prevPatches = prevFrame.getPatches();
  if (patches.empty()) {
    return;
  }
  std::sort(patches.begin(), patches.end());
  int id = 0;
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)(params_.minimumImageHeight_ / params_.occupancyResolution_,
                                     patches[0].getSizeV0());
  vector<PCCPatch> matchedPatches, tmpPatches;
  matchedPatches.clear();

  float thresholdIOU = 0.2;

  // main loop.
  for (auto &patch : prevPatches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    id++;
    float maxIou = 0.0;
    int bestIdx = -1, cId = 0;
    for (auto &cpatch : patches) {
      if ((patch.getViewId() == cpatch.getViewId()) && (cpatch.getBestMatchIdx() == -1)) {
        Rect rect = Rect(patch.getU1(), patch.getV1(), patch.getSizeU(), patch.getSizeV());
        Rect crect = Rect(cpatch.getU1(), cpatch.getV1(), cpatch.getSizeU(), cpatch.getSizeV());
        float iou = computeIOU(rect, crect);
        if (iou > maxIou) {
          maxIou = iou;
          bestIdx = cId;
        }
      }  // end of if (patch.viewId == cpatch.viewId).
      cId++;
    }

    if (maxIou > thresholdIOU) {
      // store the best match index
      // printf("Best match patchId:[%d:%d], viewId:%d, iou:%9.6f\n", id, bestIdx, patch.viewId,
      // maxIou);
      patches[bestIdx].setBestMatchIdx() = id - 1;  // the matched patch id in preivious frame.
      matchedPatches.push_back(patches[bestIdx]);
    }
  }

  // generate new patch order.
  vector<PCCPatch> newOrderPatches = matchedPatches;

  for (auto patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    if (patch.getBestMatchIdx() == -1) {
      newOrderPatches.push_back(patch);
    }
  }

  frame.setNumMatchedPatches() = matchedPatches.size();
  // remove the below logs when useless.
  printf("patches.size:%d,reOrderedPatches.size:%d,matchedpatches.size:%d\n", (int)patches.size(),
         (int)newOrderPatches.size(), (int)frame.getNumMatchedPatches());
  patches = newOrderPatches;

  for (auto &patch : patches) {
    occupancySizeU = (std::max)(occupancySizeU, patch.getSizeU0() + 1);
  }

  width = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  for (auto &patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto &occupancy = patch.getOccupancy();
    while (!locationFound) {
      // print(patch.occupancy, patch.getSizeU0(), patch.getSizeV0());
      // print(occupancyMap, occupancySizeU, occupancySizeV);
      for (size_t v = 0; v <= occupancySizeV - patch.getSizeV0() && !locationFound; ++v) {
        for (size_t u = 0; u <= occupancySizeU - patch.getSizeU0(); ++u) {
          bool canFit = true;
          for (size_t v0 = 0; v0 < patch.getSizeV0() && canFit; ++v0) {
            const size_t y = v + v0;
            for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
              const size_t x = u + u0;

              if (occupancy[v0 * patch.getSizeU0() + u0] && occupancyMap[y * occupancySizeU + x]) {
                canFit = false;
                break;
              }
            }
          }
          if (!canFit) {
            continue;
          }
          locationFound = true;
          patch.getU0() = u;
          patch.getV0() = v;
          break;
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize(occupancySizeU * occupancySizeV);
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      const size_t v = patch.getV0() + v0;
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height =
        (std::max)(height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution());
    width = (std::max)(width, (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution());
    maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }

  if (frame.getMissedPointsPatch().size() > 0) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV,
                          maxOccupancyRow);
  } else {
    printMap(occupancyMap, occupancySizeU, occupancySizeV);
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

/*void PCCEncoder::newsort(std::vector<PCCPatch>& patches){
  std::vector<PCCPatch>& newpatches;

}*/

void PCCEncoder::mergePatches(PCCPatch &patch1, PCCPatch &patch2, PCCPatch &superpatch) {
  auto &occupancy = superpatch.getOccupancy();
  auto &occupancy1 = patch1.getOccupancy();
  auto &occupancy2 = patch2.getOccupancy();
  occupancy.resize((patch1.getSizeU0() + patch2.getSizeU0()) *
                   (patch1.getSizeV0() + patch2.getSizeV0()));
  size_t occupancySizeU = patch1.getSizeU0() + patch2.getSizeU0();
  size_t occupancySizeV = patch1.getSizeV0() + patch2.getSizeV0();
  bool locationFound = false;
  size_t l1;
  size_t r1;
  while (!locationFound) {
    // print(patch.occupancy, patch.getSizeU0(), patch.getSizeV0());
    // print(occupancyMap, occupancySizeU, occupancySizeV);
    for (size_t v = 0; v <= occupancySizeV - patch1.getSizeV0() && !locationFound; ++v) {
      for (size_t u = 0; u <= occupancySizeU - patch1.getSizeU0(); ++u) {
        bool canFit = true;
        for (size_t v0 = 0; v0 < patch1.getSizeV0() && canFit; ++v0) {
          const size_t y = v + v0;
          for (size_t u0 = 0; u0 < patch1.getSizeU0(); ++u0) {
            const size_t x = u + u0;
            if (occupancy1[v0 * patch1.getSizeU0() + u0] && occupancy[y * occupancySizeU + x]) {
              canFit = false;
              break;
            }
          }
        }
        if (!canFit) {
          continue;
        }
        locationFound = true;
        l1 = u;
        r1 = v;
        break;
      }
    }
  }
  for (size_t v0 = 0; v0 < patch1.getSizeV0(); ++v0) {
    const size_t v = r1 + v0;
    for (size_t u0 = 0; u0 < patch1.getSizeU0(); ++u0) {
      const size_t u = l1 + u0;
      occupancy[v * occupancySizeU + u] =
          occupancy[v * occupancySizeU + u] || occupancy1[v0 * patch1.getSizeU0() + u0];
    }
  }
  locationFound = false;
  while (!locationFound) {
    // print(patch.occupancy, patch.getSizeU0(), patch.getSizeV0());
    // print(occupancyMap, occupancySizeU, occupancySizeV);
    for (size_t v = 0; v <= occupancySizeV - patch2.getSizeV0() && !locationFound; ++v) {
      for (size_t u = 0; u <= occupancySizeU - patch2.getSizeU0(); ++u) {
        bool canFit = true;
        for (size_t v0 = 0; v0 < patch2.getSizeV0() && canFit; ++v0) {
          const size_t y = v + v0;
          for (size_t u0 = 0; u0 < patch2.getSizeU0(); ++u0) {
            const size_t x = u + u0;
            if (occupancy2[v0 * patch2.getSizeU0() + u0] && occupancy[y * occupancySizeU + x]) {
              canFit = false;
              break;
            }
          }
        }
        if (!canFit) {
          continue;
        }
        locationFound = true;
        l1 = u;
        r1 = v;
        break;
      }
    }
  }
  for (size_t v0 = 0; v0 < patch2.getSizeV0(); ++v0) {
    const size_t v = r1 + v0;
    for (size_t u0 = 0; u0 < patch2.getSizeU0(); ++u0) {
      const size_t u = l1 + u0;
      occupancy[v * occupancySizeU + u] =
          occupancy[v * occupancySizeU + u] || occupancy2[v0 * patch2.getSizeU0() + u0];
    }
  }
  size_t maxHeight = 0;
  size_t maxWidth = 0;
  for (size_t u0 = 0; u0 < occupancySizeU; ++u0) {
    for (size_t v0 = 0; v0 < occupancySizeV; ++v0) {
      if (occupancy[v0 * occupancySizeU + u0] == 1 && v0 > maxHeight) {
        maxHeight = v0;
      }
      if (occupancy[v0 * occupancySizeU + u0] == 1 && u0 > maxWidth) {
        maxWidth = u0;
      }
    }
  }
  size_t e = 0;
  size_t i = 0;
  for (size_t u0 = occupancySizeU - i; u0 > maxWidth; u0 = u0 - 1) {
    for (size_t v0 = 0; v0 < occupancySizeV; ++v0) {
      occupancy.erase(occupancy.begin() + v0 * u0 + u0 - e - 1);
      e++;
    }
    e = 0;
    i++;
  }
  std::cout << "Superpatch\n"
            << "Superpatch\n";
  printMap(occupancy, maxWidth, maxHeight);
  std::cout << "Superpatch\n"
            << "Superpatch\n";
}

void PCCEncoder::countPixels(PCCPatch &patch) {
  size_t width = patch.getSizeU0();
  size_t height = patch.getSizeV0();
  size_t count = 0;
  auto &occupancy = patch.getOccupancy();
  for (size_t u0 = 0; u0 < width; ++u0) {
    for (size_t v0 = 0; v0 < height; ++v0) {
      if (occupancy[v0 * width + u0] == 1) {
        count++;
      }
    }
  }
  patch.getPixelCount() = count;
}

/*void PCCEncoder::bestBuddiesCompute(PCCPatch &patch, std::vector<PCCPatch> &patches) {

}*/

double PCCEncoder::computeSM(PCCPatch &father_patch, PCCPatch &bb_patch,
                             std::vector<bool> occupancy_, size_t pos_u, size_t pos_v,
                             PCCPatch &second_patch) {
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  double sum = 0.0;
  // search right
  // std::cout << "Specific " << father_patch.getDepth(0)[]
  // std::cout << "Patch Index " << father_patch.getIndex() << std::endl;
  int num_pixels = 0;
  for (size_t v = 0; v < bb_patch.getSizeV()-1; ++v) {
    size_t uright = infiniteDepth;
    size_t uleft = infiniteDepth;
    if ((int(pos_v * params_.occupancyResolution_) + int(v) -
         int((bb_patch.getSizeV0() * params_.occupancyResolution_))) >= 0 &&
        (((pos_v * params_.occupancyResolution_) + v) <
         (((bb_patch.getSizeV0()) * params_.occupancyResolution_) + father_patch.getSizeV()))) {
      // std::cout << " " << v << std::endl;
      for (size_t u = bb_patch.getSizeU()-1; u > 0; u--) {
        // std::cout << u << " " << v << std::endl;
        if (bb_patch.getDepth(0)[(v * bb_patch.getSizeU()) + u] != infiniteDepth) {
          uright = u;
          break;
        }
        // std::cout << "Estou Aqui1" << std::endl;
      }
      // std::cout << "Encontrei o mais à direita do bb " << uright << std::endl;
      for (size_t u = 0; u < father_patch.getSizeU()-1; u++) {
        if (father_patch.getDepth(0)[(((pos_v * params_.occupancyResolution_) + v -
                                       (bb_patch.getSizeV0() * params_.occupancyResolution_)) *
                                      father_patch.getSizeU()) +
                                     u] != infiniteDepth) {
          // std::cout << "LL " << (((pos_v*params_.occupancyResolution_) + v -
          //(bb_patch.getSizeV0()*params_.occupancyResolution_)) * bb_patch.getSizeU()) + u <<
          // std::endl;
          uleft = u;
          break;
        }
      }
      // std::cout << "Encontrei o mais à esquerda do father " << uleft << std::endl;
      // std::cout << "Parei no v = " << v << std::endl;
    }
    // std::cout << uleft << " " << uright << std::endl;
    if ((uleft != infiniteDepth) && (uright != infiniteDepth) &&
        (int(uleft) > (int(uright) - ((int(bb_patch.getSizeU0()) - int(pos_u)) *
                                      int(params_.occupancyResolution_))))) {
      // std::cout << "Estou Aqui" << std::endl;
      size_t pson = (v * bb_patch.getSizeU()) + uright;
      size_t pfather = (((pos_v * params_.occupancyResolution_) + v -
                         (bb_patch.getSizeV0() * params_.occupancyResolution_)) *
                        father_patch.getSizeU()) +
                       uleft;
      // std::cout << bb_patch.getColor(0).size() << " " << bb_patch.getSizeU() << " " <<
      // bb_patch.getSizeV() << std::endl;
      int sum1 =
          pow(int((father_patch.getColor(0)[pfather]) - int(bb_patch.getColor(0)[pson])), 2) +
          pow(int((father_patch.getColor(1)[pfather]) - int(bb_patch.getColor(1)[pson])), 2) +
          pow(int((father_patch.getColor(2)[pfather]) - int(bb_patch.getColor(2)[pson])), 2);
      double sum2 = sqrt((double(sum1) / (3.0 * 255.0 * 255.0)));

      // Comparison

      // std::cout << "For Comparison" << std::endl;

      // std::cout << "YFather " << int(father_patch.getColor(0)[pfather]) << " Yson " <<
      // int(bb_patch.getColor(0)[pson]) << std::endl;  std::cout << "UFather " <<
      // int(father_patch.getColor(1)[pfather]) << " Uson " << int(bb_patch.getColor(1)[pson]) <<
      // std::endl;  std::cout << "VFather " << int(father_patch.getColor(2)[pfather]) << " Vson "
      // << int(bb_patch.getColor(2)[pson]) << std::endl;

      // std::cout << "For Comparison" << std::endl;

      // std::cout << std::endl;

      // Comparison
      num_pixels++;
      // std::cout << sum1 << std::endl;
      sum = sum + sum2;
    }
  }
  if (pos_u == 11 && pos_v == 14) {
    //std::cout << "BoW " << num_pixels << " " << sum << std::endl;
  }
  // std::cout << num_pixels << std::endl;
  // search left
  // std::cout << "I die Hereeeee2" << std::endl;
  if (pos_v == 66) {
    // std::cout << "I die Hereeeee7" << std::endl;
    // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
  }
  for (size_t v = 0; v < bb_patch.getSizeV() - 1; ++v) {
    if (pos_v == 66) {
      // std::cout << "I die Hereeeee88" << std::endl;
      // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
    }
    // std::cout << v << std::endl;
    size_t uright = infiniteDepth;
    size_t uleft = infiniteDepth;
    // std::cout << "I die Hereeeee5" << std::endl;
    if ((int(pos_v * params_.occupancyResolution_) + int(v) -
         int((bb_patch.getSizeV0() * params_.occupancyResolution_))) >= 0 &&
        (((pos_v * params_.occupancyResolution_) + v) <
         (((bb_patch.getSizeV0()) * params_.occupancyResolution_) + father_patch.getSizeV()))) {
      for (size_t u = 0; u < bb_patch.getSizeU() - 1; u++) {
        if (bb_patch.getDepth(0)[(v * bb_patch.getSizeU()) + u] != infiniteDepth) {
          uleft = u;
          break;
        }
      }
      for (size_t u = father_patch.getSizeU() - 1; u > 0; u--) {
        if (father_patch.getDepth(0)[(((pos_v * params_.occupancyResolution_) + v -
                                       (bb_patch.getSizeV0() * params_.occupancyResolution_)) *
                                      father_patch.getSizeU()) +
                                     u] != infiniteDepth) {
          uright = u;
          break;
        }
      }
    }
    if (pos_v == 66) {
      // std::cout << "I die Hereeeee99" << std::endl;
      // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
    }
    // std::cout << "I die Hereeeee6" << std::endl;
    if ((uleft != infiniteDepth) && (uright != infiniteDepth) &&
        (int(uleft) > (int(uright) + ((int(bb_patch.getSizeU0()) - int(pos_u)) *
                                      int(params_.occupancyResolution_))))) {
      size_t pson = (v * bb_patch.getSizeU()) + uleft;
      size_t pfather = (((pos_v * params_.occupancyResolution_) + v -
                         (bb_patch.getSizeV0() * params_.occupancyResolution_)) *
                        father_patch.getSizeU()) +
                       uright;
      if (pos_v == 66) {
        // std::cout << pfather << " " << father_patch.getColor(0).size() << " " <<
        // father_patch.getDepth(0).size() << std::endl;
        // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
      }
      // std::cout << pfather << " " << father_patch.getColor(0).size() << " " <<
      // father_patch.getDepth(0).size() << std::endl;
      int sum1 =
          pow(int((father_patch.getColor(0)[pfather]) - int(bb_patch.getColor(0)[pson])), 2) +
          pow(int((father_patch.getColor(1)[pfather]) - int(bb_patch.getColor(1)[pson])), 2) +
          pow(int((father_patch.getColor(2)[pfather]) - int(bb_patch.getColor(2)[pson])), 2);
      double sum2 = sqrt((double(sum1) / (3.0 * 255.0 * 255.0)));
      if (pos_v == 66) {
        // std::cout << "I die Hereeeee222" << std::endl;
        // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
      }
      sum = sum + sum2;
      num_pixels++;
    }
  }
  if (pos_v == 66) {
    // std::cout << "I die Hereeeee8" << std::endl;
    // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
  }
  if (pos_u == 11 && pos_v == 14) {
    //std::cout << "BoW " << num_pixels << " " << sum << std::endl;
  }
  // std::cout << num_pixels << std::endl;
  // search up
  // std::cout << "I die Hereeeee3" << std::endl;
  for (size_t u = 0; u < bb_patch.getSizeU() - 1; ++u) {
    // std::cout << "I die Hereeeee" << std::endl;
    size_t vup = infiniteDepth;
    size_t vdown = infiniteDepth;
    if ((int(pos_u * params_.occupancyResolution_) + int(u) -
         int((bb_patch.getSizeU0() * params_.occupancyResolution_))) >= 0 &&
        (((pos_u * params_.occupancyResolution_) + u) <
         (((bb_patch.getSizeU0()) * params_.occupancyResolution_) + father_patch.getSizeU()))) {
      // std::cout << "I die Here" << std::endl;
      for (size_t v = 0; v < bb_patch.getSizeV() - 1; v++) {
        if (bb_patch.getDepth(0)[(v * bb_patch.getSizeU()) + u] != infiniteDepth) {
          vup = v;
          break;
        }
      }
      for (size_t v = father_patch.getSizeV() - 1; v > 0; v--) {
        if (father_patch.getDepth(0)[(v * father_patch.getSizeU()) +
                                     ((pos_u * params_.occupancyResolution_) + u -
                                      (bb_patch.getSizeU0() * params_.occupancyResolution_))] !=
            infiniteDepth) {
          vdown = v;
          break;
        }
      }
    }
    if ((vup != infiniteDepth) && (vdown != infiniteDepth) &&
        (int(vup) > (int(vdown) + ((int(bb_patch.getSizeV0()) - int(pos_v)) *
                                   int(params_.occupancyResolution_))))) {
      // std::cout << "I die Here7" << std::endl;
      // std::cout << "Breaks at " << pos_v << " " << vdown << " " << father_patch.getSizeV() << " "
      // <<  u << std::endl;
      size_t pson = (vup * bb_patch.getSizeU()) + u;
      size_t pfather = (vdown * father_patch.getSizeU()) +
                       ((pos_u * params_.occupancyResolution_) + u -
                        (bb_patch.getSizeU0() * params_.occupancyResolution_));
      int sum1 =
          pow(int((father_patch.getColor(0)[pfather]) - int(bb_patch.getColor(0)[pson])), 2) +
          pow(int((father_patch.getColor(1)[pfather]) - int(bb_patch.getColor(1)[pson])), 2) +
          pow(int((father_patch.getColor(2)[pfather]) - int(bb_patch.getColor(2)[pson])), 2);
      double sum2 = sqrt((double(sum1) / (3.0 * 255.0 * 255.0)));
      sum = sum + sum2;
      num_pixels++;
    }
  }
  if (pos_u == 11 && pos_v == 14) {
    //std::cout << "BoW " << num_pixels << " " << sum << std::endl;
  }
  if (pos_v == 66) {
    // std::cout << "I die Hereeeee9" << std::endl;
    // std::cout << v << " " << bb_patch.getSizeV() << std::endl;
  }
  // std::cout << num_pixels << std::endl;

  // search down
  // std::cout << "I die Hereeeee4" << std::endl;
  for (size_t u = 0; u < bb_patch.getSizeU() - 1; ++u) {
    //std::cout << "I die Hereeeee4" << std::endl;
    size_t vup = infiniteDepth;
    size_t vdown = infiniteDepth;
    if ((int(pos_u * params_.occupancyResolution_) + int(u) -
         int((bb_patch.getSizeU0() * params_.occupancyResolution_))) >= 0 &&
        (((pos_u * params_.occupancyResolution_) + u) <
         (((bb_patch.getSizeU0()) * params_.occupancyResolution_) + father_patch.getSizeU()))) {
      for (size_t v = bb_patch.getSizeV()-1; v > 0; v--) {
        if (bb_patch.getDepth(0)[(v * bb_patch.getSizeU()) + u] != infiniteDepth) {
          vdown = v;
          break;
        }
      }
      for (size_t v = 0; v < father_patch.getSizeV() - 1; v++) {
        if (father_patch.getDepth(0)[(v * father_patch.getSizeU()) +
                                     ((pos_u * params_.occupancyResolution_) + u -
                                      (bb_patch.getSizeU0() * params_.occupancyResolution_))] !=
            infiniteDepth) {
          vup = v;
          break;
        }
      }
    }
    if (pos_u == 11 && pos_v == 14) {
      //std::cout << vup << " " << vdown << std::endl;
    }
    // std::cout << vup << " " << vdown << std::endl;
    if ((vup != infiniteDepth) && (vdown != infiniteDepth) &&
        (int(vup) > (int(vdown) - ((int(bb_patch.getSizeV0()) - int(pos_v)) *
                                   int(params_.occupancyResolution_))))) {
      //std::cout << "I die Hereeeee4" << std::endl;
      size_t pson = (vdown * bb_patch.getSizeU()) + u;
      size_t pfather =
          (vup * father_patch.getSizeU()) + ((pos_u * params_.occupancyResolution_) + u -
                                             (bb_patch.getSizeU0() * params_.occupancyResolution_));
      int sum1 =
          pow(int((father_patch.getColor(0)[pfather]) - int(bb_patch.getColor(0)[pson])), 2) +
          pow(int((father_patch.getColor(1)[pfather]) - int(bb_patch.getColor(1)[pson])), 2) +
          pow(int((father_patch.getColor(2)[pfather]) - int(bb_patch.getColor(2)[pson])), 2);
      double sum2 = sqrt((double(sum1) / (3.0 * 255.0 * 255.0)));
      sum = sum + sum2;
      num_pixels++;
    }
  }
  if (pos_u == 11 && pos_v == 14) {
    //std::cout << "BoW " << num_pixels << " " << sum << std::endl;
  }
  double percentage_pixels = double(num_pixels);
  // / double((2 * bb_patch.getSizeU()) + (2 * bb_patch.getSizeV()));
  // std::cout << num_pixels << " " << percentage_pixels << std::endl;
  // getchar();
  return (sum / (double(num_pixels))) / percentage_pixels;
}

void PCCEncoder::packBestBuddies(PCCFrameContext &frame, size_t frameIndex) {
  auto &patches = frame.getPatches();
  size_t i = 0;
  std::vector<std::vector<Buddies>> best_buddies;
  best_buddies.resize(patches.size());
  std::sort(patches.begin(), patches.end());
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  size_t k = 0;
  std::vector<pcc::PCCPatch> kpatches;
  size_t firstPatch = 400;
  double bestBestInter = 1000000.0;

  for (auto &patch : patches) {
    // std::cout << i << std::endl;
    if (i == 1) {
      // printMap(patch.getOccupancy(), patch.getSizeU0(), patch.getSizeV0());
    }
    if (i != 6) {
      i++;
      continue;
    }
    /*if (i == 2) {
      break;
    }*/
    std::vector<bool> occupancyMap;
    occupancyMap.resize(patch.getSizeU0() * patch.getSizeV0(), false);
    auto &fatherOccupancy = patch.getOccupancy();
    size_t j = 0;
    size_t bigOcH = 0;
    size_t bigOcW = 0;
    size_t maxW = 0;
    size_t maxH = 0;
    for (auto &patch1 : patches) {
      if (j <= i) {
        j++;
        continue;
      }
      maxW = (std::max)(maxW, patch1.getSizeU0());
      maxH = (std::max)(maxH, patch1.getSizeV0());
    }
    maxH = patches[i + 1].getSizeV0();
    j = 0;
    // std::cout << "Chego Aqui" << std::endl;
    bigOcW = patch.getSizeU0() + (2 * maxW);
    bigOcH = patch.getSizeV0() + (2 * maxH);
    std::vector<bool> tmpOccupancyMap;
    tmpOccupancyMap.resize(bigOcW * bigOcH, false);
    std::vector<bool> tmpOccupancyMap2;
    tmpOccupancyMap2.resize(bigOcW * bigOcH, false);
    std::vector<int> valuesOccupancyMap;
    valuesOccupancyMap.resize(bigOcW * bigOcH, -1);
    std::vector<int> tmpValuesOccupancyMap;
    tmpValuesOccupancyMap.resize(patch.getSizeU0() * patch.getSizeV0(), -1);
    size_t ul1 = 0;
    size_t vl1 = 0;
    patch.getU0() = maxW;
    patch.getV0() = maxH;
    kpatches.push_back(patch);
    for (size_t u1 = maxW; u1 < maxW + patch.getSizeU0(); u1++) {
      for (size_t v1 = maxH; v1 < maxH + patch.getSizeV0(); v1++) {
        tmpOccupancyMap[(v1 * bigOcW) + u1] = fatherOccupancy[(vl1 * patch.getSizeU0()) + ul1];
        vl1 = vl1 + 1;
      }
      vl1 = 0;
      ul1 = ul1 + 1;
    }
    ul1 = 0;
    vl1 = 0;
    // printMap(tmpOccupancyMap,bigOcW,bigOcH);
    for (size_t u1 = maxW; u1 < maxW + patch.getSizeU0(); u1++) {
      for (size_t v1 = maxH; v1 < maxH + patch.getSizeV0(); v1++) {
        tmpOccupancyMap2[(v1 * bigOcW) + u1] = fatherOccupancy[(vl1 * patch.getSizeU0()) + ul1];
        vl1 = vl1 + 1;
      }
      vl1 = 0;
      ul1 = ul1 + 1;
    }
    // printMap(tmpOccupancyMap2, bigOcW, bigOcH);
    // std::cout << "Chego Aqui" << std::endl;
    for (auto &patch1 : patches) {
      /*if (i <6) {
        i++;
        continue;
      }*/
      // std::cout << j << std::endl;
      if (j <= i) {
        j++;
        continue;
      }
      /*if (patch1.getSizeU() > patch.getSizeU() || patch1.getSizeV() > patch.getSizeV()) {
        continue;
      }*/
      if (j != 7) {
        //j++;
        //continue;
      }
      if (j == 16) {
        //break;
      }
      double bestInter = 1000000.0;
      double tmpBestInter = 1000000.0;
      // std::cout << j << " " << i << " " << patch1.getSizeU0() << " " << patch1.getSizeV0() <<
      // std::endl;
      // std::cout << j << " " << patch.getSizeU0() << " " << patch.getSizeV0() << std::endl;

      // printMap(patch1.getOccupancy(), patch1.getSizeU0(), patch1.getSizeV0());

      // copy to the center
      size_t uu = 0;
      size_t vv = 0;
      int uuu = 0;
      int vvv = 0;
      int uuuu = 0;
      int vvvv = 0;
      int u5 = 0;
      int v5 = 0;

      bool out = 0;
      // if (j == i+1) {
      // std::cout << "Chego Aqui" << std::endl;
      //}
      // std::cout << "Morro no " << i << " " << j << std::endl;
      // std::cout << "one " << patch1.getSizeU0() << " " << patch.getSizeU0() << std::endl;
      // std::cout << "two " << patch1.getSizeV0() << " " << patch.getSizeV0() << std::endl;
      // std::cout << "two " << bigOcW << " " << bigOcH << std::endl;
      //printMap(tmpOccupancyMap, bigOcW, bigOcH);
      //printMap(tmpOccupancyMap2, bigOcW, bigOcH);
      std::vector<int> overlaps;
      std::vector<int> final_overlaps;
      bool fits = false;
      for (size_t v = 0; v <= patch.getSizeV0() + patch1.getSizeV0() && out == 0; ++v) {
        for (size_t u = 0; u <= patch.getSizeU0() + patch1.getSizeU0() && out == 0; ++u) {
          if (u == 0 && v == 0) {
            continue;
          }
          if (u == patch.getSizeU0() + patch1.getSizeU0() &&
              v == patch.getSizeV0() + patch1.getSizeV0()) {
            continue;
          }
          // std::cout << "ooooooo" << std::endl;
          bool canFit = true;
          bool canFit2 = true;
          for (size_t v0 = 0; v0 < patch1.getSizeV0() && canFit; ++v0) {
            const size_t y = v + maxH - patch1.getSizeV0() + v0;
            for (size_t u0 = 0; u0 < patch1.getSizeU0(); ++u0) {
              const size_t x = u + maxW - patch1.getSizeU0() + u0;
              if (patch1.getOccupancy()[v0 * patch1.getSizeU0() + u0] &&
                  tmpOccupancyMap[y * bigOcW + x]) {
                // canFit = false;
                break;
              }
              // std::cout << "eeeeee"<<std::endl;
            }
          }
          for (size_t v0 = 0; v0 < patch1.getSizeV0() && canFit2; ++v0) {
            const size_t y = v + maxH - patch1.getSizeV0() + v0;
            for (size_t u0 = 0; u0 < patch1.getSizeU0(); ++u0) {
              const size_t x = u + maxW - patch1.getSizeU0() + u0;
              if (patch1.getOccupancy()[v0 * patch1.getSizeU0() + u0] &&
                  tmpOccupancyMap[y * bigOcW + x]) {
                canFit2 = false;
                break;
              }
              // std::cout << "eeeeee"<<std::endl;
            }
          }
          if (!canFit2) {
            continue;
          }
		  //std::cout << "Pos = " << int(u + maxW - patch1.getSizeU0()) - int(maxW) << " " << int(v + maxH - patch1.getSizeV0()) - int(maxH) << std::endl;

          /*if (!canFit) {
            continue;
          }*/
          // std::cout << "chis" << uuu - int(maxW) << " " << vvv - int(maxH)
          // << std::endl;
          if (j == 1) {
            // std::cout << "chego aqui" << std::endl;
            // u = 14;
            // v = 0;
            // std::cout << "Pos = " << int(u + maxW - patch1.getSizeU0()) - int(maxW) << " " <<
            // int(v + maxH - patch1.getSizeV0()) - int(patches[i +
            // 1].getSizeV0()) << std::endl;
          }
          if (j == 3) {
            // std::cout << "Estou Aqui" << std::endl;
          }
          // std::cout << "eeeeee" << std::endl;
          uuu = int(u + maxW - patch1.getSizeU0());  // - int(maxW);
          vvv = int(v + maxH - patch1.getSizeV0());  // - int(maxH);
          // std::cout << uuu - int(maxW) << " " - int(maxH) << vvv <<
          // std::endl;
          if (j == 10) {
            // std::cout << uuu - int(maxW) << " " << vvv - int(maxH) <<
            // std::endl;
          }
          overlaps.resize(0);
          if (uuu - int(maxW) < 0 && vvv - int(maxH) < 0) {
            // std::cout << "oooooo" << std::endl;
            for (size_t u1 = 0; u1 < uuu + patch1.getSizeU0() - maxW; u1++) {
              if (occupancyMap[u1] == true) {
                canFit = false;
                overlaps.push_back(tmpValuesOccupancyMap[u1]);
              }
            }
            for (size_t v1 = 0; v1 < vvv + patch1.getSizeV0() - maxH; v1++) {
              if (occupancyMap[v1 * patch.getSizeU0()] == true) {
                canFit = false;
                overlaps.push_back(tmpValuesOccupancyMap[v1 * patch.getSizeU0()]);
              }
            }
          } else if (uuu - int(maxW) < 0) {
            for (size_t u1 = 0; u1 < uuu + patch1.getSizeU0() - maxW; u1++) {
              if (occupancyMap[u1 + ((patch.getSizeV0() - 1) * (patch.getSizeU0()))] == true) {
                canFit = false;
                overlaps.push_back(tmpValuesOccupancyMap[u1 + ((patch.getSizeV0() - 1) * (patch.getSizeU0()))]);
              }
            }
            for (size_t v1 = vvv - int(maxH);
                 (v1 < (vvv - int(maxH)) + patch1.getSizeV0()) && v1 < patch.getSizeV0(); v1++) {
              if (occupancyMap[v1 * patch.getSizeU0()] == true) {
                canFit = false;
                overlaps.push_back(tmpValuesOccupancyMap[v1 * patch.getSizeU0()]);
              }
            }
          } else if (vvv - int(maxH) < 0) {
            if (j == 10) {
              // std::cout << "oooooo" << std::endl;
            }
            for (size_t v1 = 0; v1 < vvv + patch1.getSizeV0() - maxH; v1++) {
              if (occupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1] == true) {
                overlaps.push_back(tmpValuesOccupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1]);
                canFit = false;
              }
            }
            for (size_t u1 = uuu - int(maxW);
                 (u1 < (uuu - int(maxW)) + patch1.getSizeU0()) && u1 < patch.getSizeU0(); u1++) {
              if (j == 10) {
                // std::cout << "oooooo" << std::endl;
              }
              if (occupancyMap[u1] == true) {
                canFit = false;
				overlaps.push_back(tmpValuesOccupancyMap[u1]);
              }
            }
          } else {
            for (size_t v1 = vvv - int(maxH);
                 (v1 < (vvv - int(maxH)) + patch1.getSizeV0()) && v1 < patch.getSizeV0(); v1++) {
              if (occupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1] == true) {
                overlaps.push_back(tmpValuesOccupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1]);
                canFit = false;
              }
            }
            for (size_t u1 = uuu - int(maxW);
                 (u1 < (uuu - int(maxW)) + patch1.getSizeU0()) && u1 < patch.getSizeU0(); u1++) {
              if (occupancyMap[u1 + ((patch.getSizeV0() - 1) * (patch.getSizeU0()))] == true) {
                canFit = false;
                overlaps.push_back(tmpValuesOccupancyMap[u1 + ((patch.getSizeV0() - 1) * (patch.getSizeU0()))]);
              }
            }
          }

		  double tmpInter = computeSM(patch, patch1, tmpOccupancyMap, u, v, patches[i + 1]);
          if (tmpInter < tmpBestInter) {
            tmpBestInter = tmpInter;
            final_overlaps = overlaps;
            u5 = uuu;
            v5 = vvv;
          }

          if (j == 3) {
            // std::cout << "Estou Aqui2" << std::endl;
          }
          if (j == 1) {
            // std::cout << "chego aqui2" << std::endl;
          }
          // std::cout << "ooooooo" << std::endl;
          // std::cout << "eeeeee" << std::endl;
		  //std::cout << "Pos = " << int(u + maxW - patch1.getSizeU0()) - int(maxW) << " " << int(v + maxH - patch1.getSizeV0()) - int(maxH) << std::endl;
          if (!canFit) {
            if (j == 7) {
              // std::cout << uuu - int(maxW) << " " << vvv - int(maxH) << std::endl;
              // std::cout << "lll" << std::endl;
            }
            if (j == 3) {
              // std::cout << "Estou Aqui3" << std::endl;
            }
            if (j == 28) {
              // std::cout << "Starting Computation for " << u << " and " << v << std::endl;
            }
            // std::cout << uuu - int(maxW) << " " << vvv - int(maxH) <<
            // std::endl;
            continue;
          }
		  if (j == 7) {
			//std::cout << "Pos = " << int(u + maxW - patch1.getSizeU0()) - int(maxW) << " " << int(v + maxH - patch1.getSizeV0()) - int(maxH) << std::endl;
          }
          if (j == 7) {
            // std::cout << uuu - int(maxW) << " " << vvv - int(maxH) << std::endl;
            // std::cout << "vvv" << std::endl;
          }

          if (j == 7) {
            // std::cout << "Starting Computation for " << u << " and " << v << std::endl;
            // std::cout << "one " << patch1.getSizeU0() << " " << patch.getSizeU0() << std::endl;
            // std::cout << "two " << patch1.getSizeV0() << " " << patch.getSizeV0() << std::endl;
          }
          // std::cout << uuu - int(maxW) << " " << vvv - int(maxH) <<
          // std::endl;  std::cout << "aaaaaaa" << std::endl;  std::cout << "Starting Computation
          // for "
          // << u << " and " << v << std::endl;
          // u = 0;
          // v = 55;
          if (j == 28) {
            // std::cout << "Starting Computation for " << u << " and " << v << std::endl;
          }

          double inter = tmpInter;
          if (j == 3) {
            // std::cout << "Estou Aqui5" << std::endl;
          }
          if (j == 1) {
            // std::cout << "Score = " << inter << std::endl;
          }
          // std::cout << "eeeeee" << std::endl;
          //std::cout << "Pos = " << int(u + maxW - patch1.getSizeU0()) - int(maxW) << " " << int(v + maxH - patch1.getSizeV0()) - int(maxH) << std::endl; 
		  //std::cout << "Pos = " << u << " " << v << std::endl;
		  //std::cout << "Score = "<< inter << std::endl;
          if (inter <= bestInter && inter != 0.0) {
            uuuu = int(u + maxW - patch1.getSizeU0());  // - int(maxW);
            vvvv = int(v + maxH - patch1.getSizeV0());  // - int(maxH);
            uu = u + maxW - patch1.getSizeU0();
            vv = v + maxH - patch1.getSizeV0();
            bestInter = inter;
            patch1.getU0() = uu;
            patch1.getV0() = vv;
            fits = true;
            //final_overlaps = overlaps;
            //fits = canFit;
          }
          if (j == 10) {
            // out = 1;
          }
          // out=1;
          // break;
        }
        // break;
      }
      // std::cout << tmpBestInter << std::endl;
      if (j == 4) {
        // std::cout << "Pos = " << int(uu) << " " << int(vv) << std::endl;
      }
      if (j == 1) {
        // printMap(patch1.getOccupancy(), patch1.getSizeU0(), patch1.getSizeV0());
      }
      // std::cout << "Best Score= " << bestInter / (2*patch1.getSizeU() + 2*patch1.getSizeV())
      //        << std::endl;
      if ((bestInter / (2 * patch1.getSizeU() + 2 * patch1.getSizeV())) < bestBestInter) {
        bestBestInter = bestInter / (2 * patch1.getSizeU() + 2 * patch1.getSizeV());
        firstPatch = patch.getIndex();
      }
      if (j == 5) {
        // std::cout << "Specs = " << patch1.getSizeU0() << " " << patches[i + 1].getSizeU0() <<
        // std::endl;
      }
      // std::cout << "Pos = " << int(uu) - int(maxW) << " " << int(vv) - int(patches[i +
      // 1].getSizeV0()) << std::endl;  std::cout << "Pos = " << int(uuuu) - int(maxW) << " " <<
      // int(vvvv) - int(maxH) << std::endl;
      if (!fits) {
        for (size_t k1 = 0; k1 < final_overlaps.size(); k1++) {
          std::cout << final_overlaps[k1] << " ";
        }
        std::cout << std::endl << "FFF " << j << std::endl;
		std::cout << "Pos = " << int(u5) - int(maxW) << " " << int(v5) - int(maxH) << std::endl;
        std::cout << "Dim " << patch1.getSizeU0() << " " << patch1.getSizeV0() << std::endl;
        size_t one_over = final_overlaps[0];
		bool only_one= true;
		for (size_t k1 = 1; k1 < final_overlaps.size(); k1++) {
          if(one_over!=final_overlaps[k1]){
			only_one=false;
            break;
		  }
        }
		if(only_one){
          std::cout << "Sou o Único" << std::endl;
		}
      }
      if (!fits) {
        j++;
        continue;
      }
      std::cout << "Encontro" << std::endl;
      if (uu != 0 || vv != 0) {
        //std::cout << "YOOO " << j << " " << uuuu - int(maxW) << " " << vvvv - int(maxH)
          //        << std::endl;
        if (uuuu - int(maxW) < 0 && vvvv - int(maxH) < 0) {
          for (size_t u1 = 0; u1 < uuuu + patch1.getSizeU0() - maxW; u1++) {
            occupancyMap[u1] = true;
            tmpValuesOccupancyMap[u1] = patch1.getIndex();
          }
          for (size_t v1 = 0; v1 < vvvv + patch1.getSizeV0() - maxH; v1++) {
            occupancyMap[v1 * patch.getSizeU0()] = true;
            tmpValuesOccupancyMap[v1 * patch.getSizeU0()] = patch1.getIndex();
          }
        } else if (uuuu - int(maxW) < 0) {
          // std::cout << "Estou Aqui" << std::endl;
          for (size_t u1 = 0; u1 < uuuu + patch1.getSizeU0() - maxW; u1++) {
            occupancyMap[u1 + ((patch.getSizeV0() - 1) * patch.getSizeU0())] = true;
            tmpValuesOccupancyMap[u1 + ((patch.getSizeV0() - 1) * patch.getSizeU0())] =
                patch1.getIndex();
          }
          for (size_t v1 = vvvv - int(maxH);
               (v1 < (vvvv - int(maxH)) + patch1.getSizeV0()) && v1 < patch.getSizeV0(); v1++) {
            occupancyMap[v1 * patch.getSizeU0()] = true;
            tmpValuesOccupancyMap[v1 * patch.getSizeU0()] = patch1.getIndex();
          }
        } else if (vvvv - int(maxH) < 0) {
          for (size_t v1 = 0; v1 < vvvv + patch1.getSizeV0() - maxH; v1++) {
            occupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1] = true;
            tmpValuesOccupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1] =
                patch1.getIndex();
          }
          for (size_t u1 = uuuu - int(maxW);
               (u1 < (uuuu - int(maxW)) + patch1.getSizeU0()) && u1 < patch.getSizeU0(); u1++) {
            occupancyMap[u1] = true;
            tmpValuesOccupancyMap[u1] = patch1.getIndex();
          }
        } else {
          // std::cout << vvvv - int(maxH) << std::endl;
          for (size_t v1 = vvvv - int(maxH);
               (v1 < (vvvv - int(maxH)) + patch1.getSizeV0()) && v1 < patch.getSizeV0(); v1++) {
            // std::cout << "eeeeee" << std::endl;
            occupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1] = true;
            tmpValuesOccupancyMap[v1 * patch.getSizeU0() + patch.getSizeU0() - 1] =
                patch1.getIndex();
          }
          for (size_t u1 = uuuu - int(maxW);
               (u1 < (uuuu - int(maxW)) + patch1.getSizeU0()) && u1 < patch.getSizeU0(); u1++) {
            occupancyMap[u1 + ((patch.getSizeV0() - 1) * (patch.getSizeU0()))] = true;
            tmpValuesOccupancyMap[u1 + ((patch.getSizeV0() - 1) * (patch.getSizeU0()))] =
                patch1.getIndex();
          }
        }
        // if (j == 27 || j == 28) {
        // printMap(occupancyMap, patch.getSizeU0(), patch.getSizeV0());
        //}
         //printMap(occupancyMap, patch.getSizeU0(), patch.getSizeV0());
        for (size_t v0 = 0; v0 < patch1.getSizeV0(); ++v0) {
          const size_t v = vv + v0;
          for (size_t u0 = 0; u0 < patch1.getSizeU0(); ++u0) {
            const size_t u = uu + u0;
            tmpOccupancyMap[v * bigOcW + u] = tmpOccupancyMap[v * bigOcW + u] ||
                                              patch1.getOccupancy()[v0 * patch1.getSizeU0() + u0];
          }
        }
        for (size_t v0 = 0; v0 < patch1.getSizeV0(); ++v0) {
          const size_t v = vv + v0;
          for (size_t u0 = 0; u0 < patch1.getSizeU0(); ++u0) {
            const size_t u = uu + u0;
            if (patch1.getOccupancy()[v0 * patch1.getSizeU0() + u0]) {
              valuesOccupancyMap[v * bigOcW + u] = patch1.getIndex();
            }
          }
        }
        best_buddies[patch.getIndex()].resize(best_buddies[patch.getIndex()].size() + 1);
        best_buddies[patch.getIndex()][best_buddies[patch.getIndex()].size() - 1].getFather() =
            patch;
        best_buddies[patch.getIndex()][best_buddies[patch.getIndex()].size() - 1].getPatch() =
            patch1;
        best_buddies[patch.getIndex()][best_buddies[patch.getIndex()].size() - 1].getPosU() =
            int(uu) - int(maxW);
        best_buddies[patch.getIndex()][best_buddies[patch.getIndex()].size() - 1].getPosV() =
            int(vv) - int(maxH);
        best_buddies[patch.getIndex()][best_buddies[patch.getIndex()].size() - 1].getScore() =
            bestInter / (2 * patch1.getSizeU() + 2 * patch1.getSizeV());
        std::sort(best_buddies[patch.getIndex()].begin(), best_buddies[patch.getIndex()].end());
        height = (std::max)(
            height, (patch1.getV0() + patch1.getSizeV0()) * patch1.getOccupancyResolution());
        width = (std::max)(width,
                           (patch1.getU0() + patch1.getSizeU0()) * patch1.getOccupancyResolution());
        k++;
        // printMap(tmpOccupancyMap, bigOcW , bigOcH);
        kpatches.push_back(patch1);
      }
      // printMap(tmpOccupancyMap, bigOcW, bigOcH);
      j++;
    }
    for (size_t vv = 0; vv < patch.getSizeV0(); vv++) {
      for (size_t uu = 0; uu < patch.getSizeU0(); uu++) {
        if (tmpValuesOccupancyMap[vv * patch.getSizeU0() + uu] == -1) {
          std::cout << ".";
        } else {
          std::cout << tmpValuesOccupancyMap[vv * patch.getSizeU0() + uu];
        }
      }
      std::cout << std::endl;
    }
    //printMap(occupancyMap, patch.getSizeU0(), patch.getSizeV0());
    //printMap(tmpOccupancyMap, bigOcW, bigOcH);
    /*for (size_t v0 = 0; v0 < bigOcH; ++v0) {
      for (size_t u0 = 0; u0 < bigOcW; ++u0) {
        std::cout << valuesOccupancyMap[v0*bigOcW + u0] << " ";
          }
          std::cout << std::endl;
    }*/
    i++;
    std::cout << "Acabei o " << i << std::endl;
    // std::cout << std::endl;
    /*for (size_t k = 0; k < best_buddies[0].size(); k++) {
      std::cout << "Father = " << best_buddies[0][k].getFather().getIndex() << std::endl;
      std::cout << "Patch = " << best_buddies[0][k].getPatch().getIndex() << std::endl;
      std::cout << "U = " << best_buddies[0][k].getPosU() << std::endl;
      std::cout << "V = " << best_buddies[0][k].getPosV() << std::endl;
      std::cout << "Score = " << best_buddies[0][k].getScore() << std::endl;
      std::cout << std::endl;
    }*/
  }
  // patches = kpatches;
  // std::cout << std::endl << std::endl << std::endl;
  std::cout << "Acabei" << std::endl;
  getchar();
  // std::cout << "First Patch = " << bestBestInter << std::endl;
  // std::cout << "First Patch = " << firstPatch << std::endl;
  std::vector<Buddies> buddies_pool = best_buddies[patches[0].getIndex()];
  std::cout << buddies_pool.size() << std::endl;
  std::vector<pcc::PCCPatch> kkpatches;
  // std::sort(best_buddies.begin(), best_buddies.end(), orderByFather);
  for (size_t k = 0; k < best_buddies[patches[0].getIndex()].size(); k++) {
    // std::cout << "Father = " << buddies_pool[k].getFather().getIndex() << std::endl;
    // std::cout << "Patch = " << buddies_pool[k].getPatch().getIndex() << std::endl;
    // std::cout << "U = " << buddies_pool[k].getPosU() << std::endl;
    // std::cout << "V = " << buddies_pool[k].getPosV() << std::endl;
    // std::cout << "Score = " << buddies_pool[k].getScore() << std::endl;
    // std::cout << std::endl;
  }
  std::sort(patches.begin(), patches.end(), byIndex);
  // std::cout << "Acabo" << std::endl;
  std::vector<bool> packed;
  packed.resize(patches.size(), false);
  size_t l = 1;
  packed[buddies_pool[0].getFather().getIndex()] = true;
  // std::cout << "Acabo" << std::endl;
  std::vector<bool> currOcupancy = buddies_pool[0].getFather().getOccupancy();
  std::vector<bool> finalOcupancy = buddies_pool[0].getFather().getOccupancy();
  // std::cout << "Acabo" << std::endl;
  int fOcW = buddies_pool[0].getFather().getSizeU0();
  int fOcH = buddies_pool[0].getFather().getSizeV0();
  // std::cout << "Acabo" << std::endl;
  patches[buddies_pool[0].getFather().getIndex()].getU0() = 0;
  patches[buddies_pool[0].getFather().getIndex()].getV0() = 0;
  // kkpatches.push_back(patches[buddies_pool[0].getFather().getIndex()]);
  // printMap(currOcupancy, fOcW, fOcH);
  // std::cout << "Acabo" << std::endl;
  while (l < patches.size()) {
    std::cout << "Acabo em " << l << std::endl;
    // if (l == 20) {
    //	break;
    //}
    bool found_next = 0;
    Buddies next_patch;
    size_t s = 0;
    bool nothing = false;
    while (found_next == 0) {
      if (buddies_pool.size() == 0) {
        nothing = true;
        break;
      }
      next_patch = buddies_pool[0];
      if (packed[next_patch.getPatch().getIndex()] == 0) {
        found_next = 1;
      }
      buddies_pool.erase(buddies_pool.begin());
      s++;
    }
    if (nothing == true) {
      l++;
      break;
    }
    // std::cout << "Father = " << next_patch.getFather().getIndex() << std::endl;
    // std::cout << "Patch = " << next_patch.getPatch().getIndex() << std::endl;
    // std::cout << "U = " << next_patch.getPosU() << std::endl;
    // std::cout << "V = " << next_patch.getPosV() << std::endl;
    // std::cout << "Score = " << next_patch.getScore() << std::endl;
    int newOcW;
    int newOcH;
    bool fit = false;
    while (fit == false) {
      // std::cout << "Hey" << std::endl;
      fit = true;
      // std::cout << "Father = " << next_patch.getFather().getIndex() << std::endl;
      // std::cout << "Patch = " << next_patch.getPatch().getIndex() << std::endl;
      // std::cout << "U = " << next_patch.getPosU() << std::endl;
      // std::cout << "V = " << next_patch.getPosV() << std::endl;
      // std::cout << "Score = " << next_patch.getScore() << std::endl;
      if (next_patch.getPosU() < 0) {
        newOcW = int(patches[next_patch.getFather().getIndex()].getU0()) + next_patch.getPosU();
        if (next_patch.getPatch().getIndex() == 3) {
          // std::cout << "Bunda no Chão"  << newOcW << std::endl;
        }
        if (newOcW > 0) {
          newOcW = 0;
          // no need to extend
        }
      } else {
        newOcW = next_patch.getPosU() + int(next_patch.getPatch().getSizeU0()) -
                 (fOcW - int(patches[next_patch.getFather().getIndex()].getU0()));
        // std::cout << next_patch.getPosU() << " " << next_patch.getPatch().getSizeU0()
        //                << " " << fOcW << " " <<
        // patches[next_patch.getFather().getIndex()].getU0() << std::endl;
        // std::cout << patches[next_patch.getFather().getIndex()].getU0() << std::endl;
        // std::cout << next_patch.getFather().getIndex() << std::endl;
        // std::cout << "Chego Aqui" << std::endl;
        if (newOcW < 0) {
          // std::cout << "Chego Aqui" << std::endl;
          newOcW = 0;
          // no need to extend
        }
      }
      if (next_patch.getPosV() < 0) {
        newOcH = int(patches[next_patch.getFather().getIndex()].getV0()) + next_patch.getPosV();
        if (newOcH > 0) {
          newOcH = 0;
          // no need to extend
        }
      } else {
        newOcH = next_patch.getPosV() + int(next_patch.getPatch().getSizeV0()) -
                 (fOcH - int(patches[next_patch.getFather().getIndex()].getV0()));
        // std::cout << next_patch.getPosV() << " " << next_patch.getPatch().getSizeV0()
        //        << " " << fOcH << " " <<
        // patches[next_patch.getFather().getIndex()].getV0() << std::endl;
        if (newOcH < 0) {
          // std::cout << "Chego Aqui" << std::endl;
          newOcH = 0;
          // no need to extend
        }
      }
      // std::cout << "Chego Aqui" << std::endl;
      // expand occupancy
      if (newOcW < 0 && newOcH < 0) {
        // std::cout << "Chego Aqui1" << std::endl;
        currOcupancy.resize((fOcW - newOcW) * (fOcH - newOcH), false);
        std::fill(currOcupancy.begin(), currOcupancy.end(), false);
        // printMap(currOcupancy,(fOcW - newOcW),(fOcH - newOcH));
        for (size_t u0 = 0; u0 < fOcW; u0++) {
          size_t u1 = u0 - newOcW;
          for (size_t v0 = 0; v0 < fOcH; v0++) {
            size_t v1 = v0 - newOcH;
            currOcupancy[(v1 * (fOcW - newOcW)) + u1] = finalOcupancy[(v0 * fOcW) + u0];
          }
        }
        // printMap(currOcupancy,(fOcW - newOcW),(fOcH - newOcH));
        for (size_t u0 = 0; u0 < next_patch.getPatch().getSizeU0(); u0++) {
          for (size_t v0 = 0; v0 < next_patch.getPatch().getSizeV0(); v0++) {
            if (currOcupancy[(v0 * (fOcW - newOcW)) + u0] == true &&
                next_patch.getPatch()
                        .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] == true) {
              fit = false;
              // std::cout << u0 << " " << v0 << std::endl;
              break;
            }
            currOcupancy[(v0 * (fOcW - newOcW)) + u0] =
                next_patch.getPatch()
                    .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] ||
                currOcupancy[(v0 * (fOcW - newOcW)) + u0];
          }
          if (fit == false) {
            break;
          }
        }
      } else if (newOcW < 0) {
        if (next_patch.getPatch().getIndex() == 3) {
          // std::cout << "Chego Aqui2" << std::endl;
        }
        if (next_patch.getPatch().getIndex() == 3) {
          // printMap(currOcupancy, fOcW, fOcH);
        }
        // std::cout << "Chego Aqui2" << std::endl;
        currOcupancy.resize((fOcW - newOcW) * (fOcH + newOcH), false);
        if (next_patch.getPatch().getIndex() == 3) {
          // printMap(currOcupancy, fOcW, fOcH);
        }
        std::fill(currOcupancy.begin(), currOcupancy.end(), false);
        for (size_t u0 = 0; u0 < fOcW; u0++) {
          size_t u1 = u0 - newOcW;
          // std::cout << u1 << " ";
          for (size_t v0 = 0; v0 < fOcH; v0++) {
            size_t v1 = v0;
            // std::cout << v1 << " ";
            currOcupancy[(v1 * (fOcW - newOcW)) + u1] = finalOcupancy[v0 * fOcW + u0];
          }
        }
        if (next_patch.getPatch().getIndex() == 3) {
          // printMap(currOcupancy, fOcW - newOcW, fOcH + newOcH);
        }
        for (size_t u0 = 0; u0 < next_patch.getPatch().getSizeU0(); u0++) {
          for (size_t v0 = 0; v0 < next_patch.getPatch().getSizeV0(); v0++) {
            size_t v1 =
                v0 + next_patch.getPosV() + patches[next_patch.getFather().getIndex()].getV0();
            if (currOcupancy[(v1 * (fOcW - newOcW)) + u0] == true &&
                next_patch.getPatch()
                        .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] == true) {
              if (next_patch.getPatch().getIndex() == 3) {
                // std::cout << u0 << v1 << std::endl;
              }
              fit = false;
              break;
            }
            currOcupancy[(v1 * (fOcW - newOcW)) + u0] =
                next_patch.getPatch()
                    .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] ||
                currOcupancy[(v1 * (fOcW - newOcW)) + u0];
          }
          if (fit == false) {
            break;
          }
        }
      } else if (newOcH < 0) {
        // std::cout << "Chego Aqui3" << std::endl;
        currOcupancy.resize((fOcW + newOcW) * (fOcH - newOcH), false);
        std::fill(currOcupancy.begin(), currOcupancy.end(), false);
        for (size_t u0 = 0; u0 < fOcW; u0++) {
          size_t u1 = u0;
          for (size_t v0 = 0; v0 < fOcH; v0++) {
            size_t v1 = v0 - newOcH;
            currOcupancy[(v1 * (fOcW + newOcW)) + u1] = finalOcupancy[v0 * fOcW + u0];
          }
        }
        for (size_t u0 = 0; u0 < next_patch.getPatch().getSizeU0(); u0++) {
          size_t u1 =
              u0 + next_patch.getPosU() + patches[next_patch.getFather().getIndex()].getU0();
          for (size_t v0 = 0; v0 < next_patch.getPatch().getSizeV0(); v0++) {
            if (currOcupancy[(v0 * (fOcW + newOcW)) + u1] == true &&
                next_patch.getPatch()
                        .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] == true) {
              fit = false;
              break;
            }
            currOcupancy[(v0 * (fOcW + newOcW)) + u1] =
                next_patch.getPatch()
                    .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] ||
                currOcupancy[(v0 * (fOcW + newOcW)) + u1];
          }
          if (fit == false) {
            break;
          }
        }
      } else {
        // std::cout << "Chego Aqui" << std::endl;
        // std::cout << "Chego Aqui4" << std::endl;
        currOcupancy.resize((fOcW + newOcW) * (fOcH + newOcH), false);
        std::fill(currOcupancy.begin(), currOcupancy.end(), false);
        // std::cout << newOcW << " " << newOcH << std::endl;
        for (size_t u0 = 0; u0 < fOcW; u0++) {
          size_t u1 = u0;
          for (size_t v0 = 0; v0 < fOcH; v0++) {
            size_t v1 = v0;
            // std::cout << "Chego Aqui4" << std::endl;
            currOcupancy[(v1 * (fOcW + newOcW)) + u1] = finalOcupancy[(v0 * fOcW) + u0];
          }
        }
        // std::cout << "Chego Aqui4" << std::endl;
        for (size_t u0 = 0; u0 < next_patch.getPatch().getSizeU0(); u0++) {
          size_t u1 =
              u0 + next_patch.getPosU() + patches[next_patch.getFather().getIndex()].getU0();
          for (size_t v0 = 0; v0 < next_patch.getPatch().getSizeV0(); v0++) {
            size_t v1 =
                v0 + next_patch.getPosV() + patches[next_patch.getFather().getIndex()].getV0();
            if (currOcupancy[(v1 * (fOcW + newOcW)) + u1] == true &&
                next_patch.getPatch()
                        .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] == true) {
              fit = false;
              break;
            }
            currOcupancy[(v1 * (fOcW + newOcW)) + u1] =
                next_patch.getPatch()
                    .getOccupancy()[v0 * (next_patch.getPatch().getSizeU0()) + u0] ||
                currOcupancy[(v1 * (fOcW + newOcW)) + u1];
          }
          if (fit == false) {
            break;
          }
        }
      }
      if (fit == false) {
        found_next = 0;
        size_t s = 0;
        while (found_next == 0) {
          if (buddies_pool.size() == 0) {
            nothing = true;
            break;
          }
          next_patch = buddies_pool[0];
          if (packed[next_patch.getPatch().getIndex()] == 0) {
            found_next = 1;
          }
          buddies_pool.erase(buddies_pool.begin());
          s++;
        }
        currOcupancy = finalOcupancy;
      }
      if (buddies_pool.size() == 0) {
        nothing = true;
        break;
      }
    }
    if (nothing == true) {
      l++;
      break;
    }
    // std::cout << "Father = " << next_patch.getFather().getIndex() << std::endl;
    // std::cout << "Patch = " << next_patch.getPatch().getIndex() << std::endl;
    // std::cout << "U = " << next_patch.getPosU() << std::endl;
    // std::cout << "V = " << next_patch.getPosV() << std::endl;
    // std::cout << "Score = " << next_patch.getScore() << std::endl;
    // printMap(next_patch.getPatch().getOccupancy(), next_patch.getPatch().getSizeU0(),
    // next_patch.getPatch().getSizeV0());
    if (newOcW < 0 && newOcH < 0) {
      fOcW = fOcW - newOcW;
      fOcH = fOcH - newOcH;
      patches[next_patch.getPatch().getIndex()].getU0() = 0;
      patches[next_patch.getPatch().getIndex()].getV0() = 0;
      for (auto &patch : patches) {
        if (packed[patch.getIndex()] == true) {
          patch.getU0() = -newOcW + patch.getU0();
          patch.getV0() = -newOcH + patch.getV0();
        }
      }
      // patches[next_patch.getFather().getIndex()].getU0() = -newOcW +
      // patches[next_patch.getFather().getIndex()].getU0();
      // patches[next_patch.getFather().getIndex()].getV0() = -newOcH +
      // patches[next_patch.getFather().getIndex()].getV0();
    } else if (newOcW < 0) {
      fOcW = fOcW - newOcW;
      fOcH = fOcH + newOcH;
      patches[next_patch.getPatch().getIndex()].getU0() = 0;
      patches[next_patch.getPatch().getIndex()].getV0() =
          int(next_patch.getPosV()) + int(patches[next_patch.getFather().getIndex()].getV0());
      for (auto &patch : patches) {
        if (packed[patch.getIndex()] == true) {
          patch.getU0() = -newOcW + patch.getU0();
        }
      }
      // patches[next_patch.getFather().getIndex()].getU0() = -newOcW +
      // patches[next_patch.getFather().getIndex()].getU0();
    } else if (newOcH < 0) {
      fOcW = fOcW + newOcW;
      fOcH = fOcH - newOcH;
      patches[next_patch.getPatch().getIndex()].getU0() =
          next_patch.getPosU() + patches[next_patch.getFather().getIndex()].getU0();
      patches[next_patch.getPatch().getIndex()].getV0() = 0;
      for (auto &patch : patches) {
        if (packed[patch.getIndex()] == true) {
          patch.getV0() = -newOcH + patch.getV0();
        }
      }
      // patches[next_patch.getFather().getIndex()].getV0() = -newOcH +
      // patches[next_patch.getFather().getIndex()].getV0();
    } else {
      fOcW = fOcW + newOcW;
      fOcH = fOcH + newOcH;
      patches[next_patch.getPatch().getIndex()].getU0() =
          next_patch.getPosU() + patches[next_patch.getFather().getIndex()].getU0();
      patches[next_patch.getPatch().getIndex()].getV0() =
          next_patch.getPosV() + patches[next_patch.getFather().getIndex()].getV0();
      // std::cout << "Aqui " << patches[next_patch.getPatch().getIndex()].getV0() << std::endl;
    }
    packed[next_patch.getPatch().getIndex()] = 1;
    finalOcupancy = currOcupancy;

    l++;
    // std::cout << (fOcW + newOcW) << " " << (fOcH + newOcH)<< std::endl;
    // printMap(currOcupancy, fOcW, fOcH);
    buddies_pool.insert(buddies_pool.end(), best_buddies[next_patch.getPatch().getIndex()].begin(),
                        best_buddies[next_patch.getPatch().getIndex()].end());
    // for (size_t k = 0; k < buddies_pool.size(); k++) {
    // std::cout << "Father = " << buddies_pool[k].getFather().getIndex() << std::endl;
    // std::cout << "Patch = " << buddies_pool[k].getPatch().getIndex() << std::endl;
    // std::cout << "U = " << buddies_pool[k].getPosU() << std::endl;
    // std::cout << "V = " << buddies_pool[k].getPosV() << std::endl;
    // std::cout << "Score = " << buddies_pool[k].getScore() << std::endl;
    // std::cout << std::endl;
    //}
    std::sort(buddies_pool.begin(), buddies_pool.end());
    // std::cout << std::endl;std::cout << std::endl;
    // std::cout << std::endl;std::cout << std::endl;
    for (size_t k = 0; k < buddies_pool.size(); k++) {
      // std::cout << "Father = " << buddies_pool[k].getFather().getIndex() << std::endl;
      // std::cout << "Patch = " << buddies_pool[k].getPatch().getIndex() << std::endl;
      // std::cout << "U = " << buddies_pool[k].getPosU() << std::endl;
      // std::cout << "V = " << buddies_pool[k].getPosV() << std::endl;
      // std::cout << "Score = " << buddies_pool[k].getScore() << std::endl;
      // std::cout << std::endl;
    }
  }
  for (auto &patch : patches) {
    if (packed[patch.getIndex()] == true) {
      kkpatches.push_back(patch);
    }
  }
  printMap(currOcupancy, fOcW, fOcH);
  height = fOcH * params_.occupancyResolution_;
  width = fOcW * params_.occupancyResolution_;
  // patches = kpatches;
  std::cout << "Acabei Total" << std::endl;
  patches = kkpatches;
  // std::wcout << patches.size() << std::endl;
}

void PCCEncoder::packBB(PCCFrameContext &frame, size_t frameIndex) {
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  auto &patches = frame.getPatches();
  if (patches.empty()) {
    return;
  }
  for (auto &patch : patches) {
    countPixels(patch);
  }
  if (string(params_.variance).find("bbarea") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), bBArea);
  } else if (string(params_.variance).find("ea") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), effectiveArea);
  } else if (string(params_.variance).find("ws") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), wasteSpace);
  } else {
    std::sort(patches.begin(), patches.end());
  }
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = patches[0].getSizeV0();

  for (auto &patch : patches) {
    occupancySizeU = (std::max)(occupancySizeU, patch.getSizeU0() + 1);
  }
  for (auto &patch : patches) {
    occupancySizeV = (std::max)(occupancySizeV, patch.getSizeV0() + 1);
  }

  // width = occupancySizeU * params_.occupancyResolution_;
  // height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  size_t bbU = 0;
  size_t bbV = 0;
  // For Sequential
  // patches.resize(frameIndex + 1);
  // newsort(patches);
  size_t i = 0;
  for (auto &patch : patches) {
    // For Sequential
    // if (i == frameIndex0) {
    // break;
    //}
    // if (i == 50) {
    // break;
    //}
    // printMap(patch.getOccupancy(), patch.getSizeU0(), patch.getSizeV0());
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto &occupancy = patch.getOccupancy();
    while (!locationFound) {
      for (size_t u = 0; u <= bbU && !locationFound; ++u) {
        size_t diftobb = patch.getSizeV0() > bbV ? 0 : bbV - patch.getSizeV0();
        for (size_t v = 0; v <= diftobb; ++v) {
          // std::cout << "Bounding Box insertion bbV = " << bbV << " v = " << v << " u = " << u
          //        << " size = " << patch.getSizeV0() << std::endl;
          bool canFit = true;
          for (size_t v0 = 0; v0 < patch.getSizeV0() && canFit; ++v0) {
            const size_t y = v + v0;
            for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
              const size_t x = u + u0;
              if (occupancy[v0 * patch.getSizeU0() + u0] && occupancyMap[y * occupancySizeU + x]) {
                canFit = false;
                // std::cout << "No can do\n";
                break;
              }
            }
          }
          // std::cout << "No no no\n";
          if (!canFit) {
            continue;
          }
          // std::cout << "Bounding Box insertion u0 = " << u << "v0 = " << v
          //        << "bbu =" << bbU << "bbv =" << bbV << std::endl;
          locationFound = true;
          patch.getU0() = u;
          patch.getV0() = v;
          break;
        }
      }
      if (!locationFound) {
        for (size_t u = bbU - 1; u <= occupancySizeU - patch.getSizeU0() && !locationFound; ++u) {
          for (size_t v = 0; v <= occupancySizeV - patch.getSizeV0(); ++v) {
            bool canFit = true;
            for (size_t v0 = 0; v0 < patch.getSizeV0() && canFit; ++v0) {
              const size_t y = v + v0;
              for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
                const size_t x = u + u0;
                if (occupancy[v0 * patch.getSizeU0() + u0] &&
                    occupancyMap[y * occupancySizeU + x]) {
                  canFit = false;
                  break;
                }
              }
            }
            if (!canFit) {
              continue;
            }
            locationFound = true;
            if (i < 4) {
              // std::cout << "Out insertion" << std::endl;
            }
            patch.getU0() = u;
            patch.getV0() = v;
            break;
          }
        }
        if (!locationFound) {
          // std::cout << "Out insertion INIT" << std::endl;
          bbV = bbV + patch.getSizeV0();
          occupancySizeV = bbV;
          occupancyMap.resize(occupancySizeU * occupancySizeV);
        }
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      const size_t v = patch.getV0() + v0;
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    bbU = (std::max)(bbU, patch.getU0() + patch.getSizeU0());
    bbV = (std::max)(bbV, patch.getV0() + patch.getSizeV0());

    // std::cout << "BBBBounding Box insertion u0 = " << patch.getU0() << "v0 = " << patch.getV0()
    //        << "bbu =" << bbU << "bbv =" << bbV << std::endl;

    height =
        (std::max)(height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution());
    width = (std::max)(width, (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution());
    maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    // printMap(occupancyMap, occupancySizeU, occupancySizeV);
    i++;
  }
  size_t maxHeight = 0;
  size_t maxWidth = 0;
  for (size_t u0 = 0; u0 < occupancySizeU; ++u0) {
    for (size_t v0 = 0; v0 < occupancySizeV; ++v0) {
      if (occupancyMap[v0 * occupancySizeU + u0] == 1 && v0 > maxHeight) {
        maxHeight = v0;
      }
      if (occupancyMap[v0 * occupancySizeU + u0] == 1 && u0 > maxWidth) {
        maxWidth = u0;
      }
    }
  }
  std::cout << "Max-Width = " << maxWidth << '\n';
  std::cout << "Max-Height = " << maxHeight << '\n';
  // height = (maxHeight+1) * patches[0].getOccupancyResolution();
  // width = (maxWidth+1) * patches[0].getOccupancyResolution();
  size_t e = 0;
  size_t j = 0;
  /*for (size_t u0 = maxWidth - j; u0 > maxWidth; u0 = u0 - 1) {
    for (size_t v0 = 0; v0 < occupancySizeV; ++v0) {
      occupancyMap.erase(occupancyMap.begin() + v0 * u0 + u0 - e - 1);
      e++;
    }
    e = 0;
    j++;
  }*/
  // for (size_t v0 = 0; v0 < maxHeight; ++v0) {
  // for (size_t u0 = maxWidth; u0 < occupancySizeU-1; ++u0) {
  //  occupancyMap.erase(occupancyMap.begin() + v0 * maxWidth + maxWidth+1);
  //}
  //}
  printMap(occupancyMap, occupancySizeU, occupancySizeV);
}

int PCCEncoder::computeINT(size_t U0, size_t V0, size_t sizeU0, size_t sizeV0,
                           size_t occupancySizeU, std::vector<bool> occupancy_) {
  int count = 0;
  for (size_t v0 = V0; v0 < V0 + sizeV0; ++v0) {
    for (size_t u0 = U0; u0 < U0 + sizeU0; ++u0) {
      if (occupancy_[v0 * occupancySizeU + u0]) {
        count++;
      }
    }
  }
  return count;
}

void PCCEncoder::packSOTA(PCCFrameContext &frame, size_t frameIndex) {
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  auto &patches = frame.getPatches();
  if (patches.empty()) {
    return;
  }
  for (auto &patch : patches) {
    countPixels(patch);
  }

  /*for (auto &patch : patches) {
    std::cout << "Index " << patch.getIndex() << " - BBArea " << patch.getSizeV() * patch.getSizeU()
              << " - EArea " << patch.getPixelCount() << " - Wasted "
              << patch.getSizeV() * patch.getSizeU() - patch.getPixelCount() << "\n";
  }*/
  // PCCPatch &merged = PCCPatch();
  if (string(params_.variance).find("bbarea") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), bBArea);
  } else if (string(params_.variance).find("ea") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), effectiveArea);
  } else if (string(params_.variance).find("ws") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), wasteSpace);
  } else {
    std::sort(patches.begin(), patches.end());
  }

  /*for (auto &patch : patches) {
    std::cout << "Index " << patch.getIndex() << " - BBArea "
              << patch.getSizeV0() * patch.getSizeU0() << " - EArea " << patch.getPixelCount()
              << " - Wasted " << patch.getSizeV0() * patch.getSizeU0() - patch.getPixelCount()
              << " - Per "
              << 1.0 * (patch.getSizeV0() * patch.getSizeU0() - patch.getPixelCount()) /
                     (patch.getSizeV0() * patch.getSizeU0())
              << "\n";
  }

  for (auto &patch : patches) {
    std::cout << "Patch " << patch.getIndex() << " = " << patch.getPixelCount() << "\n";
  }*/
  // mergePatches(patches[0], patches[1], merged);
  // For Sequential
  // patches.resize(frameIndex + 1);
  // newsort(patches);
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = patches[0].getSizeV0();
  for (auto &patch : patches) {
    occupancySizeU = (std::max)(occupancySizeU, patch.getSizeU0() + 1);
  }

  for (auto &patch : patches) {
    occupancySizeV = (std::max)(occupancySizeV, patch.getSizeV0() + 1);
  }

  width = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  // For Sequential
  // int i = 0;
  // size_t frameIndex0 = frameIndex + 1;
  int i = 0;
  int bestInter;
  for (auto &patch : patches) {
    // For Sequential
    // if (i == frameIndex0) {
    // break;
    //}
    if (i == 1000) {
      break;
    }
    // printMap(patch.getOccupancy(), patch.getSizeU0(), patch.getSizeV0());
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto &occupancy = patch.getOccupancy();
    bestInter = -1;
    while (!locationFound) {
      // print(patch.occupancy, patch.getSizeU0(), patch.getSizeV0());
      // print(occupancyMap, occupancySizeU, occupancySizeV);
      for (size_t v = 0; v <= occupancySizeV - patch.getSizeV0(); ++v) {
        for (size_t u = 0; u <= occupancySizeU - patch.getSizeU0(); ++u) {
          bool canFit = true;
          for (size_t v0 = 0; v0 < patch.getSizeV0() && canFit; ++v0) {
            const size_t y = v + v0;
            for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
              const size_t x = u + u0;
              if (occupancy[v0 * patch.getSizeU0() + u0] && occupancyMap[y * occupancySizeU + x]) {
                canFit = false;
                break;
              }
            }
          }
          if (!canFit) {
            continue;
          }
          int inter =
              computeINT(u, v, patch.getSizeU0(), patch.getSizeV0(), occupancySizeU, occupancyMap);
          if (i == 7) {
            // std::cout << "U0 " << u << " V0 " << v << " sizeU " << patch.getSizeU0() << " sizeV "
            //        << patch.getSizeV0() << std::endl;
            // std::cout << "bestInter " << bestInter << " inter " << inter << " iteration " << i
            //        << std::endl;
          }
          if (inter > bestInter) {
            // std::cout << "Estou Aqui \n";
            // std::cout << "bestInter " << bestInter << " inter " << inter << " iteration " << i
            //     << std::endl;
            bestInter = inter;
            locationFound = true;
            patch.getU0() = u;
            patch.getV0() = v;
          }
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize(occupancySizeU * occupancySizeV);
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      const size_t v = patch.getV0() + v0;
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height =
        (std::max)(height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution());
    width = (std::max)(width, (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution());
    maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    // printMap(occupancyMap, occupancySizeU, occupancySizeV);
    // For Sequential
    // i++;
    i++;
  }

  if (frame.getMissedPointsPatch().size() > 0) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV,
                          maxOccupancyRow);
  } else {
    printMap(occupancyMap, occupancySizeU, occupancySizeV);
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::pack(PCCFrameContext &frame, size_t frameIndex) {
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  auto &patches = frame.getPatches();
  std::vector<PCCPatch> patches2;
  if (patches.empty()) {
    return;
  }
  for (auto &patch : patches) {
    countPixels(patch);
  }

  /*for (auto &patch : patches) {
    std::cout << "Index " << patch.getIndex() << " - BBArea " << patch.getSizeV() * patch.getSizeU()
              << " - EArea " << patch.getPixelCount() << " - Wasted "
              << patch.getSizeV() * patch.getSizeU() - patch.getPixelCount() << "\n";
  }*/
  // PCCPatch &merged = PCCPatch();

  if (string(params_.variance).find("bbarea") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), bBArea);
  } else if (string(params_.variance).find("ea") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), effectiveArea);
  } else if (string(params_.variance).find("ws") != std::string::npos) {
    std::sort(patches.begin(), patches.end(), wasteSpace);
  } else {
    std::sort(patches.begin(), patches.end());
  }

  /*std::vector<pcc::PCCPatch> newpatches;
  newpatches.resize(6);
  newpatches[0] = patches[3];
  newpatches[1] = patches[1];
  newpatches[2] = patches[2];
  newpatches[3] = patches[0];
  newpatches[4] = patches[4];
  newpatches[5] = patches[5];
  patches = newpatches;*/

  /*for (auto &patch : patches) {
    std::cout << "Index " << patch.getIndex() << " - BBArea "
              << patch.getSizeV0() * patch.getSizeU0() << " - EArea " << patch.getPixelCount()
              << " - Wasted " << patch.getSizeV0() * patch.getSizeU0() - patch.getPixelCount()
              << " - Per "
              << 1.0 * (patch.getSizeV0() * patch.getSizeU0() - patch.getPixelCount()) /
                     (patch.getSizeV0() * patch.getSizeU0())
              << "\n";
  }

  for (auto &patch : patches) {
    std::cout << "Patch " << patch.getIndex() << " = " << patch.getPixelCount() << "\n";
  }*/
  // mergePatches(patches[0], patches[1], merged);
  // For Sequential
  // patches.resize(frameIndex + 1);
  // newsort(patches);
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)(params_.minimumImageHeight_ / params_.occupancyResolution_,
                                     patches[0].getSizeV0());
  for (auto &patch : patches) {
    occupancySizeU = (std::max)(occupancySizeU, patch.getSizeU0() + 1);
  }

  // patches.resize(1);

  width = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  // For Sequential
  int i = 0;
  // for (auto &patch : patches) {
  // if (patch.getViewId() == 5) {
  // patches2.push_back(patch);
  //}
  //}
  // size_t frameIndex0 = frameIndex + 1;
  for (auto &patch : patches) {
    if (true) {
      // patches2.push_back(patch);
      // For Sequential
      // if (i == frameIndex0) {
      // break;
      //}
      // printMap(patch.getOccupancy(), patch.getSizeU0(), patch.getSizeV0());
      if (i == 15) {
        /*std::cout << "Patch index " << patch.getIndex() << std::endl;
        for (int j = 0; j < patch.getColor(0).size(); j++) {
          if (j % patch.getSizeU() == 0) {
            std::cout << std::endl;
                  }
          std::cout << "C = " << static_cast<int>(patch.getColor(0)[j]) << " "
                    << static_cast<int>(patch.getColor(1)[j]) << " "
                    << static_cast<int>(patch.getColor(2)[j]) << " ";
                }*/
      }
      assert(patch.getSizeU0() <= occupancySizeU);
      assert(patch.getSizeV0() <= occupancySizeV);
      bool locationFound = false;
      auto &occupancy = patch.getOccupancy();
      while (!locationFound) {
        // print(patch.occupancy, patch.getSizeU0(), patch.getSizeV0());
        // print(occupancyMap, occupancySizeU, occupancySizeV);
        for (size_t v = 0; v <= occupancySizeV - patch.getSizeV0() && !locationFound; ++v) {
          for (size_t u = 0; u <= occupancySizeU - patch.getSizeU0(); ++u) {
            bool canFit = true;
            for (size_t v0 = 0; v0 < patch.getSizeV0() && canFit; ++v0) {
              const size_t y = v + v0;
              for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
                const size_t x = u + u0;
                if (occupancy[v0 * patch.getSizeU0() + u0] &&
                    occupancyMap[y * occupancySizeU + x]) {
                  canFit = false;
                  break;
                }
              }
            }
            if (!canFit) {
              continue;
            }
            locationFound = true;
            patch.getU0() = u;
            patch.getV0() = v;
            break;
          }
        }
        if (!locationFound) {
          occupancySizeV *= 2;
          occupancyMap.resize(occupancySizeU * occupancySizeV);
        }
      }
      for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
        const size_t v = patch.getV0() + v0;
        for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
          const size_t u = patch.getU0() + u0;
          occupancyMap[v * occupancySizeU + u] =
              occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
        }
      }

      height =
          (std::max)(height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution());
      width =
          (std::max)(width, (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution());
      maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
      // print(occupancyMap, occupancySizeU, occupancySizeV);
      // For Sequential
      i++;
    }
  }
  // patches = patches2;

  if (frame.getMissedPointsPatch().size() > 0) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV,
                          maxOccupancyRow);
  } else {
    printMap(occupancyMap, occupancySizeU, occupancySizeV);
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::packMissedPointsPatch(PCCFrameContext &frame,
                                       const std::vector<bool> &occupancyMap, size_t &width,
                                       size_t &height, size_t occupancySizeU, size_t occupancySizeV,
                                       size_t maxOccupancyRow) {
  auto &missedPointsPatch = frame.getMissedPointsPatch();
  missedPointsPatch.v0 = maxOccupancyRow;
  missedPointsPatch.u0 = 0;
  size_t missedPointsPatchBlocks =
      static_cast<size_t>(ceil(double(missedPointsPatch.size()) /
                               (params_.occupancyResolution_ * params_.occupancyResolution_)));
  size_t missedPointsPatchBlocksV =
      static_cast<size_t>(ceil(double(missedPointsPatchBlocks) / occupancySizeU));
  int occupancyRows2Add = static_cast<int>(maxOccupancyRow + missedPointsPatchBlocksV -
                                           height / params_.occupancyResolution_);
  occupancyRows2Add = occupancyRows2Add > 0 ? occupancyRows2Add : 0;
  occupancySizeV += occupancyRows2Add;
  height += occupancyRows2Add * params_.occupancyResolution_;

  vector<bool> newOccupancyMap;
  newOccupancyMap.resize(occupancySizeU * occupancySizeV);

  size_t missedPointsPatchBlocksU =
      static_cast<size_t>(ceil(double(missedPointsPatchBlocks) / missedPointsPatchBlocksV));
  missedPointsPatch.sizeV = missedPointsPatchBlocksV * params_.occupancyResolution_;
  missedPointsPatch.sizeU =
      static_cast<size_t>(ceil(double(missedPointsPatch.size()) / missedPointsPatch.sizeV));
  missedPointsPatch.sizeV0 = missedPointsPatchBlocksV;
  missedPointsPatch.sizeU0 = missedPointsPatchBlocksU;

  const int16_t infiniteValue = (std::numeric_limits<int16_t>::max)();
  missedPointsPatch.resize(missedPointsPatch.sizeU * missedPointsPatch.sizeV, infiniteValue);
  std::vector<bool> &missedPointPatchOccupancy = missedPointsPatch.occupancy;
  missedPointPatchOccupancy.resize(missedPointsPatch.sizeU0 * missedPointsPatch.sizeV0, false);

  for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
    for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
      const size_t p = v * missedPointsPatch.sizeU + u;
      if (missedPointsPatch.x[p] < infiniteValue) {
        const size_t u0 = u / missedPointsPatch.occupancyResolution;
        const size_t v0 = v / missedPointsPatch.occupancyResolution;
        const size_t p0 = v0 * missedPointsPatch.sizeU0 + u0;
        assert(u0 >= 0 && u0 < missedPointsPatch.sizeU0);
        assert(v0 >= 0 && v0 < missedPointsPatch.sizeV0);
        missedPointPatchOccupancy[p0] = true;
      }
    }
  }

  for (size_t v0 = 0; v0 < missedPointsPatch.sizeV0; ++v0) {
    const size_t v = missedPointsPatch.v0 + v0;
    for (size_t u0 = 0; u0 < missedPointsPatch.sizeU0; ++u0) {
      const size_t u = missedPointsPatch.u0 + u0;
      newOccupancyMap[v * occupancySizeU + u] =
          newOccupancyMap[v * occupancySizeU + u] ||
          missedPointPatchOccupancy[v0 * missedPointsPatch.sizeU0 + u0];
    }
  }

  // printMap(newOccupancyMap, occupancySizeU, occupancySizeV);
  for (size_t v = 0; v < maxOccupancyRow; ++v) {
    for (size_t u = 0; u < occupancySizeU; ++u) {
      const size_t p = v * occupancySizeU + u;
      newOccupancyMap[p] = occupancyMap[p];
    }
  }

  printMap(newOccupancyMap, occupancySizeU, occupancySizeV);
}

bool PCCEncoder::generateGeometryVideo(const PCCPointSet3 &source, PCCFrameContext &frame,
                                       const PCCPatchSegmenter3Parameters segmenterParams,
                                       PCCVideoGeometry &videoGeometry, PCCFrameContext &prevFrame,
                                       size_t frameIndex) {
  if (!source.getPointCount()) {
    return false;
  }

  auto &patches = frame.getPatches();
  patches.reserve(256);
  PCCPatchSegmenter3 segmenter;
  // segmenter.setNbThread(params_.nbThread_);
  // segmenter.compute(source, segmenterParams, patches);
  if (params_.losslessGeo_) {
    generateMissedPointsPatch(source, frame);
    sortMissedPointsPatch(frame);
  }
  /*std::ofstream fileoutp("spatches23.txt");
  fileoutp<<patches.size()<<"\n";
  for (const auto &patch : patches ) {
    fileoutp<<patch;
  }
  fileoutp.close();*/
  std::ifstream fileinp;
  if (string(params_.compressedStreamPath_).find("26") != std::string::npos) {
    fileinp = std::ifstream("patches26.txt");
  } else if (string(params_.compressedStreamPath_).find("25") != std::string::npos) {
    fileinp = std::ifstream("patches25.txt");
  } else if (string(params_.compressedStreamPath_).find("24") != std::string::npos) {
    fileinp = std::ifstream("patches24.txt");
  } else if (string(params_.compressedStreamPath_).find("23") != std::string::npos) {
    fileinp = std::ifstream("spatches23.txt");
  } else {
    fileinp = std::ifstream("patches27.txt");
  }
  std::vector<PCCPatch> patchesRec;
  size_t numberOfPatches;
  string line;
  std::getline(fileinp, line);
  numberOfPatches = std::stoi(line);
  for (int i = 0; i < numberOfPatches; i++) {
    PCCPatch patch;
    fileinp >> patch;
    patchesRec.push_back(patch);
  }

  fileinp.close();

  /*std::ofstream fileoutp2("patches2.txt");
  for (const auto &patch : patchesRec ) {
    fileoutp2<<patch;
  }

  fileoutp2.close();*/
  patches = patchesRec;
  packBestBuddies(frame, frameIndex);
  if ((frameIndex == 0) || (!params_.constrainedPack_)) {
    if (string(params_.variance).find("SOTA") != std::string::npos) {
      packSOTA(frame, frameIndex);
    } else if (string(params_.variance).find("BBC") != std::string::npos) {
      packBB(frame, frameIndex);
    } else {
      // pack(frame, frameIndex);
    }
  } else {
    // spatialConsistencyPack(frame , prevFrame);
    pack(frame, frameIndex);
  }
  return true;
}

void PCCEncoder::generateOccupancyMap(PCCFrameContext &frame) {
  auto &occupancyMap = frame.getOccupancyMap();
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  occupancyMap.resize(width * height, 0);
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  for (const auto &patch : frame.getPatches()) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(0)[p];
        if (d < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < width && y < height);
          occupancyMap[x + y * width] = 1;
        }
      }
    }
  }
  auto &missedPointsPatch = frame.getMissedPointsPatch();
  const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
  const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
  if (missedPointsPatch.size()) {
    for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
      for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
        const size_t p = v * missedPointsPatch.sizeU + u;
        if (missedPointsPatch.x[p] < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < width && y < height);
          occupancyMap[x + y * width] = 1;
        }
      }
    }
  }
}

void PCCEncoder::generateIntraImage(PCCFrameContext &frame, const size_t depthIndex,
                                    PCCImageGeometry &image) {
  auto &width = frame.getWidth();
  auto &height = frame.getHeight();
  image.resize(width, height);
  image.set(0);
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  size_t maxDepth = 0;
  for (const auto &patch : frame.getPatches()) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(depthIndex)[p];
        if (d < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < width && y < height);
          image.setValue(0, x, y, uint16_t(d));
          maxDepth = (std::max)(maxDepth, patch.getSizeD());
        }
      }
    }
  }
  std::cout << "maxDepth " << maxDepth << std::endl;
  if (maxDepth > 255) {
    std::cout << "Error: maxDepth > 255" << maxDepth << std::endl;
    exit(-1);
  }
  auto &missedPointsPatch = frame.getMissedPointsPatch();
  const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
  const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
  if (missedPointsPatch.size()) {
    for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
      for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
        const size_t p = v * missedPointsPatch.sizeU + u;
        if (missedPointsPatch.x[p] < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < width && y < height);
          image.setValue(0, x, y, uint16_t(missedPointsPatch.x[p]));
          if (params_.losslessGeo444_) {
            image.setValue(1, x, y, uint16_t(missedPointsPatch.y[p]));
            image.setValue(2, x, y, uint16_t(missedPointsPatch.z[p]));
          }
        }
      }
    }
  }
}

bool PCCEncoder::predictGeometryFrame(PCCFrameContext &frame, const PCCImageGeometry &reference,
                                      PCCImageGeometry &image) {
  assert(reference.getWidth() == image.getWidth());
  assert(reference.getHeight() == image.getHeight());
  const size_t refWidth = reference.getWidth();
  const size_t refHeight = reference.getHeight();
  auto &occupancyMap = frame.getOccupancyMap();
  for (size_t y = 0; y < refHeight; ++y) {
    for (size_t x = 0; x < refWidth; ++x) {
      const size_t pos1 = y * refWidth + x;
      if (occupancyMap[pos1] == 1) {
        for (size_t c = 0; c < 3; ++c) {
          const uint16_t value1 = static_cast<uint16_t>(image.getValue(c, x, y));
          const uint16_t value0 = static_cast<uint16_t>(reference.getValue(c, x, y));
          // assert(value0 <= value1);
          int_least32_t delta = (int_least32_t)value1 - (int_least32_t)value0;
          if (delta < 0) {
            delta = 0;
          }
          if (delta > 9) {
            delta = 9;
          }
          // assert(delta < 10);
          image.setValue(c, x, y, (uint8_t)delta);
        }
      }
    }
  }
  return true;
}

void PCCEncoder::generateMissedPointsPatch(const PCCPointSet3 &source, PCCFrameContext &frame) {
  auto &patches = frame.getPatches();
  auto &missedPointsPatch = frame.getMissedPointsPatch();
  missedPointsPatch.occupancy.resize(0);
  PCCPointSet3 pointsToBeProjected;
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  for (const auto &patch : patches) {
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const size_t depth0 = patch.getDepth(0)[p];
        if (depth0 < infiniteDepth) {
          PCCPoint3D point0;
          point0[patch.getNormalAxis()] = double(depth0) + patch.getD1();
          point0[patch.getTangentAxis()] = double(u) + patch.getU1();
          point0[patch.getBitangentAxis()] = double(v) + patch.getV1();
          pointsToBeProjected.addPoint(point0);
          const size_t depth1 = patch.getDepth(1)[p];
          if (depth1 > depth0) {
            PCCPoint3D point1;
            point1[patch.getNormalAxis()] = double(depth1) + patch.getD1();
            point1[patch.getTangentAxis()] = double(u) + patch.getU1();
            point1[patch.getBitangentAxis()] = double(v) + patch.getV1();
            pointsToBeProjected.addPoint(point1);
          }
        }
      }
    }
  }
  PCCStaticKdTree3 kdtreeMissedPoints;
  kdtreeMissedPoints.build(pointsToBeProjected);
  PCCPointDistInfo nNeighbor;
  PCCNNResult result = {&nNeighbor, 0};
  PCCNNQuery3 query = {PCCVector3D(0.0), (std::numeric_limits<double>::max)(), 1};
  std::vector<size_t> missedPoints;
  missedPoints.resize(0);
  for (size_t i = 0; i < source.getPointCount(); ++i) {
    query.point = source[i];
    kdtreeMissedPoints.findNearestNeighbors(query, result);
    const double dist2 = result.neighbors[0].dist2;
    if (dist2 > 0.0) {
      missedPoints.push_back(i);
    }
  }
  missedPointsPatch.occupancyResolution = params_.occupancyResolution_;
  if (params_.losslessGeo444_) {
    missedPointsPatch.resize(missedPoints.size());
    for (auto i = 0; i < missedPoints.size(); ++i) {
      const PCCPoint3D missedPoint = source[missedPoints[i]];
      missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
      missedPointsPatch.y[i] = static_cast<uint16_t>(missedPoint.y());
      missedPointsPatch.z[i] = static_cast<uint16_t>(missedPoint.z());
    }
  } else {
    const size_t mps = missedPoints.size();
    missedPointsPatch.resize(3 * mps);
    const int16_t infiniteValue = (std::numeric_limits<int16_t>::max)();
    for (auto i = 0; i < mps; ++i) {
      const PCCPoint3D missedPoint = source[missedPoints[i]];
      missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
      missedPointsPatch.x[mps + i] = static_cast<uint16_t>(missedPoint.y());
      missedPointsPatch.x[2 * mps + i] = static_cast<uint16_t>(missedPoint.z());
      missedPointsPatch.y[i] = infiniteValue;
      missedPointsPatch.y[mps + i] = infiniteValue;
      missedPointsPatch.y[2 * mps + i] = infiniteValue;
      missedPointsPatch.z[i] = infiniteValue;
      missedPointsPatch.z[mps + i] = infiniteValue;
      missedPointsPatch.z[2 * mps + i] = infiniteValue;
    }
  }
}

void PCCEncoder::sortMissedPointsPatch(PCCFrameContext &frame) {
  auto &missedPointsPatch = frame.getMissedPointsPatch();
  const size_t maxNeighborCount = 5;
  const size_t neighborSearchRadius = 5;
  size_t missedPointCount =
      params_.losslessGeo444_ ? missedPointsPatch.size() : missedPointsPatch.size() / 3;

  vector<size_t> sortIdx;
  sortIdx.reserve(missedPointCount);
  PCCPointSet3 missedPointSet;
  missedPointSet.resize(missedPointCount);

  for (size_t i = 0; i < missedPointCount; i++) {
    missedPointSet[i] =
        params_.losslessGeo444_
            ? PCCPoint3D(missedPointsPatch.x[i], missedPointsPatch.y[i], missedPointsPatch.z[i])
            : PCCPoint3D(missedPointsPatch.x[i], missedPointsPatch.x[i + missedPointCount],
                         missedPointsPatch.x[i + missedPointCount * 2]);
  }
  PCCStaticKdTree3 kdtreeMissedPointSet;
  kdtreeMissedPointSet.build(missedPointSet);
  PCCPointDistInfo nNeighbor1[maxNeighborCount];
  PCCNNResult result = {nNeighbor1, 0};
  PCCNNQuery3 query = {PCCVector3D(0.0), neighborSearchRadius, maxNeighborCount};
  std::vector<size_t> fifo;
  fifo.reserve(missedPointCount);
  std::vector<bool> flags(missedPointCount, true);

  for (size_t i = 0; i < missedPointCount; i++) {
    if (flags[i]) {
      flags[i] = false;
      sortIdx.push_back(i);
      fifo.push_back(i);
      while (!fifo.empty()) {
        const size_t currentIdx = fifo.back();
        fifo.pop_back();
        query.point = missedPointSet[currentIdx];
        kdtreeMissedPointSet.findNearestNeighbors(query, result);
        for (size_t j = 0; j < result.resultCount; j++) {
          size_t n = result.neighbors[j].index;
          if (flags[n]) {
            flags[n] = false;
            sortIdx.push_back(n);
            fifo.push_back(n);
          }
        }
      }
    }
  }
  for (size_t i = 0; i < missedPointCount; ++i) {
    const PCCPoint3D missedPoint = missedPointSet[sortIdx[i]];
    if (params_.losslessGeo444_) {
      missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
      missedPointsPatch.y[i] = static_cast<uint16_t>(missedPoint.y());
      missedPointsPatch.z[i] = static_cast<uint16_t>(missedPoint.z());
    } else {
      missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
      missedPointsPatch.x[i + missedPointCount] = static_cast<uint16_t>(missedPoint.y());
      missedPointsPatch.x[i + missedPointCount * 2] = static_cast<uint16_t>(missedPoint.z());
    }
  }
}
bool PCCEncoder::generateGeometryVideo(const PCCGroupOfFrames &sources, PCCContext &context) {
  bool res = true;
  PCCPatchSegmenter3Parameters segmenterParams;
  segmenterParams.nnNormalEstimation = params_.nnNormalEstimation_;
  segmenterParams.maxNNCountRefineSegmentation = params_.maxNNCountRefineSegmentation_;
  segmenterParams.iterationCountRefineSegmentation = params_.iterationCountRefineSegmentation_;
  segmenterParams.occupancyResolution = params_.occupancyResolution_;
  segmenterParams.minPointCountPerCCPatchSegmentation =
      params_.minPointCountPerCCPatchSegmentation_;
  segmenterParams.maxNNCountPatchSegmentation = params_.maxNNCountPatchSegmentation_;
  segmenterParams.surfaceThickness = params_.surfaceThickness_;
  segmenterParams.maxAllowedDepth = params_.maxAllowedDepth_;
  segmenterParams.maxAllowedDist2MissedPointsDetection =
      params_.maxAllowedDist2MissedPointsDetection_;
  segmenterParams.maxAllowedDist2MissedPointsSelection =
      params_.maxAllowedDist2MissedPointsSelection_;
  segmenterParams.lambdaRefineSegmentation = params_.lambdaRefineSegmentation_;
  auto &videoGeometry = context.getVideoGeometry();
  auto &frames = context.getFrames();

  for (size_t i = 0; i < frames.size(); i++) {
    size_t preIndex = i > 0 ? (i - 1) : 0;
    if (!generateGeometryVideo(sources[i], frames[i], segmenterParams, videoGeometry,
                               frames[preIndex], i)) {
      res = false;
      break;
    }
  }
  return res;
}

bool PCCEncoder::resizeGeometryVideo(PCCContext &context) {
  size_t maxWidth = 0, maxHeight = 0;
  auto &videoGeometry = context.getVideoGeometry();
  for (auto &frame : context.getFrames()) {
    maxWidth = (std::max)(maxWidth, frame.getWidth());
    maxHeight = (std::max)(maxHeight, frame.getHeight());
  }
  for (auto &frame : context.getFrames()) {
    frame.getWidth() = maxWidth;
    frame.getHeight() = maxHeight;
    frame.getOccupancyMap().resize((maxWidth / params_.occupancyResolution_) *
                                   (maxHeight / params_.occupancyResolution_));
  }
  std::cout << "Max-Width = " << maxWidth << '\n';
  std::cout << "Max-Height = " << maxHeight << '\n';
  return true;
}

bool PCCEncoder::dilateGeometryVideo(PCCContext &context) {
  auto &videoGeometry = context.getVideoGeometry();
  auto &videoGeometryD1 = context.getVideoGeometryD1();
  for (auto &frame : context.getFrames()) {
    generateOccupancyMap(frame);
    const size_t shift = videoGeometry.getFrameCount();

    if (!params_.absoluteD1_) {
      videoGeometry.resize(shift + 1);
      videoGeometryD1.resize(shift + 1);
      auto &frame1 = videoGeometry.getFrame(shift);
      generateIntraImage(frame, 0, frame1);
      auto &frame2 = videoGeometryD1.getFrame(shift);
      generateIntraImage(frame, 1, frame2);
      dilate(frame, videoGeometry.getFrame(shift), videoGeometry.getFrame(shift), 0);
    } else {
      videoGeometry.resize(shift + 2);
      for (size_t f = 0; f < 2; ++f) {
        auto &frame1 = videoGeometry.getFrame(shift + f);
        generateIntraImage(frame, f, frame1);
        dilate(frame, videoGeometry.getFrame(shift + f), videoGeometry.getFrame(shift + f), 0);
      }
    }
  }
  return true;
}

template <typename T>
void PCCEncoder::dilate(PCCFrameContext &frame, PCCImage<T, 3> &image, PCCImage<T, 3> &simage,
                        bool texture, const PCCImage<T, 3> *reference) {
  auto occupancyMapTemp = frame.getOccupancyMap();
  const size_t pixelBlockCount = params_.occupancyResolution_ * params_.occupancyResolution_;
  const size_t occupancyMapSizeU = image.getWidth() / params_.occupancyResolution_;
  const size_t occupancyMapSizeV = image.getHeight() / params_.occupancyResolution_;
  const int64_t neighbors[4][2] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  const size_t MAX_OCCUPANCY_RESOLUTION = 64;
  assert(params_.occupancyResolution_ <= MAX_OCCUPANCY_RESOLUTION);
  size_t count[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];
  PCCVector3<int32_t> values[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];

  std::vector<PCCVector3<int32_t>> consecutivePatches;
  // foravgpadd
  size_t yavg = 0;
  size_t uavg = 0;
  size_t vavg = 0;
  size_t npixels = 0;
  for (size_t v1 = 0; v1 < occupancyMapSizeV; ++v1) {
    const int64_t v0 = v1 * params_.occupancyResolution_;
    for (size_t u1 = 0; u1 < occupancyMapSizeU; ++u1) {
      const int64_t u0 = u1 * params_.occupancyResolution_;
      size_t nonZeroPixelCount = 0;
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
          const size_t location0 = y0 * image.getWidth() + x0;
          nonZeroPixelCount += (occupancyMapTemp[location0] == 1);
          // foravgpadd
          if (occupancyMapTemp[location0] == 1) {
            yavg = yavg + image.getValue(0, u0 + u2, v0 + v2);
            uavg = uavg + image.getValue(1, u0 + u2, v0 + v2);
            vavg = vavg + image.getValue(2, u0 + u2, v0 + v2);
          }
        }
      }
      // for avgpadd
      npixels = npixels + nonZeroPixelCount;
      if (!nonZeroPixelCount) {
        if (reference) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              image.setValue(0, x0, y0, reference->getValue(0, x0, y0));
              image.setValue(1, x0, y0, reference->getValue(1, x0, y0));
              image.setValue(2, x0, y0, reference->getValue(2, x0, y0));
              if (texture) {
                simage.setValue(0, x0, y0, 100);
                simage.setValue(1, x0, y0, 150);
                simage.setValue(2, x0, y0, 60);
              }
            }
          }
        } else if (u1 > 0) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert(x0 > 0);
              const size_t x1 = x0 - 1;
              image.setValue(0, x0, y0, image.getValue(0, x1, y0));
              image.setValue(1, x0, y0, image.getValue(1, x1, y0));
              image.setValue(2, x0, y0, image.getValue(2, x1, y0));
              if (texture) {
                simage.setValue(0, x0, y0, 0);  // 100);
                simage.setValue(1, x0, y0, 0);  // 150);
                simage.setValue(2, x0, y0, 0);  // 60);
              }
            }
          }
        } else if (v1 > 0) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert(y0 > 0);
              const size_t y1 = y0 - 1;
              image.setValue(0, x0, y0, image.getValue(0, x0, y1));
              image.setValue(1, x0, y0, image.getValue(1, x0, y1));
              image.setValue(2, x0, y0, image.getValue(2, x0, y1));
              if (texture) {
                simage.setValue(0, x0, y0, 0);  // 100);
                simage.setValue(1, x0, y0, 0);  // 150);
                simage.setValue(2, x0, y0, 0);  // 60);
              }
            }
          }
        }
        continue;
      }
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          values[v2][u2] = 0;
          count[v2][u2] = 0UL;
        }
      }
      uint32_t iteration = 1;
      while (nonZeroPixelCount < pixelBlockCount) {
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            const int64_t x0 = u0 + u2;
            const int64_t y0 = v0 + v2;
            assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
            const size_t location0 = y0 * image.getWidth() + x0;
            if (occupancyMapTemp[location0] == iteration) {
              for (size_t n = 0; n < 4; ++n) {
                const int64_t x1 = x0 + neighbors[n][0];
                const int64_t y1 = y0 + neighbors[n][1];
                const size_t location1 = y1 * image.getWidth() + x1;
                if (x1 >= u0 && x1 < int64_t(u0 + params_.occupancyResolution_) && y1 >= v0 &&
                    y1 < int64_t(v0 + params_.occupancyResolution_) &&
                    occupancyMapTemp[location1] == 0) {
                  const int64_t u3 = u2 + neighbors[n][0];
                  const int64_t v3 = v2 + neighbors[n][1];
                  assert(u3 >= 0 && u3 < int64_t(params_.occupancyResolution_));
                  assert(v3 >= 0 && v3 < int64_t(params_.occupancyResolution_));
                  for (size_t k = 0; k < 3; ++k) {
                    values[v3][u3][k] += image.getValue(k, x0, y0);
                  }
                  ++count[v3][u3];
                }
              }
            }
          }
        }
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            if (count[v2][u2]) {
              ++nonZeroPixelCount;
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              const size_t location0 = y0 * image.getWidth() + x0;
              const size_t c = count[v2][u2];
              const size_t c2 = c / 2;
              occupancyMapTemp[location0] = iteration + 1;
              for (size_t k = 0; k < 3; ++k) {
                image.setValue(k, x0, y0, T((values[v2][u2][k] + c2) / c));
              }
              if (texture) {
                simage.setValue(0, x0, y0, 0);  // 100);
                simage.setValue(1, x0, y0, 0);  // 150);
                simage.setValue(2, x0, y0, 0);  // 60);
                // for (size_t k = 0; k < 3; ++k) {
                // simage.setValue(k, x0, y0, T((values[v2][u2][k] + c2) / c));
                //}
              }
              values[v2][u2] = 0;
              count[v2][u2] = 0UL;
            }
          }
        }
        ++iteration;
      }
    }
  }
  size_t ufirst;
  size_t vfirst;
  size_t sequence = 0;
  for (size_t v1 = 0; v1 < occupancyMapSizeV; ++v1) {
    const int64_t v0 = v1 * params_.occupancyResolution_;
    for (size_t u1 = 0; u1 < occupancyMapSizeU; ++u1) {
      const int64_t u0 = u1 * params_.occupancyResolution_;
      size_t nonZeroPixelCount = 0;
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
          const size_t location0 = y0 * image.getWidth() + x0;
          nonZeroPixelCount += (occupancyMapTemp[location0] == 1);
        }
      }
      // std::cout << "Estou Aqui" << std::endl;
      if (!nonZeroPixelCount) {
        if (u1 == occupancyMapSizeU - 1) {
          if (sequence != 0) {
            // std::cout << "Estou Aqui2" << std::endl;
            consecutivePatches.resize(consecutivePatches.size() + 1);
            consecutivePatches[consecutivePatches.size() - 1][0] = ufirst;
            consecutivePatches[consecutivePatches.size() - 1][1] = vfirst;
            consecutivePatches[consecutivePatches.size() - 1][2] = sequence + 1;
          } else {
            consecutivePatches.resize(consecutivePatches.size() + 1);
            consecutivePatches[consecutivePatches.size() - 1][0] = u1;
            consecutivePatches[consecutivePatches.size() - 1][1] = v1;
            consecutivePatches[consecutivePatches.size() - 1][2] = 1;
          }
          sequence = 0;
        } else {
          if (sequence == 0) {
            // std::cout << "Estou Aqui" << std::endl;
            ufirst = u1;
            vfirst = v1;
          }
          sequence = sequence + 1;
        }
      } else {
        if (sequence != 0) {
          consecutivePatches.resize(consecutivePatches.size() + 1);
          consecutivePatches[consecutivePatches.size() - 1][0] = ufirst;
          consecutivePatches[consecutivePatches.size() - 1][1] = vfirst;
          consecutivePatches[consecutivePatches.size() - 1][2] = sequence;
        }
        sequence = 0;
      }
    }
  }

  // for avgpadding
  /*for (size_t v1 = 0; v1 < occupancyMapSizeV; ++v1) {
    const int64_t v0 = v1 * params_.occupancyResolution_;
    for (size_t u1 = 0; u1 < occupancyMapSizeU; ++u1) {
      const int64_t u0 = u1 * params_.occupancyResolution_;
      size_t nonZeroPixelCount = 0;
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
          const size_t location0 = y0 * image.getWidth() + x0;
          nonZeroPixelCount += (occupancyMapTemp[location0] == 1);
        }
      }
      if (!nonZeroPixelCount) {
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            const int64_t x0 = u0 + u2;
            const int64_t y0 = v0 + v2;
            if (texture) {
              image.setValue(0, x0, y0, int(yavg / npixels));
              image.setValue(1, x0, y0, int(uavg / npixels));
              image.setValue(2, x0, y0, int(vavg / npixels));
                        }
          }
        }
      }
    }
  }*/

  // std::cout << "Estou Aqui" << std::endl;
  // weighted line
  if (string(params_.variancepadd).find("WML") != std::string::npos) {
    if (texture) {
      for (int k = 0; k < consecutivePatches.size(); k++) {
        std::cout << consecutivePatches[k][0] << " " << consecutivePatches[k][1] << " "
                  << consecutivePatches[k][2] << std::endl;
        if (consecutivePatches[k][0] + consecutivePatches[k][2] == occupancyMapSizeU) {
        } else if (consecutivePatches[k][0] == 0) {
          for (size_t v2 = consecutivePatches[k][1] * params_.occupancyResolution_;
               v2 < (consecutivePatches[k][1] + 1) * params_.occupancyResolution_; ++v2) {
            for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_;
                 u2 < (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                          params_.occupancyResolution_;
                 ++u2) {
              image.setValue(0, u2, v2,
                             image.getValue(0,
                                            (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                                params_.occupancyResolution_,
                                            v2));
              image.setValue(1, u2, v2,
                             image.getValue(1,
                                            (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                                params_.occupancyResolution_,
                                            v2));
              image.setValue(2, u2, v2,
                             image.getValue(2,
                                            (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                                params_.occupancyResolution_,
                                            v2));
            }
          }
        } else {
          for (size_t v2 = consecutivePatches[k][1] * params_.occupancyResolution_;
               v2 < (consecutivePatches[k][1] + 1) * params_.occupancyResolution_; ++v2) {
            for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_;
                 u2 < (consecutivePatches[k][0] * params_.occupancyResolution_) +
                          ((consecutivePatches[k][2] * params_.occupancyResolution_) / 2);
                 ++u2) {
              float weight =
                  float(u2 - ((consecutivePatches[k][0] * params_.occupancyResolution_) - 1)) /
                  float(consecutivePatches[k][2] * params_.occupancyResolution_);
              // std::cout << weight << std::endl;
              weight = 1 - weight;
              float wavg =
                  weight *
                      (image.getValue(
                          0, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1, v2)) +
                  (1 - weight) *
                      (image.getValue(0,
                                      (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                          params_.occupancyResolution_,
                                      v2));
              // std::cout << wavg << " ";
              image.setValue(0, u2, v2, int(wavg));
              wavg = weight * (image.getValue(
                                  1, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2)) +
                     (1 - weight) *
                         (image.getValue(1,
                                         (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                             params_.occupancyResolution_,
                                         v2));
              // std::cout << wavg << " ";
              image.setValue(1, u2, v2, int(wavg));
              wavg = weight * (image.getValue(
                                  2, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2)) +
                     (1 - weight) *
                         (image.getValue(2,
                                         (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                             params_.occupancyResolution_,
                                         v2));
              // std::cout << weight << std::endl;
              image.setValue(2, u2, v2, int(wavg));
            }
            for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_ +
                             ((consecutivePatches[k][2] * params_.occupancyResolution_) / 2);
                 u2 < (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                          params_.occupancyResolution_;
                 ++u2) {
              float weight = float(((consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                    params_.occupancyResolution_) -
                                   u2) /
                             float(consecutivePatches[k][2] * params_.occupancyResolution_);
              float wavg =
                  weight *
                      (image.getValue(
                          0, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1, v2)) +
                  (1 - weight) *
                      (image.getValue(0,
                                      (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                          params_.occupancyResolution_,
                                      v2));
              image.setValue(0, u2, v2, int(wavg));
              wavg = weight * (image.getValue(
                                  1, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2)) +
                     (1 - weight) *
                         (image.getValue(1,
                                         (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                             params_.occupancyResolution_,
                                         v2));
              image.setValue(1, u2, v2, int(wavg));
              wavg = weight * (image.getValue(
                                  2, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2)) +
                     (1 - weight) *
                         (image.getValue(2,
                                         (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                             params_.occupancyResolution_,
                                         v2));
              image.setValue(2, u2, v2, int(wavg));
            }
          }
        }
      }
    }
  }
  // non-weighted line
  else if (string(params_.variancepadd).find("ML") != std::string::npos) {
    if (texture) {
      for (int k = 0; k < consecutivePatches.size(); k++) {
        // std::cout << consecutivePatches[k][0] << " " << consecutivePatches[k][1] << " "
        //        << consecutivePatches[k][2] << std::endl;
        if (consecutivePatches[k][0] + consecutivePatches[k][2] == occupancyMapSizeU) {
        } else if (consecutivePatches[k][0] == 0) {
          for (size_t v2 = consecutivePatches[k][1] * params_.occupancyResolution_;
               v2 < (consecutivePatches[k][1] + 1) * params_.occupancyResolution_; ++v2) {
            for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_;
                 u2 < (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                          params_.occupancyResolution_;
                 ++u2) {
              image.setValue(0, u2, v2,
                             image.getValue(0,
                                            (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                                params_.occupancyResolution_,
                                            v2));
              image.setValue(1, u2, v2,
                             image.getValue(1,
                                            (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                                params_.occupancyResolution_,
                                            v2));
              image.setValue(2, u2, v2,
                             image.getValue(2,
                                            (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                                params_.occupancyResolution_,
                                            v2));
            }
          }
        } else {
          for (size_t v2 = consecutivePatches[k][1] * params_.occupancyResolution_;
               v2 < (consecutivePatches[k][1] + 1) * params_.occupancyResolution_; ++v2) {
            for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_;
                 u2 < (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                          params_.occupancyResolution_;
                 ++u2) {
              image.setValue(
                  0, u2, v2,
                  (image.getValue(0, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2) +
                   image.getValue(0,
                                  (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                      params_.occupancyResolution_,
                                  v2)) /
                      2);
              image.setValue(
                  1, u2, v2,
                  (image.getValue(1, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2) +
                   image.getValue(1,
                                  (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                      params_.occupancyResolution_,
                                  v2)) /
                      2);
              image.setValue(
                  2, u2, v2,
                  (image.getValue(2, (consecutivePatches[k][0] * params_.occupancyResolution_) - 1,
                                  v2) +
                   image.getValue(2,
                                  (consecutivePatches[k][0] + consecutivePatches[k][2]) *
                                      params_.occupancyResolution_,
                                  v2)) /
                      2);
            }
          }
        }
      }
    }
  }
  // 4 neighbors weighted
  if (string(params_.variancepadd).find("WM4") != std::string::npos) {
    if (texture) {
      for (int k = 0; k < consecutivePatches.size(); k++) {
        for (size_t v2 = consecutivePatches[k][1] * params_.occupancyResolution_;
             v2 < (consecutivePatches[k][1] + 1) * params_.occupancyResolution_; ++v2) {
          for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_;
               u2 <
               (consecutivePatches[k][0] + consecutivePatches[k][2]) * params_.occupancyResolution_;
               ++u2) {
            size_t rleft, gleft, bleft, dleft, isleft;
            size_t rup, gup, bup, dup, isup;
            size_t rright, gright, bright, dright, isright;
            size_t rdown, gdown, bdown, ddown, isdown;

            // left search

            if (u2 != 0) {
              rleft = image.getValue(0, u2 - 1, v2);
              gleft = image.getValue(1, u2 - 1, v2);
              bleft = image.getValue(2, u2 - 1, v2);
              dleft = 1;
              isleft = 1;
            } else {
              isleft = 0;
            }
            // up search

            if (v2 != 0) {
              rup = image.getValue(0, u2, v2 - 1);
              gup = image.getValue(1, u2, v2 - 1);
              bup = image.getValue(2, u2, v2 - 1);
              dup = 1;
              isup = 1;
            } else {
              isup = 0;
            }

            // right search

            if (u2 != (occupancyMapSizeU * params_.occupancyResolution_) - 1) {
              isright = 0;
              for (size_t u3 = u2 + 1; u3 < occupancyMapSizeU * params_.occupancyResolution_;
                   u3++) {
                const size_t location3 =
                    (v2 * occupancyMapSizeU * params_.occupancyResolution_) + u3;
                if (occupancyMapTemp[location3] == 1) {
                  rright = image.getValue(0, u3, v2);
                  gright = image.getValue(1, u3, v2);
                  bright = image.getValue(2, u3, v2);
                  dright = u3 - u2;
                  isright = 1;
                  // std::cout << rright << gright << bright << dright << std::endl;
                  break;
                }
              }
            } else {
              isright = 0;
            }

            // down search

            if (v2 != (occupancyMapSizeV * params_.occupancyResolution_) - 1) {
              isdown = 0;
              for (size_t v3 = v2 + 1; v3 < occupancyMapSizeV * params_.occupancyResolution_;
                   v3++) {
                const size_t location3 =
                    (v3 * occupancyMapSizeU * params_.occupancyResolution_) + u2;
                if (occupancyMapTemp[location3] == 1) {
                  rdown = image.getValue(0, u2, v3);
                  gdown = image.getValue(1, u2, v3);
                  bdown = image.getValue(2, u2, v3);
                  ddown = v3 - v2;
                  isdown = 1;
                  break;
                }
              }
            } else {
              isdown = 0;
            }
            float rnum = 0;
            float gnum = 0;
            float bnum = 0;
            float den = 0;
            if (isleft) {
              rnum = rnum + (float(rleft) / float(dleft));
              gnum = gnum + (float(gleft) / float(dleft));
              bnum = bnum + (float(bleft) / float(dleft));
              den = den + (1.0 / float(dleft));
            }
            if (isup) {
              rnum = rnum + (float(rup) / float(dup));
              gnum = gnum + (float(gup) / float(dup));
              bnum = bnum + (float(bup) / float(dup));
              den = den + (1.0 / float(dup));
            }
            if (isright) {
              rnum = rnum + (float(rright) / float(dright));
              gnum = gnum + (float(gright) / float(dright));
              bnum = bnum + (float(bright) / float(dright));
              den = den + (1.0 / float(dright));
            }
            if (isdown) {
              rnum = rnum + (float(rdown) / float(ddown));
              gnum = gnum + (float(gdown) / float(ddown));
              bnum = bnum + (float(bdown) / float(ddown));
              den = den + (1.0 / float(ddown));
            }
            // std::cout << int(rnum/den) << " " << int(gnum/den) << " " << int(bnum/den) <<
            // std::endl;
            image.setValue(0, u2, v2, int(rnum / den));
            image.setValue(1, u2, v2, int(gnum / den));
            image.setValue(2, u2, v2, int(bnum / den));
          }
        }
      }
    }
  }
  // non-weighted 4 neighbors
  else if (string(params_.variancepadd).find("M4") != std::string::npos) {
    if (texture) {
      for (int k = 0; k < consecutivePatches.size(); k++) {
        for (size_t v2 = consecutivePatches[k][1] * params_.occupancyResolution_;
             v2 < (consecutivePatches[k][1] + 1) * params_.occupancyResolution_; ++v2) {
          for (size_t u2 = consecutivePatches[k][0] * params_.occupancyResolution_;
               u2 <
               (consecutivePatches[k][0] + consecutivePatches[k][2]) * params_.occupancyResolution_;
               ++u2) {
            size_t rleft, gleft, bleft, dleft, isleft;
            size_t rup, gup, bup, dup, isup;
            size_t rright, gright, bright, dright, isright;
            size_t rdown, gdown, bdown, ddown, isdown;

            // left search

            if (u2 != 0) {
              rleft = image.getValue(0, u2 - 1, v2);
              gleft = image.getValue(1, u2 - 1, v2);
              bleft = image.getValue(2, u2 - 1, v2);
              isleft = 1;
            } else {
              isleft = 0;
            }
            // up search

            if (v2 != 0) {
              rup = image.getValue(0, u2, v2 - 1);
              gup = image.getValue(1, u2, v2 - 1);
              bup = image.getValue(2, u2, v2 - 1);
              isup = 1;
            } else {
              isup = 0;
            }

            // right search

            if (u2 != (occupancyMapSizeU * params_.occupancyResolution_) - 1) {
              isright = 0;
              for (size_t u3 = u2 + 1; u3 < occupancyMapSizeU * params_.occupancyResolution_;
                   u3++) {
                const size_t location3 =
                    (v2 * occupancyMapSizeU * params_.occupancyResolution_) + u3;
                if (occupancyMapTemp[location3] == 1) {
                  rright = image.getValue(0, u3, v2);
                  gright = image.getValue(1, u3, v2);
                  bright = image.getValue(2, u3, v2);
                  isright = 1;
                  // std::cout << rright << gright << bright << dright << std::endl;
                  break;
                }
              }
            } else {
              isright = 0;
            }

            // down search

            if (v2 != (occupancyMapSizeV * params_.occupancyResolution_) - 1) {
              isdown = 0;
              for (size_t v3 = v2 + 1; v3 < occupancyMapSizeV * params_.occupancyResolution_;
                   v3++) {
                const size_t location3 =
                    (v3 * occupancyMapSizeU * params_.occupancyResolution_) + u2;
                if (occupancyMapTemp[location3] == 1) {
                  rdown = image.getValue(0, u2, v3);
                  gdown = image.getValue(1, u2, v3);
                  bdown = image.getValue(2, u2, v3);
                  isdown = 1;
                  break;
                }
              }
            } else {
              isdown = 0;
            }
            float rnum = 0;
            float gnum = 0;
            float bnum = 0;
            float den = 0;
            size_t c = 0;
            if (isleft) {
              rnum = rnum + rleft;
              gnum = gnum + gleft;
              bnum = bnum + bleft;
              c++;
            }
            if (isup) {
              rnum = rnum + rup;
              gnum = gnum + gup;
              bnum = bnum + bup;
              c++;
            }
            if (isright) {
              rnum = rnum + rright;
              gnum = gnum + gright;
              bnum = bnum + bright;
              c++;
            }
            if (isdown) {
              rnum = rnum + rdown;
              gnum = gnum + gdown;
              bnum = bnum + bdown;
              c++;
            }
            // std::cout << int(rnum/den) << " " << int(gnum/den) << " " << int(bnum/den) <<
            // std::endl;
            image.setValue(0, u2, v2, int(rnum / c));
            image.setValue(1, u2, v2, int(gnum / c));
            image.setValue(2, u2, v2, int(bnum / c));
          }
        }
      }
    }
  }
}

bool PCCEncoder::generateTextureVideo(const PCCGroupOfFrames &sources,
                                      PCCGroupOfFrames &reconstructs, PCCContext &context) {
  auto &frames = context.getFrames();
  auto &videoTexture = context.getVideoTexture();
  bool ret = true;
  for (size_t i = 0; i < frames.size(); i++) {
    auto &frame = frames[i];
    assert(frame.getWidth() == context.getWidth() && frame.getHeight() == context.getHeight());
    sources[i].transfertColors(reconstructs[i], int32_t(params_.bestColorSearchRange_),
                               params_.losslessTexture_ == 1);
    ret &= generateTextureVideo(reconstructs[i], frame, videoTexture, 2);
  }
  return ret;
}

bool PCCEncoder::generateTextureVideo(const PCCPointSet3 &reconstruct, PCCFrameContext &frame,
                                      PCCVideoTexture &video, const size_t frameCount) {
  auto &pointToPixel = frame.getPointToPixel();
  const size_t pointCount = reconstruct.getPointCount();
  if (!pointCount || !reconstruct.hasColors()) {
    return false;
  }
  const size_t shift = video.getFrameCount();
  video.resize(shift + frameCount);
  for (size_t f = 0; f < frameCount; ++f) {
    auto &image = video.getFrame(f + shift);
    image.resize(frame.getWidth(), frame.getHeight());
    image.set(0);
  }
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const PCCColor3B color = reconstruct.getColor(i);
    const size_t u = location[0];
    const size_t v = location[1];
    const size_t f = location[2];
    auto &image = video.getFrame(f + shift);
    image.setValue(0, u, v, color[0]);
    image.setValue(1, u, v, color[1]);
    image.setValue(2, u, v, color[2]);
  }
  return true;
}
int PCCEncoder::compressHeader(PCCContext &context, pcc::PCCBitstream &bitstream) {
  bitstream.write<uint8_t>((uint8_t)(context.size()));
  if (!context.size()) {
    return 0;
  }
  bitstream.write<uint16_t>(uint16_t(context.getWidth()));
  bitstream.write<uint16_t>(uint16_t(context.getHeight()));
  bitstream.write<uint8_t>(uint8_t(params_.occupancyResolution_));
  bitstream.write<uint8_t>(uint8_t(params_.radius2Smoothing_));
  bitstream.write<uint8_t>(uint8_t(params_.neighborCountSmoothing_));
  bitstream.write<uint8_t>(uint8_t(params_.radius2BoundaryDetection_));
  bitstream.write<uint8_t>(uint8_t(params_.thresholdSmoothing_));
  bitstream.write<uint8_t>(uint8_t(params_.losslessGeo_));
  bitstream.write<uint8_t>(uint8_t(params_.losslessTexture_));
  bitstream.write<uint8_t>(uint8_t(params_.noAttributes_));
  bitstream.write<uint8_t>(uint8_t(params_.losslessGeo444_));
  bitstream.write<uint8_t>(uint8_t(params_.absoluteD1_));
  bitstream.write<uint8_t>(uint8_t(params_.binArithCoding_));
  return 1;
}

void PCCEncoder::compressOccupancyMap(PCCContext &context, PCCBitstream &bitstream) {
  for (auto &frame : context.getFrames()) {
    if (params_.losslessGeo_) {
      auto &patches = frame.getPatches();
      auto &missedPointsPatch = frame.getMissedPointsPatch();
      const size_t patchIndex = patches.size();
      patches.resize(patchIndex + 1);
      PCCPatch &dummyPatch = patches[patchIndex];
      dummyPatch.getIndex() = patchIndex;
      dummyPatch.getU0() = missedPointsPatch.u0;
      dummyPatch.getV0() = missedPointsPatch.v0;
      dummyPatch.getSizeU0() = missedPointsPatch.sizeU0;
      dummyPatch.getSizeV0() = missedPointsPatch.sizeV0;
      dummyPatch.getU1() = 0;
      dummyPatch.getV1() = 0;
      dummyPatch.getD1() = 0;
      dummyPatch.getNormalAxis() = 0;
      dummyPatch.getTangentAxis() = 1;
      dummyPatch.getBitangentAxis() = 2;
      dummyPatch.getOccupancyResolution() = missedPointsPatch.occupancyResolution;
      dummyPatch.getOccupancy() = missedPointsPatch.occupancy;
      compressOccupancyMap(frame, bitstream);
      patches.pop_back();
    } else {
      compressOccupancyMap(frame, bitstream);
    }
  }
}
void PCCEncoder::compressOccupancyMap(PCCFrameContext &frame, PCCBitstream &bitstream) {
  auto &patches = frame.getPatches();
  const size_t patchCount = patches.size();
  bitstream.write<uint32_t>(uint32_t(patchCount));
  bitstream.write<uint8_t>(uint8_t(params_.occupancyPrecision_));
  bitstream.write<uint8_t>(uint8_t(params_.maxCandidateCount_));
  size_t maxU0 = 0;
  size_t maxV0 = 0;
  size_t maxU1 = 0;
  size_t maxV1 = 0;
  size_t maxD1 = 0;
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    maxU0 = (std::max)(maxU0, patch.getU0());
    maxV0 = (std::max)(maxV0, patch.getV0());
    maxU1 = (std::max)(maxU1, patch.getU1());
    maxV1 = (std::max)(maxV1, patch.getV1());
    maxD1 = (std::max)(maxD1, patch.getD1());
  }
  const uint8_t bitCountU0 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU0 + 1)));
  const uint8_t bitCountV0 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV0 + 1)));
  const uint8_t bitCountU1 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU1 + 1)));
  const uint8_t bitCountV1 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV1 + 1)));
  const uint8_t bitCountD1 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxD1 + 1)));

  bitstream.write<uint8_t>(bitCountU0);
  bitstream.write<uint8_t>(bitCountV0);
  bitstream.write<uint8_t>(bitCountU1);
  bitstream.write<uint8_t>(bitCountV1);
  bitstream.write<uint8_t>(bitCountD1);
  PCCBistreamPosition startPosition = bitstream.getPosition();
  bitstream += (uint64_t)4;  // placehoder for bitstream size

  bool bBinArithCoding = params_.binArithCoding_ && (!params_.losslessGeo_) &&
                         (params_.occupancyResolution_ == 16) && (params_.occupancyPrecision_ == 4);

  o3dgc::Arithmetic_Codec arithmeticEncoder;
  arithmeticEncoder.set_buffer(uint32_t(bitstream.capacity() - bitstream.size()),
                               bitstream.buffer() + bitstream.size());
  arithmeticEncoder.start_encoder();
  o3dgc::Static_Bit_Model bModel0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
  o3dgc::Adaptive_Data_Model orientationModel(4);

  o3dgc::Adaptive_Bit_Model orientationModel2;

  int64_t prevSizeU0 = 0;
  int64_t prevSizeV0 = 0;
  // arithmeticEncoder.encode( params_.absoluteD1_, bModelAbsoluteD1 );
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    EncodeUInt32(uint32_t(patch.getU0()), bitCountU0, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV0()), bitCountV0, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getU1()), bitCountU1, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV1()), bitCountV1, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getD1()), bitCountD1, arithmeticEncoder, bModel0);
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
    const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0,
                                      bModelSizeU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0,
                                      bModelSizeV0);
    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();

    if (bBinArithCoding) {
      if (patch.getNormalAxis() == 0) {
        arithmeticEncoder.encode(0, orientationModel2);
      } else if (patch.getNormalAxis() == 1) {
        arithmeticEncoder.encode(1, orientationModel2);
        arithmeticEncoder.encode(0, bModel0);
      } else {
        arithmeticEncoder.encode(1, orientationModel2);
        arithmeticEncoder.encode(1, bModel0);
      }
    } else {
      arithmeticEncoder.encode(uint32_t(patch.getNormalAxis()), orientationModel);
    }
  }
  const size_t blockToPatchWidth = frame.getWidth() / params_.occupancyResolution_;
  const size_t blockToPatchHeight = frame.getHeight() / params_.occupancyResolution_;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
  auto &blockToPatch = frame.getBlockToPatch();
  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    const auto &occupancy = patch.getOccupancy();
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if (occupancy[v0 * patch.getSizeU0() + u0]) {
          blockToPatch[(v0 + patch.getV0()) * blockToPatchWidth + (u0 + patch.getU0())] =
              patchIndex + 1;
        }
      }
    }
  }
  std::vector<std::vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0;
       --patchIndex) {  // add actual patches based on their bounding box
    const auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        candidatePatches[(patch.getV0() + v0) * blockToPatchWidth + (patch.getU0() + u0)].push_back(
            patchIndex + 1);
      }
    }
  }
  for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
    candidatePatch.push_back(0);
  }

  o3dgc::Adaptive_Bit_Model candidateIndexModelBit[4];

  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t(params_.maxCandidateCount_ + 2));
  const uint32_t bitCountPatchIndex =
      PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount + 1));
  for (size_t p = 0; p < blockCount; ++p) {
    const size_t patchIndex = blockToPatch[p];
    const auto &candidates = candidatePatches[p];
    if (candidates.size() == 1) {
      // empty
    } else {
      const uint32_t candidateCount =
          uint32_t((std::min)(candidates.size(), params_.maxCandidateCount_));
      bool found = false;
      for (uint32_t i = 0; i < candidateCount; ++i) {
        if (candidates[i] == patchIndex) {
          found = true;
          if (bBinArithCoding) {
            if (i == 0) {
              arithmeticEncoder.encode(0, candidateIndexModelBit[0]);
            } else if (i == 1) {
              arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
              arithmeticEncoder.encode(0, candidateIndexModelBit[1]);
            } else if (i == 2) {
              arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
              arithmeticEncoder.encode(1, candidateIndexModelBit[1]);
              arithmeticEncoder.encode(0, candidateIndexModelBit[2]);
            } else if (i == 3) {
              arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
              arithmeticEncoder.encode(1, candidateIndexModelBit[1]);
              arithmeticEncoder.encode(1, candidateIndexModelBit[2]);
              arithmeticEncoder.encode(0, candidateIndexModelBit[3]);
            }
          } else {
            arithmeticEncoder.encode(i, candidateIndexModel);
          }
          break;
        }
      }
      if (!found) {
        if (bBinArithCoding) {
          arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
          arithmeticEncoder.encode(1, candidateIndexModelBit[1]);
          arithmeticEncoder.encode(1, candidateIndexModelBit[2]);
          arithmeticEncoder.encode(1, candidateIndexModelBit[3]);
        } else {
          arithmeticEncoder.encode(uint32_t(params_.maxCandidateCount_), candidateIndexModel);
        }
        EncodeUInt32(uint32_t(patchIndex), bitCountPatchIndex, arithmeticEncoder, bModel0);
      }
    }
  }

  const size_t blockSize0 = params_.occupancyResolution_ / params_.occupancyPrecision_;
  const size_t pointCount0 = blockSize0 * blockSize0;
  const size_t traversalOrderCount = 4;
  std::vector<std::vector<std::pair<size_t, size_t>>> traversalOrders;
  traversalOrders.resize(traversalOrderCount);
  for (size_t k = 0; k < traversalOrderCount; ++k) {
    auto &traversalOrder = traversalOrders[k];
    traversalOrder.reserve(pointCount0);
    if (k == 0) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(std::make_pair(u1, v1));
        }
      }
    } else if (k == 1) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(std::make_pair(v1, u1));
        }
      }
    } else if (k == 2) {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (size_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
             int64_t(u1) < (std::min)(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(std::make_pair(u1, v1));
        }
      }
    } else {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (size_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
             int64_t(u1) < (std::min)(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(std::make_pair(blockSize0 - (1 + u1), v1));
        }
      }
    }
  }
  o3dgc::Adaptive_Bit_Model fullBlockModel, occupancyModel;

  o3dgc::Adaptive_Bit_Model traversalOrderIndexModel_Bit0;
  o3dgc::Adaptive_Bit_Model traversalOrderIndexModel_Bit1;
  o3dgc::Adaptive_Bit_Model runCountModel2;
  o3dgc::Adaptive_Bit_Model runLengthModel2[4];
  static size_t runLengthTable[16] = {0, 1, 2, 3, 13, 7, 10, 4, 14, 9, 11, 5, 12, 8, 6, 15};

  o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
  o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
  o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

  std::vector<bool> block0;
  std::vector<size_t> bestRuns;
  std::vector<size_t> runs;
  block0.resize(pointCount0);
  auto &width = frame.getWidth();
  auto &occupancyMap = frame.getOccupancyMap();
  for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
    for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
      const size_t patchIndex = blockToPatch[v0 * blockToPatchWidth + u0];
      if (patchIndex) {
        size_t fullCount = 0;
        for (size_t v1 = 0; v1 < blockSize0; ++v1) {
          const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            const size_t u2 = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
            bool isFull = false;
            for (size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3) {
              for (size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3) {
                isFull |= occupancyMap[(v2 + v3) * width + u2 + u3] == 1;
              }
            }
            block0[v1 * blockSize0 + u1] = isFull;
            fullCount += isFull;
            for (size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3) {
              for (size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3) {
                occupancyMap[(v2 + v3) * width + u2 + u3] = isFull;
              }
            }
          }
        }

        if (fullCount == pointCount0) {
          arithmeticEncoder.encode(true, fullBlockModel);
        } else {
          arithmeticEncoder.encode(false, fullBlockModel);
          bestRuns.clear();
          size_t bestTraversalOrderIndex = 0;
          for (size_t k = 0; k < traversalOrderCount; ++k) {
            auto &traversalOrder = traversalOrders[k];
            const auto &location0 = traversalOrder[0];
            bool occupancy0 = block0[location0.second * blockSize0 + location0.first];
            size_t runLength = 0;
            runs.clear();
            for (size_t p = 1; p < traversalOrder.size(); ++p) {
              const auto &location = traversalOrder[p];
              const bool occupancy1 = block0[location.second * blockSize0 + location.first];
              if (occupancy1 != occupancy0) {
                runs.push_back(runLength);
                occupancy0 = occupancy1;
                runLength = 0;
              } else {
                ++runLength;
              }
            }
            runs.push_back(runLength);
            if (k == 0 || runs.size() < bestRuns.size()) {
              bestRuns = runs;
              bestTraversalOrderIndex = k;
            }
          }

          assert(bestRuns.size() >= 2);
          const uint32_t runCountMinusOne = uint32_t(bestRuns.size() - 1);
          const uint32_t runCountMinusTwo = uint32_t(bestRuns.size() - 2);

          if (bBinArithCoding) {
            size_t bit1 = bestTraversalOrderIndex >> 1;
            size_t bit0 = bestTraversalOrderIndex & 0x1;
            arithmeticEncoder.encode(uint32_t(bit1), traversalOrderIndexModel_Bit1);
            arithmeticEncoder.encode(uint32_t(bit0), traversalOrderIndexModel_Bit0);
            arithmeticEncoder.ExpGolombEncode(uint32_t(runCountMinusTwo), 0, bModel0,
                                              runCountModel2);
          } else {
            arithmeticEncoder.encode(uint32_t(bestTraversalOrderIndex), traversalOrderIndexModel);
            arithmeticEncoder.encode(runCountMinusTwo, runCountModel);
          }

          const auto &location0 = traversalOrders[bestTraversalOrderIndex][0];
          bool occupancy0 = block0[location0.second * blockSize0 + location0.first];
          arithmeticEncoder.encode(occupancy0, occupancyModel);
          for (size_t r = 0; r < runCountMinusOne; ++r) {
            if (bBinArithCoding) {
              size_t runLengthIdx = runLengthTable[bestRuns[r]];
              size_t bit3 = (runLengthIdx >> 3) & 0x1;
              size_t bit2 = (runLengthIdx >> 2) & 0x1;
              size_t bit1 = (runLengthIdx >> 1) & 0x1;
              size_t bit0 = runLengthIdx & 0x1;
              arithmeticEncoder.encode(uint32_t(bit3), runLengthModel2[3]);
              arithmeticEncoder.encode(uint32_t(bit2), runLengthModel2[2]);
              arithmeticEncoder.encode(uint32_t(bit1), runLengthModel2[1]);
              arithmeticEncoder.encode(uint32_t(bit0), runLengthModel2[0]);
            } else {
              arithmeticEncoder.encode(uint32_t(bestRuns[r]), runLengthModel);
            }
          }
        }
      }
    }
  }
  uint32_t compressedBitstreamSize = arithmeticEncoder.stop_encoder();
  bitstream += (uint64_t)compressedBitstreamSize;
  bitstream.write<uint32_t>(compressedBitstreamSize, startPosition);
}
