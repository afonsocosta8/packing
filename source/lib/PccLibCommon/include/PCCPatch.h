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

#ifndef PCCPatch_h
#define PCCPatch_h

#include <iterator>
#include <math.h>
#include "PCCCommon.h"

namespace pcc {


class PCCPatch {
 public:
  PCCPatch(){};
  ~PCCPatch(){
    depth_[0].clear();
    depth_[1].clear();
    occupancy_.clear();
  };
  size_t&                     getIndex()                     { return index_;                 }
  size_t&					  getPixelCount()				 { return pixelCount;			  }
  size_t&                     getU1()                        { return u1_;                    }
  size_t&                     getV1()                        { return v1_;                    }
  size_t&                     getD1()                        { return d1_;                    }
  size_t&                     getSizeD()                     { return sizeD_;                 }
  size_t&                     getSizeU()                     { return sizeU_;                 }
  size_t&                     getSizeV()                     { return sizeV_;                 }
  size_t&                     getU0()                        { return u0_;                    }
  size_t&                     getV0()                        { return v0_;                    }
  size_t&                     getSizeU0()                    { return sizeU0_;                }
  size_t&                     getSizeV0()                    { return sizeV0_;                }
  size_t&                     setViewId()                    { return viewId_;                }
  size_t&                     setBestMatchIdx()              { return bestMatchIdx_;          }
  size_t&                     getOccupancyResolution()       { return occupancyResolution_;   }
  size_t&                     getNormalAxis()                { return normalAxis_;            }
  size_t&                     getTangentAxis()               { return tangentAxis_;           }
  size_t&                     getBitangentAxis()             { return bitangentAxis_;         }
  std::vector<int16_t>&       getDepth( int i )              { return depth_[i];              }
  std::vector<uint8_t>&       getColor( int i )              { return color_[i];              }
  std::vector<bool>&          getOccupancy()                 { return occupancy_;             }

  size_t                      getIndex()               const { return index_;                 }
  size_t					  getPixelCount()		   const { return pixelCount;			  }
  size_t                      getU1()                  const { return u1_;                    }
  size_t                      getV1()                  const { return v1_;                    }
  size_t                      getD1()                  const { return d1_;                    }
  size_t                      getSizeD()               const { return sizeD_;                 }
  size_t                      getSizeU()               const { return sizeU_;                 }
  size_t                      getSizeV()               const { return sizeV_;                 }
  size_t                      getU0()                  const { return u0_;                    }
  size_t                      getV0()                  const { return v0_;                    }
  size_t                      getSizeU0()              const { return sizeU0_;                }
  size_t                      getSizeV0()              const { return sizeV0_;                }
  size_t                      getOccupancyResolution() const { return occupancyResolution_;   }
  size_t                      getNormalAxis()          const { return normalAxis_;            }
  size_t                      getTangentAxis()         const { return tangentAxis_;           }
  size_t                      getBitangentAxis()       const { return bitangentAxis_;         }
  size_t                      getViewId()              const { return viewId_;                }
  size_t                      getBestMatchIdx()        const { return bestMatchIdx_;          }
  const std::vector<int16_t>& getDepth( int i )        const { return depth_[i];              }
  const std::vector<uint8_t>& getColor( int i )        const { return color_[i];              }
  const std::vector<bool>&    getOccupancy()           const { return occupancy_;             }

  void print() const {
    printf("Patch[%3zu] uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu / %4zu %4zu %4zu \n",
           index_, u0_, v0_, sizeU0_, sizeV0_, u1_, v1_, d1_, sizeU_, sizeV_, sizeD_ );
  }

  //this method implements the writing of one the patch to the file
  friend std::ostream & operator << (std::ostream &out, const PCCPatch & rhs) {
    // the first attributes of type syze_t are inserted in seperate lines
	  out << rhs.index_ << "\n" <<rhs.u1_<<"\n"<<rhs.v1_<<"\n"<<
    rhs.d1_ << "\n" <<rhs.sizeD_<<"\n"<<rhs.sizeU_<<"\n"<<rhs.sizeV_<<"\n"<<
    rhs.u0_ << "\n" <<rhs.v0_<<"\n"<<rhs.sizeU0_<<"\n"<<rhs.sizeV0_<<"\n"<<
    rhs.occupancyResolution_ << "\n" <<rhs.normalAxis_<<"\n"<<rhs.tangentAxis_<<"\n"<<rhs.bitangentAxis_<<"\n";

    //now i need to send the size of the vector of depths(depth map)
    //this size is the same for D0 and D1
    out << rhs.depth_[0].size() << "\n";

    //the next 2 blocks write the depth maps to the file
    //each position of the vector is inserted separated by empty spaces
    for(int i=0;i<rhs.depth_[0].size();i++){
      out << rhs.depth_[0][i]<<" ";
    }
    out << "\n";

    for(int i=0;i<rhs.depth_[1].size();i++){
      out << rhs.depth_[1][i]<<" ";
    }

    out << "\n";

	for (int i = 0; i < rhs.depth_[0].size(); i++) {
      out << int(rhs.color_[0][i]) << " ";
    }
    out << "\n";

    for (int i = 0; i < rhs.depth_[1].size(); i++) {
      out << int(rhs.color_[1][i]) << " ";
    }

	out << "\n";

    for (int i = 0; i < rhs.depth_[1].size(); i++) {
      out << int(rhs.color_[2][i]) << " ";
    }

    out << "\n";
    //this block writes the occupancy map to the file also separated by empty spaces
    //no need to send th size since in the reading process it is possible to extract it
    for(int i=0;i<rhs.occupancy_.size();i++){
      out << rhs.occupancy_[i]<<" ";
    }

    out << "\n";
    //the last 2 syze_t variables are written to file
    out <<rhs.viewId_<< "\n" << rhs.bestMatchIdx_ << "\n" << rhs.pixelCount <<
    std::endl;

	  return out;
  }

  friend std::istream & operator >> (std::istream &in, PCCPatch &rhs){

    std::string depth0;
    std::string depth1;
    std::string soccupancy;
    size_t patchsize;
    // the first variables of type size_t are extracted from file and inserted in the respective variables
    // size of the depth maps is also extracted(patchsize)
    in >> rhs.index_;
  	in >> rhs.u1_;
  	in >> rhs.v1_;
    in >> rhs.d1_;
    in >> rhs.sizeD_;
    in >> rhs.sizeU_;
    in >> rhs.sizeV_;
    in >> rhs.u0_;
    in >> rhs.v0_;
    in >> rhs.sizeU0_;
    in >> rhs.sizeV0_;
    in >> rhs.occupancyResolution_;
    in >> rhs.normalAxis_;
    in >> rhs.tangentAxis_;
    in >> rhs.bitangentAxis_;
    in >> patchsize;

    //initialize the depth vectors
    rhs.depth_[0].resize(patchsize);
    rhs.depth_[1].resize(patchsize);
    rhs.color_[0].resize(patchsize);
    rhs.color_[1].resize(patchsize);
    rhs.color_[2].resize(patchsize);
    //initialize the occupancy map
    //previously extracted information allows the extraction of occupancy map size
    rhs.occupancy_.resize(ceil(1.0*rhs.sizeU_/rhs.occupancyResolution_)*ceil(1.0*rhs.sizeV_/rhs.occupancyResolution_));
    //the next 2 blocks read the lines containing the depth vectors separated by empty spaces
    for (int i=0;i<patchsize;i++){
      in >> rhs.depth_[0][i];
    }

    for (int i=0;i<patchsize;i++){
      in >> rhs.depth_[1][i];
    }
    for (int i = 0; i < patchsize; i++) {
      uint16_t taw;
      in >> taw;
      rhs.color_[0][i]=taw;
    }

    for (int i = 0; i < patchsize; i++) {
      uint16_t taw;
      in >> taw;
      rhs.color_[1][i]=taw;
    }
    for (int i = 0; i < patchsize; i++) {
      uint16_t taw;
      in >> taw;
      rhs.color_[2][i]=taw;
    }
    //the next block reads the line containing the occupancy vector separated by empty spaces
    for (int i=0;i<ceil(1.0*rhs.sizeU_/rhs.occupancyResolution_)*ceil(1.0*rhs.sizeV_/rhs.occupancyResolution_);i++){
      in>>soccupancy;
      rhs.occupancy_[i] = (soccupancy=="1");
    }
    //the last 2 variables are extracted
    in >> rhs.viewId_;
    in >> rhs.bestMatchIdx_;
    in >> rhs.pixelCount;
  	return in;
  }

  //Empty Space Percentage
  /*friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return (lhs.sizeV0_ * lhs.sizeU0_ - lhs.pixelCount)/(lhs.sizeV0_ * lhs.sizeU0_) != (rhs.sizeV0_ * rhs.sizeU0_ - rhs.pixelCount) / (rhs.sizeV0_ * rhs.sizeU0_)
               ? (lhs.sizeV0_ * lhs.sizeU0_ - lhs.pixelCount) / (lhs.sizeV0_ * lhs.sizeU0_) >
                     (rhs.sizeV0_ * rhs.sizeU0_ - rhs.pixelCount) / (rhs.sizeV0_ * rhs.sizeU0_)
               : lhs.index_ < rhs.index_;
  }*/

  // Empty Space
  /*friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return (lhs.sizeV0_ * lhs.sizeU0_ - lhs.pixelCount) != (rhs.sizeV0_ * rhs.sizeU0_ - rhs.pixelCount)
               ? (lhs.sizeV0_ * lhs.sizeU0_ - lhs.pixelCount) >
                     (rhs.sizeV0_ * rhs.sizeU0_ - rhs.pixelCount)
               : lhs.index_ < rhs.index_;
  }*/ 

  // Effective Area one - Block level
  /*friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return lhs.pixelCount != rhs.pixelCount
               ? lhs.pixelCount > rhs.pixelCount
               : lhs.index_ < rhs.index_;
  }*/

  //Area one
  /*friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return (lhs.sizeV_ * lhs.sizeU_) != (rhs.sizeV_ * rhs.sizeU_) 
			   ? (lhs.sizeV_ * lhs.sizeU_) > (rhs.sizeV_ * rhs.sizeU_)
               : lhs.index_ < rhs.index_;
  }*/ 

  //TMC2 one
  friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return lhs.sizeV_ != rhs.sizeV_
               ? lhs.sizeV_ > rhs.sizeV_
               : (lhs.sizeU_ != rhs.sizeU_ ? lhs.sizeU_ > rhs.sizeU_ : lhs.index_ < rhs.index_);
  }

  //viewID
  /*friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return lhs.viewId_ != rhs.viewId_
               ? lhs.viewId_ < rhs.viewId_
               : lhs.index_ < rhs.index_;
  }*/

  //Width
  //friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
  //  return lhs.sizeU_ != rhs.sizeU_
  //      ? lhs.sizeU_ > rhs.sizeU_
  //          : (lhs.sizeV_ != rhs.sizeV_ ? lhs.sizeV_ > rhs.sizeV_ : lhs.index_ < rhs.index_);
  //}
 private:
  size_t index_;                   // patch index
  size_t u1_;                      // tangential shift
  size_t v1_;                      // bitangential shift
  size_t d1_;                      // depth shift
  size_t sizeD_;                   // size for depth
  size_t sizeU_;                   // size for depth
  size_t sizeV_;                   // size for depth
  size_t u0_;                      // location in packed image
  size_t v0_;                      // location in packed image
  size_t sizeU0_;                  // size of occupancy map
  size_t sizeV0_;                  // size of occupancy map
  size_t occupancyResolution_;     // ocupacy map resolution

  size_t normalAxis_;              // x
  size_t tangentAxis_;             // y
  size_t bitangentAxis_;           // z
  std::vector<int16_t> depth_[2];  // depth
  std::vector<uint8_t> color_[3];  // color
  std::vector<bool> occupancy_;    // occupancy map

  size_t viewId_;                  // viewId in [0,1,2,3,4,5]
  size_t bestMatchIdx_;            // index of matched patch from pre-frame patch.
  size_t pixelCount;			   // number of occupied pixels
};

struct PCCMissedPointsPatch {
  size_t sizeU;
  size_t sizeV;
  size_t u0;
  size_t v0;
  size_t sizeV0;
  size_t sizeU0;
  size_t occupancyResolution;
  std::vector<bool> occupancy;
  std::vector<uint16_t> x;
  std::vector<uint16_t> y;
  std::vector<uint16_t> z;
  void resize(const size_t size) {
    x.resize(size);
    y.resize(size);
    z.resize(size);
  }
  void resize(const size_t size, const uint16_t val) {
    x.resize(size, val);
    y.resize(size, val);
    z.resize(size, val);
  }
  const size_t size() { return x.size(); }
};
}

#endif /* PCCPatch_h */
