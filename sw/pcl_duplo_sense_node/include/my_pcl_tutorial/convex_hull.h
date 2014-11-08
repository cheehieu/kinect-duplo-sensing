/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* $Id: convex_hull.h 2568 2011-09-22 20:32:16Z aaldoma $
*
*/
 
//#include <pcl/pcl_config.h>
#include "pcl_config.h"
#ifdef HAVE_QHULL
 
#ifndef PCL_CONVEX_HULL_2D_H_
#define PCL_CONVEX_HULL_2D_H_

// PCL includes
#include "pcl/pcl_base.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/PolygonMesh.h"
#include <math.h>

namespace pcl
{
  inline bool
  comparePoints2D (const std::pair<int, Eigen::Vector4f> & p1, const std::pair<int, Eigen::Vector4f> & p2)
  {
    double angle1 = atan2 (p1.second[1], p1.second[0]) + M_PI;
    double angle2 = atan2 (p2.second[1], p2.second[0]) + M_PI;
    return (angle1 > angle2);
  }


  template<typename PointInT>
  class ConvexHull : public PCLBase<PointInT>
  {
    using PCLBase<PointInT>::input_;
    using PCLBase<PointInT>::indices_;
    using PCLBase<PointInT>::initCompute;
    using PCLBase<PointInT>::deinitCompute;

    public:
      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      ConvexHull () : total_area_(0), total_volume_(0)
      {
        keep_information_ = false;
        compute_area_ = false;
      };

      void
      reconstruct (PointCloud &points, 
                   std::vector<pcl::Vertices> &polygons);

   void
      reconstruct (PointCloud &output);

      void
      setKeepInformation (bool value)
      {
        keep_information_ = value;
      }

      void
      setComputeAreaVolume (bool value)
      {
        compute_area_ = value;
      }

      double
      getTotalArea ()
      {
        return total_area_;
      }

      double
      getTotalVolume ()
      {
        return total_volume_;
      }

     private:
      void
      performReconstruction (PointCloud &points, 
                             std::vector<pcl::Vertices> &polygons, 
                             bool fill_polygon_data = false);

      bool keep_information_;
      bool compute_area_;
      double total_area_;
      double total_volume_;

    protected:
      std::string
      getClassName () const
      {
        return ("ConvexHull");
      }
    };
}

#endif  //#ifndef PCL_CONVEX_HULL_2D_H_
#endif
