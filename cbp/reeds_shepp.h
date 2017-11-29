/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Guan-Horng Liu.
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
*   * Neither the name of the the copyright holder nor the names of its
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
* Author:  Guan-Horng Liu
*********************************************************************/

#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <vector>
#include <iostream>
#include <deque>
#include <queue>

typedef int (*ReedsSheppPathSamplingCallback)(double q[3], void *user_data);
typedef int (*ReedsSheppPathTypeCallback)(int t, void *user_data);

using namespace std;
using namespace cv;
class ReedsSheppStateSpace
{
  public:
    /** \brief The Reeds-Shepp path segment types */
    enum ReedsSheppPathSegmentType
    {
        RS_NOP = 0,
        RS_LEFT = 1,
        RS_STRAIGHT = 2,
        RS_RIGHT = 3
    };

    /** \brief Reeds-Shepp path types */
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

    /** \brief Complete description of a ReedsShepp path */
    class ReedsSheppPath
    {
      public:
        ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                       double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                       double w = 0., double x = 0.);

        double length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType *type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
        /**
	* @brief The overload of the operator <
	* \brief
	* @param
	* @param
	* @param
	* @return
	*/
        friend bool operator<(const ReedsSheppPath lhs, const ReedsSheppPath rhs)
        {
            if (lhs.length() > rhs.length())
            {
                return true;
            }
            return false;
        }
    };

    ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

    double distance(double q0[3], double q1[3]);

    void type(double q0[3], double q1[3], ReedsSheppPathTypeCallback cb, void *user_data);

    ReedsSheppPath sample(double q0[3], double q1[3], double step_size, ReedsSheppPathSamplingCallback cb, void *user_data);

    std::vector<ReedsSheppPath> sample_many(double q0[3], double q1[3], double step_size,
                                            std::vector<std::vector<geometry_msgs::Point32>> &data,
                                            std::vector<std::vector<geometry_msgs::Polygon>> &bounding_polygon);

    /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

    /*The following parts merged by yinjian*/
    inline double mod2pi(double x);

    inline void polar(double x, double y, double &r, double &theta);

    inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega);

    // formula 8.1 in Reeds-Shepp paper
    inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v);

    // formula 8.2
    inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v);

    void CSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path);

    // formula 8.3 / 8.4  *** TYPO IN PAPER ***
    inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v);

    void CCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path);

    // formula 8.7
    inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v);

    // formula 8.8
    inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v);

    void CCCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path);

    // formula 8.9
    inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v);

    // formula 8.10
    inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v);

    void CCSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path);

    // formula 8.11 *** TYPO IN PAPER ***
    inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v);

    void CCSCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path);

    ReedsSheppStateSpace::ReedsSheppPath reedsShepp(double x, double y, double phi);
    /************************/

  protected:
    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]);

    std::priority_queue<ReedsSheppPath> pathqueue;

    /** \brief Turning radius */
    double rho_;
};

#endif
