/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file PoseOptimizerGN.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include <boost/math/tools/minima.hpp>
#include "PoseOptimizerGN.h"
#include "MyUtil.h"

using namespace std;

////////

// データ対応づけ固定のもと、初期値initPoseを与えてロボット位置の推定値estPoseを求める
double PoseOptimizerGN::optimizePose(Pose2D &initPose, Pose2D &estPose) {
  chrono_time t0 = clock();  
//  printf("curLps.size=%zu, refLps.size=%zu\n", curLps.size(), refLps.size());

  const static int MAX_STEPS = 10;
  Pose2D pose = initPose;
  double prevErr = 1000000;
  for (int i=0; i<MAX_STEPS; ++i) {
    Pose2D npose;
    double curErr = calGaussNewton(pose, curLps, refLps, npose);

    if (abs(prevErr - curErr) <= evthre) {   // 収束
      if (curErr < prevErr) {
        pose = npose;
        prevErr = curErr;
      }
      break;  
    }
    if (curErr < prevErr) { 
      pose = npose;
      prevErr = curErr;
    } 
    else   
      break;
  }
  estPose = pose;

//  printf("initPose=(%g %g %g)\n", initPose.tx, initPose.ty, initPose.th);
//  printf("estPose= (%g %g %g)\n", estPose.tx, estPose.ty, estPose.th);

  chrono_time t1 = clock();  
  double dur = duration(t0, t1);
  totalTime += dur;
  ++timeCnt;

//  printf("Elapsed time: poptGN=%g\n", dur);

  return(prevErr);
}

//////

// 推定位置pose、現在スキャン点群curLps、参照スキャン点群refLps
double PoseOptimizerGN::calGaussNewton(const Pose2D &pose, vector<const LPoint2D*> &curLps, vector<const LPoint2D*> &refLps, Pose2D &newPose) {
  double tx = pose.tx;
  double ty = pose.ty;
  double th = pose.th;
  double a = DEG2RAD(th);
  double cs = cos(a);
  double sn = sin(a);

  Eigen::Matrix3d JWJ = Eigen::Matrix3d::Zero(3,3);  
  Eigen::Vector3d JWe = Eigen::Vector3d::Zero(3);  
  double totalErr=0;

  for (size_t i=0; i<curLps.size(); i++) {
    const LPoint2D *clp = curLps[i];                         // 現在スキャンの点
    const LPoint2D *rlp = refLps[i];                         // 参照スキャンの点

    if (rlp->type != LINE)                                   // 法線のない点は使わない
      continue;

    double cx = clp->x;
    double cy = clp->y;

    if (hasOutliers)
      addNoise(i, cx, cy);       // 外れ値テスト

    Eigen::Matrix2d W;
    W(0,0) = rlp->nx*rlp->nx;
    W(1,0) = rlp->ny*rlp->nx;
    W(0,1) = rlp->nx*rlp->ny;
    W(1,1) = rlp->ny*rlp->ny;

    Eigen::Vector2d e;
    e(0) = (cs*cx - sn*cy + tx) - rlp->x;          // clpを推定位置で座標変換
    e(1) = (sn*cx + cs*cy + ty) - rlp->y;
    double err = e.transpose()*W*e;


    double drho = 1;
    if (beRobust)
      drho = robustWeightHuber(err);
//      drho = robustWeightTukey(err);

    Eigen::Matrix<double, 2, 3> J;
    J(0,0) = 1;
    J(0,1) = 0;
    J(0,2) = -sn*cx - cs*cy;
    J(1,0) = 0;
    J(1,1) = 1;
    J(1,2) = cs*cx - sn*cy;

    Eigen::Matrix<double, 3, 2> A = drho*J.transpose()*W;    

    JWJ += A*J;
    JWe += A*e;
    totalErr += drho*err;
  }
  
  Eigen::Vector3d d = -JWJ.inverse()*JWe;
  newPose.setVal(tx+d(0), ty+d(1), MyUtil::add(th, RAD2DEG(d(2))));

//  printf("newPose=(%g %g %g), totalErr=%g\n", newPose.tx, newPose.ty, newPose.th, totalErr);

  return(totalErr);
}
