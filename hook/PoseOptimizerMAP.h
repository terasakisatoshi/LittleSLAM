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
 * @file PoseOptimizerMAP.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _POSE_OPTIMIZER_MAP_H_
#define _POSE_OPTIMIZER_MAP_H_

#include "PoseOptimizer.h"
#include "PointCloudMap.h"
#include "PoseFuser.h"

// 直線探索つきの最急降下法でコスト関数を最小化する
class PoseOptimizerMAP : public PoseOptimizer
{
private:  
  double evlimit;
  bool hasOutliers;
  bool beRobust;

  std::vector<const LPoint2D*> curLps;         // 対応がとれた現在スキャンの点群
  std::vector<const LPoint2D*> refLps;         // 対応がとれた参照スキャンの点群

  Pose2D predPose;
  Eigen::Matrix3d W;                           // オドメトリの共分散行列
  PointCloudMap *pcmap;                   // 点群地図  
  PoseFuser pfu;

public:
  PoseOptimizerMAP() : evlimit(0.05), hasOutliers(false), beRobust(false), pcmap(nullptr) {
  }

  ~PoseOptimizerMAP() {
  }

  // DataAssociatorで対応のとれた点群cur, refを設定
  virtual void setPoints(std::vector<const LPoint2D*> &cur, std::vector<const LPoint2D*> &ref) {
  //  cfunc->setPoints(curLps, refLps);    
    curLps = cur;
    refLps = ref;
  }

  void setHasOutliers(bool t) {
    hasOutliers = t;
  }

  void setBeRobust(bool t) {
    beRobust = t;
  }

  // Huber robust function
  double robustWeightHuber(double e) {
    double drho;
    if (e < evlimit*evlimit)
      drho = 1;
    else 
      drho = evlimit/sqrt(e);

    return(drho)  ;
  }

  // Tukey robust function
  double robustWeightTukey(double e) {
    double drho;
    if (e < evlimit*evlimit) {
      double r = e/(evlimit*evlimit);
      drho = (1-r)*(1-r);      
    }
    else 
      drho = 0;

    return(drho)  ;
  }

  void addNoise(size_t i, double &cx, double &cy) {
    if (i%10 == 0) {
      cx += 0.3;
      cy += 0.3;
    }
  }

  void setPointCloudMap(PointCloudMap *m) {
    pcmap = m;
  }

/////

  virtual double optimizePose(Pose2D &initPose, Pose2D &estPose);
  double calGaussNewton(const Pose2D &pose, std::vector<const LPoint2D*> &curLps, std::vector<const LPoint2D*> &refLps, Pose2D &newPose);
  void calOdometryCovariance(const Pose2D &predPose, Eigen::Matrix3d &mcov);
};

#endif 
