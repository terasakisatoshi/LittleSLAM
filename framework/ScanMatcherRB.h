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
 * @file ScanMatcherRB.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SCAN_MATCHER2D_RBTEST_H_
#define SCAN_MATCHER2D_RBTEST_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"
#include "RefScanMaker.h"
#include "ScanPointResampler.h"
#include "ScanPointAnalyser.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"
#include "ScanMatcher2D.h"

// ICPを用いてスキャンマッチングを行う
class ScanMatcherRB : public ScanMatcher2D
{
public:
  ScanMatcherRB() {
  }

  ~ScanMatcherRB() {
  }
  
//////////

  virtual bool matchScan(Scan2D &scan);

};

#endif
