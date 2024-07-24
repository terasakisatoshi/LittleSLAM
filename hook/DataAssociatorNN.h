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
 * @file DataAssociatorNN.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef DATA_ASSOCIATOR_NN_H_
#define DATA_ASSOCIATOR_NN_H_

#include "DataAssociator.h"
#include "NNFinder2D.h"

// 格子テーブルを用いて、現在スキャンと参照スキャン間の点の対応づけを行う
class DataAssociatorNN : public DataAssociator
{
private:
  std::vector<const LPoint2D*> allLps;
  NNFinder2D *nnfin;                        // 格子テーブル
  
public:
  DataAssociatorNN() {
    nnfin = new NNFinder2D(1000);
  }

  ~DataAssociatorNN() {
    delete nnfin;
  }
  
  // 参照スキャンの点rlpsをポインタにしてnntabに入れる
  virtual void setRefBase(const std::vector<LPoint2D> &rlps) {
    allLps.clear();
    for (size_t i=0; i<rlps.size(); i++) 
      allLps.push_back(&rlps[i]);              // ポインタにして格納
    nnfin->makeIndex(allLps);
  }

/////////

  virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif
