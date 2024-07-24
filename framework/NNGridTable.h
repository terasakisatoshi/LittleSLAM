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
 * @file NNGridTable.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _NN_GRID_TABLE_H_
#define _NN_GRID_TABLE_H_

#include <vector>
#include "MyUtil.h"
#include "Pose2D.h"

struct NNGridCell
{
  std::vector<const LPoint2D*> lps;         // このセルに格納されたスキャン点群

  void clear() {
    lps.clear();                            // 空にする
  }
};

///////

// 格子テーブル
class NNGridTable
{
private:
  double dthre;
  double csize;                       // セルサイズ[m]
  double rsize;                       // 対象領域のサイズ[m]。正方形の1辺の半分。
  int tsize;                          // テーブルサイズの半分
  std::vector<NNGridCell> table;      // テーブル本体

public:
  NNGridTable() : dthre(0.2), csize(0.05), rsize(40){            // セル5cm、対象領域40x2m四方
    tsize = static_cast<int>(rsize/csize);           // テーブルサイズの半分
    size_t w = 2*static_cast<size_t>(tsize)+1;       // テーブルサイズ
    table.resize(w*w);                               // 領域確保
    clear();                                         // tableの初期化
  }

  ~NNGridTable() {
  }
  
  void clear() {
    for (size_t i=0; i<table.size(); i++)
      table[i].clear();                         // 各セルを空にする
  }

  void setDthre(double d) {
    dthre = d;
  }
  
////////////

  void addPoint(const LPoint2D *lp);
  const LPoint2D *findClosestPoint(const LPoint2D *clp, const Pose2D &predPose);
  void makeCellPoints(size_t nthre, std::vector<LPoint2D> &ps);
};

#endif

