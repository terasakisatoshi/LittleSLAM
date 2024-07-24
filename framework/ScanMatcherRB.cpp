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
 * @file ScanMatcherRB.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "ScanMatcherRB.h"

using namespace std;

/////////

// スキャンマッチングの実行
// ロバストコスト関数専用。オドメトリ値を使わない
bool ScanMatcherRB::matchScan(Scan2D &curScan) {
  ++cnt;

  printf("----- ScanMatcherRB: cnt=%d start -----\n", cnt);

  // spresが設定されていれば、スキャン点間隔を均一化する
  if (spres != nullptr)
    spres->resamplePoints(&curScan);

  // spanaが設定されていれば、スキャン点の法線を計算する
  if (spana != nullptr)
    spana->analysePoints(curScan.lps);

  // 最初のスキャンは単に地図に入れるだけ
  if (cnt == 0) {
    growMap(curScan, initPose);
    prevScan = curScan;                      // 直前スキャンの設定
    return(true);
  }
  
  // Scanに入っているオドメトリ値を用いて移動量を計算する
  Pose2D odoMotion;                                                   // オドメトリに基づく移動量
  Pose2D::calRelativePose(curScan.pose, prevScan.pose, odoMotion);    // 前スキャンとの相対位置が移動量

  Pose2D lastPose = pcmap->getLastPose();                        // 直前位置
  // predPoseは、ロバストコスト関数のテストでは入れない
  Pose2D predPose;                                               // オドメトリによる予測位置
  Pose2D::calGlobalPose(odoMotion, lastPose, predPose);          // 直前位置に移動量を加えて予測位置を得る

  const Scan2D *refScan = rsm->makeRefScan();                    // 参照スキャンの生成
  estim->setScanPair(&curScan, refScan);                         // ICPにスキャンを設定
  printf("curScan.size=%zu, refScan.size=%zu\n", curScan.lps.size(), refScan->lps.size());

  Pose2D estPose;                                                // ICPによる推定位置
//  estim->estimatePose(lastPose, estPose);         // 前回位置を初期値にしてICPを実行
  estim->estimatePose(predPose, estPose);         // 前回位置を初期値にしてICPを実行
//  size_t usedNum = estim->getUsedNum();

  // オドメトリアークのための共分散行列を求める
  Pose2D fusedPose;                       // ダミー
  Eigen::Matrix3d fusedCov;               // センサ融合後の共分散
  pfu->setRefScan(refScan);
  pfu->fusePose(&curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov);
  cov = fusedCov;

  growMap(curScan, estPose);               // 地図にスキャン点群を追加
  prevScan = curScan;                      // 直前スキャンの設定

  // 確認用
//  printf("lastPose: tx=%g, ty=%g, th=%g\n", lastPose.tx, lastPose.ty, lastPose.th);
  printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);

  // 累積走行距離の計算（確認用）
  Pose2D estMotion;                                                    // 推定移動量
  Pose2D::calRelativePose(estPose, lastPose, estMotion);
  atd += sqrt(estMotion.tx*estMotion.tx + estMotion.ty*estMotion.ty); 
  printf("atd=%g\n", atd);

  return(true);
}
