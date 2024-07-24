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
 * @file PoseEstimatorICP.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "PoseEstimatorICP.h"

using namespace std;

//////////////

// 初期値initPoseを与えて、ICPによりロボット位置の推定値estPoseを求める
double PoseEstimatorICP::estimatePose(Pose2D &initPose, Pose2D &estPose){
  chrono_time t0 = clock();

  double evmin = HUGE_VAL;             // コスト最小値。初期値は大きく
  double evthre = 0.000001;            // コスト変化閾値。変化量がこれ以下なら繰り返し終了
//  double evthre = 0.00000001;            // コスト変化閾値。変化量がこれ以下なら繰り返し終了
  popt->setEvthre(evthre);
  popt->setEvlimit(0.2);               // evlimitは外れ値の閾値[m]
  double ev = 0;                       // コスト
  double evold = evmin;                // 1つ前の値。収束判定のために使う。
  Pose2D pose = initPose;
  Pose2D poseMin = initPose;
  for (int i=0; abs(evold-ev) > evthre && i<100; i++) {           // i<100は振動対策
    if (i > 0)
      evold = ev;
    dass->findCorrespondence(curScan, pose);      // データ対応づけ
    Pose2D newPose;
    popt->setPoints(dass->curLps, dass->refLps);                  // 対応結果を渡す
    ev = popt->optimizePose(pose, newPose);                       // その対応づけにおいてロボット位置の最適化
    pose = newPose;

    if (ev < evmin) {                                             // コスト最小結果を保存
      poseMin = newPose;
      evmin = ev;
    }

//    printf("dass.curLps.size=%zu, dass.refLps.size=%zu\n", dass->curLps.size(), dass->refLps.size());
//    printf("mratio=%g\n", mratio);
//    printf("i=%d: ev=%g, evold=%g\n", i, ev, evold);
  }

  pnrate = popt->getPnrate();
  usedNum = dass->curLps.size();

  estPose = poseMin;

  printf("finalError=%g, pnrate=%g\n", evmin, pnrate);
  printf("estPose:  tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);      // 確認用

  chrono_time t1 = clock();
  double dur = duration(t0, t1);
  printf("PoseEstimatorICP: dur=%g\n", dur);                 // 処理時間

  if (evmin < HUGE_VAL)
    totalError += evmin;                                   // 誤差合計
  totalTime += dur;                                         // 処理時間合計
  printf("totalError=%g, totalTime=%g\n", totalError, totalTime);    // 確認用

  dass->averageProcTime();
  popt->averageProcTime();

  return(evmin);
}
