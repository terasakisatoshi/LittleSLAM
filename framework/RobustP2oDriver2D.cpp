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
 * @file RobustP2oDriver2D.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "RobustP2oDriver2D.h"
#include "MyUtil.h"
#include "p2o.h"

using namespace std;

////////

// kslamを用いてポーズグラフpgをポーズ調整し、その結果のロボット軌跡をnewPosesに格納する。
void RobustP2oDriver2D::doP2o( PoseGraph &pg, vector<Pose2D> &newPoses, int N) {
  vector<PoseNode*> &nodes = pg.nodes;                        // ポーズノード
  vector<PoseArc*> &arcs = pg.arcs;                           // ポーズアーク

  printf("nodes.size=%zu, arcs.size=%zu\n", nodes.size(), arcs.size());
  
  vector<p2o::Pose2D> pnodes;                                  // p2oのポーズノード集合
  // ポーズノードをp2o用に変換
  for (size_t i=0; i<nodes.size(); i++) {
    PoseNode *node = nodes[i];
    Pose2D pose = node->pose;                                  // ノードの位置
    pnodes.push_back(p2o::Pose2D(pose.tx, pose.ty, DEG2RAD(pose.th)));   // 位置だけ入れる
  }

//  printf("npose     =(%g %g %g)\n", npose.tx, npose.ty, npose.th);
//  printf("pnode.pose=(%g %g %g)\n", pnode->pose.tx, pnode->pose.ty, pnode->pose.th);

  p2o::Con2DVec pcons;                             // p2oのポーズアーク集合
  int outNum=0;  
  double sum=0; 
  int num=0;
  for (size_t i=0; i<arcs.size(); i++) {
    PoseArc *arc = arcs[i];
    PoseNode *src = arc->src;
    PoseNode *dst = arc->dst;
    Pose2D &relPose = arc->relPose;

    p2o::Con2D con;
    con.id1 = src->nid;
    con.id2 = dst->nid;
    con.t = p2o::Pose2D(relPose.tx, relPose.ty, DEG2RAD(relPose.th));

    // ここをロバスト化する。ループアークにswitch変数で重み付け
    double sw=1;                 // switch variable
    if (dst->nid != src->nid+1) {
      if (hasOutliers)
        addNoise(arc);                // テスト用

      Pose2D &spose = src->pose;
      Pose2D &dpose = dst->pose;
      double chi2 = calChiError(spose, dpose, relPose, arc->inf);

      double phi = 1;
      if (beRobust)
        sw = min(1.0, 2*phi/(phi + chi2));

      sum += chi2;
      ++num;
      arc->sw = sw;

//      printf("sw=%g, chi2=%g, src=%d, dst=%d\n", sw, chi2, src->nid, dst->nid);
      printf("sw=%1.5lf, src=%d, dst=%d\n", sw, src->nid, dst->nid);

      if (chi2 > 1.0)
        ++outNum;
    }

    for (int k=0; k<3; k++) {
      for (int m=0; m<3; m++) {
        con.info(k, m) = sw*sw*arc->inf(k,m);
      }
    }
    pcons.push_back(con);
  }  
  printf("outNum=%d, sum/num=%g\n", outNum, sum/num);

//  printf("knodes.size=%zu, kcons.size=%zu\n", knodes.size(), kcons.size());    // 確認用

  p2o::Optimizer2D opt;                                          // p2oインスタンス
  std::vector<p2o::Pose2D> result = opt.optimizePath(pnodes, pcons, N);  // N回実行

   // 結果をnewPoseに格納する
  for (size_t i=0; i<result.size(); i++) {
    p2o::Pose2D newPose = result[i];                                      // i番目のノードの修正された位置
    Pose2D pose(newPose.x, newPose.y, RAD2DEG(newPose.th));
    newPoses.emplace_back(pose);
  }
}

////////////////////////

double RobustP2oDriver2D::calChiError(const Pose2D &spose, const Pose2D &dpose, const Pose2D &relPose, Eigen::Matrix3d &inf) {
  Pose2D rpose;
  Pose2D::calRelativePose(dpose, spose, rpose);
  double dx = rpose.tx - relPose.tx;
  double dy = rpose.ty - relPose.ty;
  double dth = DEG2RAD(MyUtil::add(rpose.th, -relPose.th));
  double result = dx*dx*inf(0,0) + dy*dy*inf(1,1) + dth*dth*inf(2,2) + dx*dy*(inf(0,1)+inf(1,0));
                // とりあえず、並進・回転間の相関はなしとする。
//                + dy*dth*(inf(1,2) + inf(2,1)) + dth*dx*(inf(0,2) + inf(2,0));
                // とりあえず、並進・回転間の相関はなしとする。

//  printf("dx=%g, dy=%g, dth=%g, inf: %g, %g, %g\n", dx, dy, dth, inf(0,0), inf(1,1), inf(2,2));

  return(result);
}

//////////////

void RobustP2oDriver2D::addNoise(PoseArc *arc) {
  PoseNode *src = arc->src;
  Pose2D &relPose = arc->relPose;

  if (src->nid%2 == 0) {            // 始点ノードidが偶数ならノイズ入れる
    relPose.tx += 2; 
    relPose.ty += 1; 
  }
}


