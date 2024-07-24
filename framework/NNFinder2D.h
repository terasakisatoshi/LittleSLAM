/*******************************************************************
 * Copyright (C) 2017 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *                    All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * fuRo. ("Confidential Information").  You shall not disclose such
 * Confidential Information and shall use it only in accordance with
 * the terms of the license agreement you entered into with fuRo.
 *
 * @file NNFinder2D.h
 * @author Masahiro Tomono
 * @brief Wrapper of nanoflann for 2D nearest neighbor search
 * Note: This program uses open software NanoFlann (BSD license).
 *******************************************************************/

#ifndef NNFINDER2D_H_
#define NNFINDER2D_H_

#include <vector>
#include <nanoflann.hpp>
#include "MyUtil.h"
#include "LPoint2D.h"

//////////////////////

// Tはポインタ
class NanoFlannIFp2D
{
public:
  std::vector<const LPoint2D*>  lps;

  NanoFlannIFp2D() {
  }

  ~NanoFlannIFp2D() {
  }

///////

  size_t kdtree_get_point_count() const { 
    return lps.size(); 
  }

  const double kdtree_distance(const double *p1, const size_t idx_p2, size_t /*size*/) const	{
    const double d0 = p1[0] - lps[idx_p2]->x;
    const double d1 = p1[1] - lps[idx_p2]->y;
    return (d0*d0 + d1*d1);
  }

  double kdtree_get_pt(const size_t idx, int dim) const	{
    if (dim==0) 
      return lps[idx]->x;
    else 
      return lps[idx]->y;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

/////////////////////

class NNFinder2D
{
public:
  NanoFlannIFp2D *fmap;

  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, NanoFlannIFp2D>, NanoFlannIFp2D, 2> my_kd_tree_t;
  my_kd_tree_t *index;

public:
  NNFinder2D() : index(nullptr) {
    fmap = new NanoFlannIFp2D();
  }

  NNFinder2D(int pmn) : index(nullptr) {
    fmap = new NanoFlannIFp2D();
  }

  ~NNFinder2D() {
    delete fmap;
    if (index != nullptr)
      delete index;
  }

///////////
  
  void makeIndex(const std::vector<const LPoint2D*> &refLps) {
    deleteIndex();
    std::vector<const LPoint2D*> &flps = fmap->lps;
    flps.clear();
    flps.resize(refLps.size());
    for (size_t i=0; i<refLps.size(); i++) {
      flps[i] = refLps[i];
    }

    index = new my_kd_tree_t(2, *fmap, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index->buildIndex();
  }

  void deleteIndex() {
    if (index != nullptr) {
      delete index;
      index = nullptr;
    }
  }

  const LPoint2D *getNearestNeighbor(const LPoint2D *clp, const std::vector<const LPoint2D*> &lps, double dthre) {
    double query[2] = {static_cast<double>(clp->x), static_cast<double>(clp->y)};
    size_t num_results = 1;
    std::vector<uint32_t> ret_index(num_results);
    std::vector<double> out_dist_sqr(num_results);
    index->knnSearch(query, num_results, &ret_index[0], &out_dist_sqr[0]);

    if (!ret_index.empty()) {
      double d = out_dist_sqr[0];            // 二乗距離
      if (d <= dthre*dthre) {
        size_t idx = ret_index[0];
        const LPoint2D *lp = lps[idx];
        return(lp);
      }
    }
    return(nullptr);
  }
};

#endif
