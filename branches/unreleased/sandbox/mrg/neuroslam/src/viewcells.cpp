/*
 * viewcells.cpp
 *
 *  Created on: Feb 23, 2011
 *      Author: hordurj
 */

#include <iostream>
#include "neuroslam/viewcells.hpp"

namespace neuroslam {

double ViewCells::calculateScore(const View & view1, const View & view2)
{
  double d = (view1-view2).lpNorm<1>();
  double e = 0.5;
  if (d > d_max_) return 0.0;
  else return (1.0/(d+e));
}

void ViewCells::update()
{
}

bool ViewCells::setView(const View & view)
{
  bool added = false;
  double total_score = 0.0;
  for (size_t i=0; i<views_.size(); ++i) {
    score_[i] = calculateScore(view, views_[i]);
    total_score += score_[i];
  }

  active_cells_.clear();

  int max_id = -1;
  int max_score = 0.0;

  if (total_score == 0.0) {
    views_.push_back(view);
    score_.push_back(1.0);
    max_id = views_.size()-1;
    added = true;
  } else {
    for (size_t i=0; i<views_.size(); ++i) {
      score_[i] = score_[i]/total_score;
      if (score_[i]>0.0) {
        if (score_[i]>max_score) {
          max_id = i;
          max_score = score_[i];
        }
        active_cells_.push_back(std::pair<int, double>(i, score_[i]));
        std::cout << "ViewCells::setView: view = " << i << " score = " << score_[i] << " n: " << views_.size() << std::endl;
      }
    }
  }
  max_cell_id_ = max_id;

  std::cout << "ViewCells::setView: diff: " << (view-views_[max_cell_id_]).lpNorm<1>() << std::endl;

  return added;
}
}
