/*
 * viewcells.hpp
 *
 *  Created on: Feb 23, 2011
 *      Author: hordurj
 */

#ifndef VIEWCELLS_HPP_
#define VIEWCELLS_HPP_

#include <vector>
#include <Eigen/Dense>

namespace neuroslam
{

typedef Eigen::VectorXd View;
class ViewCells
{
public:
  ViewCells() : lambda_(0.36), m_min_(0.1), delta_(0.1), d_max_(1000.0), max_cell_id_(-1)
  {

  }

  /**
   * Returns the number of view cells
   */
  size_t count() const {return views_.size();}

  /**
   * Returns the current score of a view cell
   */
  double score(size_t idx) const {return score_[idx];}

  /**
   * Sets the current view.
   *
   * This method sets the active view. The score is updated relative to this
   * view. If the maximum score is below a certain threshold the view will
   * be added as a new view cell.
   *
   * @param view is the current active view.
   *
   * @return true if a new view is added
   */
  bool setView(const View & view);

  /**
   * Get the maximally activated view cell.
   *
   * @return id of the maximally activated cell
   *         -1 if none is active
   *
   */
  int getMaxView() {return max_cell_id_;}

  /**
   * Return currently active cells.
   */
  const std::vector<std::pair<int, double> > & getActiveCells() {return active_cells_;};

  /**
   * Update the view cell activity using the last received view.
   */
  void update();

private:
  double calculateScore(const View & view1, const View & view2);

  double lambda_;
  double m_min_;
  double delta_;
  double d_max_;

  std::vector<View > views_;
  std::vector<double> score_;

  // A list of currently active cells.
  std::vector<std::pair<int, double> > active_cells_;

  int max_cell_id_;
};

}

#endif /* VIEWCELLS_HPP_ */
