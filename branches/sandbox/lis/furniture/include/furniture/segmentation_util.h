#ifndef SEGMENTATION_UTIL_H_
#define SEGMENTATION_UTIL_H_

#include <furniture/common.h>
#include <furniture/color_util.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace furniture {

  class Edge {
  public:
    double color_dist, norm_angle;
    int node1, node2;
    
    Edge(int node1, int node2, double color_dist, double norm_angle) {
      if (node1 < node2) {
	this->node1 = node1;
	this->node2 = node2;
      } else {
	this->node2 = node1;
	this->node1 = node2;
      }
      this->color_dist = color_dist;
      this->norm_angle = norm_angle;
    }
  };

  bool edge_comparator(Edge e1, Edge e2) {
    if (e1.node1 != e2.node1)
      return e1.node1 < e2.node1;
    else return e1.node2 < e2.node2;
  }

  // Find nearest neighbors for each point. Have a hashmap (edgepair -> boolean) to mark processed edges. If edge is not processed and the points are not far away, calculate the distance. This will form the graph.

} // namespace

#endif
