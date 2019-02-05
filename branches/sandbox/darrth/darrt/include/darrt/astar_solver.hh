#ifndef __DARRT_ASTAR_SOLVER_HH__
#define __DARRT_ASTAR_SOLVER_HH__

#include <arm_navigation_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <planning_environment/models/collision_models_interface.h>

#include <ompl/base/spaces/RealVectorBounds.h>

#include <vector>
#include <map>

namespace gm = geometry_msgs;
namespace an = arm_navigation_msgs;

namespace darrt {
  
  double distance2D(const gm::Point &p1, const gm::Point &p2);
  
  class GridPoint {
  public:
    int x, y;
    GridPoint() {x = 0; y = 0;}
    GridPoint(int nx, int ny) {x = nx; y = ny;}
    
    bool operator<(const GridPoint &other) const {
      if (x != other.x) {
	return x < other.x;
      }
      return y < other.y;
    }

    bool operator==(const GridPoint &other) {
      return x == other.x && y == other.y;
    }
  };

  double distanceTaxi(const GridPoint &p1, const GridPoint &p2);

  class HeapItem {
  public:
    GridPoint point;
    double cost, g, h;

    bool operator<(const HeapItem &other) {
      return cost < other.cost;
    }
  };

  typedef std::map<GridPoint, unsigned int> PointKeyMap;
  typedef std::map<GridPoint, GridPoint> PointMap;
  typedef std::vector<HeapItem> HeapVec;  


  class MinHeap {
  public:
    bool push(HeapItem h);
    HeapItem pop();
    bool decrease_key(unsigned int index, HeapItem new_item);
    bool clear();
    unsigned int find(HeapItem h) const;
    HeapItem at(unsigned int i) const;
    size_t size() const {return heap_.size();}
    bool empty() const {return heap_.empty();}

  protected:
    PointKeyMap points_;
    HeapVec heap_;
    unsigned int parent(unsigned int index);
    bool heapify(unsigned int index);
    bool exchange(unsigned int i1, unsigned int i2);
    std::string str();
  };


  class AStarSolver {
  public:
    class Info {
    public:
      double resolution;
      ompl::base::RealVectorBounds bounds;
      
      Info() : bounds(2) 
      {resolution = 0.05; bounds.setLow(0); bounds.setHigh(0);}
    };
    typedef std::vector<gm::Point> Path;

    AStarSolver(const an::CollisionObject &obj,
		planning_environment::CollisionModelsInterface
		*collision_models_interface);
    bool configure(const Info &info);
    bool reset();
    bool plan(const gm::PoseStamped &goal, 
	      const gm::Point *start=NULL);
    const Path &path() const {return path_;}
    void display_solution() const;
    
  protected:
    MinHeap heap_;
    PointMap parents_;
    Path path_;
    an::CollisionObject object_;
    planning_environment::CollisionModelsInterface *collision_models_interface_;
    Info info_;
    const gm::PoseStamped *last_goal_;
    const gm::Point *last_start_;
    ros::Publisher rpub_;
    bool configured_;
    
    bool update_object_location(const gm::Point &point) const;
    bool revert_object_location() const;
    bool is_valid(const gm::Point &point) const;
    gm::Point convert(const GridPoint &g, const gm::Point offset) const;
    GridPoint convert(const gm::Point &p, const gm::Point offset) const;
  };
}
#endif //astar_solver.hh
