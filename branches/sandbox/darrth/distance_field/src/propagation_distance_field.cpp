/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan, Ken Anderson */

#include <distance_field/propagation_distance_field.h>
#include <visualization_msgs/Marker.h>

namespace distance_field
{

PropagationDistanceField::~PropagationDistanceField()
{
}

PropagationDistanceField::PropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      DistanceField<PropDistanceFieldVoxel>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, PropDistanceFieldVoxel(max_distance))
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  bucket_queue_.resize(max_distance_sq_+1);

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution;
}

int PropagationDistanceField::eucDistSq(int3 point1, int3 point2)
{
  int dx = point1.x() - point2.x();
  int dy = point1.y() - point2.y();
  int dz = point1.z() - point2.z();
  return dx*dx + dy*dy + dz*dz;
}

void PropagationDistanceField::updatePointsInField(const std::vector<tf::Vector3>& points, bool iterative)
{
  if( iterative )
  {
    VoxelSet points_added;
    VoxelSet points_removed(object_voxel_locations_);

    // Compare and figure out what points are new,
    // and what points are to be deleted
    for( unsigned int i=0; i<points.size(); i++)
    {
      // Convert to voxel coordinates
      int3 voxel_loc;
      bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(),
                                voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );
      if( valid )
      {
        if( iterative )
        {
          bool already_obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
          if( !already_obstacle_voxel )
          {
            // Not already in set of existing obstacles, so add to voxel list
            object_voxel_locations_.insert(voxel_loc);

            // Add point to the set or expansion
            points_added.insert(voxel_loc);
          }
          else
          {
            // Already an existing obstacle, so take off removal list
            points_removed.erase(voxel_loc);
          }
        }
        else
        {
          object_voxel_locations_.insert(voxel_loc);
          points_added.insert(voxel_loc);
        }
      }
    }

   removeObstacleVoxels( points_removed );
   addNewObstacleVoxels( points_added );
  }

  else	// !iterative
  {
    VoxelSet points_added;
    reset();

    for( unsigned int i=0; i<points.size(); i++)
    {
      // Convert to voxel coordinates
      int3 voxel_loc;
      bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(),
                                voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );
      if( valid )
      {
        object_voxel_locations_.insert(voxel_loc);
        points_added.insert(voxel_loc);
      }
    }
    addNewObstacleVoxels( points_added );
  }
}

void PropagationDistanceField::addPointsToField(const std::vector<tf::Vector3>& points)
{
  VoxelSet voxel_locs;

  for( unsigned int i=0; i<points.size(); i++)
  {
    // Convert to voxel coordinates
    int3 voxel_loc;
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(),
                              voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );

    if( valid )
    {
      bool already_obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
      if( !already_obstacle_voxel )
      {
        // Not already in set of existing obstacles, so add to voxel list
        object_voxel_locations_.insert(voxel_loc);

        // Add point to the queue for expansion
        voxel_locs.insert(voxel_loc);
      }
    }
  }

  addNewObstacleVoxels( voxel_locs );
}

void PropagationDistanceField::addNewObstacleVoxels(const VoxelSet& locations)
{
  int x, y, z;
  int initial_update_direction = getDirectionNumber(0,0,0);
  bucket_queue_[0].reserve(locations.size());

  VoxelSet::const_iterator it = locations.begin();
  for( it=locations.begin(); it!=locations.end(); ++it)
  {
    int3 loc = *it;
    x = loc.x();
    y = loc.y();
    z = loc.z();
    bool valid = isCellValid( x, y, z);
    if (!valid)
      continue;
    PropDistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.distance_square_ = 0;
    voxel.closest_point_ = loc;
    voxel.location_ = loc;
    voxel.update_direction_ = initial_update_direction;
    bucket_queue_[0].push_back(&voxel);
  }

  propogate();
}

void PropagationDistanceField::removeObstacleVoxels(const VoxelSet& locations )
{
  std::vector<int3> stack;
  int initial_update_direction = getDirectionNumber(0,0,0);

  stack.reserve( num_cells_[DIM_X] * num_cells_[DIM_Y] * num_cells_[DIM_Z] );
  bucket_queue_[0].reserve(locations.size());

  // First reset the obstacle voxels,
  VoxelSet::const_iterator it = locations.begin();
  for( it=locations.begin(); it!=locations.end(); ++it)
  {
    int3 loc = *it;
    bool valid = isCellValid( loc.x(), loc.y(), loc.z());
    if (!valid)
      continue;
    PropDistanceFieldVoxel& voxel = getCell(loc.x(), loc.y(), loc.z());
    voxel.distance_square_ = max_distance_sq_;
    voxel.closest_point_ = loc;
    voxel.location_ = loc;
    voxel.update_direction_ = initial_update_direction;
    stack.push_back(loc);
  }

  // Reset all neighbors who's closest point is now gone.
  while(stack.size() > 0)
  {
    int3 loc = stack.back();
    stack.pop_back();

    for( int neighbor=0; neighbor<27; neighbor++ )
    {
      int3 diff = getLocationDifference(neighbor);
      int3 nloc( loc.x() + diff.x(), loc.y() + diff.y(), loc.z() + diff.z() );

      if( isCellValid(nloc.x(), nloc.y(), nloc.z()) )
      {
        PropDistanceFieldVoxel& nvoxel = getCell(nloc.x(), nloc.y(), nloc.z());
        int3& close_point = nvoxel.closest_point_;
        PropDistanceFieldVoxel& closest_point_voxel = getCell( close_point.x(), close_point.y(), close_point.z() );

        if( closest_point_voxel.distance_square_ != 0 )
        {	// closest point no longer exists
          if( nvoxel.distance_square_!=max_distance_sq_)
          {
            nvoxel.distance_square_ = max_distance_sq_;
            nvoxel.closest_point_ = nloc;
            nvoxel.location_ = nloc;
            nvoxel.update_direction_ = initial_update_direction;
            stack.push_back(nloc);
          }
        }
        else
        {	// add to queue so we can propogate the values
          bucket_queue_[0].push_back(&nvoxel);
        }
      }
    }
  }

  propogate();
}

void PropagationDistanceField::propogate()
{
  int x, y, z, nx, ny, nz;
  int3 loc;

  // now process the queue:
  for (unsigned int i=0; i<bucket_queue_.size(); ++i)
  {
    std::vector<PropDistanceFieldVoxel*>::iterator list_it = bucket_queue_[i].begin();
    while(list_it!=bucket_queue_[i].end())
    {
      PropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        PropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->distance_square_)
        {
          // update the neighboring voxel
          neighbor->distance_square_ = new_distance_sq;
          neighbor->closest_point_ = vptr->closest_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    bucket_queue_[i].clear();
  }

}

void PropagationDistanceField::getOccupiedVoxelMarkers(const std::string & frame_id, 
						       const ros::Time stamp,
						       const tf::Transform& cur,
						       visualization_msgs::Marker& inf_marker)
{
  inf_marker.points.clear();
  inf_marker.header.frame_id = frame_id;
  inf_marker.header.stamp = stamp;
  inf_marker.ns = "distance_field_occupied_cells";
  inf_marker.id = 1;
  inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
  inf_marker.action = 0;
  inf_marker.scale.x = resolution_[DIM_X];
  inf_marker.scale.y = resolution_[DIM_Y];
  inf_marker.scale.z = resolution_[DIM_Z];
  inf_marker.color.r = 0.0;
  inf_marker.color.g = 0.0;
  inf_marker.color.b = 1.0;
  inf_marker.color.a = 0.1;
  //inf_marker.lifetime = ros::Duration(30.0);

  inf_marker.points.reserve(100000);

  VoxelSet::iterator iter;
  for(iter = object_voxel_locations_.begin(); iter != object_voxel_locations_.end(); iter++)
  {
    int last = inf_marker.points.size();
    inf_marker.points.resize(last + 1);
    double nx, ny, nz;
    int x = (*iter).x();
    int y = (*iter).y();
    int z = (*iter).z();
    this->gridToWorld(x,y,z,nx, ny, nz);
    tf::Vector3 vec(nx,ny,nz);
    vec = cur*vec;
    inf_marker.points[last].x = vec.x();
    inf_marker.points[last].y = vec.y();
    inf_marker.points[last].z = vec.z();    
  }
}


void PropagationDistanceField::reset()
{
  VoxelGrid<PropDistanceFieldVoxel>::reset(PropDistanceFieldVoxel(max_distance_sq_));
  object_voxel_locations_.clear();
}

void PropagationDistanceField::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx=-1; dx<=1; ++dx)
  {
    for (int dy=-1; dy<=1; ++dy)
    {
      for (int dz=-1; dz<=1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        int3 n_point( dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n=0; n<2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx=-1; dx<=1; ++dx)
    {
      for (int dy=-1; dy<=1; ++dy)
      {
        for (int dz=-1; dz<=1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx=-1; tdx<=1; ++tdx)
          {
            for (int tdy=-1; tdy<=1; ++tdy)
            {
              for (int tdz=-1; tdz<=1; ++tdz)
              {
                if (tdx==0 && tdy==0 && tdz==0)
                  continue;
                if (n>=1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                    continue;
                  if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                    continue;
                }
                int3 n_point(tdx,tdy,tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          //printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz, neighborhoods_[n][direction_number].size());
        }
      }
    }
  }

}

int PropagationDistanceField::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx+1)*9 + (dy+1)*3 + dz+1;
}

int3 PropagationDistanceField::getLocationDifference(int directionNumber) const
{
  return direction_number_to_direction_[ directionNumber ];
}

SignedPropagationDistanceField::~SignedPropagationDistanceField()
{
}

SignedPropagationDistanceField::SignedPropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      DistanceField<SignedPropDistanceFieldVoxel>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, SignedPropDistanceFieldVoxel(max_distance,0))
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution;
}

int SignedPropagationDistanceField::eucDistSq(int3 point1, int3 point2)
{
  int dx = point1.x() - point2.x();
  int dy = point1.y() - point2.y();
  int dz = point1.z() - point2.z();
  return dx*dx + dy*dy + dz*dz;
}

void SignedPropagationDistanceField::addPointsToField(const std::vector<tf::Vector3>& points)
{
  // initialize the bucket queue
  positive_bucket_queue_.resize(max_distance_sq_+1);
  negative_bucket_queue_.resize(max_distance_sq_+1);

  positive_bucket_queue_[0].reserve(points.size());
  negative_bucket_queue_[0].reserve(points.size());

  for(int x = 0; x < num_cells_[0]; x++)
  {
    for(int y = 0; y < num_cells_[1]; y++)
    {
      for(int z = 0; z < num_cells_[2]; z++)
      {
        SignedPropDistanceFieldVoxel& voxel = getCell(x,y,z);
        voxel.closest_negative_point_.x() = x;
        voxel.closest_negative_point_.y() = y;
        voxel.closest_negative_point_.z() = z;
        voxel.negative_distance_square_ = 0;
      }
    }
  }

  // first mark all the points as distance=0, and add them to the queue
  int x, y, z, nx, ny, nz;
  int3 loc;
  int initial_update_direction = getDirectionNumber(0,0,0);
  for (unsigned int i=0; i<points.size(); ++i)
  {
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
    if (!valid)
      continue;
    SignedPropDistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.positive_distance_square_ = 0;
    voxel.negative_distance_square_ = max_distance_sq_;
    voxel.closest_positive_point_.x() = x;
    voxel.closest_positive_point_.y() = y;
    voxel.closest_positive_point_.z() = z;
    voxel.closest_negative_point_.x() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.closest_negative_point_.y() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.closest_negative_point_.z() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.location_.x() = x;
    voxel.location_.y() = y;
    voxel.location_.z() = z;
    voxel.update_direction_ = initial_update_direction;
    positive_bucket_queue_[0].push_back(&voxel);
  }

  // now process the queue:
  for (unsigned int i=0; i<positive_bucket_queue_.size(); ++i)
  {
    std::vector<SignedPropDistanceFieldVoxel*>::iterator list_it = positive_bucket_queue_[i].begin();
    while(list_it!=positive_bucket_queue_[i].end())
    {
      SignedPropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_positive_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->positive_distance_square_)
        {
          // update the neighboring voxel
          neighbor->positive_distance_square_ = new_distance_sq;
          neighbor->closest_positive_point_ = vptr->closest_positive_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          positive_bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    positive_bucket_queue_[i].clear();
  }


  for(unsigned int i = 0; i < points.size(); i++)
    {
      bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
      if(!valid)
        continue;

      for(int dx = -1; dx <= 1; dx ++)
      {
        for(int dy = -1; dy<= 1; dy ++)
        {
          for(int dz = -1; dz <= 1; dz++)
          {
            nx = x + dx;
            ny = y + dy;
            nz = z + dz;

            if(!isCellValid(nx, ny, nz))
              continue;

            SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);

            if(neighbor->closest_negative_point_.x() != SignedPropDistanceFieldVoxel::UNINITIALIZED)
            {
              neighbor->update_direction_ = initial_update_direction;
              negative_bucket_queue_[0].push_back(neighbor);
            }
          }
        }
      }

    }

  for (unsigned int i=0; i<negative_bucket_queue_.size(); ++i)
  {
    std::vector<SignedPropDistanceFieldVoxel*>::iterator list_it = negative_bucket_queue_[i].begin();
    while(list_it!=negative_bucket_queue_[i].end())
    {
      SignedPropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_negative_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->negative_distance_square_)
        {
          // update the neighboring voxel
          neighbor->negative_distance_square_ = new_distance_sq;
          neighbor->closest_negative_point_ = vptr->closest_negative_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          negative_bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    negative_bucket_queue_[i].clear();
  }

}

void SignedPropagationDistanceField::reset()
{
  VoxelGrid<SignedPropDistanceFieldVoxel>::reset(SignedPropDistanceFieldVoxel(max_distance_sq_, 0));
}

void SignedPropagationDistanceField::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx=-1; dx<=1; ++dx)
  {
    for (int dy=-1; dy<=1; ++dy)
    {
      for (int dz=-1; dz<=1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        int3 n_point( dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n=0; n<2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx=-1; dx<=1; ++dx)
    {
      for (int dy=-1; dy<=1; ++dy)
      {
        for (int dz=-1; dz<=1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx=-1; tdx<=1; ++tdx)
          {
            for (int tdy=-1; tdy<=1; ++tdy)
            {
              for (int tdz=-1; tdz<=1; ++tdz)
              {
                if (tdx==0 && tdy==0 && tdz==0)
                  continue;
                if (n>=1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                    continue;
                  if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                    continue;
                }
                int3 n_point(tdx,tdy,tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          //printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz, neighborhoods_[n][direction_number].size());
        }
      }
    }
  }



}

int SignedPropagationDistanceField::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx+1)*9 + (dy+1)*3 + dz+1;
}


}
