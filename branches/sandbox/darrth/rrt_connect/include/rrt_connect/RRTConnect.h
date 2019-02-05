/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan*/

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRT_CONNECT_
#define OMPL_CONTROL_PLANNERS_RRT_RRT_CONNECT_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "rrt_connect/BiDirectedControlSampler.h"

namespace ompl
{
  
  namespace control
  {
    
    /**
       @anchor gRRTC
       @par Short description
       The basic idea is to grow to RRTs, one from the start and
       one from the goal, and attempt to connect them.
       @par External documentation
       J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844730">10.1109/ROBOT.2000.844730</a><br>
       <a href="http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246">[PDF]</a>
       <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
    */
    
    /** \brief RRT-Connect (RRTConnect) */
    class RRTConnect : public base::Planner
    {
    public:
      
      /** \brief Constructor */
      RRTConnect(const SpaceInformationPtr &si);
      
      virtual ~RRTConnect(void);
      
      virtual void getPlannerData(base::PlannerData &data) const;
      
      virtual base::PlannerStatus solve
	(const base::PlannerTerminationCondition &ptc);
      
      virtual void clear(void);

      /** \brief Set the range the planner is supposed to use.
	  
	  This parameter greatly influences the runtime of the
	  algorithm. It represents the maximum length of a
	  motion to be added in the tree of motions. */
      void setRange(double distance)
      {
	maxDistance_ = distance;
      }
      
      /** \brief Get the range the planner is using */
      double getRange(void) const
      {
	return maxDistance_;
      }
      
      /** \brief Set a different nearest neighbors datastructure */
      template<template<typename T> class NN>
	void setNearestNeighbors(void)
	{
	  tStart_.reset(new NN<Motion*>());
	  tGoal_.reset(new NN<Motion*>());
	}
      
      virtual void setup(void);

      /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself */
      bool getIntermediateStates(void) const
      {
	return addIntermediateStates_;
      }
      
      /** \brief Specify whether the intermediate states generated along motions are to be added to the tree itself */
      void setIntermediateStates(bool addIntermediateStates)
      {
	addIntermediateStates_ = addIntermediateStates;
      }

      void setMaxConnectionAttempts(int maxConnectionAttempts) {
	maxConnectionAttempts_ = maxConnectionAttempts;
      }

      int getMaxConnectionAttempts() const {
	return maxConnectionAttempts_;
      }

				    
      
    protected:
      
      /** \brief Representation of a motion */
      class Motion
      {
      public:
	
      Motion(void) : root(NULL), state(NULL), parent(NULL)
	  {
	    parent = NULL;
	    state  = NULL;
	    control = NULL;
	    steps = 0;
	    free_control = false;
	  }
	
      Motion(const SpaceInformationPtr si) : 
	root(NULL), state(si->allocState()), parent(NULL), 
	  control(si->allocControl())
	  {
	    steps = 0;
	    free_control = true;
	  }
	
	~Motion(void)
	  {
	  }
	
	const base::State *root;
	base::State       *state;
	Motion            *parent;
	Control           *control;
	int               steps;
	bool              free_control;

	
      };

      /** \brief A nearest-neighbor datastructure representing a tree of motions */
      typedef boost::shared_ptr< NearestNeighbors<Motion*> > TreeData;
      
      /** \brief Information attached to growing a tree of motions (used internally) */
      struct TreeGrowingInfo
      {
	const base::Goal *goal;
	base::State         *xstate;
	Motion              *xmotion;
	bool                 start;
      };

      /** \brief The state of the tree after an attempt to extend it */
      enum GrowState
      {
	/// no progress has been made
	TRAPPED,
	/// progress has been made towards the randomly sampled state
	ADVANCED,
	/// the randomly sampled state was reached
	REACHED,
	/// the goal was reached
	GOAL
      };
      
      /** \brief Free the memory allocated by this planner */
      void freeMemory(void);
      
      /** \brief Compute distance between motions (actually distance between contained states) */
      double distanceFunction(const Motion* a, const Motion* b) const
      {
	return si_->distance(a->state, b->state);
      }

      /** \brief Compute distance between motions (actually distance between contained states) */
      double distanceFunctionBackwards(const Motion* a, const Motion* b) const
      {
	return si_->distance(b->state, a->state);
      }

      
      /** \brief Grow a tree towards a random state */
      GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);
      
      /** \brief State sampler */
      base::StateSamplerPtr         sampler_;
      
      /** \brief The start tree */
      TreeData                      tStart_;
      
      /** \brief The goal tree */
      TreeData                      tGoal_;
      
      /** \brief The maximum length of a motion to be added to a tree */
      double                        maxDistance_;
      
      /** \brief The random number generator */
      RNG                           rng_;
      
      /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
      std::pair<Motion *, Motion *>      connectionPoint_;

      SpaceInformationPtr             siC_;

      BiDirectedControlSampler *controlSampler_;

      bool addIntermediateStates_;

      int maxConnectionAttempts_;
    };
    
  }
}

#endif
