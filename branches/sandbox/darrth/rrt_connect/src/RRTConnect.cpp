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

/* Author: Ioan Sucan */

#include "rrt_connect/RRTConnect.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::control::RRTConnect::RRTConnect(const SpaceInformationPtr &si) : 
  Planner(si, "RRTConnect") {
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.directed = true;
  
  maxDistance_ = 0.0;
  siC_ = si;

  addIntermediateStates_ = false;
  maxConnectionAttempts_ = -1;
  
  Planner::declareParam<double>
    ("range", this, &RRTConnect::setRange, &RRTConnect::getRange);
  connectionPoint_ = std::make_pair<Motion *, Motion *>(NULL, NULL);
}

ompl::control::RRTConnect::~RRTConnect(void)
{
  freeMemory();
}

void ompl::control::RRTConnect::setup(void)
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);
  
  if (!tStart_)
    tStart_.reset(new NearestNeighborsGNAT<Motion*>());
  if (!tGoal_)
    tGoal_.reset(new NearestNeighborsGNAT<Motion*>());
  tStart_->setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, 
					   this, _1, _2));
  tGoal_->setDistanceFunction(boost::bind(&RRTConnect::distanceFunctionBackwards, 
					  this, _1, _2));
}

void ompl::control::RRTConnect::freeMemory(void)
{
  std::vector<Motion*> motions;
  
  if (tStart_)
    {
      tStart_->list(motions);
      for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
	  if (motions[i]->state)
	    si_->freeState(motions[i]->state);
	  if (motions[i]->control && motions[i]->free_control) {
	    siC_->freeControl(motions[i]->control);
	  }
	  delete motions[i];
        }
    }
  
  if (tGoal_)
    {
      tGoal_->list(motions);
      for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
	  if (motions[i]->state)
	    si_->freeState(motions[i]->state);
	  if (motions[i]->control && motions[i]->free_control) {
	    siC_->freeControl(motions[i]->control);
	  }
	  delete motions[i];
        }
    }
}

void ompl::control::RRTConnect::clear(void)
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (tStart_)
    tStart_->clear();
  if (tGoal_)
    tGoal_->clear();
  connectionPoint_ = std::make_pair<Motion *, Motion *>(NULL, NULL);
}

ompl::control::RRTConnect::GrowState ompl::control::RRTConnect::growTree
(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{

    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);
    
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
    // so we check that one first
    int cd; //if we're sampling backwards, cd will be negative
    Control *rctrl = rmotion->control;
    if (tgi.start) {
      //sample a random control 
      cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state,
				     rmotion->state);
    } else {
      cd = controlSampler_->sampleFrom(rctrl, nmotion->control, nmotion->state,
				       rmotion->state);
    }
    unsigned int nturns;
    Control *cc = NULL;
    if (addIntermediateStates_) {
      std::vector<base::State *> pstates;
      nturns = 
	siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
      if (nturns >= siC_->getMinControlDuration()) {
	Motion *lastmotion = nmotion;
	unsigned int p = 0;
	for ( ; p < pstates.size(); ++p) 
	  {
	    Motion *motion = new Motion();
	    motion->state = pstates[p];
	    if (p == 0) {
	      motion->control = siC_->allocControl();
	      siC_->copyControl(motion->control, rctrl);	
	      cc = motion->control;
	      motion->free_control = true;
	    } else {
	      motion->control = cc;
	      motion->free_control = false;
	    }
	    motion->steps = -1;
	    motion->parent = lastmotion;
	    motion->root = nmotion->root;
	    lastmotion = motion;
	    tree->add(motion);
	    if (tgi.start && tgi.goal && tgi.goal->isSatisfied(pstates[p])) {
	      logInform("Goal found during forward growth");
	      tgi.xmotion = lastmotion;
	      siC_->copyState(tgi.xstate, lastmotion->state);
	      return GOAL;
	    }
	  }
	tgi.xmotion = lastmotion;
	siC_->copyState(tgi.xstate, lastmotion->state);
      } 
    } else {
      nturns = siC_->propagateWhileValid(nmotion->state, rctrl, cd, rmotion->state);
      if (nturns < siC_->getMinControlDuration()) {
	return TRAPPED;
      }
      /* create a motion */
      Motion *motion = new Motion(siC_);
      siC_->copyState(motion->state, rmotion->state);
      siC_->copyControl(motion->control, rctrl);
      motion->steps = cd;
      motion->parent = nmotion;
      motion->root = nmotion->root;
      tgi.xmotion = motion;
      siC_->copyState(tgi.xstate, rmotion->state);
      
      tree->add(motion);
      if (tgi.start && tgi.goal && tgi.goal->isSatisfied(rmotion->state)) {
	logInform("Goal found during forward growth");
	return GOAL;
      }
    }
    logInform("TOTAL TURNS: %u, EXPECTED TURNS: %d", nturns, abs(cd));
    if (nturns < 2 || nturns < siC_->getMinControlDuration()) {
      return TRAPPED;
    }
    if (nturns < static_cast<unsigned int>(abs(cd))) {
      return ADVANCED;
    }
    return REACHED;
}

ompl::base::PlannerStatus ompl::control::RRTConnect::solve
(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = 
      dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
      logError("Unknown type of goal (or goal undefined)");
      return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    DirectedControlSamplerPtr dc = siC_->allocDirectedControlSampler();
    
    controlSampler_ = dynamic_cast<BiDirectedControlSampler *>(dc.get());

    if (!controlSampler_) {
      logError("Sampler is not bi-directional!");
      return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
      {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
      }

    if (tStart_->size() == 0)
      {
        logError("Motion planning start tree could not be initialized!");
        return base::PlannerStatus::INVALID_START;
      }
    
    if (!goal->couldSample())
      {
        logError("Insufficient states in sampleable goal region");
        return base::PlannerStatus::INVALID_GOAL;
      }
    
    if (!sampler_)
      sampler_ = si_->allocStateSampler();

    logInform("Starting with %d states", (int)(tStart_->size() + 
					       tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();
    tgi.goal = goal;
    
    Motion   *rmotion   = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl      = rmotion->control;
    bool startTree      = true;
    bool solved         = false;

    double size_to_goal_ratio = 2.0;
    if (addIntermediateStates_) {
      //we get a LOT more states in the goal tree
      size_to_goal_ratio = 500.0;
    }

    while (ptc() == false)
      {
        TreeData &tree      = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;
	
	logInform("Goals sampled: %u, Size of goal tree: %u, Size of start tree: %u",
		  pis_.getSampledGoalsCount(), tGoal_->size(), tStart_->size());

        if (tGoal_->size() == 0 || 
	    pis_.getSampledGoalsCount() < tGoal_->size() / size_to_goal_ratio)
	  {
	    logInform("Adding a goal state to the tree!");
            const base::State *st = tGoal_->size() == 
	      0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
	      {
                Motion* motion = new Motion(siC_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
	      } else 
	      {
		logWarn("Unable to get another goal state");
	      }
	    
            if (tGoal_->size() == 0)
	      {
                logError("Unable to sample any valid states for goal tree");
                break;
	      }
	  }
	
        /* sample random state */
        sampler_->sampleUniform(rstate);
	
	logInform("\nGrowing %s tree towards random state",
		  tgi.start ? "Forward" : "Backward");

        GrowState gs = growTree(tree, tgi, rmotion);
	switch (gs) {
	case TRAPPED:
	  logInform("\ngrow resulted in TRAPPED");
	  break;
	case REACHED:
	  logInform("\ngrow resulted in REACHED");
	  break;
	case ADVANCED:
	  logInform("\ngrow resulted in ADVANCED");
	  break;
	case GOAL:
	  logInform("\ngrow resulted in GOAL");
	  break;
	default:
	  logInform("\ngrow resulted in unknown state %u", gs);
	  break;
	}	  
	
        if (gs != TRAPPED)
	  {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;
	    
            /* attempt to connect trees */
	    si_->copyState(rstate, tgi.xstate);

            GrowState gsc = gs;
	    if (gs != GOAL) {
	      gsc = ADVANCED;
	      tgi.start = startTree;
	    }

	    int grows = 0;
            while (gsc == ADVANCED && (maxConnectionAttempts_ < 0 || 
				       grows < maxConnectionAttempts_)) {
	      logInform("\nGrowing %s tree towards tree we just extended", 
			tgi.start ? "Forward" : "Backward");
	      gsc = growTree(otherTree, tgi, rmotion);
	      grows++;
	    }	    

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion  = tgi.start ? addedMotion : tgi.xmotion;
	    
            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if ((gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root)) || gsc == GOAL)
	      {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not NULL. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                // if (startMotion->parent)
		//   startMotion = startMotion->parent;
                // else
		//   goalMotion = goalMotion->parent;
		if (gsc == REACHED) {
		  connectionPoint_ = std::make_pair<Motion *, Motion *>(startMotion, goalMotion);

		} else {
		  if (goal->isSatisfied(startMotion->state)) {
		    //double check
		    connectionPoint_ = std::make_pair<Motion *, Motion *>(startMotion, NULL);		    
		  } else {
		    logError("Tree returned GOAL but the goal was not found in the motion!");
		    continue;
		  }
		}

                /* construct the solution path */
                Motion *solution = connectionPoint_.first;
                std::vector<Motion*> mpath1;
                while (solution != NULL)
		  {
                    mpath1.push_back(solution);
                    solution = solution->parent;
		  }
		
                solution = connectionPoint_.second;
                std::vector<Motion*> mpath2;
                while (solution != NULL)
		  {
                    mpath2.push_back(solution);
                    solution = solution->parent;
		  }
		
                PathControl *path = new PathControl(si_);
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
		  path->append(mpath1[i]->state, mpath1[i]->control, 
			       mpath1[i]->steps*siC_->getPropagationStepSize());
                for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
		  path->append(mpath2[i]->state, mpath2[i]->control,
			       mpath2[i]->steps*siC_->getPropagationStepSize());
		
                pdef_->addSolutionPath(base::PathPtr(path), false, 0.0);
                solved = true;
                break;
	    }
	  }
      }

    siC_->freeState(tgi.xstate);
    siC_->freeState(rstate);
    siC_->freeControl(rctrl);
    delete rmotion;
    
    logInform("Created %u states (%u start + %u goal)", tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

//this function is really wrong
void ompl::control::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    

    // std::vector<Motion*> motions;
    // if (tStart_)
    //     tStart_->list(motions);

    // for (unsigned int i = 0 ; i < motions.size() ; ++i)
    // {
    //     if (motions[i]->parent == NULL)
    //         data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
    //     else
    //     {
    //         data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
    //                      base::PlannerDataVertex(motions[i]->state, 1));
    //     }
    // }

    // motions.clear();
    // if (tGoal_)
    //     tGoal_->list(motions);

    // for (unsigned int i = 0 ; i < motions.size() ; ++i)
    // {
    //     if (motions[i]->parent == NULL)
    //         data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
    //     else
    //     {
    //         // The edges in the goal tree are reversed to be consistent with start tree
    //         data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
    //                      base::PlannerDataVertex(motions[i]->parent->state, 2));
    //     }
    // }

    // // Add the edge connecting the two trees
    // data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
