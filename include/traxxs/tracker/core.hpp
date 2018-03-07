// This file is a part of the traxxs framework.
// Copyright 2018, AKEOLAB S.A.S.
// Main contributor(s): Aurelien Ibanez, aurelien@akeo-lab.com
// 
// This software is a computer program whose purpose is to help create and manage trajectories.
// 
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use, 
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info". 
// 
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability. 
// 
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or 
// data to be ensured and,  more generally, to use and operate it in the 
// same conditions as regards security. 
// 
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

#ifndef TRAXXS_TRACKER_CORE_H
#define TRAXXS_TRACKER_CORE_H

#include <traxxs/trajectory/trajectory.hpp>

namespace traxxs {
namespace tracker {

enum class TrackerStatus
{
    Undefined       = -1, // not defined
    Error           = 0,  // an error occurred at the tracker or trajectory level
    NotInitialized  ,     // unitialized status
    Stalled         ,     // the trajectory is stalling, i.e. we did not increment
    Incremented     ,     // the trajectory has been incremented: we got a new desired state
    Finished              // the end of the trajectory has been reached
};

enum class TrackerValidatorStatus
{
    Failure         = -1,
    Error           = 0,
    Success         = 1
};

/** 
 * \brief An abstract Tracker interface 
 */
class Tracker
{
 public:
  Tracker(){};
  virtual ~Tracker(){};
 
 public: // the non-virtual public interface
  /**
   * \brief gets the next state at +dt for current state 
   * \param[in]   dt the time increment at which we fetch the new state
   * \param[in]   current_state the current system state, might be unused
   * \param[out]  new_state_out the next state
   */
  virtual TrackerStatus next( double dt, const trajectory::TrajectoryState& current_state, trajectory::TrajectoryState& new_state_out ) final { 
    next_last_status_ = this->do_next( dt, current_state, new_state_out ); 
    return next_last_status_;
  }
  
  /** 
   * \brief resets the Tracker
   * \param[in]   trajectory optional: the new trajectory, use nullptr to keep the previous trajectory
   * \warning you might need to reset the trajectory itself yourself.
   */
  virtual bool reset( std::shared_ptr< trajectory::Trajectory > trajectory = nullptr ) final {
    if ( trajectory != nullptr )
      trajectory_ = trajectory;
    current_virtual_t_ = 0.0;
    next_last_status_ = TrackerStatus::Undefined;
    return this->do_reset( trajectory );
  }
  
 public:
  void setTrajectory( std::shared_ptr< trajectory::Trajectory >& trajectory ) { this->trajectory_ = trajectory; }
  std::shared_ptr< trajectory::Trajectory > getTrajectory() { return this->trajectory_; }
  double getVirtualTime() { return this->current_virtual_t_; }
     
 protected: // the (pure virtual) implementation
  virtual TrackerStatus do_next( double dt, const trajectory::TrajectoryState& current_state, trajectory::TrajectoryState& new_state_out ) = 0;
  virtual bool do_reset( std::shared_ptr< trajectory::Trajectory > trajectory = nullptr ) { return true; };
  
 protected:
  /** \brief the trajectory the Tracker tracks */
  std::shared_ptr< trajectory::Trajectory > trajectory_ = nullptr;
  /** \brief the internal virtual time */
  double current_virtual_t_ = 0.0;
  /** \brief the last output trackerstatus */
  TrackerStatus next_last_status_ = TrackerStatus::Undefined;
    
};

} // namespace tracker
} // namespace traxxs 

#endif //TRAXXS_TRACKER_CORE_H
