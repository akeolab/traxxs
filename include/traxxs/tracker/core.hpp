#ifndef TRAXXS_TRACKER_CORE_H
#define TRAXXS_TRACKER_CORE_H

#include <traxxs/trajectory/trajectory.hpp>

namespace traxxs {
namespace tracker {

enum class TrackerStatus
{
    Undefined       = -1,
    Error           = 0,
    NotInitialized  , 
    Stalled         ,
    Incremented     ,
    Finished    
};

/** 
 * \brief An abstract Tracker interface 
 */
class TrackerBase
{
 public:
  TrackerBase(){};
  virtual ~TrackerBase(){};
 
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
