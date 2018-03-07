#ifndef TRAXXS_TRACKER_TRACKER_TIME_H
#define TRAXXS_TRACKER_TRACKER_TIME_H

#include <traxxs/tracker/core.hpp>

namespace traxxs {
namespace tracker {

/** 
 * \brief A Time-pursuit trajectory tracker
 * Simply, the tracker keeps track of the virtual time.
 * On every call to TrackerTimePursuit::next(), the virtual time will be incremented by the input dt until the end of the trajectory is met
 */
class TrackerTimePursuit : public TrackerBase
{
 public:
  TrackerTimePursuit() : TrackerBase() {};
  
 protected:
  virtual TrackerStatus do_next( double dt, const trajectory::TrajectoryState& current_state, trajectory::TrajectoryState& new_state_out ) override {
    bool is_beyond;
    if ( this->trajectory_ == nullptr )
      return TrackerStatus::NotInitialized;
    bool ret = this->trajectory_->getState( this->current_virtual_t_ + dt, new_state_out, nullptr, &is_beyond );
    if ( !ret )
      return TrackerStatus::Error;
    // if beyond
    if ( is_beyond )
      return TrackerStatus::Finished;
    // keep track of the increment
    this->current_virtual_t_ += dt;
    return TrackerStatus::Incremented;
  }
};

} // namespace tracker
} // namespace traxxs

#endif //TRAXXS_TRACKER_TRACKER_TIME_H
