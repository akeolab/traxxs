#ifndef TRAXXS_ARC_HELPERS_H
#define TRAXXS_ARC_HELPERS_H

#include <list>

#include "arctrajgen.hpp"

namespace traxxs {
namespace arc {
  
/** \brief a segment at constant jerk */
struct JerkSegment 
{
  double duration;
  double j;
};

/** \brief integrates a JerkSegment from c_in, return in c_out */
bool integrate( const arc::JerkSegment& segment, const arc::ArcConditions& c_in, arc::ArcConditions& c_out );
arc::ArcConditions integrate( const arc::JerkSegment& segment, const arc::ArcConditions& c_in );
/** \brief integrates a list of JerkSegment from c_in, return in c_out */
bool integrate( const std::list< arc::JerkSegment >& segments, const arc::ArcConditions& c_in, arc::ArcConditions& c_out, double t = std::nan("") );

/** \brief finds the duration to reach v_target from c_in at constant jerk c_in.j */
double findJerkDurationToReachVelocity( const arc::ArcConditions& c_in, double v_target );

/**
 * \brief computes 3 jerk segments with braking at ALMOST maximum jerk in order to reach conditions compliant with c_max from c_in
 * 
 * The focus is put on reaching a null acceleration to maximize viability
 * 
 * 1st segment: reach the closest acceleration bound (will be of duration 0 if the initial acceleration is already within the bounds
 * 2nd segment: reach a velocity limit OR another acceleration limit (in the direction of the velocity limit. This is the main deceleration phase
 * 3rd segment: linear acceleration until the velocity limit is reached with null acceleration
 * Optional segment: linear acceleration until null acceleration is reached
 * 
 * \note this is time-suboptimal: in the last segment(s), the maximum jerk is not used to ensure viability.
 * \todo consider using a trajectory generator for this. Any acc-bounded generator will work, since we work with velocities (objective: reach closest bound with null acceleration)
 * 
 * \note only c_in.ds and c_in.dds will be used as inputs
 * \note c_max must have ds, dds and j set
 */ 
std::list< arc::JerkSegment > maxJerkBrakeToLimits( const arc::ArcConditions& c_in, const arc::ArcConditions& c_max );
  
} // namespace arc
} // namespace traxxs

#endif // TRAXXS_ARC_HELPERS_H
