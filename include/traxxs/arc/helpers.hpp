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
