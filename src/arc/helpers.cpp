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

#include <traxxs/arc/helpers.hpp>

#include <stdexcept>

#include <traxxs/constants.hpp>

traxxs::arc::ArcConditions traxxs::arc::integrate( const traxxs::arc::JerkSegment& segment, const traxxs::arc::ArcConditions& c_in )
{
  arc::ArcConditions c_out;
  integrate( segment, c_in, c_out );
  return c_out;
}

bool traxxs::arc::integrate( const traxxs::arc::JerkSegment& segment, const traxxs::arc::ArcConditions& c_in, traxxs::arc::ArcConditions& c_out ) 
{
  return arc::integrate( std::list< arc::JerkSegment >{segment}, c_in, c_out );
}

bool traxxs::arc::integrate( const std::list<traxxs::arc::JerkSegment>& segments, const traxxs::arc::ArcConditions& c_in, traxxs::arc::ArcConditions& c_out, double t /*= std::nan("")*/ ) 
{
  double dt, j;
  c_out = c_in;
  double t_total = 0.0;
  for ( const auto& segment : segments ) {
    dt = segment.duration;
    if ( !std::isnan( t ) && t >= t_total && t < (t_total + dt) ) // we are in the last segment
      dt = std::max( 0.0, t - t_total );
    j = segment.j;
    c_out.s = c_out.s + dt * c_out.ds + dt*dt/2.0 * c_out.dds + dt*dt*dt/6.0 * j;
    c_out.ds = c_out.ds + dt * c_out.dds + dt*dt/2.0 * j;
    c_out.dds = c_out.dds + dt * j ;
    c_out.j = j;
    if ( !std::isnan( c_out.t ) )
      c_out.t += dt;
    t_total += dt;
    if ( dt < segment.duration ) // we have reached the last segment
      break;
  }
  return true;
}


double traxxs::arc::findJerkDurationToReachVelocity( const traxxs::arc::ArcConditions& c_in, double v_target )
{
  double t = 0;
  if ( std::fabs( c_in.j ) < constants::kZero ) {
    if ( std::fabs( c_in.dds ) < constants::kZero ) 
      t = 0.0;
    else 
      t = ( v_target - c_in.ds ) / c_in.dds;
  } else {
    double a, b, c; // a.t^2 + b.t + c = 0
    a = c_in.j / 2.0;
    b = c_in.dds;
    c = c_in.ds - v_target;
    double delta = b*b - 4.0 * a * c;
    if ( delta < 0 )
      throw std::runtime_error( "Infeasibility encountered in findJerkDurationToReachVelocity." ); // shold NEVER happen
    double t1, t2;
    t1 = ( -b - std::sqrt( delta ) ) / ( 2 * a );
    t2 = ( -b + std::sqrt( delta ) ) / ( 2 * a );
    if ( t1 < 0 )
      t = t2;
    else if ( t2 < 0 )
      t = t1;
    else 
      t = std::min( t1, t2 );
  }
  return t;
}

std::list< traxxs::arc::JerkSegment > traxxs::arc::maxJerkBrakeToLimits( const traxxs::arc::ArcConditions& c_in, const traxxs::arc::ArcConditions& c_max )
{
  std::list< arc::JerkSegment > segments;
  
  if ( std::isnan( c_in.dds ) || std::isnan( c_in.ds ) || std::isnan( c_max.j ) || std::isnan( c_max.dds ) || std::isnan( c_max.ds ) )
    throw std::invalid_argument( "maxJerkBrake expects ds, dds inputs and ds, dds, j limits." );
  
  if ( std::fabs( c_max.j ) < constants::kZero )
    throw std::invalid_argument( "c_max.j too close to zero in maxJerkBrake." ); // division by zero 
    
  double target_v = std::nan("");
  double target_a = std::nan("");
  arc::JerkSegment seg;
  arc::ArcConditions cur;
  
  //
  // 1st segment: reach the closest acceleration bound (will be of duration 0 if the initial acceleration is already within the bounds
  //
  if ( c_in.dds > +c_max.dds ) // above its limit
    target_a = +c_max.dds; // next target is the upper bound
  if ( c_in.dds < -c_max.dds ) // below its limit
    target_a = -c_max.dds; // next target is the lower bound
    
  if ( std::isnan( target_a ) && c_in.ds <= c_max.ds && c_in.ds >= -c_max.ds ) // no need to brake, everything is within the limits
    return segments;
  
  seg = JerkSegment();
  seg.j = 0;
  seg.duration = 0;
  if ( !std::isnan( target_a ) ) {
    if ( c_in.dds < target_a ) // we need j > 0 to make dds increase
      seg.j = +c_max.j;
    if ( c_in.dds > target_a ) // we need j < 0 to make dds decrease
      seg.j = -c_max.j;
    seg.duration = ( target_a - c_in.dds ) / seg.j ;
    segments.push_back( seg );
  }
  // otherwise, ignore (1st seg duration = 0)
  
  // at this point, acceleration is within the bounds. Only velocity might be out of bounds
  cur = arc::integrate( seg, c_in );
  
  //
  // 2nd segment: reach a velocity limit OR another acceleration limit (in the direction of the velocity limit. This is the main deceleration phase
  //
  target_v = cur.ds; // by default, we keep ds as it is
  // otherwise, if above/below limits, set target to upper/lower bound
  if ( cur.ds > +c_max.ds )
    target_v = +c_max.ds;
  if ( cur.ds < -c_max.ds )
    target_v = -c_max.ds;
  
  // deduce a target acceleration based on velocity
  target_a = 0.0; // by default, we keep constant velocity
  if ( cur.ds > target_v ) // we need to decelerate, i.e. reach dds = -maxAcc
    target_a = -c_max.dds; 
  if ( cur.ds < target_v ) // we need to accelerate, i.e. reach dds = +maxAcc
    target_a = +c_max.dds; 
  
  seg = JerkSegment();
  seg.j = 0;
  seg.duration = 0;
  if ( cur.dds < target_a ) // we need j > 0
    seg.j = +c_max.j;
  if ( cur.dds > target_a ) // we need j < 0
    seg.j = -c_max.j;
  
  // now find the duration of the 2nd segment
  // first the duration to reach the target velocity
  cur.j = seg.j;
  double t_v = findJerkDurationToReachVelocity( cur, target_v );
  // to be compared to the duration to reach one of the dds bounds
  target_a = 0;
  if ( seg.j > 0 ) // we might hit the upper bound
    target_a = +c_max.dds;
  else if ( seg.j < 0 ) // we might hit the lower bound
    target_a = -c_max.dds;
  double t_a = constants::kInf;
  if ( std::fabs( seg.j ) > constants::kZero ) 
    t_a = ( target_a - cur.dds ) / seg.j;
  
  // we take the minimum. If acceleration hits its bound faster than velocity, the third segment will keep dds constant to let the velocity decrease/increase until compliance
  seg.duration = std::fmin( t_v, t_a );
  segments.push_back( seg );
  
  cur = arc::integrate( seg, cur );
  
  //
  // 3rd segment: constant acceleration until the velocity limit is reached (will be 0 if the velocity limit has been reached during the 2nd segment)
  // while the final state might be compliant with bounds, this does not mean that it is a viable state.
  // e.g. velocity is above bounds and acceleration is positive. 
  // The algorithm will strongly decelerate until velocity reaches the upper bound. 
  // If jerk is too limited, the time for acceleration to reach 0 for example will lead velocity below the lower bound.
  // to fix this, the deceleration should be slower according to the jerk limit
  
  //   cur.j = seg.j; // = 0, constant acceleration
  //   seg.duration = findJerkDurationToReachVelocity( cur, target_v ); // we keep the previous target velocity, in case not reached yet in the second segment
  
  //
  // Instead, we prefer as a third segment a linear acceleration profile to reach the velocity limit with acc = 0
  //
  seg = JerkSegment();
  seg.j = 0;
  seg.duration = 0;
  
  if ( std::fabs( cur.dds ) > constants::kZero ) {
    seg.duration =  2.0 * ( target_v - cur.ds ) / cur.dds;
    if ( std::fabs( seg.duration ) > constants::kZero )
      seg.j = -cur.dds / seg.duration;
  }
  
  segments.push_back( seg );
  
  cur = arc::integrate( seg, cur );
  
  //
  // Optional segment: make sure we reach acceleration = 0 at maximum jerk
  //
  /** \fixme this does not guarantee that after this segment the velocity is not out of bounds ? */
  seg = JerkSegment();
  seg.j = 0;
  seg.duration = 0;
  if ( std::fabs( cur.dds ) > constants::kZero ) {
    if  ( cur.dds < 0 ) // we need j > 0
      seg.j = +c_max.j;
    if  (cur.dds > 0 ) // we need j < 0
      seg.j = -c_max.j;
    seg.duration = -cur.dds / seg.j;
    segments.push_back( seg );
  }
    
  
  
  return segments;
}
