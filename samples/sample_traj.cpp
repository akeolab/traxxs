/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/trajectory/trajectory.hpp>
#include "arctrajgensoftmotion.hpp"
#include "samples_helpers.hpp"

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

int main(void) {
  
//   std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  path::PathConditions pt_start(2), pt_wpt(2), pt_end(2), wpt_blend_s(2), wpt_blend_e(2);
  sptr< traxxs::path::PathSegment > seg_start, seg_blend, seg_end;
  
  pt_start.x  << 0, 0 ;
  pt_wpt.x    << 1, 0 ;
  pt_end.x    << 1, 1 ;
  
  pt_start.dx << 0, 0;
  pt_end.dx   << 0, 0;
  
  wpt_blend_s = pt_start;
  wpt_blend_e = pt_end;
  blendStartEndFromWaypoints( wpt_blend_s, wpt_blend_e, pt_wpt );
  
  path::PathBounds path_bounds(2);
  path_bounds.dx << 1.0, 1.0;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  // first the blend, so that we can use the start and end of the blend
  seg_blend = std::make_shared< traxxs::path::CircularBlend >( wpt_blend_s, wpt_blend_e, path_bounds,  pt_wpt.x, 0.1 );

  seg_blend->init();
  wpt_blend_s.x = seg_blend->getConfiguration( 0.0 );
  wpt_blend_e.x = seg_blend->getConfiguration( seg_blend->getLength() );
  
  seg_start = std::make_shared< traxxs::path::LinearSegment >( pt_start, wpt_blend_s, path_bounds );
  seg_end = std::make_shared< traxxs::path::LinearSegment >( wpt_blend_e, pt_end, path_bounds );
  
  std::vector< sptr< traxxs::path::PathSegment > > segments{ seg_start, seg_blend, seg_end };
  for( auto& seg : segments )
    seg->init();
  traxxs::path::Path path( segments );
  path.init();
  

  std::vector< std::shared_ptr< arc::ArcTrajGen > > arctrajgens;
  for ( unsigned int i = 0; i < path.getSegments().size(); ++i )
    arctrajgens.push_back( std::make_shared< ArcTrajGenSoftMotion >() );
  trajectory::Trajectory trajectory;
  
  trajectory.set( path.getSegments(), arctrajgens );
  
//   return 0;
  
  int seg_idx;
  trajectory::TrajectoryState state;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  for ( double t = 0; t < 2.5; t+=0.0001 ) {
    if ( !trajectory.getArcConditions( t, conds, seg, &seg_idx ) )
      break;
    trajectory.getState( t, state );
    std::cout << t << ";" << seg_idx
      << ";" << conds.s << ";" << conds.ds << ";" << conds.dds  << ";" << conds.j 
      << ";" << toCSV( state.x )
      << ";" << toCSV( state.dx ) << ";" << toCSV( state.ddx ) << ";" << toCSV( state.j ) << std::endl;
  }
  
  
  
}
