/**
 * In this file we check that stacking path segments 
 * can effectively ensure non-null velocity continuity
 * by using SmoothStep7 segments
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <path/path.hpp>
#include <path/segment.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

int main(void) {
  
  
  sptr< PathSegment > top_seg_start, top_seg_1_blend, top_seg_end;
  sptr< PathSegment > bottom_seg_start, bottom_seg_1_blend, bottom_seg_end;
  
  sptr< PathSegment > stack_seg_start, stack_seg_1_blend, stack_seg_end;
  
  PathConditions wpt_start, wpt_1, wpt_end, wpt_blend_s, wpt_blend_e;
  PathConditions bottom_wpt_start, bottom_wpt_1, bottom_wpt_end;
  wpt_start.x = Eigen::Vector2d();
  wpt_1.x = Eigen::Vector2d();
  wpt_end.x = Eigen::Vector2d();
  
  bottom_wpt_start.x = Eigen::Matrix<double, 1, 1>();
  bottom_wpt_1.x = Eigen::Matrix<double, 1, 1>();
  bottom_wpt_end.x = Eigen::Matrix<double, 1, 1>();
  
  wpt_start.x  << 0, 0 ;
  wpt_1.x      << 1, 0 ;
  wpt_end.x    << 1, 1 ;
  
  bottom_wpt_start.x << 0;
  bottom_wpt_1.x     << 1;
  bottom_wpt_end.x   << 0;
  
  PathBounds path_bounds, bottom_path_bounds;
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(2);
  path_bounds.dx  = 1 * ones;
  path_bounds.ddx = 10 * ones;
  path_bounds.j   = 100 * ones;
  
  ones = Eigen::VectorXd::Ones(1);
  bottom_path_bounds.dx  = 1 * ones;
  bottom_path_bounds.ddx = 10 * ones;
  bottom_path_bounds.j   = 100 * ones;
  
  //
  // the top segments
  //
  // first the blend, so that we can use the start and end of the blend
  top_seg_1_blend = std::make_shared< CircularBlend >( wpt_start, wpt_end, path_bounds, wpt_1.x, 0.1 );
  top_seg_1_blend->init();
  wpt_blend_s.x = top_seg_1_blend->getConfiguration( 0.0 );
  wpt_blend_e.x = top_seg_1_blend->getConfiguration( top_seg_1_blend->getLength() );
  
  top_seg_start = std::make_shared< LinearSegment >( wpt_start, wpt_blend_s, path_bounds );
  // seg_1_blend = 
  top_seg_end = std::make_shared< LinearSegment >( wpt_blend_e, wpt_end, path_bounds );
  
  //
  // the bottom segments
  //
  bottom_seg_start = std::make_shared< SmoothStep7 >( bottom_wpt_start, bottom_wpt_1, bottom_path_bounds );
  bottom_seg_1_blend = std::make_shared< SmoothStep7 >( bottom_wpt_1, bottom_wpt_1, bottom_path_bounds );
  bottom_seg_end = std::make_shared< SmoothStep7 >( bottom_wpt_1, bottom_wpt_end, bottom_path_bounds );
  
  //
  // Stack them
  // 
  stack_seg_start = std::make_shared< StackedSegments >( top_seg_start, bottom_seg_start );
  stack_seg_1_blend = std::make_shared< StackedSegments >( top_seg_1_blend, bottom_seg_1_blend );
  stack_seg_end = std::make_shared< StackedSegments >( top_seg_end, bottom_seg_end );
  
  std::vector< sptr< PathSegment > > segments{ stack_seg_start, stack_seg_1_blend, stack_seg_end };
  for ( auto& seg : segments )
    seg->init();
  Path path( segments );
  path.init();
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  sptr< PathSegment > cur, prev;
  Eigen::VectorXd x_m, x_p, dx_m, dx_p, ddx_m, ddx_p, j_m, j_p;
  ArcConditions conds_m, conds_p;
  
  // check continuity
  for ( unsigned int iseg=1; iseg<segments.size(); ++iseg ) {
    cur = segments[iseg];
    prev = segments[iseg-1];
    std::cout << "------- Segments " << iseg-1 << ", " << iseg << std::endl;
    conds_m = prev->getEndArcConditions();
    conds_p = cur->getStartArcConditions();
    
    x_m = prev->getConfiguration( prev->getLength() );
    x_p = cur->getConfiguration( 0.0 );
    
    dx_m = conds_m.ds * prev->getDerivative( 1, prev->getLength() );
    dx_p = conds_p.ds * cur->getDerivative( 1, 0.0 );
    
    ddx_m = conds_m.dds * prev->getDerivative( 1, prev->getLength() ) + conds_m.ds*conds_m.ds * prev->getDerivative( 2, prev->getLength() );
    ddx_p = conds_p.dds * cur->getDerivative( 1, 0.0 ) + conds_p.ds*conds_p.ds * cur->getDerivative( 2, 0.0 );
    
    if ( (x_p - x_m).norm() > kZero ) 
      std::cerr << "Position continuity fail." << std::endl;
    if ( (dx_p - dx_m).norm() > kZero ) 
      std::cerr << "Velocity continuity fail." << std::endl;
    if ( (ddx_p - ddx_m).norm() > kZero ) 
      std::cerr << "Acceleration continuity fail." << std::endl;
    
    std::cout << "Conds at (-): ds=" << conds_m.ds << ", dds=" << conds_m.dds << ", j=" << conds_m.j << std::endl;
    std::cout << "Conds at (+): ds=" << conds_p.ds << ", dds=" << conds_p.dds << ", j=" << conds_p.j << std::endl;
    std::cout << "x continuity:\tdelta=" << (x_p - x_m).norm() << std::endl;
    std::cout << "dx continuity:\tdelta=" << (dx_p - dx_m).norm() << ", |dx|=" << dx_p.norm() << std::endl;
    std::cout << "ddx continuity:\tdelta=" << (ddx_p - ddx_m).norm() << ", |ddx|=" << ddx_p.norm() << std::endl;
  }
  
}
