/**
 * In this file we check that building a path effectively ensures:
 *  - that continuity is attempted if possible (x, dx, ddx, dddx)
 *  - that bounds on the path are correctly converted into bounds on the arc. 
 */
#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/path/path.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs::constants;

int main(void) {
  
  bool ret_cont_pos = true;
  bool ret_cont_vel = true;
  bool ret_cont_acc = true;
  bool ret_bounds = true;
  
  int ret_count_dx_nonnull = 0;
  
  sptr< traxxs::path::PathSegment > seg_start, seg_1_blend, seg_2, seg_3, seg_end;
  
  traxxs::path::PathWaypoint wpt_start, wpt_1, wpt_2, wpt_3, wpt_end, wpt_blend_s, wpt_blend_e;
  wpt_start.x = Eigen::Vector2d();
  wpt_1.x = Eigen::Vector2d();
  wpt_2.x = Eigen::Vector2d();
  wpt_3.x = Eigen::Vector2d();
  wpt_end.x = Eigen::Vector2d();
  
  wpt_start.x  << 0, 0 ;
  wpt_1.x      << 1, 0 ;
  wpt_2.x      << 1, 1 ;
  wpt_3.x      << .5, 1 ;
  wpt_end.x    << 0, 1 ;
  
  traxxs::path::PathBounds path_bounds;
  Eigen::Vector2d ones = Eigen::VectorXd::Ones(2);
  path_bounds.dx  = 1 * ones;
  path_bounds.ddx = 10 * ones;
  path_bounds.j   = 100 * ones;
  
  // first the blend, so that we can use the start and end of the blend
  seg_1_blend = std::make_shared< traxxs::path::CircularBlend >( wpt_start, wpt_2, path_bounds,  wpt_1.x, 0.1 );
  
  
  seg_1_blend->init();
  wpt_blend_s.x = seg_1_blend->getConfiguration( 0.0 );
  wpt_blend_e.x = seg_1_blend->getConfiguration( seg_1_blend->getLength() );
  
  std::cout<< wpt_blend_s.x.transpose() << std::endl;
  std::cout<< wpt_blend_e.x.transpose() << std::endl;
  
  seg_start = std::make_shared< traxxs::path::LinearSegment >( wpt_start, wpt_blend_s, path_bounds );
  // seg_1_blend = 
  seg_2 = std::make_shared< traxxs::path::LinearSegment >( wpt_blend_e, wpt_2, path_bounds );
  seg_3 = std::make_shared< traxxs::path::LinearSegment >( wpt_2, wpt_3, path_bounds );
  seg_end = std::make_shared< traxxs::path::LinearSegment >( wpt_3, wpt_end, path_bounds );
  
  std::vector< sptr< traxxs::path::PathSegment > > segments{ seg_start, seg_1_blend, seg_2, seg_3, seg_end };
  for( auto& seg : segments )
    seg->init();
  traxxs::path::Path path( segments );
  path.init();
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  sptr< traxxs::path::PathSegment > cur, prev;
  Eigen::VectorXd x_m, x_p, dx_m, dx_p, ddx_m, ddx_p, j_m, j_p;
  traxxs::arc::ArcConditions conds_m, conds_p;
  
  for ( unsigned int iseg=0; iseg<segments.size(); ++iseg ) {
    cur = segments[iseg];
    std::cout << "Segment " << iseg << " :\nfrom\t" << cur->getStartArcConditions() << "\nto\t" << cur->getEndArcConditions() << std::endl;
  }
  
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
    
    ret_cont_pos &= ( (x_p - x_m).norm() < kZero );
    ret_cont_vel &= ( (dx_p - dx_m).norm() < kZero );
    ret_cont_acc &= ( (ddx_p - ddx_m).norm() < kZero );
    
    if ( dx_p.norm() > kZero ) ret_count_dx_nonnull++;
  }
  
  // check bounds
  std::cout << "------- Checking bounds " << std::endl;
  traxxs::arc::ArcConditions arc_bounds;
  Eigen::VectorXd fp, fpp, fppp;
  Eigen::VectorXd dx, ddx, dddx;
  for ( const auto& seg : segments ) {
    arc_bounds = seg->getArcBounds();
    fp = seg->getDerivativeCwiseAbsMax( 1 );
    fpp = seg->getDerivativeCwiseAbsMax( 2 );
    fppp = seg->getDerivativeCwiseAbsMax( 3 );
    
    dx = fp * arc_bounds.ds;
    ddx = fp * arc_bounds.dds + fpp * arc_bounds.ds*arc_bounds.ds;
    dddx = fp * arc_bounds.j + 3.0 * fpp * arc_bounds.ds*arc_bounds.dds + fppp * arc_bounds.ds*arc_bounds.ds*arc_bounds.ds;
    
    if ( ( path_bounds.dx - dx ).minCoeff() < 0 ) // negative
      std::cerr << "Velocity bound fail with residual " << ( path_bounds.dx - dx ).minCoeff() << std::endl;
    if ( ( path_bounds.ddx - ddx ).minCoeff() < 0 ) // negative
      std::cerr << "Acceleration bound fail with residual " << ( path_bounds.ddx - ddx ).minCoeff() << std::endl;
    if ( ( path_bounds.j - dddx ).minCoeff() < 0 ) // negative
      std::cerr << "Jerk bound fail with residual " << ( path_bounds.j - dddx ).minCoeff() << std::endl;
    
    ret_bounds &= ( ( path_bounds.dx - dx ).minCoeff() >= 0 );
    ret_bounds &= ( ( path_bounds.ddx - ddx ).minCoeff() >= 0 );
    ret_bounds &= ( ( path_bounds.j - dddx ).minCoeff() >= 0 );
    
    std::cout << "Bounds: ds=" << arc_bounds.ds << ",\tdds=" << arc_bounds.dds << ",\tj=" << arc_bounds.j << std::endl;
  }
  
  std::cout << "--------\nSUMMARY\n--------" << std::endl;
  if ( ret_cont_pos ) std::cout << "All position continuities respected." << std::endl;
  else                std::cerr << "Position continuities not respected." << std::endl;
  if ( ret_cont_vel ) std::cout << "All velocity continuities respected." << std::endl;
  else                std::cerr << "Velocity continuities not respected." << std::endl;
  if ( ret_cont_acc ) std::cout << "All acceleration continuities respected." << std::endl;
  else                std::cerr << "Acceleration continuities not respected." << std::endl;
  
  if ( ret_bounds ) std::cout << "All path bounds respected." << std::endl;
  else              std::cerr << "Path bounds not respected." << std::endl;
  
  std::cout << ret_count_dx_nonnull << "/" << (segments.size()-1) << " non-null continuities." << std::endl;
  
}
