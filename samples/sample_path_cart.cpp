/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <path/path.hpp>
#include <path/segment.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

int main(void) {
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  CartesianPathConditions pt_start, pt_wpt, pt_wpt_l, pt_wpt_r, pt_end;
  
  sptr< CartesianSegment< LinearSegment, SmoothStep7 > > seg_start, seg_end; 
  sptr< CartesianSegment< CircularBlend, SmoothStep7 > > seg_blend;
  
  pt_start.position.p  << 0, 0, 0;
  pt_wpt.position.p    << 1, 0, 0;
  pt_end.position.p    << 1, 1, 0;
  
  pt_start.position.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.position.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.position.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
  pt_wpt_l = pt_start;
  pt_wpt_l.position.q = pt_wpt.position.q;
  pt_wpt_r = pt_end;
  pt_wpt_r.position.q = pt_wpt.position.q;
  
  PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 0.1;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  ArcConditions tmp;
  seg_blend = std::make_shared< CartesianSegment< CircularBlend, SmoothStep7 > >( pt_wpt_l, pt_wpt_r, path_bounds, pt_wpt.position.p, 0.1 );
  tmp.s = 0.0;
  seg_blend->getPosition(tmp);
  pt_wpt_l.position = CartesianPoint( seg_blend->getPosition(tmp) ) ;
  seg_start = std::make_shared< CartesianSegment< LinearSegment, SmoothStep7 > >( pt_start, pt_wpt_l, path_bounds );
  tmp.s = seg_blend->getLength();
  pt_wpt_r.position = CartesianPoint( seg_blend->getPosition(tmp) ) ;
  seg_end = std::make_shared< CartesianSegment< LinearSegment, SmoothStep7 > >( pt_wpt_r, pt_end, path_bounds );
  
  sptr< PathSegment > seg;

  std::cout << "Seg start:\n" ;
  seg = seg_start;
  tmp.s = 0.0;
  std::cout << "  from:\t" <<  CartesianPoint( seg->getPosition(tmp) ).p.transpose() << " , " 
                          << CartesianPoint( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
  tmp.s = seg->getLength();
  std::cout << "  to:\t" <<  CartesianPoint( seg->getPosition(tmp) ).p.transpose() << " , " 
                          << CartesianPoint( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
  
  std::cout << "Seg blend:\n" ;
  seg = seg_blend;
  tmp.s = 0.0;
  std::cout << "  from:\t" <<  CartesianPoint( seg->getPosition(tmp) ).p.transpose() << " , " 
                          << CartesianPoint( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
  tmp.s = seg->getLength();
  std::cout << "  to:\t" <<  CartesianPoint( seg->getPosition(tmp) ).p.transpose() << " , " 
                          << CartesianPoint( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
                          
  std::cout << "Seg end:\n" ;
  seg = seg_end;
  tmp.s = 0.0;
  std::cout << "  from:\t" <<  CartesianPoint( seg->getPosition(tmp) ).p.transpose() << " , " 
                          << CartesianPoint( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
  tmp.s = seg->getLength();
  std::cout << "  to:\t" <<  CartesianPoint( seg->getPosition(tmp) ).p.transpose() << " , " 
                          << CartesianPoint( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
 
}
