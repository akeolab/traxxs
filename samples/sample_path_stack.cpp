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

/**
 * In this file we check that stacking path segments 
 * can effectively ensure non-null velocity continuity
 * by using SmoothStep7 segments
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/path/path.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

int main(void) {
  
  sptr< path::PathSegment > top_seg_start, top_seg_1_blend, top_seg_end;
  sptr< path::PathSegment > bottom_seg_start, bottom_seg_1_blend, bottom_seg_end;
  
  sptr< path::PathSegment > stack_seg_start, stack_seg_1_blend, stack_seg_end;
  
  path::PathWaypoint wpt_start, wpt_1, wpt_end, wpt_blend_s, wpt_blend_e;
  path::PathWaypoint bottom_wpt_start, bottom_wpt_1, bottom_wpt_end;
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
  
  path::PathBounds path_bounds, bottom_path_bounds;
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
  top_seg_1_blend = std::make_shared< path::CircularBlend >( wpt_start, wpt_end, path_bounds, wpt_1.x, 0.1 );
  top_seg_1_blend->init();
  wpt_blend_s.x = top_seg_1_blend->getConfiguration( 0.0 );
  wpt_blend_e.x = top_seg_1_blend->getConfiguration( top_seg_1_blend->getLength() );
  
  top_seg_start = std::make_shared< path::LinearSegment >( wpt_start, wpt_blend_s, path_bounds );
  // seg_1_blend = 
  top_seg_end = std::make_shared< path::LinearSegment >( wpt_blend_e, wpt_end, path_bounds );
  
  //
  // the bottom segments
  //
  bottom_seg_start = std::make_shared< path::SmoothStep7 >( bottom_wpt_start, bottom_wpt_1, bottom_path_bounds );
  bottom_seg_1_blend = std::make_shared< path::SmoothStep7 >( bottom_wpt_1, bottom_wpt_1, bottom_path_bounds );
  bottom_seg_end = std::make_shared< path::SmoothStep7 >( bottom_wpt_1, bottom_wpt_end, bottom_path_bounds );
  
  //
  // Stack them
  // 
  stack_seg_start = std::make_shared< path::StackedSegments >( top_seg_start, bottom_seg_start );
  stack_seg_1_blend = std::make_shared< path::StackedSegments >( top_seg_1_blend, bottom_seg_1_blend );
  stack_seg_end = std::make_shared< path::StackedSegments >( top_seg_end, bottom_seg_end );
  
  std::vector< sptr< path::PathSegment > > segments{ stack_seg_start, stack_seg_1_blend, stack_seg_end };
  for ( auto& seg : segments )
    seg->init();
  path::Path path( segments );
  path.init();
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  sptr< path::PathSegment > cur, prev;
  Eigen::VectorXd x_m, x_p, dx_m, dx_p, ddx_m, ddx_p, j_m, j_p;
  arc::ArcConditions conds_m, conds_p;
  
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
    
    if ( (x_p - x_m).norm() > constants::kZero ) 
      std::cerr << "Position continuity fail." << std::endl;
    if ( (dx_p - dx_m).norm() > constants::kZero ) 
      std::cerr << "Velocity continuity fail." << std::endl;
    if ( (ddx_p - ddx_m).norm() > constants::kZero ) 
      std::cerr << "Acceleration continuity fail." << std::endl;
    
    std::cout << "Conds at (-): ds=" << conds_m.ds << ", dds=" << conds_m.dds << ", j=" << conds_m.j << std::endl;
    std::cout << "Conds at (+): ds=" << conds_p.ds << ", dds=" << conds_p.dds << ", j=" << conds_p.j << std::endl;
    std::cout << "x continuity:\tdelta=" << (x_p - x_m).norm() << std::endl;
    std::cout << "dx continuity:\tdelta=" << (dx_p - dx_m).norm() << ", |dx|=" << dx_p.norm() << std::endl;
    std::cout << "ddx continuity:\tdelta=" << (ddx_p - ddx_m).norm() << ", |ddx|=" << ddx_p.norm() << std::endl;
  }
  
}
