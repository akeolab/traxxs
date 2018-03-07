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
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/path/path.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

int main(void) {
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  path::CartesianPathWaypoint pt_start, pt_wpt, pt_end;
  
  sptr< path::CartesianSegment< path::LinearSegment, path::SmoothStep7 > > seg_start, seg_end; 
  sptr< path::CartesianSegment< path::CircularBlend, path::SmoothStep7 > > seg_blend;
  
  pt_start.x.p  << 0, 0, 0;
  pt_wpt.x.p    << 1, 0, 0;
  pt_end.x.p    << 1, 1, 0;
  
  pt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.x.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.x.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
  path::PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 0.1;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  using JoiningSegment_t  = path::CartesianSegment< path::LinearSegment, path::SmoothStep7 >;
  using BlendSegment_t    = path::CartesianSegment< path::CircularBlend, path::SmoothStep7 >;
  std::vector< path::CartesianPathWaypoint > waypoints = { pt_start, pt_wpt, pt_end};
  std::vector< sptr< path::PathSegment > > segments = path::blendedSegmentsFromWaypoints< path::CartesianPathWaypoint, JoiningSegment_t, BlendSegment_t, double>( 
   path_bounds, waypoints, 0.1 );
  path::Path path( segments );
  
  path.init();
  
  arc::ArcConditions tmp;
  for ( auto& seg : segments ) {
    tmp.s = 0.0;
    std::cout << "from:\t"  << path::Pose().fromVector( seg->getPosition(tmp) )->p.transpose() << "\n\t" 
                              << path::Pose().fromVector( seg->getPosition(tmp) )->q.coeffs().transpose() <<  "\n" ;
    tmp.s = seg->getLength();
    std::cout << "to:\t"    << path::Pose().fromVector( seg->getPosition(tmp) )->p.transpose() << "\n\t" 
                              << path::Pose().fromVector( seg->getPosition(tmp) )->q.coeffs().transpose() <<  "\n" ;
  }
  
}
