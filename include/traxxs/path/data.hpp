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

#ifndef TRAXXS_PATH_DATA_H
#define TRAXXS_PATH_DATA_H

#include <Eigen/Dense>

namespace traxxs {
namespace path {
  
struct Pose {
  Pose() {};
  
  /** \brief this constructor is needed to be able to compare traxxs::path::Pose to Eigen::VectorXd */
  Pose( const Eigen::VectorXd &rep_7d ) { 
    try {
      this->fromVector( rep_7d );
    } catch ( const std::invalid_argument* e ) {
      this->p.fill( std::nan("") );
      this->q = Eigen::Quaterniond( std::nan(""), std::nan(""), std::nan(""), std::nan("") );
    }
  }
  
  /**
   * \warning will throw std::invalid_argument if the input vector is not of size 7
   * \param[in] rep_7d The 7D representation of the cartesian point [x, y, z, qx, qy, qz, qw]
   */
  Pose* fromVector( const Eigen::VectorXd &rep_7d ) {
    if ( rep_7d.size() != 7 )
      throw std::invalid_argument( "Input Waypoints should be of size 7 [x, y, z, qx, qy, qz, qw]" );
    this->p = rep_7d.segment( 0, 3 );
    this->q = Eigen::Quaterniond( rep_7d[6], rep_7d[3], rep_7d[4], rep_7d[5] ); // Eigen expects w x y z
    return this;
  }
  
  /** \brief returns a pose vector in the form x, y, z, qx, qy, qz, qw */
  Eigen::VectorXd toVector() {
    Eigen::VectorXd rep_7d( 7 );
    rep_7d.segment( 0, 3 ) = this->p;
    rep_7d[3] = this->q.x();
    rep_7d[4] = this->q.y();
    rep_7d[5] = this->q.z();
    rep_7d[6] = this->q.w();
    return rep_7d;
  }
  
  /** the position x y z */
  Eigen::Vector3d     p;
  /** the orientation q */
  Eigen::Quaterniond  q;
};

} // namespace path
} // namespace traxxs

#endif // TRAXXS_PATH_DATA_H
