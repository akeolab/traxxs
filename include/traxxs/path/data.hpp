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
