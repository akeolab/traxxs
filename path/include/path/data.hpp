#ifndef TRAJGEN_PATH_DATA_H
#define TRAJGEN_PATH_DATA_H

#include <Eigen/Dense>

struct Pose {
    Pose() {};
  /**
   * \warning will throw std::invalid_argument if the input vector is not of size 7
   * \param[in] rep_7d The 7D representation of the cartesian point [x, y, z, qx, qy, qz, qw]
   */
    Pose( const Eigen::VectorXd &rep_7d ) {
    if ( rep_7d.size() != 7 )
      throw std::invalid_argument( "Input Waypoints should be of size 7 [x, y, z, qx, qy, qz, qw]" );
    this->p = rep_7d.segment( 0, 3 );
    this->q = Eigen::Quaterniond( rep_7d[6], rep_7d[3], rep_7d[4], rep_7d[5] ); // Eigen expects w x y z
  }
  
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

#endif // TRAJGEN_PATH_DATA_H
