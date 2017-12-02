#include <iostream>
#include <iomanip>
#include <SCurveProfile.hpp>

int main(void) {
  SCurveProfile scurve;
  
  double dt = 0.1;
  
  scurve.set_period( dt );
  
  scurve.config( 
    0, 0, 0,
    0.1, 0, 0,
    1.0, 1.0, 1.0
  );
  
  scurve.compute_curves();
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  for ( int i=0 ; i<scurve.t_vect_.size() ; ++i ) {
    std::cout << scurve.t_vect_[i] << " ; " << scurve.s_vect_[i] << " ; " << scurve.v_vect_[i] << " ; " << scurve.a_vect_[i] << " ; " << scurve.j_vect_[i] << std::endl;
  }   
  return 0;
}
