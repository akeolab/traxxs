#include <iostream>
#include <arctrajgensoftmotion.hpp>

int main(void) {
  ArcTrajGenSoftMotion arcTrajGen;
  traxxs::arc::ArcConditions c_i, c_f, c_max;
  traxxs::arc::ArcConditions c_cur;
  
  double dt = 0.1;
  
  c_i.s = 0;
  c_i.ds = 2.5;
  c_i.dds = 0.0;
  
  c_f.s = 1.0;
  c_f.ds = 0.0;
  c_f.dds = 0;
  
  c_max.ds = 1.0;
  c_max.dds = 100.0;
  c_max.j = 1000.0;
  
  arcTrajGen.setDt( dt );
  arcTrajGen.setInitialConditions( c_i );
  arcTrajGen.setFinalConditions( c_f );
  arcTrajGen.setMaxConditions( c_max );
  
  if ( !arcTrajGen.compute() ) { 
    std::cerr << "Compute failed." << std::endl;
    return 1;
  }
  
  std::cout << "From " << c_i << "  to " << c_f << " s.t. " << c_max << std::endl;
  std::cout << "Duration : " << arcTrajGen.getDuration() << std::endl;
  for ( double t = 0; t <= arcTrajGen.getDuration() + dt ; t += dt ) {
    arcTrajGen.getConditionsAtTime( t, c_cur );
    std::cout << c_cur << std::endl;
  }
  
  return 0;
}
