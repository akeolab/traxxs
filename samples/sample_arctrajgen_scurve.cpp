#include <iostream>
#include <arctrajgenscurve.hpp>

int main(void) {
  ArcTrajGenSCurve arcTrajGen;
  traxxs::arc::ArcConditions c_i, c_f, c_max;
  traxxs::arc::ArcConditions c_cur;
  
  double dt = 0.001;
  
  c_i.s = 0;
  c_i.ds = 0;
  c_i.dds = 0;
  
  c_f.s = 0.1;
  c_f.ds = 0;
  c_f.dds = 0;
  
  c_max.ds = 1.0;
  c_max.dds = 1.0;
  c_max.j = 1.0;
  
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
  std::vector< traxxs::arc::ArcConditions > profile = arcTrajGen.getConditionsProfile();
  for ( auto& c : profile ) {
    std::cout << c << std::endl;
  }
  
  return 0;
}
