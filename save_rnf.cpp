#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <fstream>
#include "RoadNetwork.hpp"

std::string save_rnf(struct RoadNetwork rn)
{
  //Defining road network file
  std::size_t found = rn.kml_name.find_last_of(".");
  std::string rnf_name = rn.kml_name.substr(0,found) + ".rnf";

  //Saving road network
  std::ofstream save_road_network(rnf_name.c_str());
  boost::archive::text_oarchive oa(save_road_network);
  oa << rn;
  save_road_network.close();
  
  std::cout << "Road network file saved as \"" << rnf_name<< "\"." << "\n";
  return rnf_name;
}
    