#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <geo/Ecef.hpp>
#include <geo/Lla.hpp>
#include <geo/convert.hpp>
#include <boost/filesystem.hpp>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <vector>
#include <utility>
#include "RoadNetwork.hpp"
#include "rn_analyzer.hpp"

void rn_to_GOG(std::string kml_file, struct RoadNetwork rn)
{
  //Extract kml name
  std::size_t found = rn.kml_name.find_last_of(".");
  std::string gog_name = rn.kml_name.substr(0,found) + ".gog";
  
  //Create GOG file 
  //***********************************
  std::string filename;
  std::ostringstream oss;
  oss << gog_name;
  filename=oss.str();
  std::ofstream gog_file;
  gog_file.open(filename.c_str(),std::ofstream::trunc);
  //***********************************
  
   std::cout << "Constructing " << gog_name << "..." << std::endl;
  for(int p=0;p<rn.road_map.size();++p){
    gog_file << "start" << std::endl;
    gog_file << "line" << std::endl; 
    gog_file << "linewidth 3" << std::endl; 
   
    for(int q=1;q<rn.road_map[p].size();++q)
    {

      gog_file << std::fixed << "ll " << std::get<0>(rn.road_map[p][q]) << " " << 
           std::get<1>(rn.road_map[p][q]) << std::endl;
    }
    gog_file << "linecolor yellow" << std::endl; 
    gog_file << "end" << std::endl; 
    gog_file << "\n" << std::endl;
  }
  std::cout << gog_name << " complete" << "\n";
 
}