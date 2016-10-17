#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/tokenizer.hpp>
#include "get_kml_list.hpp"
#include "kml_to_rn.hpp"
#include "RoadNetwork.hpp"
#include "save_rnf.hpp"
#include "load_rnf.hpp"
#include "rn_to_GOG.hpp"
#include "rn_analyzer.hpp"


int main(int argc, char** argv)
{
    std::cout << "Program starting...\n";
   
    //Generating kml file list
    std::vector<std::string> kml_files = get_kml_list();
    
    //Initializing program variables
    std::string kml_file;
    RoadNetwork rn1;
    
    //Iterating over kml file list
    for(int i=0; i < kml_files.size(); i++)
    {
      kml_file = kml_files.at(i);
      
      //Transforming kml file to a Road Network struct
      double node_spacing = 0.24383; //minimum distance between nodes
      rn1 = kml_to_rn(kml_file,node_spacing);
      
      //Saving/Serializing generated road_network
      std::string rnf_file = save_rnf(rn1);
      
       //Creating GOG File
      rn_to_GOG(kml_file, rn1);
      
      //Loading generated road_network
      RoadNetwork rn2 = load_rnf(rnf_file);
  
    }
    
    //Moving all gog files to "build" folder
    std::system("mv ../*.gog .");
   
  return 0;
}
