#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <fstream>
#include "RoadNetwork.hpp"

struct RoadNetwork load_rnf(std::string rnf_file)
{
   std::string ext;
   
  //Defining road network file
  try
  {
    std::size_t found = rnf_file.find_last_of(".");
    ext = rnf_file.substr(found+1);
    
    if(ext == "rnf")
    {
      //Creating Road Network struct
      RoadNetwork rn;
      
      //Loading road network
      std::ifstream load_road_network(rnf_file.c_str());
      boost::archive::text_iarchive ia(load_road_network);
      ia >> rn;  
      std::cout << rn.kml_name << " road network successfully loaded!" << "\n";
      
      return rn;
    }
    else
    {
	try
	{
	  throw 42;
	}
	catch (int i)
	{
	  std::cout << "Road network could not be loaded. Invalid file extension" << "\n";
	}
    }
  }
  catch(const std::exception& e)
  {
    //Reference to string that does not contain "."
    std::cout << "File does not contain an extension" << "\n";
  }
 
}
   