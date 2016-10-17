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
    RoadNetwork rn = load_rnf("../APL_roads.rnf");
    
    Edge e = edge(1000,4809, rn.GRAPH).first;
    rn_analyzer rn_analyzer1;
    
    rn_analyzer1.road_curvature(e, rn);
   
    rn_analyzer1.print_rn_map_index(rn);
    //rn_analyzer1.get_intersections(rn);
  return 0;
}
