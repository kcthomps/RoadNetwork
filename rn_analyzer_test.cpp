 #include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include "RoadNetwork.hpp"
#include "load_rnf.hpp"
#include "rn_analyzer.hpp"

int main()
{
 //Testing road network analyzer
  struct RoadNetwork rn1 = load_rnf("chicago_routes.rnf");
   Vertex v1;
   Vertex v2;
   int i = 0;
    for (rn1.vp = vertices(rn1.GRAPH); rn1.vp.first != rn1.vp.second; ++rn1.vp.first)
    {
      if (i==0)
      {
	v1 = *rn1.vp.first;
      }
      if (i==1)
      {
	v2 = *rn1.vp.first;
	break;
      }
    }
    rn_analyzer rn_analyzer1;
    double t = rn_analyzer1.slope_of_edge(v1,v2,rn1);
    return 0;
}