/*
 * frontier_detector_node.cpp
 *
 *  Created on: Mar 18, 2021
 *      Author: hankm
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <octomap_server/mapframedata.h>

#include "frontier_detector_dms.hpp"

using namespace autoexplorer;

int main(int argc, char** argv)
{

    if( argc != 2)
    {
        printf("args: %s <numthreads> \n", argv[0]);
        return -1;
    }
    
  int numthreads = atoi(argv[1]);

    FrontierDetectorDMS front_detector_dms(numthreads);

    // load gridmap and costmap
    //front_detector_dms.SetNumThreads(numthreads);

    string homedir = getenv("HOME");
    string costmapimgfile = homedir+"/results/autoexploration/tmp/deutschemuseum/outimg00390.png" ;
    string gridmapimgfile = homedir+"/results/autoexploration/tmp/deutschemuseum/outimg00390.png" ;
    string mapinfofile    = homedir+"/results/autoexploration/tmp/deutschemuseum/mapinfo00390.txt";
    front_detector_dms.loadGridMap(gridmapimgfile, mapinfofile);
printf("gridmap loaded\n");
    front_detector_dms.loadCostMap(costmapimgfile, mapinfofile);
printf("costmap loaded\n");
    front_detector_dms.processMap() ;

  return 0;
}


