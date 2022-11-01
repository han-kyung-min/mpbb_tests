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
        printf("args: %s <frameidx> \n", argv[0]);
        return -1;
    }
    
  int nframeidx = atoi(argv[1]);

    FrontierDetectorDMS front_detector_dms(8);

    // load gridmap and costmap
    //front_detector_dms.SetNumThreads(numthreads);

    string homedir = getenv("HOME");
    string strcostmap  = (boost::format("/media/data/results/autoexplorer/costmap%04d.txt")  % nframeidx ).str() ;
    string strfrontier = (boost::format("/media/data/results/autoexplorer/frontier%04d.txt") % nframeidx ).str() ;
    string mapinfofile = (boost::format("/media/data/results/autoexplorer/cminfo%04d.txt")   % nframeidx ).str() ;

    front_detector_dms.loadCostMap(strcostmap, mapinfofile);
printf("costmap loaded\n");

	nav_msgs::Path msg_frontiers;
	front_detector_dms.loadFrontierPoints( strfrontier, msg_frontiers  ) ;

    front_detector_dms.planToFrontierPoints( nframeidx, msg_frontiers  ) ;

  return 0;
}


