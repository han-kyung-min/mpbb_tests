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

    FrontierDetectorDMS front_detector_dms(nframeidx);

    // load gridmap and costmap
    //front_detector_dms.SetNumThreads(numthreads);

    string homedir = getenv("HOME");
    string strcostmap  = (boost::format("/home/hankm/results/autoexploration/costmap%05d.txt")  % nframeidx ).str() ;
    string strfrontier = (boost::format("/home/hankm/results/autoexploration/frontier%05d.txt") % nframeidx ).str() ;
    string mapinfofile = (boost::format("/home/hankm/results/autoexploration/cminfo%05d.txt")   % nframeidx ).str() ;

    front_detector_dms.loadCostMap(strcostmap, mapinfofile);
printf("costmap loaded\n");

	nav_msgs::Path msg_frontiers;
	front_detector_dms.loadFrontierPoints( strfrontier, msg_frontiers  ) ;

    front_detector_dms.planToFrontierPoints( msg_frontiers  ) ;

  return 0;
}


