/*
 * frontier_detector_dms.hpp
 *
 *  Created on: Sep 25, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_FRONTIER_DETECTOR_DMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_DMS_HPP_

#include <chrono>
#include <time.h>
#include "thread_utility_meas.hpp"
#include "frontier_detector.hpp"
#include "frontier_point.hpp"
#include "frontier_filter.hpp"
#include "global_planning_handler.hpp"
#include <omp.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <chrono>
//#include "std_msgs/Empty.h"

//#define OCCUPANCY_THR (60)
//#define FD_DEBUG_MODE
#define ROI_OFFSET (10)
#define DIST_HIGH  (1.0e8)

namespace autoexplorer
{

using namespace std;

class FrontierDetectorDMS: public FrontierDetector
{
public:
	FrontierDetectorDMS( int numthreads );
	virtual ~FrontierDetectorDMS();

	void loadGridMap( const string& filename);
	void loadCostMap( const string& filename);
	void loadGridMap( const string& imgfilename, const string& mapinfofile) ;
	void loadCostMap( const string& imgfilename, const string& mapinfofile);

	void setCostMap(const string& costmapfile);
	void setGridMap(const string& gridmapfile);
	void processMap() ;

	//inline void SetInitMotionCompleted(){ m_isInitMotionCompleted = true;  }
	inline void SetNumThreads(int numthreads){ mn_numthreads = numthreads; }

	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );

	cv::Point world2gridmap( cv::Point2f img_pt_roi );
	cv::Point2f gridmap2world( cv::Point grid_pt );

protected:

	visualization_msgs::Marker m_points, m_cands;
	cv::Point2f m_robotpose ;

	int mn_numthreads;
	int m_nglobalcostmapidx ;
	string m_str_debugpath ;
	string m_str_inputparams ;

	cv::Mat m_uMapImg, m_uMapImgROI ;

	FrontierFilter m_oFrontierFilter;

	//GlobalPlanningHandler* mpo_gph ;

	GlobalPlanningHandler mo_gph ;
	costmap_2d::Costmap2D* mpo_costmap;

	uint8_t* mp_cost_translation_table;

	ofstream m_ofs_time ;
private:
	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_gridmap;
	std::mutex mutex_costmap;
	std::mutex mutex_upperbound;
	std::mutex mutex_timing_profile;

	omp_lock_t m_mplock;

	ThreadUtilityMeas* mp_threadutil; ;
};

}




#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
