/*
 * frontier_detector_dms.cpp
 *
 *  Created on: Sep 25, 2021
 *      Author: hankm
 */

// frontier detection for dynamic map size cases (cartographer generated maps)

#include "frontier_detector_dms.hpp"


namespace autoexplorer
{

FrontierDetectorDMS::FrontierDetectorDMS( int numthreads ):
m_nglobalcostmapidx(0), mn_numthreads(numthreads),
mpo_gph(NULL),
mp_cost_translation_table(NULL)
{
	// gridmap generated from octomap might be downsampled !!
	m_str_debugpath = string("/home/hankm/results/autoexploration");

	m_noccupancy_thr = 40;
	m_fRobotRadius = 0.3f;
	m_fResolution = 0.05f;
	m_nGlobalMapWidth = 4000;
	m_nGlobalMapHeight = 4000;
	m_nROISize = 0;
	m_nScale = 1;
	m_nCorrectionWindowWidth = 0;
	m_nlethal_cost_thr = 80;
	float fgridmap_conf_thr = 0.6 ;
	float fcostmap_conf_thr = 0.4;
	m_nNumPyrDownSample = 1;

	m_nScale = pow(2, m_nNumPyrDownSample) ;
	m_frontiers_region_thr = 10 / m_nScale ;
	m_nROISize = 12; //static_cast<int>( round( m_fRobotRadius / m_fResolution ) ) * 2 ; // we never downsample costmap !!! dont scale it with roisize !!
	m_nGlobalMapCentX = m_nGlobalMapWidth  / 2 ;
	m_nGlobalMapCentY = m_nGlobalMapHeight / 2 ;

	m_uMapImg  	  = cv::Mat(m_nGlobalMapHeight, m_nGlobalMapWidth, CV_8U, cv::Scalar(127));

	int ncostmap_roi_size = m_nROISize / 2 ;
	int ngridmap_roi_size = m_nROISize ;
	m_nCorrectionWindowWidth = m_nScale * 2 + 1 ; // the size of the correction search window

	m_oFrontierFilter = FrontierFilter(
			ncostmap_roi_size, ngridmap_roi_size, m_str_debugpath, m_nNumPyrDownSample,
			fgridmap_conf_thr, fcostmap_conf_thr, m_noccupancy_thr, m_nlethal_cost_thr,
			m_nGlobalMapWidth, m_nGlobalMapHeight,
			m_fResolution);

// global_planning_handler
	//mpo_gph = new GlobalPlanningHandler( ) ;
	mpo_costmap = new costmap_2d::Costmap2D();

	if (mp_cost_translation_table == NULL)
	{
		mp_cost_translation_table = new uint8_t[101];

		// special values:
		mp_cost_translation_table[0] = 0;  // NO obstacle
		mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
		mp_cost_translation_table[100] = 254;  // LETHAL obstacle
//		mp_cost_translation_table[-1] = 255;  // UNKNOWN

		// regular cost values scale the range 1 to 252 (inclusive) to fit
		// into 1 to 98 (inclusive).
		for (int i = 1; i < 99; i++)
		{
			mp_cost_translation_table[ i ] = uint8_t( ((i-1)*251 -1 )/97+1 );
		}
	}

	m_ofs_time = ofstream(m_str_debugpath+"/planning_time.txt");

	string str_filepath = m_str_debugpath+"/threadutility.txt";
	mp_threadutil = new ThreadUtilityMeas(mn_numthreads, str_filepath);

}

FrontierDetectorDMS::~FrontierDetectorDMS()
{
	if(mp_cost_translation_table != nullptr)
		delete [] mp_cost_translation_table;
	if(mp_threadutil != nullptr)
		delete mp_threadutil ;
}


cv::Point2f FrontierDetectorDMS::gridmap2world( cv::Point img_pt_roi  )
{
	// grid_x = (map_x - map.info.origin.position.x) / map.info.resolution
	// grid_y = (map_y - map.info.origin.position.y) / map.info.resolution
	// img_x = (gridmap_x - gridmap.info.origin.position.x) / gridmap.info.resolution
	// img_y = (gridmap_y - gridmap.info.origin.position.y) / gridmap.info.resolution

	float fgx =  static_cast<float>(img_pt_roi.x) * m_fResolution + m_gridmap.info.origin.position.x  ;
	float fgy =  static_cast<float>(img_pt_roi.y) * m_fResolution + m_gridmap.info.origin.position.y  ;

	return cv::Point2f( fgx, fgy );
}

cv::Point FrontierDetectorDMS::world2gridmap( cv::Point2f grid_pt)
{
	float fx = (grid_pt.x - m_gridmap.info.origin.position.x) / m_gridmap.info.resolution ;
	float fy = (grid_pt.y - m_gridmap.info.origin.position.y) / m_gridmap.info.resolution ;

	return cv::Point( (int)fx, (int)fy );
}

void FrontierDetectorDMS::loadGridMap( const string& gridmapfile)
{
	ifstream ifs_map( gridmapfile );
	int nheight ;
	int nwidth ;
	int value ;
	float origx ;
	float origy ;
	float resolution ;
	ifs_map >> nwidth >> nheight  >> origx >> origy >> resolution;
	m_gridmap.info.height = nheight ;
	m_gridmap.info.width  = nwidth ;
	m_gridmap.info.origin.position.x = origx ;
	m_gridmap.info.origin.position.y = origy ;
	m_gridmap.info.resolution = resolution ;
	for( int ridx=0; ridx < nheight; ridx++ )
	{
		for( int cidx=0; cidx < nwidth; cidx++ )
		{
			ifs_map >> value ;
			m_gridmap.data.push_back(value);
		}
	}
	ifs_map.close();
}

void FrontierDetectorDMS::loadCostMap( const string& costmapfile)
{
	ifstream ifs_map( costmapfile );
	int nheight ;
	int nwidth ;
	int value ;
	float origx ;
	float origy ;
	float res ;
	ifs_map >> nwidth >> nheight >> origx >> origy >> res;
	m_globalcostmap.info.height = nheight ;
	m_globalcostmap.info.width  = nwidth ;
	m_globalcostmap.info.origin.position.x = origx ;
	m_globalcostmap.info.origin.position.y = origy ;
	m_globalcostmap.info.resolution = res ;
	for( int ridx=0; ridx < nheight; ridx++ )
	{
		for( int cidx=0; cidx < nwidth; cidx++ )
		{
			ifs_map >> value ;
			m_globalcostmap.data.push_back(value);
		}
	}
	ifs_map.close();
}

void FrontierDetectorDMS::setCostMap(const string& costmapfile)
{

}

void FrontierDetectorDMS::setGridMap(const string& gridmapfile)
{

}

// mapcallback for dynamic mapsize (i.e for the cartographer)
void FrontierDetectorDMS::processMap()
{

	float gmresolution ;
	uint32_t gmheight, gmwidth;

	nav_msgs::OccupancyGrid globalcostmap;
	float cmresolution, cmstartx, cmstarty;
	uint32_t cmwidth, cmheight;
	std::vector<signed char> cmdata;

	gmresolution = m_gridmap.info.resolution ;
	gmheight = m_gridmap.info.height ;
	gmwidth = m_gridmap.info.width ;

	//const std::unique_lock<mutex> lock(mutex_costmap);
	globalcostmap = m_globalcostmap;
	cmresolution=globalcostmap.info.resolution;
	cmstartx=globalcostmap.info.origin.position.x;
	cmstarty=globalcostmap.info.origin.position.y;
	cmwidth =globalcostmap.info.width;
	cmheight=globalcostmap.info.height;
	cmdata  =globalcostmap.data;


	if( gmheight == 0 || gmwidth == 0
		|| gmwidth  != cmwidth
		|| gmheight != cmheight)
	{
		printf("unreliable grid map input h/w (%d, %d) gcostmap h/w (%d, %d) \n",
				gmheight, gmwidth,
				cmheight, cmwidth);
		return;
	}
	m_nroi_origx = m_nGlobalMapCentX ; // - (int)round( m_gridmap.info.origin.position.x / m_fResolution ) ;
	m_nroi_origy = m_nGlobalMapCentY ; //- (int)round( m_gridmap.info.origin.position.y / m_fResolution ) ;

	cv::Rect roi( m_nroi_origx, m_nroi_origy, gmwidth, gmheight );

	m_uMapImgROI = m_uMapImg(roi);
printf("roi: %d %d \n", m_uMapImgROI.rows, m_uMapImgROI.cols);

	for( int ii =0 ; ii < gmheight; ii++)
	{
		for( int jj = 0; jj < gmwidth; jj++)
		{
			int8_t occupancy = m_gridmap.data[ ii * gmwidth + jj ]; // dynamic gridmap size
			int y_ = (m_nroi_origy + ii) ;
			int x_ = (m_nroi_origx + jj) ;

			if ( occupancy < 0 )
			{
				m_uMapImg.data[ y_ * m_nGlobalMapWidth + x_ ] = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
			}
			else if( occupancy >= 0 && occupancy < m_noccupancy_thr)
			{
				m_uMapImg.data[ y_ * m_nGlobalMapWidth + x_ ] = static_cast<uchar>(ffp::MapStatus::FREE) ;
			}
			else
			{
				m_uMapImg.data[ y_ * m_nGlobalMapWidth + x_ ] = static_cast<uchar>(ffp::MapStatus::OCCUPIED) ;
			}
		}
	}

	cv::Mat img_ ; // valid map area only (ox oy w h)
	img_ = m_uMapImg( roi ); //m_uMapImgROI.clone();

//cv::namedWindow("tmp");
//cv::imshow("tmp", img_);
//cv::waitKey(0);

	if( m_nNumPyrDownSample > 0)
	{
		// be careful here... using pyrDown() interpolates occ and free, making the boarder area (0 and 127) to be 127/2 !!
		// 127 reprents an occupied cell !!!
		//downSampleMap(img);
		for(int iter=0; iter < m_nNumPyrDownSample; iter++ )
		{
			int nrows = img_.rows; //% 2 == 0 ? img.rows : img.rows + 1 ;
			int ncols = img_.cols; // % 2 == 0 ? img.cols : img.cols + 1 ;
			pyrDown(img_, img_, cv::Size( ncols/2, nrows/2 ) );
		}
	}
	clusterToThreeLabels( img_ );

// We need to zero-pad around img b/c m_gridmap dynamically increases
	//uint8_t ukn = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
	cv::Mat img_padded = cv::Mat( img_.rows + ROI_OFFSET*2, img_.cols + ROI_OFFSET*2, CV_8U, cv::Scalar(ffp::MapStatus::UNKNOWN) ) ;
	cv::Rect myroi( ROI_OFFSET, ROI_OFFSET, img_.cols, img_.rows );
	cv::Mat img_roi = img_padded(myroi) ;
	img_.copyTo(img_roi) ;

	ffp::FrontPropagation oFP(img_padded); // image uchar
	oFP.update(img_padded, cv::Point(0,0));
	oFP.extractFrontierRegion( img_padded ) ;

	cv::Mat img_frontiers = oFP.GetFrontierContour() ;

	cv::Mat dst, img_labeled;
	cvtColor(img_frontiers, dst, cv::COLOR_GRAY2BGR);
	cvtColor(img_padded, img_labeled, cv::COLOR_GRAY2BGR);
// locate the most closest labeled points w.r.t the centroid pts

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	cv::findContours( img_frontiers, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );

	if( contours.size() == 0 )
		return;

	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
//ROS_INFO("hierarchy: %d \n", idx);
		cv::Scalar color( rand()&255, rand()&255, rand()&255 );
		drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
		drawContours( img_labeled, contours, idx, cv::Scalar(0,0,255), CV_FILLED, 8, hierarchy );
	}


#ifdef FD_DEBUG_MODE
	string outfilename =  m_str_debugpath + "/global_mapimg.png" ;
	cv::imwrite( outfilename.c_str(), m_uMapImg);
	cv::imwrite(m_str_debugpath + "/map_w_fr.png", img_labeled);
	cv::imwrite(m_str_debugpath + "/img_frontiers.png",img_frontiers);
#endif

	vector<cv::Point2f> fcents;
	for(int i=0; i < contours.size(); i++)
	{
		float fx =0, fy =0 ;
		float fcnt = 0 ;
		vector<cv::Point> contour = contours[i];
		for( int j=0; j < contour.size(); j++)
		{
			fx += static_cast<float>( contour[j].x ) ;
			fy += static_cast<float>( contour[j].y ) ;
			fcnt += 1.0;
		}
		fx = fx/fcnt ;
		fy = fy/fcnt ;

		cv::Point2f fcent( fx,  fy ) ;
		fcents.push_back(fcent);
	}

	// get closest frontier pt to each cent
	// i.e.) the final estimated frontier points
	vector<FrontierPoint> voFrontierCands;

	for( int i = 0; i < contours.size(); i++ )
	{
		vector<cv::Point> contour = contours[i] ;
		if(contour.size() < m_frontiers_region_thr ) // don't care about small frontier regions
			continue ;

		float fcentx = fcents[i].x ;
		float fcenty = fcents[i].y ;

		float fmindist = 1000 ;
		int nmindistidx = -1;

		for (int j=0; j < contour.size(); j++)
		{
			float fx = static_cast<float>(contour[j].x) ;
			float fy = static_cast<float>(contour[j].y) ;
			float fdist = std::sqrt( (fx - fcentx) * (fx - fcentx) + (fy - fcenty) * (fy - fcenty) );
			if(fdist < fmindist)
			{
				fmindist = fdist ;
				nmindistidx = j ;
			}
		}

		CV_Assert(nmindistidx >= 0);
		cv::Point frontier = contour[nmindistidx];

		if(
			(ROI_OFFSET > 0) &&
			(frontier.x <= ROI_OFFSET || frontier.y <= ROI_OFFSET ||
			 frontier.x >= gmwidth + ROI_OFFSET || frontier.y >= gmheight + ROI_OFFSET)
		   )
		{
			continue;
		}

		frontier.x = frontier.x - ROI_OFFSET ;
		frontier.y = frontier.y - ROI_OFFSET ;

		//frontiers_cand.push_back(frontier) ;
		FrontierPoint oPoint( frontier, gmheight, gmwidth,
								m_gridmap.info.origin.position.y, m_gridmap.info.origin.position.x,
			   // m_nGlobalMapCentY, m_nGlobalMapCentX,
					   //0,0,
					   m_fResolution, m_nNumPyrDownSample );

// //////////////////////////////////////////////////////////////////
// 				We need to run position correction here
/////////////////////////////////////////////////////////////////////
		cv::Point init_pt 		= oPoint.GetInitGridmapPosition() ; 	// position @ ds0 (original sized map)
		cv::Point corrected_pt	= oPoint.GetCorrectedGridmapPosition() ;
		correctFrontierPosition( m_gridmap, init_pt, m_nCorrectionWindowWidth, corrected_pt  );
//printf("init_pt:\t %d %d \n", init_pt.x, init_pt.y);
//printf("corrected_pt:\t %d %d \n", corrected_pt.x, corrected_pt.y);
		oPoint.SetCorrectedCoordinate(corrected_pt);
		voFrontierCands.push_back(oPoint);
	}

	geometry_msgs::Point p;
	m_cands.points.clear();
	m_points.points.clear();

	const float fcm_conf = m_oFrontierFilter.GetCostmapConf() ;
	const float fgm_conf = m_oFrontierFilter.GetGridmapConf() ;

	for(size_t idx=0; idx < voFrontierCands.size(); idx++)
	{
		cv::Point2f frontier_in_world = voFrontierCands[idx].GetCorrectedWorldPosition() ;
		p.x = frontier_in_world.x ;
		p.y = frontier_in_world.y ;
		p.z = 0.0 ;
		m_cands.points.push_back(p);
//ROS_INFO("frontier cands: %f %f \n", p.x, p.y);
	}

	// eliminate frontier points at obtacles
	vector<size_t> valid_frontier_indexs;
	if( globalcostmap.info.width > 0 )
	{
		//frontiers = eliminateSupriousFrontiers( m_globalcostmap, frontiers_cand, m_nROISize) ;
		m_oFrontierFilter.measureCostmapConfidence(globalcostmap, voFrontierCands);
		m_oFrontierFilter.measureGridmapConfidence(m_gridmap, voFrontierCands);

		for(size_t idx=0; idx < voFrontierCands.size(); idx++)
			voFrontierCands[idx].SetFrontierFlag( fcm_conf, fgm_conf );

//		set<pointset, pointset> unreachable_frontiers;
//		{
//			//const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
//			unreachable_frontiers = m_unreachable_frontier_set ;
//			//m_oFrontierFilter.computeReachability( unreachable_frontiers, voFrontierCands );
//		}
	}
	else
	{
		//ROS_INFO("costmap hasn't updated \n");
		//frontiers = frontiers_cand ; // points in img coord
	}

#ifdef FD_DEBUG_MODE
	string strcandfile = m_str_debugpath + "/front_cand.txt" ;
	ofstream ofs_cand(strcandfile);
	for( int idx=0; idx < voFrontierCands.size(); idx++ )
	{
		cv::Point pt( voFrontierCands[idx].GetDownSampledPosition().x, voFrontierCands[idx].GetDownSampledPosition().y );
		cv::circle(img_labeled, cv::Point(pt.x+ROI_OFFSET, pt.y+ROI_OFFSET), 2, cv::Scalar(0,255,0), 1,8, 0);
		ofs_cand << voFrontierCands[idx].GetCorrectedGridmapPosition().x << " " << voFrontierCands[idx].GetInitGridmapPosition().y << endl;
	}
	cv::imwrite(m_str_debugpath + "/map_w_fcents.png", img_labeled);
	ofs_cand.close();
#endif

	for (size_t idx=0; idx < voFrontierCands.size(); idx++)
	{
		if( voFrontierCands[idx].isConfidentFrontierPoint() )
			valid_frontier_indexs.push_back( idx );
	}

	if( valid_frontier_indexs.size() == 0 )
	{
		mb_explorationisdone = true;
		return;
	}

	// set exploration goals
	for(size_t idx=0; idx < valid_frontier_indexs.size(); idx++)
	{
		// scale then conv 2 gridmap coord
		size_t vidx = valid_frontier_indexs[idx];
//printf("valid idx: %d\n", vidx);
		cv::Point2f frontier_in_world = voFrontierCands[vidx].GetCorrectedWorldPosition();
		geometry_msgs::PoseWithCovarianceStamped mygoal ; // float64
		//mygoal.header.stamp= 0;
		mygoal.header.frame_id = m_worldFrameId;
		mygoal.pose.pose.position.x= frontier_in_world.x ;
		mygoal.pose.pose.position.y= frontier_in_world.y ;
		mygoal.pose.pose.position.z=0.0;
		//m_exploration_goal.push_back(mygoal) ;

		//m_targetspub.publish(mygoal);
		p.x = mygoal.pose.pose.position.x ;
		p.y = mygoal.pose.pose.position.y ;
		p.z = 0.0 ;
		m_points.points.push_back(p);

//printf("frontier pts found: %f %f \n", frontier_in_world.x, frontier_in_world.y);
	}

printf("num cand pts: %d  num val fpts: %d \n", voFrontierCands.size(), valid_frontier_indexs.size() );

#ifdef FFD_DEBUG_MODE
		imwrite(m_str_debugpath+"/frontier_cents.png", dst);
#endif

	//ROS_INFO("costmap info: %f %f %f %f \n", resolution, Xstartx, Xstarty, width);
	//ROS_INFO("frontier: %f %f \n", m_points.points[0].x, m_points.points[0].y );

// generate a path trajectory
// call make plan service

// Here we do motion planning

	geometry_msgs::PoseStamped start = GetCurrPose( );
	start.header.frame_id = m_worldFrameId;

vector<uint32_t> plan_len;
plan_len.resize( m_points.points.size() );

//mpo_gph->setCostmap(Data, m_globalcostmap.info.width, m_globalcostmap.info.height, m_globalcostmap.info.resolution,
//					m_globalcostmap.info.origin.position.x, m_globalcostmap.info.origin.position.y) ;

//ROS_INFO("resizing mpo_costmap \n");
mpo_costmap->resizeMap( cmwidth, cmheight, cmresolution,
					    cmstartx, cmstarty );
//ROS_INFO("mpo_costmap has been reset \n");
unsigned char* pmap = mpo_costmap->getCharMap() ;
//ROS_INFO("w h datlen : %d %d %d \n", cmwidth, cmheight, cmdata.size() );

for(uint32_t ridx = 0; ridx < cmheight; ridx++)
{
	for(uint32_t cidx=0; cidx < cmwidth; cidx++)
	{
		uint32_t idx = ridx * cmwidth + cidx ;
//ROS_INFO("here idx: %d \n", idx);
		signed char val = cmdata[idx];

		pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
//ROS_INFO("idx val tablev : %d %d %u\t", idx, val, mp_cost_translation_table[val] );
//printf("(%d %d)\t", pmap[idx], mp_cost_translation_table[val]);
	}
}

//ROS_INFO("mpo_costmap has been set\n");
///////////////////////////////////////////////////////////////////////
// 1. estimate dist to each goal using euclidean distance heuristic (we need sorting here)
///////////////////////////////////////////////////////////////////////
	float fstartx = static_cast<float>( start.pose.position.x ) ;
	float fstarty = static_cast<float>( start.pose.position.y ) ;
	float fmindist = DIST_HIGH ;
	size_t min_heuristic_idx = 0;

//	vector< pair<size_t, float> > init_heuristic;
//	for(size_t idx=0; idx < m_points.points.size(); idx++)
//	{
//		//size_t vidx = valid_frontier_indexs[idx];
//		//cv::Point2f frontier_in_world = voFrontierCands[vidx].GetCorrectedWorldPosition();
//
//		geometry_msgs::Point point = m_points.points[idx] ;
//
//		float fxd = fstartx - (float)point.x ;
//		float fyd = fstarty - (float)point.y ;
//		float hxsq  =  sqrt( fxd * fxd + fyd * fyd ) ;
//		init_heuristic.push_back( pair<size_t, float>(idx, hxsq) );
////		if(hxsq < fmindist)
////		{
////			min_heuristic_idx = vidx ;
////			fmindist = hxsq ;
////		}
//	}
//
//	std::stable_sort(init_heuristic.begin(), init_heuristic.end(),
//			[&init_heuristic](pair<size_t, float> i1, pair<size_t, float> i2)
//			{return i1.second < i2.second; } );

//////////////////////////////////////////////////////////////////////////////////
// 2. use the fp corresponds to the min distance as the init fp. epsilon = A*(fp)
// 	i)  We first sort fpts based on their euc heuristic(), then try makePlan() for each of fpts in turn.
// 	ii) We need to sort them b/c the one with best heuristic could fail
//////////////////////////////////////////////////////////////////////////////////

	//mpo_gph = new GlobalPlanningHandler();

	float fupperbound;
	std::vector<geometry_msgs::PoseStamped> initplan;
	float fendpot = POT_HIGH;
	//const float initbound = static_cast<float>(DIST_HIGH) ;
	fupperbound = static_cast<float>(DIST_HIGH) ;

//ros::WallTime initStartTime = ros::WallTime::now();
//
//	size_t tmpidx;
//	for( size_t idx =0; idx < init_heuristic.size(); idx++)
//	{
//		mpo_gph->reinitialization( mpo_costmap ) ;
//
//		min_heuristic_idx = init_heuristic[idx].first ;
//		//cv::Point2f frontier_in_world = voFrontierCands[min_heuristic_idx].GetCorrectedWorldPosition();
//		geometry_msgs::Point point = m_points.points[min_heuristic_idx] ;
//		geometry_msgs::PoseStamped initgoal = StampedPosefromSE2( (float)point.x, (float)point.y, 0.f );
//		initgoal.header.frame_id = m_worldFrameId ;
//		bool bsuccess = mpo_gph->makePlan(0, initbound, true, start, initgoal, initplan, fendpot);
//		fupperbound = fendpot ;
//
//		if(bsuccess)
//		{
//			ROS_INFO("[init condition found] (%f %f) to (%f %f) corresponds to min idx: %d  bound pot: %f \n ",
//					start.pose.position.x, start.pose.position.y,
//					initgoal.pose.position.x, initgoal.pose.position.y, min_heuristic_idx, fendpot);
//			break;
//		}
//	}
//
//	delete mpo_gph;
//
//ros::WallTime initEndTime = ros::WallTime::now();
//		//ROS_INFO("thread: %d %p %p \n", omp_get_thread_num(), mpo_gph, mpo_costmap);
//double init_time = (initEndTime - initStartTime).toNSec() * 1e-6;
//{
//	const std::unique_lock<mutex> lock(mutex_timing_profile) ;
//	m_ofs_time << "init time: " << init_time << " init bound " << fupperbound << " initplan len " << initplan.size() << endl;
//	m_ofs_time << "pid \t" << "tid \t" << "plan size \t" << "mp_time " << "min_heuristic_idx " << "fendpot " << "fupperbound \t" << endl;
//}
////ROS_INFO("(%d) init bound %f init plan time %f  init plan len %d",tmpidx, min_heuristic_idx, fupperbound, init_time, initplan.size() );


//exit(-1);
///////////////////////// /////////////////////////////////////////////////////////
// 3. Do BB based openmp search
//////////////////////////////////////////////////////////////////////////////////
//exit(-1);

int numthreads;// = omp_get_num_threads() ;

vector< uint32_t > gplansizes( m_points.points.size(), 0 ) ;

printf("\n\n\n ******************************************************** \n");
printf("***                          begin GP here ******************** \n");
printf("*************************************************************** \n\n\n");

int nrepeat = 1000;
//std::clock_t GPstartTime = clock();

auto begin_time = std::chrono::high_resolution_clock::now();

//vector< float	 > endpotentials( numthreads );
omp_set_num_threads(mn_numthreads);

mp_threadutil->read_procstat_old();

for(int repeatidx=0; repeatidx < nrepeat; repeatidx++)
{

#pragma omp parallel firstprivate( mpo_gph ) shared( mpo_costmap, gplansizes, fupperbound )
{
	mpo_gph = new GlobalPlanningHandler();
	numthreads = mn_numthreads; //omp_get_num_threads() ;

	#pragma omp for schedule(dynamic)
	for (size_t idx=0; idx < m_points.points.size(); idx++)
	{
		int tid = omp_get_thread_num() ;

//printf("processing (%f %f) with thread %d/%d : %d", p.x, p.y, omp_get_thread_num(), omp_get_num_threads(), idx );
		//#pragma omp atomic

		//pogph = new GlobalPlanningHandler();
		mpo_gph->reinitialization( mpo_costmap ) ;
//ROS_INFO("setting costmap \n");
//		pogph->setCostmap(Data, m_globalcostmap.info.width, m_globalcostmap.info.height, m_globalcostmap.info.resolution,
//						m_globalcostmap.info.origin.position.x, m_globalcostmap.info.origin.position.y) ;

		p = m_points.points[idx];  // just for now... we need to fix it later
		geometry_msgs::PoseStamped goal = StampedPosefromSE2( p.x, p.y, 0.f );
		goal.header.frame_id = m_worldFrameId ;
		std::vector<geometry_msgs::PoseStamped> plan;
//ros::WallTime mpStartTime = ros::WallTime::now();

		float fendpot;
		bool bplansuccess = mpo_gph->makePlan(tid, fupperbound, true, start, goal, plan, fendpot);

//printf("[success: %d] [tid %d:] processed %d th point (%f %f) to (%f %f) marked %f potential \n ",
//										  bplansuccess, tid, idx,
//										  start.pose.position.x, start.pose.position.y,
//										  goal.pose.position.x, goal.pose.position.y, fendpot);


//ros::WallTime mpEndTime = ros::WallTime::now();
		gplansizes[idx] = plan.size();

//ros::WallTime mpAtomicStartTime = ros::WallTime::now();
		if( fendpot < fupperbound )
		{
			#pragma omp atomic write
				fupperbound = fendpot; // set new bound;
		}
	}
	delete mpo_gph;
}

} // end of repeatidx

std::vector<geometry_msgs::PoseStamped> best_plan ;
size_t best_len = 100000000 ;
size_t best_idx = 0;
for(size_t idx=0; idx < gplansizes.size(); idx++ )
{
	size_t curr_len = gplansizes[idx] ;
	if(curr_len < best_len && curr_len > MIN_TARGET_DIST)
	{
		best_len = curr_len ;
		best_idx = idx ;
		//best_plan = plan ;
	}
}

//std::clock_t GPEndTime = clock();

auto end_time = std::chrono::high_resolution_clock::now();
auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - begin_time);

float avg_gp_time_mm = elapsed.count() * 1e-6 /((float)nrepeat) ;
printf("numthreads: <%d>  gp_time : <%f mm>\n", mn_numthreads, avg_gp_time_mm);
m_ofs_time << numthreads << " " << m_points.points.size() << " " << avg_gp_time_mm << endl;
m_ofs_time << endl;

m_ofs_time.close();

mp_threadutil->read_procstat_cur();
mp_threadutil->meas_cpu_percent( mn_numthreads, m_points.points.size() );


p = m_points.points[best_idx];  // just for now... we need to fix it later

// display and debugging
//cv::Mat img_rgb;
//cv::cvtColor(img_padded, img_rgb, CV_GRAY2RGB);
//for(size_t idx=0; idx < voFrontierCands.size(); idx++)
//{
//	cv::Point pt = voFrontierCands[idx].GetCorrectedGridmapPosition() ;
//	//cv::Point pt = cv::Point(fpt.x, fpt.y) ;
//	cv::circle(img_rgb, cv::Point(pt.x , pt.y ), 3, cv::Scalar(0,255,0), 1, 8, 0 );
//}
//cv::namedWindow("tmp", 1);
//cv::imshow("tmp",img_rgb);
//cv::waitKey(0);


}


}

