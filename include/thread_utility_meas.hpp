/*
 * thread_utility_meas.hpp
 *
 *  Created on: Jan 28, 2022
 *      Author: hankm
 */

#ifndef INCLUDE_THREAD_UTILITY_MEAS_HPP_
#define INCLUDE_THREAD_UTILITY_MEAS_HPP_

//#include <ros/ros.h>
//#include <ros/console.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <thread>
#include <mutex>
#include <cassert>

#define BUF_MAX 1024
#define MAX_CPU 128
#define NUMTOTCPU (16)

namespace autoexplorer
{

class ThreadUtilityMeas
{
public:
	ThreadUtilityMeas();
	ThreadUtilityMeas(int numthreads, std::string str_filepath);
	virtual ~ThreadUtilityMeas();
	int RequestFinish();

	inline void set_numtotcpu( uint32_t ncpu ){ mn_numtotcpus = ncpu; }
	int read_fields(FILE* fp, uint64_t* pufileds );
	int read_procstat_old( ) ;
	int read_procstat_cur( ) ;
	int meas_cpu_percent( int nthreads, int nfpts ) ;
	int RunMeas();

protected:
	bool mb_finishrequested;

	uint32_t mn_numtotcpus ;
	int mn_numcpus; // num cpus
	FILE *m_fp;

	uint64_t *mpu_fields, *mpu_total, *mpu_total_old, *mpu_idle, *mpu_idle_old, *mpu_del_total, *mpu_del_idle;
	double* mpd_cpu_usage_percent;
	int mn_update_cycle, mn_count;
	std::clock_t m_time_old, m_time ;
	std::ofstream m_ofs;

private:

	std::mutex m_mutex_finishrequest ;
};

}


#endif /* INCLUDE_THREAD_UTILITY_MEAS_HPP_ */
