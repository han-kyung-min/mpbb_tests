/*
 * thread_utility_meas.cpp
 *
 *  Created on: Jan 28, 2022
 *      Author: hankm
 */

#include "thread_utility_meas.hpp"

namespace autoexplorer
{

ThreadUtilityMeas::ThreadUtilityMeas(){}

ThreadUtilityMeas::ThreadUtilityMeas(int numthreads, std::string str_filepath):
mn_numcpus(numthreads+1),  // cpu cpu0, cpu1, .... cpu15
mn_update_cycle(0), mn_count(0),
mb_finishrequested(false)
{
	m_fp = nullptr;
	mpu_fields 		=	new uint64_t[10];
	mpu_total 		= 	new uint64_t[MAX_CPU];
	mpu_total_old	=	new uint64_t[MAX_CPU];
	mpu_idle 		=	new uint64_t[MAX_CPU];
	mpu_idle_old 	=	new uint64_t[MAX_CPU];
	mpu_del_total	= 	new uint64_t[MAX_CPU];
	mpu_del_idle 	=	new uint64_t[MAX_CPU];
	mpd_cpu_usage_percent= new double[MAX_CPU];
	m_ofs = std::ofstream(str_filepath);
}

ThreadUtilityMeas::~ThreadUtilityMeas()
{
	delete [] mpu_fields ;
	delete [] mpu_total ;
	delete [] mpu_total_old ;
	delete [] mpu_idle ;
	delete [] mpu_idle_old ;
	delete [] mpu_del_total ;
	delete [] mpu_del_idle ;
	delete [] mpd_cpu_usage_percent;
	m_ofs.close();
}

int ThreadUtilityMeas::RequestFinish()
{
	std::unique_lock<std::mutex> lock( m_mutex_finishrequest );
	{
		mb_finishrequested = true;
	}
}

int ThreadUtilityMeas::read_fields (FILE* fp, uint64_t* pufields)
{
  int retval;
  char buffer[BUF_MAX];

  if (!fgets (buffer, BUF_MAX, fp))
  { perror ("Error"); }
  /* line starts with c and a string. This is to handle cpu, cpu[0-9]+ */
  // the reading includes the very first cpu (which is an avg est of cpu0~cpuN)
  retval = sscanf (buffer, "c%*s %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
                            &pufields[0], // user
                            &pufields[1], // nice
                            &pufields[2], // system
                            &pufields[3], // idle
                            &pufields[4], // iowait
                            &pufields[5], // irq
                            &pufields[6], // softirq
                            &pufields[7], // steal
                            &pufields[8], // guest
                            &pufields[9]); // guest_nice
  if (retval == 0)
  { return -1; }
  if (retval < 4) /* Atleast 4 fields is to be read */
  {
	fprintf (stdout,"%lu %lu %lu %lu %lu %lu %lu\n", pufields[0], pufields[1],pufields[2],pufields[3],pufields[4],pufields[5],pufields[6]);
    fprintf (stderr, "Error reading /proc/stat cpu field\n");
    return 0;
  }
  return 1;
}

int ThreadUtilityMeas::read_procstat_old( )
{

	m_time_old = clock();
	m_fp = fopen ("/proc/stat", "r");
	if (m_fp == NULL)
	{
		perror ("Error!! cannot open /proc/stat");
	}

	for (int count = 0; count < mn_numcpus; count++)
	{
		  if (!read_fields (m_fp, mpu_fields))
		  { return 0; }

		  uint64_t iowait = mpu_fields[4] ;
		  uint64_t idle_time = mpu_fields[3] + mpu_fields[4] ; // idle + iowait
		  uint64_t non_idle_time = mpu_fields[0] + mpu_fields[1] + mpu_fields[2] + mpu_fields[5] +
				  	  	  	  	   mpu_fields[6] + mpu_fields[7] ;
		  uint64_t tot_time = idle_time + non_idle_time;
		  mpu_total_old[count] = tot_time ;
		  mpu_idle_old[count]  = idle_time ;
	}
	fclose(m_fp);
}

int ThreadUtilityMeas::read_procstat_cur( )
{
	m_time = clock();
	m_fp = fopen ("/proc/stat", "r");
	if (m_fp == NULL)
	{
		perror ("Error!! cannot open /proc/stat");
	}

	for (int count = 0; count < mn_numcpus; count++)
	{
		  if (!read_fields (m_fp, mpu_fields))
		  { return 0; }

		  uint64_t iowait = mpu_fields[4] ;
		  uint64_t idle_time = mpu_fields[3] + mpu_fields[4] ;
		  uint64_t non_idle_time = mpu_fields[0] + mpu_fields[1] + mpu_fields[2] + mpu_fields[5] +
				  	  	  	  	   mpu_fields[6] + mpu_fields[7];
		  uint64_t tot_time = idle_time + non_idle_time;
		  mpu_total[count] = tot_time ;
		  mpu_idle[count]  = idle_time ;
	}
	fclose(m_fp);
}

int ThreadUtilityMeas::meas_cpu_percent( int nthreads, int nfpts )
{
	//	        cpu : user nice system idle iowait irq softirq steal guest guest_nice
	//	        cpu0 : ...
	//	        idle_time = idle + iowait
	//	        non_idle_time = user + nice + system + irq + softirq + steal
	//	        total = idle_time + non_idle_time
	//
	//	        previous_total = previous_idle + previous_non_idle
	//	        current_total = current_idle + current_non_idle
	//	        diff_total = current_total - previous_total
	//	        diff_idle = current_idle - previous_idle
	//	        cpu_usage_percentage = ( diff_total - diff_idle )/ diff_total * 100

	assert(mn_numtotcpus > 0);

	float elapsed_time_mm = (float)(m_time - m_time_old) / CLOCKS_PER_SEC * 1000;
	m_ofs << nthreads << " " << nfpts << " \n"; //<< elapsed_time_mm << ": \t";

	double fthread_percent = (double)nthreads/ ( (double)mn_numtotcpus ) ;
	// cpu 0
	mpu_del_total[0] 	= mpu_total[0] - mpu_total_old[0] ;
	mpu_del_idle[0]	= mpu_idle[0]  - mpu_idle_old[0] ;
	mpd_cpu_usage_percent[0] =((double)(mpu_del_total[0] - mpu_del_idle[0]) / (double) mpu_del_total[0]) * 100;
	m_ofs << mpd_cpu_usage_percent[0] << " " << mpd_cpu_usage_percent[0] / fthread_percent << " \n";

	for(int idx=1; idx < mn_numtotcpus+1; idx++)
	{
		mpu_del_total[idx] 	= mpu_total[idx] - mpu_total_old[idx] ;
		mpu_del_idle[idx]	= mpu_idle[idx]  - mpu_idle_old[idx] ;
		mpd_cpu_usage_percent[idx] =((double)(mpu_del_total[idx] - mpu_del_idle[idx]) / (double) mpu_del_total[idx]) * 100;
//		m_ofs << mpd_cpu_usage_percent[idx] << " " << mpu_idle_old[idx] << " " << mpu_idle[idx] << " "
//											<<  mpu_total_tick_old[idx] << " " << mpu_total_tick[idx] << " "
//											<<  mpu_del_idle[idx] <<  " " << mpu_del_total_tick[idx] << std::endl;
		m_ofs << mpd_cpu_usage_percent[idx] << " " ;
	}
	m_ofs << std::endl;
}

int ThreadUtilityMeas::RunMeas()
{
//	mn_update_cycle = 0,
//	mn_count = 0;
//	double percent_usage;
//	std::clock_t currtime;
//
//	while (!mb_finishrequested)
//	{
//		if( 1 ) //
//		{
////	        cpu : user nice system idle iowait irq softirq steal guest guest_nice
////	        cpu0 : ...
////	        idle_time = idle + iowait
////	        non_idle_time = user + nice + system + irq + softirq + steal
////	        total = idle_time + non_idle_time
////
////	        previous_total = previous_idle + previous_non_idle
////	        current_total = current_idle + current_non_idle
////	        diff_total = current_total - previous_total
////	        diff_idle = current_idle - previous_idle
////	        cpu_usage_percentage = ( diff_total - diff_idle )/ diff_total * 100
//
//			for (int count = 0; count < mn_numcpus; count++)
//			{
//			  if (!read_fields (m_fp, mpu_fields))
//			  { return 0; }
//
//			  uint64_t iowait = mpu_fields[4] ;
//			  uint64_t idle_time = mpu_fields[3] + mpu_fields[4] ;
//			  uint64_t non_idle_time = mpu_fields[0] + mpu_fields[1] + mpu_fields[2] + mpu_fields[5] + mpu_fields[6] + mpu_fields[7];
//			  uint64_t tot_time = idle_time + non_idle_time;
//			for ( int ncpuid =0; ncpuid < mn_numcpus+1; ncpuid++ )
//			{
//				read_fields(m_fp, mpu_fields);
//				for (int i=0; i<10; i++)
//				{ mpu_total_tick_old[i] += mpu_fields[i]; }
//				mpu_idle_old[ncpuid] = mpu_fields[3]; /* idle ticks index */
//			}
//			int count = 0;
//
//			  mpu_del_total_tick[count] = mpu_total_tick[count] - mpu_total_tick_old[count];
//			  mpu_del_idle[count] = mpu_idle[count] - mpu_idle_old[count];
//
//			  percent_usage = ((double)(mpu_del_total_tick[count] - mpu_del_idle[count]) / (double) mpu_del_total_tick[count]) * 100;
//			  currtime = clock();
//
//			  if (count == 0)
//			  { //printf ("Total CPU Usage: %3.2lf%%\n", percent_usage); }
//				  for(int i=0; i < 10; i++)
//				  	  m_ofs << mpu_fields[i] << " ";
//				  m_ofs << std::endl;
//				  m_ofs << mpu_total_tick[count] << " " << mpu_total_tick_old[count] << std::endl;
//				  m_ofs <<  mn_update_cycle << " Total CPU usage: " << percent_usage << "("<< mpu_del_idle[count] <<"/"
//						<<  mpu_del_total_tick[count] << ")"<< std::endl;
//			  }
//			  else
//			  {
//				  m_ofs << "\t CPU("<< count-1 << ") " << percent_usage << "("<< mpu_del_idle[count] <<"/"
//													   <<  mpu_del_total_tick[count] << ")"<< std::endl;
//			  }
//
//			  mpu_total_tick_old[count] = mpu_total_tick[count];
//			  mpu_idle_old[count] = mpu_idle[count];
//
//			}
//			mn_update_cycle++;
//			fclose(m_fp);
//		}
//	}

	/* Ctrl + C quit, therefore this will not be reached. We rely on the kernel to close this file */
	//fclose (m_fp);

	return 0;
}


}
