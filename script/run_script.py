import subprocess
import signal
import os
import roslaunch
import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import subprocess
import shlex
import sys
import signal
import psutil
import time
import shutil


def main(argv):
    
    global exploration_status
    num_rounds  = int(argv[1])

    #num_thread_dev=[1,2,4,6,8,10,12,14,16]
    for num_threads in range(1,17):#num_thread_dev:
    
        for roundidx in range(1,num_rounds+1):
            
            start_time = time.time()
            elapsed_time = time.time() - start_time
            
            global exploration_status

            os.system('/home/hankm/binary_ws/mpbb_tests/bin/mpbb_run {}'.format(num_threads))

            outfiletxt = '/home/hankm/results/autoexploration/mpbb/planning_time_{}_{}.txt'.format(num_threads,roundidx)
            shutil.copy('/home/hankm/results/autoexploration/planning_time.txt', outfiletxt)

            cpuprofiletxt = '/home/hankm/results/autoexploration/cpu_utility/threadutility_{}.txt'.format(num_threads)
            shutil.copy('/home/hankm/results/autoexploration/threadutility.txt',cpuprofiletxt)
            time.sleep(1)
    
if __name__ == '__main__':
    main(sys.argv)

