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
    
    if len(sys.argv) != 3:
        print "usage: {} <num_rounds> <num_threads>".format(argv[0])
        return -1
    
    global exploration_status
    num_rounds  = int(argv[1])
    num_threads = int(argv[2])

    #num_thread_dev=[1,2,4,6,8,10,12,14,16]
    for num_threads in range(1,num_threads+1):#num_thread_dev:
    
        for roundidx in range(1,num_rounds+1):
            
            start_time = time.time()
            elapsed_time = time.time() - start_time
            
            homedir = os.getenv("HOME")
            
            global exploration_status

            os.system('{}/binary_ws/mpbb_tests/bin/mpbb_run {}'.format(homedir,num_threads))

            outfiletxt = '{}/results/autoexploration/mp_process/mp_for/runtime_{}_{}.txt'.format(homedir,num_threads,roundidx)
            shutil.copy('{}/results/autoexploration/mp_process/runtime.txt'.format(homedir), outfiletxt)

            time.sleep(1)
    
if __name__ == '__main__':
    main(sys.argv)

