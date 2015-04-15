#!/usr/bin/python

import time
import os
import subprocess


import string
import commands

################################################################################
#### INITIALISE ################################################################
################################################################################


if __name__ == '__main__':
    
    ################################################################################
    if os.path.isdir('/home/lvs1e09/tmp'):
        print 'Deleting tmp dir'
        os.system('rm -r /home/lvs1e09/tmp')
        print 'Making tmp dir'
        os.mkdir('/home/lvs1e09/tmp')
    else:
        print 'Making tmp dir'
        os.mkdir('/home/lvs1e09/tmp')
        
    ################################################################################
    folders       = string.split(commands.getoutput('ls ~/.ros/logFiles/ --group-directories-first'),'\n')
    latest_folder = folders[0]
    print 'Latest folder = ',latest_folder
    
    ################################################################################
    print 'Making current folder'
    dir = '/home/lvs1e09/tmp/' + latest_folder
    os.mkdir(dir)
    
    ################################################################################
    print 'Copying logfiles to tmp directory'
    cmd = 'cp ~/.ros/logFiles/' + latest_folder + '/*.*' + ' ~/tmp/' + latest_folder
    os.system(cmd)
    
    ################################################################################
    print 'Copying code to tmp directory'
    cmd = 'cp -r ~/delphin-ros/DelphinROSv2/nodes ~/tmp/' + latest_folder + '/code'
    os.system(cmd)
    cmd = 'cp -r ~/delphin-ros/DelphinROSv2/launch ~/tmp/' + latest_folder + '/code'
    os.system(cmd)
    cmd = 'cp -r ~/delphin-ros/DelphinROSv2/msg ~/tmp/' + latest_folder + '/code'
    os.system(cmd)
    
    ################################################################################
    cmd = 'zip -r ~/tmp/DATA ~/tmp/' + latest_folder
    os.system(cmd)