#!/usr/bin/env python
import distribute, cloud, subprocess, getpass, time, sys

if __name__=="__main__" and len(sys.argv) > 1:
    hostfile = sys.argv[1]
else:
    hostfile = None

hosts = cloud.connect(hostfile=hostfile)

#cmd = 'rm -rf /tmp/*'
#distribute.run_on_all(hosts,cmd)

#cmd = 'rm -rf /var/tmp/objrec/data/hold-out'
#distribute.run_on_all(hosts,cmd)

#distribute.copy(hosts,'/var/tmp/objrec/data/hold_out_set.py','/var/tmp/objrec/data')

#cmd = 'cd /var/tmp/objrec/data; ./hold_out_set.py > /tmp/test.sh; chmod u+x /tmp/test.sh; /tmp/test.sh; rm /tmp/test.sh'
#distribute.run_on_all(hosts,cmd)


def clean_old_jobs(hosts):
    jobs = []
    for h in hosts:
        j = distribute.job('killall -9 detect learn Xvfb')
        j.start(h)
        jobs.append(j)
    while len([1 for j in jobs if not j.isdone(verbose=False)])>0:
        time.sleep(1)

clean_old_jobs(hosts)

print 'starting xvfb...'
cmd = 'start-stop-daemon --start -b -x /var/tmp/objrec/scripts/start_xvfb.sh'
distribute.run_on_all(hosts,cmd)

cmd = 'mkdir /tmp/hold-out; mkdir /tmp/experiment'
distribute.run_on_all(hosts,cmd)

cmd = 'sudo dpkg --configure -a; sudo apt-get update; sudo apt-get install -y subversion make g++ libopencv-dev xvfb libgl1-mesa-glx mesa-common-dev libglapi-mesa libglu1-mesa-dev mpi-default-bin mpi-default-dev rsync'
distribute.run_on_all(hosts,cmd)

cmd = 'mkdir -p /var/tmp/objrec'
distribute.run_on_all(hosts,cmd)


distribute.copy(hosts,'/var/tmp/objrec/*','/var/tmp/objrec/')

cmd = 'mkdir -p /tmp/hold-out'
distribute.run_on_all(hosts,cmd)

distribute.copy(hosts,'/tmp/hold-out/*','/tmp/hold-out/')



cmd = 'cd /var/tmp/objrec/src; make clean; make'
#distribute.run_on_all(hosts,cmd)

#distribute.copy(hosts[:-1],'/var/tmp/objrec/scripts/filter_stderr.py','/var/tmp/objrec/scripts/')

#distribute.copy(hosts,home+'/ros/objrec/src/*','/tmp/objrec/src')
#distribute.copy(hosts,home+'/ros/objrec','/tmp/objrec')
#distribute.copy(hosts[1:],'/tmp/objrec.tar.bz2','/tmp/objrec/tar.bz2')

#cmd = 'cd /tmp/objrec/src; make clean; make'
#distribute.run_on_all(hosts,cmd)

#cmd = 'cd /tmp; bunzip2 -c objrec.tar.bz2 | tar xf -'
#distribute.run_on_all(hosts,cmd)

#cmd = 'mv /tmp/objrec /var/tmp'
#distribute.run_on_all(hosts[1:],cmd)

#old stuff:
#run on the local machine first:
#cmd = 'cd ~/ros; tar cvfh - objrec | bzip2 -9 - > /scratch/sdavies/objrec.tar.bz2'
#cmd = 'cd /tmp; bunzip2 -c objrec.tar.bz2 | tar xf -'
#distribute.run_on_all(hosts,cmd)
#
#alternatively via rsync:
#
#cp ~/.ssh/sdavies-thesis.pem /tmp
#rsync -avz -e 'ssh -l ubuntu -i /tmp/sdavies-thesis.pem' * 128.52.160.125:/var/tmp/objrec/data/
#rm /tmp/sdavies-thesis.pem
#
#attach a volume on CSAIL openstack:
#sudo cfdisk /dev/vdb
#sudo mkfs.ext4 /dev/vdb1
#sudo mount /dev/vdb1 /mnt
#sudo chown ubuntu /mnt
#ls /mnt
