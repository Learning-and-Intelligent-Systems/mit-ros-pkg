import paramiko
import time
import math
import sys
import socket
import getpass
import datetime
import os
import subprocess

class job:
    def __init__(this,command,disk_outfile=None):
        this.command = command
        this.started = False
        this.done = False
        this.chan = None
        this.disk_outfile = disk_outfile
        this.out = ''
        this.err = ''
    def start(this,host):
        if this.disk_outfile is not None:
            if os.path.exists(this.disk_outfile):
                this.done = True
                this.chan = []
                return
            with open(this.disk_outfile+'.part','w') as f:
                f.write('')
        this.started = True
        this.host = host
        this.done = False
        this.ssh = host.ssh #to prevent it from being garbage collected
        try:
            #print 'about to open a new session to '+host.name
            this.chan = host.ssh.get_transport().open_session()
            #print 'done.'
            this.chan.set_combine_stderr(False)
            this.outfile = this.chan.makefile('rb')
            this.errfile = this.chan.makefile_stderr('rb')
            #this.chan.get_pty()
            #this.chan = host.ssh.invoke_shell()
        except (paramiko.ChannelException,paramiko.SSHException) as e:
            print 'host '+this.host.name+':',
            print e
            this.host.restart()
            try:
                this.chan = host.ssh.get_transport().open_session()
                this.chan.set_combine_stderr(False)
                this.outfile = this.chan.makefile('rb')
                this.errfile = this.chan.makefile_stderr('rb')
                #this.chan.get_pty()
            except (paramiko.ChannelException,paramiko.SSHException) as e:
                print 'host '+this.host.name+':',
                print e
                raise e
        except socket.error,e:
            print 'host '+this.host.name+':',
            print e
            raise e
        if this.chan is None:
            print 'Could not open a channel to host '+this.host
        try:
            this.chan.exec_command(this.command)
            this.host.n_current_jobs = this.host.n_current_jobs + 1
        except paramiko.SSHException,e:
            print 'host '+this.host.name+':',
            print e
            raise e

    def cancel(this):
        if this.disk_outfile is not None:
            with open(this.disk_outfile+'.part','a') as f:
                f.write(this.outfile.read(len(this.chan.in_buffer)))
        else:
            this.out = this.out + this.outfile.read(len(this.chan.in_buffer))
        this.err = this.err + this.outfile.read(len(this.chan.in_stderr_buffer))
        this.exit_status = -1
        this.done = True
        this.host.n_current_jobs = this.host.n_current_jobs - 1
        this.ssh = None
        this.chan.close()
        this.outfile.close()
        this.errfile.close()
        
    def isdone(this, verbose=True, auto_restart=False):
        if this.chan is None:
            return False
        if this.done:
            return True
        #empty the buffers to prevent them from getting full
        if this.disk_outfile is not None:
            with open(this.disk_outfile+'.part','a') as f:
                f.write(this.outfile.read(len(this.chan.in_buffer)))
        else:
            this.out = this.out + this.outfile.read(len(this.chan.in_buffer))
        this.err = this.err + this.errfile.read(len(this.chan.in_stderr_buffer))
        if this.chan.exit_status_ready():
            this.err = this.err + this.errfile.read()
            this.err.replace('libdc1394 error: Failed to initialize libdc1394\n','')### a hack for an OpenCV error
            if this.disk_outfile is not None:
                with open(this.disk_outfile+'.part','a') as f:
                    f.write(this.outfile.read())
            else:
                this.out = this.out + this.outfile.read()
            this.exit_status = this.chan.recv_exit_status()
            this.done = True
            this.host.n_current_jobs = this.host.n_current_jobs - 1
            this.ssh = None #so it can be garbage collected
            this.chan.close()
            this.outfile.close()
            this.errfile.close()

            if this.err!='' or this.chan.recv_exit_status()!=0:
                if verbose:
                    print 'it is now: '+datetime.datetime.today().isoformat(' ')+' '+\
                        str((time.time()-this.host.start_time)/60./60.)+' hours since (re)starting host '+\
                        this.host.name+' command: "'+\
                        this.command+'" error: '+this.err+\
                        ' output: '+this.out+\
                        ' exit status: '+str(this.exit_status)
                    if auto_restart:
                        print 'restarting job now'
                if auto_restart:
                    tmp_host = this.host
                    this.__init__(this.command,this.disk_outfile)
                    this.start(tmp_host)
                    return False
                else:
                    if this.disk_outfile is not None:
                        os.rename(this.disk_outfile+'.part',this.disk_outfile)
                    return True
            else:
                if this.disk_outfile is not None:
                    os.rename(this.disk_outfile+'.part',this.disk_outfile)
                return True
        else:
            return False

SSH_PORT = 22
def connect_keyboard_interactive(self, hostname, port=SSH_PORT, username=None, password=None, pkey=None,
                key_filename=None, timeout=None, allow_agent=True, look_for_keys=True,
                compress=False):
    """
    modified from paramiko.SSHClient.connect() and
    paramiko.Transport.auth_interactive() so that it will not cause failed
    authentication attempts on hosts that allow "keyboard_interactive"
    but not "password" authentication (even though those two are
    essentially the same thing).
    """
    
    for (family, socktype, proto, canonname, sockaddr) in socket.getaddrinfo(hostname, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
        if socktype == socket.SOCK_STREAM:
            af = family
            addr = sockaddr
            break
    else:
        # some OS like AIX don't indicate SOCK_STREAM support, so just guess. :(
        af, _, _, _, addr = socket.getaddrinfo(hostname, port, socket.AF_UNSPEC, socket.SOCK_STREAM)
    sock = socket.socket(af, socket.SOCK_STREAM)
    if timeout is not None:
        try:
            sock.settimeout(timeout)
        except:
            pass
    sock.connect(addr)
    t = self._transport = paramiko.Transport(sock)
    t.use_compression(compress=compress)
    if self._log_channel is not None:
        t.set_log_channel(self._log_channel)
    t.start_client()
    paramiko.resource.ResourceManager.register(self, t)

    server_key = t.get_remote_server_key()
    keytype = server_key.get_name()

    if port == SSH_PORT:
        server_hostkey_name = hostname
    else:
        server_hostkey_name = "[%s]:%d" % (hostname, port)
    our_server_key = self._system_host_keys.get(server_hostkey_name, {}).get(keytype, None)
    if our_server_key is None:
        our_server_key = self._host_keys.get(server_hostkey_name, {}).get(keytype, None)
    if our_server_key is None:
        # will raise exception if the key is rejected; let that fall out
        self._policy.missing_host_key(self, server_hostkey_name, server_key)
        # if the callback returns, assume the key is ok
        our_server_key = server_key

    if server_key != our_server_key:
        raise paramiko.BadHostKeyException(hostname, server_key, our_server_key)

    if username is None:
        username = getpass.getuser()

    if key_filename is None:
        key_filenames = []
    elif isinstance(key_filename, (str, unicode)):
        key_filenames = [ key_filename ]
    else:
        key_filenames = key_filename
    def handler(title, instructions, fields):
        if len(fields) > 1:
            raise SSHException('Fallback authentication failed.')
        if len(fields) == 0:
            # for some reason, at least on os x, a 2nd request will
            # be made with zero fields requested.  maybe it's just
            # to try to fake out automated scripting of the exact
            # type we're doing here.  *shrug* :)
            return []
        return [ password ]
    self._transport.auth_interactive(username, handler)


paramiko.SSHClient.connect_keyboard_interactive = connect_keyboard_interactive

class host:
    def __init__(this,hostname,username=None,password=None,private_key=None,leave_n_cpu_cores=0):
        this.name = hostname
        this.username = username
        this.password = password
        this.private_key = os.path.expanduser(private_key)
        this.leave_n_cpu_cores=leave_n_cpu_cores
        this.restart()
    def restart(this):
        this.n_current_jobs = 0
        this.start_time = time.time()
        this.ssh = paramiko.SSHClient()
        this.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        if this.password is not None:
            try:
                this.ssh.connect_keyboard_interactive(this.name,
                                                      username=this.username,
                                                      password=this.password,
                                                      timeout=60)
            except paramiko.AuthenticationException:
                print 'Authentication failed while connecting to '+this.name+'.'
            except socket.error, e:
                print str(e)+' for host '+this.name
            except paramiko.SSHException, e:
                print str(e)+' for host '+this.name
        else:
            try:
                this.ssh.connect(this.name,
                                 username=this.username,
                                 key_filename=this.private_key,timeout=60)
            except socket.error, e:
                print str(e)+' for host '+this.name
            except paramiko.SSHException, e:
                print str(e)+' for host '+this.name
                
    def sample(this):
        j = job('cat /proc/stat')
        j.start(this)
        begin = time.time()
        while not j.isdone() and time.time()-begin<8.0:
            time.sleep(.01)
        if not j.isdone():
            print 'Timeout while sampling host '+this.name
            j.cancel()
            return (this.cpu_used, this.cpu_total)
        lines = j.out.splitlines()
        used = []
        total = []
        for line in lines[1:]:
            if line[0:3]=='cpu':
                tokens = line.split()
                idle = long(tokens[4])
                work = long(tokens[1])+long(tokens[2])+long(tokens[3])
                used.append(work)
                total.append(work+idle)
        return (used, total)
    def begin_n_free_cpus(this):
        this.n_free_cpus_ready = False
        (this.cpu_used, this.cpu_total) = this.sample()
    def get_n_free_cpus(this):
        if not this.n_free_cpus_ready:
            (u,t) = this.sample()
            used = [u[j]-this.cpu_used[j] for j in range(len(u))]
            total = [t[j]-this.cpu_total[j] for j in range(len(t))]
            cpu_load = [float(used[j])/float(total[j]) if total[j]!=0 else 1.0 \
                        for j in range(len(used))]
            this.n_free_cpus = len(filter(lambda x: x<0.1, cpu_load))
            #the following if statement is not thread safe,
            #but we are not using threads
            if this.n_free_cpus > len(t) - this.n_current_jobs:
                this.n_free_cpus = len(t) - this.n_current_jobs
            this.n_free_cpus_ready = True
            return min(1,this.n_free_cpus)
        else:
            return min(1,this.n_free_cpus)
    def __del__(this):
        this.ssh.close()

def n_available_cpus(hosts):
    for h in hosts:
        h.begin_n_free_cpus()
    time.sleep(2)
    n = [h.get_n_free_cpus()-h.leave_n_cpu_cores for h in hosts]
    #print str(n)+' free cpus'
    return n


def time_string(t):
    minutes = math.floor(t/60.0)
    seconds = t%60
    return '%02d:%02d' % (minutes,seconds)

def progress(completed,total,start_time):
    elapsed_time = time.time()-start_time
    if completed==0:
        time_left = '??:??'
    else:
        seconds_left = elapsed_time/completed*(total-completed)
        time_left = time_string(seconds_left)
    print "\r"+str(total-completed) + " of " + str(total) + \
          " jobs remaining. "\
          +time_string(elapsed_time)+" elapsed, " + \
          time_left + " left.       ",
    sys.stdout.flush()

def run_on_all(hosts,cmd):
    jobs = [job(cmd) for h in range(len(hosts))]
    start_time = time.time()
    for j in range(len(jobs)):
        jobs[j].start(hosts[j])
    n_remaining = len([1 for j in jobs if j.isdone()])
    while n_remaining<len(jobs):
        n_remaining = len([1 for j in jobs if j.isdone()])
        progress(n_remaining,len(jobs),start_time)
        time.sleep(1)
    print ''
    for j in jobs:
        if j.err!='':
            print 'error on host '+j.host.name+':'
            print j.err

def copy(hosts,src,dst):
    print 'copying from '+src+' to '+dst+' on '+str(len(hosts))+' hosts'
    start_time = time.time()
    commands = ['rsync -avz --delete '+src+' '+h.username+'@'+h.name+':'+dst for h in hosts]
    #print commands
    procs = [subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE) for cmd in commands]

    n_completed = 0
    while n_completed<len(procs):
        n_completed = len([1 for p in procs if p.poll() is not None])
        progress(n_completed,len(procs),start_time)
        time.sleep(1)
    print 'copying took '+str(time.time()-start_time)+' seconds'
