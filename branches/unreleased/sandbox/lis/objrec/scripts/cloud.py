import distribute

def connect(private_key='~/.ssh/sdavies-thesis.pem',hostfile='/var/tmp/objrec/hostfile-experiment'):
    cloud_host_names = []
    with open(hostfile) as f:
        lines = f.readlines()
    lines = [l.strip()[:-8].strip() for l in lines if l.strip().endswith('slots=24')]
    lab_host_names = lines
    cloud_hosts = [distribute.host(h,username='ubuntu',private_key=private_key) for h in cloud_host_names]
    lab_hosts = [distribute.host(h,username='ubuntu',private_key=private_key) for h in lab_host_names]
    #hosts = [distribute.host(h,password,3) for h in group_host_names]
    hosts = [h for h in (cloud_hosts+lab_hosts) if h.ssh.get_transport() is not None]
    return hosts
