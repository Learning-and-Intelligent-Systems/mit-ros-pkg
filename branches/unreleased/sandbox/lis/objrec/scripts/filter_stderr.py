#!/usr/bin/env python
import sys, subprocess
cmd = ''
for arg in sys.argv[1:]:
    cmd += arg + ' '

p = subprocess.Popen(cmd,
                     shell=True,
                     stdout=subprocess.PIPE,
                     stdin=subprocess.PIPE,
                     stderr=subprocess.PIPE)

(out,err) = p.communicate('')

err = err.replace('libdc1394 error: Failed to initialize libdc1394\n','')

sys.stdout.write(out)
sys.stderr.write(err)
sys.exit(p.returncode)
