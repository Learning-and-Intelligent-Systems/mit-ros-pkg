# HitPolicies.msg
# Table of {ball trajectory params --> hit params}

float32[] ball_bounce_params  # N-by-8: (x,y, vx,vy,vz, sx,sy,sz)
float32[] hit_params          # N-by-9: (x,y,z, vx,vy,vz, nx,ny,nz)
