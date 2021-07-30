function force_out = volt2force(v,x_dot,kt,kb,Rm,r)

force_out = (kt*v*r - kt*kb*x_dot)/(Rm*(r^2));