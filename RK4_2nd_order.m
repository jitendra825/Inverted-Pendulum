function X_next = RK4_2nd_order(Xddot,dt,u, M,m,g,l,c,b,I)
x = Xddot(1:2)
z = Xddot(3:4)

k1 = z;
l1 = Inverted_Pendulum2ode(x,z,u, M,m,g,l,c,b,I);
k2 = z + (dt/2).*l1;
l2 = Inverted_Pendulum2ode( x+(dt/2).*k1, z+(dt/2).*l1, u, M,m,g,l,c,b,I);
k3 = z + (dt/2).*l2;
l3 = Inverted_Pendulum2ode( x+(dt/2).*k2, z+(dt/2).*l2, u, M,m,g,l,c,b,I);
k4 = z + dt.*l3;
l4 = Inverted_Pendulum2ode( x+dt.*k3, z+dt.*l3, u, M,m,g,l,c,b,I);

x_update = x + (dt/6).*(k1 + 2.*k2 + 2.*k3 + k4);
z_update = z + (dt/6).*(l1 + 2.*l2 + 2.*l3 + l4);

X_next = [x_update;z_update]