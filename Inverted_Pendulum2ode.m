function dxdt = Inverted_Pendulum2ode(X,X_dot,u,M,m,g,l,c,b,I)

theta     = X(2);
x_dot     = X_dot(1);
theta_dot = X_dot(2);

% u
F = u;

%% Compute dxdt
alpha_a = ((m^2)*(l^2)*((sin(theta))^2)+ M*m*l^2 +(M+m)*I);

x_ddot  = (b*m*l*theta_dot*cos(theta) + (m^2)*(l^2)*g*sin(theta)*cos(theta) + (I + m*(l^2))*(F-c*x_dot+ m*l*sin(theta)*theta_dot^2) )/alpha_a;

theta_ddot = -(F*m*l*cos(theta)-c*m*l*x_dot*cos(theta) + (m^2)*(l^2)*(theta_dot^2)*sin(theta)*cos(theta)+ (M+m)*(b*theta_dot + m*g*l*sin(theta)))/alpha_a;

dxdt = [x_ddot; theta_ddot];
