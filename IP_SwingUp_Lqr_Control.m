% this code is written by Jitendra Singh
% Swing Up & LQR control design for Inverted Pendulum

clear all
close all
clc
%% parameters
r   = 0.006;       % radius of motor shaft
M   = 0.135;       % Mass of cart
m   = 0.1;         % mass of pendulum
I   = 0.0007176;   % MOI of Pendulum
l   = 0.2;         % COM of Pendulum
g   = 9.81;        % Gravity Constant
b   = 0.00007892;  % viscous damping at pivot of Pendulum
L   = 0.046;       % Motor Inductance
Rm  = 12.5;        % Motor Resistance
kb  = 0.031;       % Motor back emf constant
kt  = 0.031;       % Motor Torque constant
c   = 0.63;        % friction coefficient of cart

Er  = 2*m*g*l;     % Potential Energy of Pendulum
n   = 3;
k_swing = 1.2;
%% LQR control design 

% calculation of A Matrix
AA = I*(M+m) + M*m*(l^2);
aa = (((m*l)^2)*g)/AA;
bb = ((I +m*(l^2))/AA)*(c + (kb*kt)/(Rm*(r^2)));
cc  = (b*m*l)/AA;
dd  = (m*g*l*(M+m))/AA;
ee  = ((m*l)/AA)*(c + (kb*kt)/(Rm*(r^2)));
ff  = ((M+m)*b)/AA;
mm = ((I +m*(l^2))*kt)/(AA*Rm*r);
nn = (m*l*kt)/(AA*Rm*r);
A  =  [0 0 1 0; 0 0 0 1; 0 aa -bb -cc; 0 dd -ee -ff];
B = [0;0; mm; nn]; 

% calculation of LQR gain
Q = diag([200 1000 0 0]);
R  = 0.035;
KK = lqr(A,B,Q,R)

%% Close Loop Control simulation
Ts = 0.01; % sample time for simulation 
Tf = 9;   % simulation end time

X0    = [0; 1*(pi/180); 0;0];  % initial state
X_des = [0; pi; 0; 0];            % desired state
u0    = 0;
i = 0;
sat = @(x, x_max, x_min) min( x_max, max(x_min,x) ); % Saturation Function
for k = 0:Ts:Tf
    
    i = i+1;
    new_state = RK4_2nd_order(X0,Ts,u0, M,m,g,l,c,b,I);
    
    % take theta as (360- theta), when theta < 0, otherwise theta = theta
    % only for LQR control 
    if new_state(2) < 0
       th = 2*pi-abs(new_state(2));
       updated_state = [new_state(1); th; new_state(3); new_state(4)];
    else
        updated_state = new_state;
    end
    
    Xp(i,:) = updated_state'; % for plot 
    t(i)    = k;
    X0      = new_state; % update states for simulate system ode
    theta   = new_state(2);
    x_dot   = new_state(3);
    theta_dot  = new_state(4);
        
    % Total Energy of pendulum
    E = m*g*l*(1-cos(theta)) + (1/2)*(I + m*l^2)*(theta_dot^2);
    
    % Energy based swing up control 
    accel = 2*(E-Er)*sign(theta_dot*cos(theta));
    accel = k_swing*g*(sat(accel, n*g, -n*g));
    
    % feedback Linearization
    u_swing = (M+m)*(accel)+ 0*x_dot-m*l*( (theta_dot)^2)*sin(theta)- m*l*(cos(theta))*( ( b*theta_dot + m*l*accel*cos(theta) + m*g*l*sin(theta) )/(I+m*l^2) );
    
    % LQR control Design 
    u_volt = -KK*(updated_state-X_des); % u = -kX
    u_volt = sat(u_volt,12,-12);
    u_lqr  = volt2force(u_volt,X0(3),kt,kb,Rm,r);
   
    % Control Switching Condition
    if (abs(X_des(2)-updated_state(2)))*(180/pi) <= 30 % condition for lqr control
        u0 = u_lqr;
    else                                   % condition for swing up control
        u0 = u_swing;  
    end
        
end

%% Animation plot of Inverted Pedulum System
hf = figure()
for i = 1:8:length(Xp)
   IP_Animation(Xp(i,1),Xp(i,2));
   pause(0.01);
  % movieVector(i) =  getframe(hf);
   hold off
end

% %% Save the movie
% myWriter = VideoWriter('IP_SwingUp', 'Motion JPEG AVI');
% %myWriter = VideoWriter('IP_animation1', 'MPEG-4');
% myWriter.Quality    = 100;
% myWritter.FrameRate = 180;
% 
% % Open the VideoWriter object, write the movie, and class the file
% open(myWriter);
% writeVideo(myWriter, movieVector);
% close(myWriter);
%% plot results
figure()
axis(gca,'equal');
subplot(2,2,1);
plot(t,Xp(:,1));
grid on;
ylabel('X (m)');
xlabel('time [sec]');

subplot(2,2,2);
plot(t, (180/pi.*Xp(:,2)));
grid on;
ylabel('\theta (deg)');
xlabel('time [sec]');

subplot(2,2,3);
plot(t,Xp(:,3));
grid on;
ylabel('x dot (m/sec)');
xlabel('time [sec]');

subplot(2,2,4);
plot(t,(180/pi.*Xp(:,4)));
grid on;
ylabel('\theta dot (deg/sec)');
xlabel('time [sec]');
