% this code is written by Jitendra Singh
% LQR control design for Inverted Pendulum

clear all
close all
clc
%% parameters
r   = 0.006;       % radius of motor shaft
M = 0.135;       % Mass of cart
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

%% LQR control design 

% calculation of Sate Space Matrix/System Matrix (A,B)

AA = I*(M+m) + M*m*(l^2);
aa = (((m*l)^2)*g)/AA;
bb = ((I +m*(l^2))/AA)*(c + (kb*kt)/(Rm*(r^2)));
cc = (b*m*l)/AA;
dd = (m*g*l*(M+m))/AA;
ee = ((m*l)/AA)*(c + (kb*kt)/(Rm*(r^2)));
ff = ((M+m)*b)/AA;
mm = ((I +m*(l^2))*kt)/(AA*Rm*r);
nn = (m*l*kt)/(AA*Rm*r);

A  =  [0 0 1 0; 0 0 0 1; 0 aa -bb -cc; 0 dd -ee -ff];
B  = [0;0; mm; nn]; 

% calculation of LQR gain
Q = diag([1200 1500 0 0]);
R  = 0.035;
KK = lqr(A,B,Q,R)

%% Close Loop Control simulation
Ts = 0.01; % sample time for simulation 
Tf = 10;   % simulation end time

X0    = [0.2; 160*(pi/180); 0;0]; % initial state/ take the (Initial Value of theta) > 0 {MUST}
                                  % such as angle will always measured from vertical downward axis(from 4th quadrant) 
X_des = [0; pi; 0; 0];            % desired state
u0    = 0;
i = 0;
sat = @(x, x_max, x_min) min( x_max, max(x_min,x) ); % Saturation Function
for k = 0:Ts:Tf
    
    i = i+1;
    new_state = RK4_2nd_order(X0,Ts,u0, M,m,g,l,c,b,I);
    Xp(i,:) = new_state'; % for plot 
    t(i)    = k;
    X0      = new_state;
    
    % LQR control Design 
    u_volt = -KK*(X0-X_des);
    u0     = volt2force(u_volt,X0(3),kt,kb,Rm,r);
    u0 = sat(u0,12,-12);
end

%% Animation plot of Inverted Pedulum System
figure()
for i = 1:12:length(Xp)
   IP_Animation(Xp(i,1),Xp(i,2))
   pause(0.01);
    % movieVector(i) =  getframe(hf);
   hold off
end

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
