% Edwards, Grace
% Choi, Derrick
% Trunko, Adam
% Conzet, Addison
% ASEN 3128
% Lab 3
% Created 9/25/2020
 clear
 close all 
 clc 
 
 %% Constants 
m = 0.068; %Quadrotor mass kg 
radius = 0.060; %Radial distance from CG to propeller m
km = 0.0024; %Control moment coefficient N*m/(N)
Ix = 6.8E-5; %Body x-axis Moment of Inertia kg*m^2
Iy = 9.2E-5; %Body y-axis Moment of Inertia kg*m^2
Iz = 1.35E-4; %Body z-axis Moment of Inertia kg*m^2
n = 1E-3; %Aerodynamic force coefficient N/(m/s)^2
mu = 2E-6; %Aerodynamic moment coefficient N*m/(rad/s)^2
g = 9.81; %m/s^2
global uncontrolled uncontrolled_3D controlled controlled_3D;
%% Problem 3
% Steady hovering flight all F must equal mg to balance forces
Lc = 0;
Mc = 0;
Nc = 0;
% Zc equals sum of forces 
Zc  = -m*g;
tspan = [0 10];
% State Vector for hoevring steady flight with 5 degree roll
x0 = [0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t, out] = ode45(@(t,x) quadrotorODE(t,x,m,Ix,Iy,Iz,n,mu,Zc,Lc,Mc,Nc), tspan,x0); 

% State Vector for hoevring steady flight with 5 degree pitch
x0 = [0 ;0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t1, out1] = ode45(@(t1,x) quadrotorODE(t1,x,m,Ix,Iy,Iz,n,mu,Zc,Lc,Mc,Nc), tspan,x0);

% State Vector for hoevring steady flight with 5 degree yaw
x0 = [0 ;0 ;0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t2, out2] = ode45(@(t2,x) quadrotorODE(t2,x,m,Ix,Iy,Iz,n,mu,Zc,Lc,Mc,Nc), tspan,x0);

% State Vector for hoevring steady flight with 0.1 rad/sec roll rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t3, out3] = ode45(@(t3,x) quadrotorODE(t3,x,m,Ix,Iy,Iz,n,mu,Zc,Lc,Mc,Nc), tspan,x0);

% State Vector for hoevring steady flight with 0.1 rad/sec pitch rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t4, out4] = ode45(@(t4,x) quadrotorODE(t4,x,m,Ix,Iy,Iz,n,mu,Zc,Lc,Mc,Nc), tspan,x0);

% State Vector for hovering steady flight with 0.1 rad/sec yaw rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1)]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t5, out5] = ode45(@(t5,x) quadrotorODE(t5,x,m,Ix,Iy,Iz,n,mu,Zc,Lc,Mc,Nc), tspan,x0);

% State Vector for hoevring steady controlled flight with 0.1 rad/sec roll rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0 ;0; Zc; -0.004*rad2deg(0.1); 0; 0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t6, out6] = ode45(@(t6,x) quadrotorODE_controlled(t6,x,m,Ix,Iy,Iz,n,mu), tspan,x0);

% State Vector for hoevring steady controlled flight with 0.1 rad/sec pitch rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1); 0; Zc; 0; -0.004*rad2deg(0.1); 0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t7, out7] = ode45(@(t7,x) quadrotorODE_controlled(t7,x,m,Ix,Iy,Iz,n,mu), tspan,x0);

% State Vector for hoevring steady controlled flight with 0.1 rad/sec yaw rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1); Zc; 0; 0; -0.004*rad2deg(0.1)]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t8, out8] = ode45(@(t8,x) quadrotorODE_controlled(t8,x,m,Ix,Iy,Iz,n,mu), tspan,x0);


%% Plots for part 3
uncontrolled = {'5^o roll', '5^o pitch', '5^o yaw', '0.1 rad/s roll rate', '0.1 rad/s pitch rate', '0.1 rad/s yaw rate'...
    '5^o roll (linear)', '5^o pitch (linear)', '5^o yaw (linear)', '0.1 rad/s roll rate (linear)', '0.1 rad/s pitch rate (linear)', '0.1 rad/s yaw rate (linear)'};
controlled = {'0.1 rad/s roll rate', '0.1 rad/s pitch rate', '0.1 rad/s yaw rate'};
uncontrolled_3D = {'5^o roll','5^o pitch','5^o yaw','0.1 rad/s roll rate','0.1 rad/s pitch rate',...
    '0.1 rad/s yaw rate','5^o roll (linear)','5^o pitch (linear)','5^o yaw (linear)','0.1 rad/s roll rate (linear)','0.1 rad/s pitch rate (linear)',...
    '0.1 rad/s yaw rate (linear)','start of flight path','end of flight path'};
controlled_3D = {'0.1 rad/s roll rate','0.1 rad/s pitch rate','0.1 rad/s yaw rate','start of flight path',...
    'end of flight path'};
PlotAircraftSim(t,out,ones(length(t),4)*Zc/4,'-b',1)
%%
PlotAircraftSim(t1,out1,ones(length(t1),4)*Zc/4,'-r',1)
PlotAircraftSim(t2,out2,ones(length(t2),4)*Zc/4,'-k',1)
PlotAircraftSim(t3,out3,ones(length(t3),4)*Zc/4,':b',1)
PlotAircraftSim(t4,out4,ones(length(t4),4)*Zc/4,':r',1)
PlotAircraftSim(t5,out5,ones(length(t5),4)*Zc/4,':k',1)

force6 = ComputeMotorForces(out6(:,13),out6(:,14),out6(:,15),out6(:,16),radius,km);
force7 = ComputeMotorForces(out7(:,13),out7(:,14),out7(:,15),out7(:,16),radius,km);
force8 = ComputeMotorForces(out8(:,13),out8(:,14),out8(:,15),out8(:,16),radius,km);

PlotAircraftSim(t6,out6,force6','-b',2)
PlotAircraftSim(t7,out7,force7','-r',2)
PlotAircraftSim(t8,out8,force8','-k',2)
%% Problem 4
% State Vector for hoevring steady flight with 5 degree roll
Tfinal = 2;
x0 = [0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[out9,t9] = LinearizedEOM(x0,Tfinal);

% State Vector for hoevring steady flight with 5 degree pitch
x0 = [0 ;0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[out10,t10] = LinearizedEOM(x0,Tfinal);

% State Vector for hoevring steady flight with 5 degree yaw
x0 = [0 ;0 ;0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[out11,t11] = LinearizedEOM(x0,Tfinal);

% State Vector for hoevring steady flight with 0.1 rad/sec roll rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[out12,t12] = LinearizedEOM(x0,Tfinal);

% State Vector for hoevring steady flight with 0.1 rad/sec pitch rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[out13,t13] = LinearizedEOM(x0,Tfinal);

% State Vector for hovering steady flight with 0.1 rad/sec yaw rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1)]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[out14,t14] = LinearizedEOM(x0,Tfinal);
%% Plots for part 4
%change colors 
PlotAircraftSim(t9,out9,ones(length(t9),4)*Zc/4,'-c',1)
PlotAircraftSim(t10,out10,ones(length(t10),4)*Zc/4,'- 0.75 0 0.75',1)
PlotAircraftSim(t11,out11,ones(length(t11),4)*Zc/4,'- 0 0.5 0',1)
PlotAircraftSim(t12,out12,ones(length(t12),4)*Zc/4,'- 1 0.4 0.7',1)
PlotAircraftSim(t13,out13,ones(length(t13),4)*Zc/4,'- 0.9290 0.6940 0.1250',1)
PlotAircraftSim(t14,out14,ones(length(t14),4)*Zc/4,'- 0.8 0.3 0.3',1)


