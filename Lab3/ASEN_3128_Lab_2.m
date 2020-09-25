% Mayhan, Nicolas
% Oliver, John
% Troche, Justin
% Conzet, Addison
% ASEN 3128
% FILENAME 
% Lab 2
% Created 9/11/2020
 clear all 
 close all 
 clc 
 
 %% Constants 
m= 0.068; %Quadrotor mass kg 
radius = 0.060; %Radial distance from CG to propeller m
km = 0.0024; %Control moment coefficient N*m/(N)
Ix = 6.8E-5; %Body x-axis Moment of Inertia kg*m^2
Iy = 9.2E-5; %Body y-axis Moment of Inertia kg*m^2
Iz = 1.35E-4; %Body z-axis Moment of Inertia kg*m^2
n = 1E-3; %Aerodynamic force coefficient N/(m/s)^2
mu = 2E-6; %Aerodynamic moment coefficient N*m/(rad/s)^2
g = 9.81; %m/s^2

%% Problem 3
% Steady hovering flight all F must equal mg to balance forces
f1 = .25*m*g;
f2 = .25*m*g;
f3 = .25*m*g;
f4 = .25*m*g;
% Zc equals sum of forces 
Zc  = -f1-f2-f3-f4;
tspan = [0 10];
% State Vector for hoevring steady flight with 5 degree roll
x0 = [0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t, out] = ode45(@(t,x) quadrotorODE(t,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0); 

% State Vector for hoevring steady flight with 5 degree pitch
x0 = [0 ;0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t1, out1] = ode45(@(t1,x) quadrotorODE(t1,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0);

% State Vector for hoevring steady flight with 5 degree yaw
x0 = [0 ;0 ;0 ;0 ;0 ;5 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t2, out2] = ode45(@(t2,x) quadrotorODE(t2,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0);

% State Vector for hoevring steady flight with 0.1 rad/sec roll rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t3, out3] = ode45(@(t3,x) quadrotorODE(t3,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0);

% State Vector for hoevring steady flight with 0.1 rad/sec pitch rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1) ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t4, out4] = ode45(@(t4,x) quadrotorODE(t4,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0);

% State Vector for hoevring steady flight with 0.1 rad/sec yaw rate
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;rad2deg(0.1)]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t5, out5] = ode45(@(t5,x) quadrotorODE(t5,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0);

%% Plots for part 3

PlotAircraftSim(t,out,ones(length(t),4)*f1,'-b')
hold on
PlotAircraftSim(t1,out1,ones(length(t1),4)*f1,'-r')
hold on
PlotAircraftSim(t2,out2,ones(length(t2),4)*f1,'-k')
hold on
PlotAircraftSim(t3,out3,ones(length(t3),4)*f1,':b')
hold on
PlotAircraftSim(t4,out4,ones(length(t4),4)*f1,':r')
hold on
PlotAircraftSim(t5,out5,ones(length(t5),4)*f1,':k')



