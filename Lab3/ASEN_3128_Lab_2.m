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
x0 = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
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

%% Plots for part 3a
figure
tiledlayout(3,1)
% X - Position
nexttile
plot(t,out(:,1),'LineWidth',2)
title('x- Position vs Time (3a)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Y - position
nexttile
plot(t,out(:,2),'LineWidth',2)
title('y- Position vs Time (3a)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Z - Position
nexttile
plot(t,out(:,3),'LineWidth',2)
title('z- Position vs Time (3a)')
xlabel('Time (sec)')
ylabel('Distance (m)')

figure
tiledlayout(3,1)
nexttile
plot(t,out(:,4),'LineWidth',2)
title('Phi vs Time (3a)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Y - position
nexttile
plot(t,out(:,5),'LineWidth',2)
title('Theta vs Time (3a)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Z - Position
nexttile
plot(t,out(:,6),'LineWidth',2)
title('Psi vs Time (3a)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')

figure
tiledlayout(3,1)
nexttile
plot(t,out(:,7),'LineWidth',2)
title('x- Velocity vs Time (3a)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Y - position
nexttile
plot(t,out(:,8),'LineWidth',2)
title('y- Velocity vs Time (3a)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Z - Position
nexttile
plot(t,out(:,9),'LineWidth',2)
title('z- Velocity vs Time (3a)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')

figure
tiledlayout(3,1)
nexttile
plot(t,out(:,10),'LineWidth',2)
title('x- Angular Velocity vs Time (3a)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Y - position
nexttile
plot(t,out(:,11),'LineWidth',2)
title('y- Angular Velocity vs Time (3a)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Z - Position
nexttile
plot(t,out(:,12),'LineWidth',2)
title('z- Angular Velocity vs Time (3a)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')

%% Plots for part 3d
figure
tiledlayout(3,1)
% X - Position
nexttile
plot(t3,out3(:,1),'LineWidth',2)
title('x- Position vs Time (3d)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Y - position
nexttile
plot(t3,out3(:,2),'LineWidth',2)
title('y- Position vs Time (3d)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Z - Position
nexttile
plot(t3,out3(:,3),'LineWidth',2)
title('z- Position vs Time (3d)')
xlabel('Time (sec)')
ylabel('Distance (m)')

figure
tiledlayout(3,1)
nexttile
plot(t3,out3(:,4),'LineWidth',2)
title('Phi vs Time (3d)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Y - position
nexttile
plot(t3,out3(:,5),'LineWidth',2)
title('Theta vs Time (3d)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Z - Position
nexttile
plot(t3,out3(:,6),'LineWidth',2)
title('Psi vs Time (3d)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')

figure
tiledlayout(3,1)
nexttile
plot(t3,out3(:,7),'LineWidth',2)
title('x- Velocity vs Time (3d)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Y - position
nexttile
plot(t3,out3(:,8),'LineWidth',2)
title('y- Velocity vs Time (3d)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Z - Position
nexttile
plot(t3,out3(:,9),'LineWidth',2)
title('z- Velocity vs Time (3d)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')

figure
tiledlayout(3,1)
nexttile
plot(t3,out3(:,10),'LineWidth',2)
title('x- Angular Velocity vs Time (3d)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Y - position
nexttile
plot(t3,out3(:,11),'LineWidth',2)
title('y- Angular Velocity vs Time (3d)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Z - Position
nexttile
plot(t3,out3(:,12),'LineWidth',2)
title('z- Angular Velocity vs Time (3d)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')

