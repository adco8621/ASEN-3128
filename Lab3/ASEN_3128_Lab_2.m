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

 %% Problem 1 
 % Steady hovering flight all F must equal mg to balance forces
f1 = .25*m*g;
f2 = .25*m*g;
f3 = .25*m*g;
f4 = .25*m*g;
% Zc equals sum of forces 
Zc  = -f1-f2-f3-f4;
tspan = [0 10];
% State Vector for hoevring steady flight 
x0 = [0 ;0 ;10 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t, out] = ode45(@(t,x) quadrotorODE(t,x,m,radius,km,Ix,Iy,Iz,n,mu,Zc,f1,f2,f3,f4), tspan,x0);
%Trim is all forces equal mg so the quadrotor is in steady flight 


%% Problem 2 
% Methodology for determining trim:
% Began by finding variables that were unchanging or zero
% 1)Used the first set of equations of motion to solve for velocity of the
% body relative to inertial
% 2)Solved the aero forces equation for X,Y,Z
% 3)Substitute into the third set of equations (with p,q,r = 0 & acceleration = 0)
% 4) Solve for angle (phi / theta) and Z_c
g = 9.81; %m/s^2
%Solving equations for 2b
syms Z_c phi
eqns = [0 ==  g*sind(phi) + ((-25*n*cosd(phi))/m), 0 == g*cosd(phi) + ((25*n*sind(phi))/m) + (Z_c/m)];
S = solve(eqns, [Z_c,phi]);

PHI = double(S.phi(2,1));
ZSUBc = double(S.Z_c(2,1));
% Using initial conditions 
f1_2 = ZSUBc/4;
f2_2 = ZSUBc/4;
f3_2 = ZSUBc/4;
f4_2 = ZSUBc/4;
x0_2 = [0 ;0 ;10 ;PHI ;0 ;0 ;0 ;5*cosd(PHI) ;-5*sind(PHI) ;0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t2, out2] = ode45(@(t,x) quadrotorODE(t,x,m,radius,km,Ix,Iy,Iz,n,mu,ZSUBc,f1_2,f2_2,f3_2,f4_2), tspan,x0_2);

%Problem 2c
syms Z_c2 theta
eqns2 = [0 ==  -g*sind(theta) + ((-25*n*cosd(theta))/m), 0 == g*cosd(theta) + ((-25*n*sind(theta))/m) + (Z_c2/m)];
S2 = solve(eqns2, [Z_c2,theta]);
THETA = double(S2.theta(2,1));
ZSUBc2 = double(S2.Z_c2(2,1));
f1_3 = ZSUBc2/4;
f2_3 = ZSUBc2/4;
f3_3 = ZSUBc2/4;
f4_3 = ZSUBc2/4;

x0_3 = [0 ;0 ;10 ;0 ;THETA ;90 ;5*cosd(THETA);0 ;5*sind(THETA);0 ;0 ;0]; % x0 = [x_E ;y_E ;z_E ;phi ;theta ;psi ;u_E ;v_E ;w_E ;p ;q ;r]
[t3, out3] = ode45(@(t,x) quadrotorODE(t,x,m,radius,km,Ix,Iy,Iz,n,mu,ZSUBc2,f1_3,f2_3,f3_3,f4_3), tspan,x0_3);
% The only trim difference maintaining a yaw angle of 90 degrees has is the
% angle which the quadrotor must rotate about to travel in the east
% direction. Because the body frame is now facing the east direction, phi
% must be zero and to translate east the quadrotor must pitch creating a
% negative theta. 
 
%% Plots for part 1b
figure (1)
tiledlayout(3,1)
% X - Position
nexttile
plot(t,out(:,1),'LineWidth',2)
title('x- Position vs Time (1b)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Y - position
nexttile
plot(t,out(:,2),'LineWidth',2)
title('y- Position vs Time (1b)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Z - Position
nexttile
plot(t,out(:,3),'LineWidth',2)
title('z- Position vs Time (1b)')
xlabel('Time (sec)')
ylabel('Distance (m)')

figure (2)
tiledlayout(3,1)
nexttile
plot(t,out(:,4),'LineWidth',2)
title('Phi vs Time (1b)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Y - position
nexttile
plot(t,out(:,5),'LineWidth',2)
title('Theta vs Time (1b)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Z - Position
nexttile
plot(t,out(:,6),'LineWidth',2)
title('Psi vs Time (1b)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')

figure (3)
tiledlayout(3,1)
nexttile
plot(t,out(:,7),'LineWidth',2)
title('x- Velocity vs Time (1b)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Y - position
nexttile
plot(t,out(:,8),'LineWidth',2)
title('y- Velocity vs Time (1b)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Z - Position
nexttile
plot(t,out(:,9),'LineWidth',2)
title('z- Velocity vs Time (1b)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')

figure (4)
tiledlayout(3,1)
nexttile
plot(t,out(:,10),'LineWidth',2)
title('x- Angular Velocity vs Time (1b)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Y - position
nexttile
plot(t,out(:,11),'LineWidth',2)
title('y- Angular Velocity vs Time (1b)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Z - Position
nexttile
plot(t,out(:,12),'LineWidth',2)
title('z- Angular Velocity vs Time (1b)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')

%% Plots for part 2b
figure (5)
tiledlayout(3,1)
% X - Position
nexttile
plot(t2,out2(:,1),'LineWidth',2)
title('x- Position vs Time (2b)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Y - position
nexttile
plot(t2,out2(:,2),'LineWidth',2)
title('y- Position vs Time (2b)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Z - Position
nexttile
plot(t2,out2(:,3),'LineWidth',2)
title('z- Position vs Time (2b)')
xlabel('Time (sec)')
ylabel('Distance (m)')

figure (6)
tiledlayout(3,1)
nexttile
plot(t2,out2(:,4),'LineWidth',2)
title('Phi vs Time (2b)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Y - position
nexttile
plot(t2,out2(:,5),'LineWidth',2)
title('Theta vs Time (2b)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Z - Position
nexttile
plot(t2,out2(:,6),'LineWidth',2)
title('Psi vs Time (2b)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')

figure (7)
tiledlayout(3,1)
nexttile
plot(t2,out2(:,7),'LineWidth',2)
title('x- Velocity vs Time (2b)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Y - position
nexttile
plot(t2,out2(:,8),'LineWidth',2)
title('y- Velocity vs Time (2b)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Z - Position
nexttile
plot(t2,out2(:,9),'LineWidth',2)
title('z- Velocity vs Time (2b)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')

figure (8)
tiledlayout(3,1)
nexttile
plot(t2,out2(:,10),'LineWidth',2)
title('x- Angular Velocity vs Time (2b)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Y - position
nexttile
plot(t2,out2(:,11),'LineWidth',2)
title('y- Angular Velocity vs Time (2b)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Z - Position
nexttile
plot(t2,out2(:,12),'LineWidth',2)
title('z- Angular Velocity vs Time (2b)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')



%% Plots for part 2c
figure (9)
tiledlayout(3,1)
% X - Position
nexttile
plot(t3,out3(:,1),'LineWidth',2)
title('x- Position vs Time (2c)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Y - position
nexttile
plot(t3,out3(:,2),'LineWidth',2)
title('y- Position vs Time (2c)')
xlabel('Time (sec)')
ylabel('Distance (m)')
% Z - Position
nexttile
plot(t3,out3(:,3),'LineWidth',2)
title('z- Position vs Time (2c)')
xlabel('Time (sec)')
ylabel('Distance (m)')

figure (10)
tiledlayout(3,1)
nexttile
plot(t3,out3(:,4),'LineWidth',2)
title('Phi vs Time (2c)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Y - position
nexttile
plot(t3,out3(:,5),'LineWidth',2)
title('Theta vs Time (2c)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')
% Z - Position
nexttile
plot(t3,out3(:,6),'LineWidth',2)
title('Psi vs Time (2c)')
xlabel('Time (sec)')
ylabel('Angle (degrees)')

figure (11)
tiledlayout(3,1)
nexttile
plot(t3,out3(:,7),'LineWidth',2)
title('x- Velocity vs Time (2c)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Y - position
nexttile
plot(t3,out3(:,8),'LineWidth',2)
title('y- Velocity vs Time (2c)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')
% Z - Position
nexttile
plot(t3,out3(:,9),'LineWidth',2)
title('z- Velocity vs Time (2c)')
xlabel('Time (sec)')
ylabel('Velocity (m/s)')

figure (12)
tiledlayout(3,1)
nexttile
plot(t3,out3(:,10),'LineWidth',2)
title('x- Angular Velocity vs Time (2c)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Y - position
nexttile
plot(t3,out3(:,11),'LineWidth',2)
title('y- Angular Velocity vs Time (2c)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
% Z - Position
nexttile
plot(t3,out3(:,12),'LineWidth',2)
title('z- Angular Velocity vs Time (2c)')
xlabel('Time (sec)')
ylabel('Velocity (rad/s)')
