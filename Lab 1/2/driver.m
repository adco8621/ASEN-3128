% Addison Conzet
% ASEN 3128
% driver.m
% Created: 8/28/20

% housekeeping
clear;clc;close all;

%% initial
tspan = [0 5]; %setting integration bounds
x0 = [0 0 0 0 -20 20]; %initial state vector

%given values
rho = 1.2754; % at sea level
Cd = 0.6;
m = 0.03;
A = pi * 0.03^2 / 4;
wind = [0 0 0];
g = 9.81;

% regualr flight
[t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,wind),tspan,x0);

% trimming once it hits the ground
y = find(x(:,3)<=0,2);
x = x(1:y(end),:);

% initial landing position
initial_landing = x(end,1:2);

% plotting 3d trajectory
figure;
plot3(x(:,1),x(:,2),x(:,3))
title('3D trajectory of golf ball thrown 20 m/s east and 20 m/s up')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')


%% wind sensitivity

% alloctaing for different landings (only 1D because only x pos will vary)
landings = zeros(41,1);

% varying wind speed from -20 to 20 m/s
for i = -20:20
    
    % setting wind
    wind = [i 0 0];
    
    % integrating
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,wind),tspan,x0);

    % trimming post landing data
    y = find(x(:,3)<=0,2);
    x = x(1:y(end),:);
    
    % storing landing positions
    landings(i+21,1) = x(end,1);
    
end

% calculating distances from original pos
dist = landings(:) - initial_landing(:,1);

% plot
figure;
plot([-20:20],dist)
title('Landing location sensitivity to wind speed')
xlabel('wind speed in the x direction (north) [m/s]')
ylabel('distance from original landing location [m]')



%% kinetic limits and mass

% finding KE
KEmax = .5 * m * (norm(x0(4:6)))^2;

% varying mass
masses = linspace(0.025,.3,100);

% calculating velocity components wrt new mass and 
Velocities = sqrt(2*KEmax./masses);
components = sqrt((Velocities.^2)/2);

% alloctaing and assigning velocity and distance matrix
Vs = zeros(100,6);
Vs(:,5:6) = [-components', components'];
dist = zeros(100,1);

% resetting wind to 0
wind = [0,0,0];

% same internals as the wind part, just varying initial mass and velocity
for i = 1:100
        
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,masses(i),g,wind),tspan,Vs(i,:));

    y = find(x(:,3)<=0,2);
    x = x(1:y(end),:);
    
    landings(i) = x(end,2);
    
    dist(i) = norm(landings(i));
    
end

% plot
figure;
plot(masses,dist)
title('Change in distance traveled by varying mass while holding initial kinetic energy constant')
xlabel('mass [kg]')
ylabel('distance from launch point [m]')
