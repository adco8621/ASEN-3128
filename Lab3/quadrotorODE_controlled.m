% Mayhan, Nicolas
% Oliver, John
% Troche, Justin
% Conzet, Addison
% ASEN 3128
% quadrotorODE.m 
% Lab 2
% Problem 1
% Created 9/11/2020
function xdot = quadrotorODE_controlled(t,x,m,Ix,Iy,Iz,n,mu)
%
% Inputs: Constants, Forces, state vector, control forces and moments
%
% Outputs: Derivative of state vector or the results of the equations of
% motion
%
% Methodology: 
%Input state vector and solve 12 equations of motion using the ODE45
%function to integrate through each equation numerically 
%[x_E,y_E,z_E,phi,theta,psi,u_E,v_E,w_E,p,q,r] = x(1:12);
x_E = x(1); y_E = x(2); z_E = x(3); phi = x(4); theta = x(5); psi = x(6); u_E = x(7); v_E = x(8); w_E = x(9); p = x(10); q = x(11); r = x(12); 
g = 9.81; %Gravity m/s^2
aeroMoment = -mu*norm([p;q;r])*[p;q;r]; % Calculates Aero moments 
V_a = norm([u_E;v_E;w_E]);
aeroForces = -n*V_a*[u_E;v_E;w_E]; % Calculates Aero forces 

%Control moments 
Z_c = -m*g;
aeroMomentControl = -0.004*[p;q;r];
%Transformation matrix also in first set of equations of motion 
transMat = [cosd(theta)*cosd(psi) sind(phi)*sind(theta)*cosd(psi)-cosd(phi)*sind(psi) cosd(phi)*sind(theta)*cosd(psi)+sind(phi)*sind(psi);cosd(theta)*sind(psi) sind(phi)*sind(theta)*sind(psi)+cosd(phi)*cosd(psi) cosd(phi)*sind(theta)*sind(psi)-sind(phi)*cosd(psi);-sind(theta) sind(phi)*cosd(theta) cosd(phi)*cosd(theta)];

%All 12 equations of motion below 
inertial_velocity = transMat * [u_E;v_E;w_E];
angle_dot = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta); 0 cosd(phi) -sind(phi);0 sind(phi)*(1/cosd(theta)) cosd(phi)*(1/cosd(theta))] * [p;q;r];
inertial_acceleration = [r*v_E-q*w_E; p*w_E-r*u_E; q*u_E-p*v_E] + g*[-sind(theta);cosd(theta)*sind(phi);cosd(theta)*cosd(phi)] + (1/m)*aeroForces + (1/m)*[0;0;Z_c];
angular_acceleration = [((Iy - Iz)/Ix)*q*r; ((Iz - Ix)/Iy)*q*r; ((Ix - Iy)/Iz)*q*r] + [(1/Ix)*aeroMoment(1); (1/Iy)*aeroMoment(2); (1/Iz)*aeroMoment(3)] + [(1/Ix)*aeroMomentControl(1); (1/Iy)*aeroMomentControl(2); (1/Iz)*aeroMomentControl(3)];

force_change = [0;-0.004*angular_acceleration];

%Setting the results of the equations of motion equal to the function
%output 
xdot = [inertial_velocity;angle_dot;inertial_acceleration;angular_acceleration;force_change];

end