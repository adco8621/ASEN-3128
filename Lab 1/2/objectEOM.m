% Addison Conzet
% ASEN 3128
% objectEOM.m
% Created: 8/28/20

function dots = objectEOM(t,x,rho,Cd,A,m,g,wind)
%
% Inputs:   t    = time (necessary for ode45) [s]
%           x    = state vector 
%                = [x-coord [m], y-coord [m], z-coord [m], x-velocity [m/s],
%                   y-velocity [m/s], z-velocity [m/s]]
%           rho  = air density [kg/m^3]
%           Cd   = drag coefficient [unitless]
%           A    = cross sectional area [m^2]
%           m    = mass [kg]
%           g    = gravitational acceleration [m/s^2]
%           wind = wind velocity vector
%                = [x-component [m/s], y-component [m/s], z-component [m/s]]
%
% Outputs:  dots = [xdot, ydot, zdot, xdotdot, ydotdot, zdotdot]
%           xdot, ydot, zdot          = position derivatives [m/s]
%           xdotdot, ydotdot, zdotdot = velocity derivatives [m/s^2]
%
% Methodology: 3D EOM factoring in wind and drag



    dots = zeros(6,1); %allocating
    
    % assigning state vector values to variables
    xdot = x(4);
    ydot = x(5);
    zdot = x(6);
    
    % finding individual drag components
    dragx = rho * xdot^2 * Cd * A / 2;
    dragy = rho * ydot^2 * Cd * A / 2;
    dragz = rho * zdot^2 * Cd * A / 2;
    
    % adjusting negative drags
    if xdot < 0
        dragx = -dragx;
    end
    if ydot < 0
        dragy = -dragy;
    end
    if zdot < 0
        dragz = -dragz;
    end
    
    % EOM
    xdotdot = -dragx/m + wind(1);
    ydotdot = -dragy/m + wind(2);
    zdotdot = -dragz/m - g + wind(3);
    
    % condensing outputs
    dots = [xdot ydot zdot xdotdot ydotdot zdotdot]';
end