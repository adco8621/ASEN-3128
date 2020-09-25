clc;clear;close all;

const.m  = 0.068;       % total mass [kg]
const.r  = 0.060;       % dist from CG to prop [m]
const.km = 0.0024;      % control moment coeff [Nm/N]
const.Ix = 6.8*10^(-5); % MOI [kg*m^2]
const.Iy = 9.2*10^(-5);
const.Iz = 1.35*10^(-4);
const.v  = 1*10^(-3);   % force coeff [N/(m/s)^2]
const.mu = 2*10^(-6);   % moment coeff [N/(rad/s)^2]
const.g  = 9.81;        % m/s^2



state0 = [0 0 0 0 0 0 0 0 0 0 0 0];

tspan = [0 5];

[t,state] = ode45(@(t,state) quadEOM(t,state,const),tspan,state0);

% trying to find equations for 5m/s east
syms x y z phi theta psi u v w p q r
state1 = [x y z phi theta psi u v w p q r];
quadEOM(t, state1, const)