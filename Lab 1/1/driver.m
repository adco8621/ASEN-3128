clear;clc;close all;

tspan = [0 5];
y0 = [0.01 0.01 0.01];

[t,y] = ode45(@(t,y) odefun(t,y),tspan,y0);

figure;

subplot(3,1,1)
plot(t,y(:,1))
subplot(3,1,2)
plot(t,y(:,2))
subplot(3,1,3)
plot(t,y(:,3))