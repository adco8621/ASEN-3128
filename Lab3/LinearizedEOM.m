function [x,t] = LinearizedEOM(x0,Tfinal)
% Inputs: x0 = initial state vector
%         Tfinal = final time of simulation
% Ouputs: x = state vector at time t
%         t = time
% Methodology: use linearized quadrotor equations of motion to simulate
% behavior of quadrotor for deviations from steady hover trim

g = 9.81; %[m/s^2] acceleration due to gravity
%state space matrix
A = zeros(12,12);
A(1,7) = 1;
A(2,8) = 1;
A(3,9) = 1;
A(4,10) = 1;
A(5,11) = 1;
A(6,12) = 1;
A(7,5) = -g;
A(8,4) = g;
% create state space system and simulate
sys = ss(A,zeros(12,1),eye(12),0);
[x,t] = initial(sys,x0,Tfinal);
end