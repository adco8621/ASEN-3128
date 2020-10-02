function [] = PlotAircraftSim(time,aircraft_state_array, control_input_array, col,x)
%% Plots each of the 12 states versus time for a given input of time and
% Aircraft_state_array fed out from the ODE call
% control_input_array is the motor forces
% Specify color using col
% x is for if you want to run it again while keeping the previous graphs
% but not plotting over them. enter a number relative to what set of 6
% graphs you arte currently plotting on


%% X Y Z Position
figure(1*x)
subplot(3,1,1);
plot(time,aircraft_state_array(:,1), col);
hold on;
ylabel('X Position')
title('Position vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,2), col);
hold on;
ylabel('Y Position')

subplot(3,1,3);
plot(time,aircraft_state_array(:,3), col);
hold on;
xlabel('Time')
ylabel('Z Position')
set(gcf,'position',[200 350 510 330])

%% Euler Angles
figure(2*x)
subplot(3,1,1);
plot(time,aircraft_state_array(:,4), col);
hold on;
ylabel('Roll [deg]')
title('Euler Angles vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,5), col);
hold on;
ylabel('Pitch [deg]')

subplot(3,1,3);
plot(time,aircraft_state_array(:,6), col);
hold on;
xlabel('Time')
ylabel('Yaw [deg]')
set(gcf,'position',[720 350 510 330])

%% Velocity versus Time
figure(3*x)
subplot(3,1,1);
plot(time,aircraft_state_array(:,7), col);
hold on;
ylabel('uE [m/s]')
title('Velocity vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,8), col);
hold on;
ylabel('vE [m/s]')

subplot(3,1,3);
plot(time,aircraft_state_array(:,9), col);
hold on;
xlabel('Time')
ylabel('wE [m/s]')
set(gcf,'position',[200 35 510 340])

%% Angular Velocity versus Time
figure(4*x)
subplot(3,1,1);
plot(time,aircraft_state_array(:,10), col);
hold on;
ylabel('p [rad/s]')
title('Angular Velocity vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,11), col);
hold on;
ylabel('q [rad/s]')

subplot(3,1,3);
plot(time,aircraft_state_array(:,12), col);
hold on;
xlabel('Time')
ylabel('r [rad/s]')
set(gcf,'position',[720 35 510 340])
%% Control forces
figure(5*x)
subplot(4, 1, 1);
plot(time,control_input_array(:,1), col);
hold on;
ylabel('f1 (N)')
title('Control Forces vs. Time')

subplot(4, 1, 2);
plot(time,control_input_array(:,2), col);
hold on;
ylabel('f2 (N)')

subplot(4, 1, 3);
plot(time,control_input_array(:,3), col);
hold on;
ylabel('f3 (N)')

subplot(4, 1, 4);
plot(time,control_input_array(:,4), col);
hold on;
ylabel('f4 (N)')
xlabel('Time')
%% 3D flight path plot
figure(6*x)
plot3(aircraft_state_array(:, 1), aircraft_state_array(:, 2), aircraft_state_array(:, 3), col);
hold on;
grid on;
plot3(aircraft_state_array(1, 1), aircraft_state_array(1, 2), aircraft_state_array(1, 3), 'go');
plot3(aircraft_state_array(end, 1), aircraft_state_array(end, 2), aircraft_state_array(end, 3), 'ro');
title('Flight Path');
end

