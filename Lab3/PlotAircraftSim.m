function [] = PlotAircraftSim(time,aircraft_state_array, control_input_array, col)
%% Plots each of the 12 states versus time for a given input of time and
%% YOUT values fed back from an ODE call

%% X Y Z Position
figure(1)
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
figure(2)
subplot(3,1,1);
plot(time,aircraft_state_array(:,4))
ylabel('Roll [deg]')
title('Euler Angles vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,5))
ylabel('Pitch [deg]')

subplot(3,1,3);
plot(time,aircraft_state_array(:,6))
xlabel('Time')
ylabel('Yaw [deg]')
set(gcf,'position',[720 350 510 330])

%% Velocity versus Time
figure(3)
subplot(3,1,1);
plot(time,aircraft_state_array(:,7))
ylabel('uE [m/s]')
title('Velocity vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,8))
ylabel('vE [m/s]')

subplot(3,1,3);
plot(time,aircraft_state_array(:,9))
xlabel('Time')
ylabel('wE [m/s]')
set(gcf,'position',[200 35 510 340])

%% Angular Velocity versus Time
figure(4)
subplot(3,1,1);
plot(time,aircraft_state_array(:,10))
ylabel('p [rad/s]')
title('Angular Velocity vs Time')

subplot(3,1,2);
plot(time,aircraft_state_array(:,11))
ylabel('q [rad/s]')

subplot(3,1,3);
plot(time,aircraft_state_array(:,12))
xlabel('Time')
ylabel('r [rad/s]')
set(gcf,'position',[720 35 510 340])
end

