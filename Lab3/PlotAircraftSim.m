function [] = PlotAircraftSim(time,aircraft_state_array, control_input_array, col,x)
%% Plots each of the 12 states versus time for a given input of time and
% Aircraft_state_array fed out from the ODE call
% control_input_array is the motor forces
% Specify color using col
% x is for if you want to run it again while keeping the previous graphs
% but not plotting over them. enter a number relative to what set of 6
% graphs you arte currently plotting on
global uncontrolled uncontrolled_3D controlled controlled_3D;

    %break up col input into a part specifying line style and another
    %specifying color
    linestyle = extractBefore(col,2);
    colorinfo = strtrim(extractAfter(col,1));
    if length(colorinfo) ~=1
        col = str2double(strsplit(colorinfo));
    else 
        col  = colorinfo;
    end
x=x-1;

%% X Y Z Position
figure(1+6*x)
subplot(7,1,[1,2]);
plot(time,aircraft_state_array(:,1),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('X Position [m]')
title('Position vs Time')

subplot(7,1,[3,4]);
plot(time,aircraft_state_array(:,2),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('Y Position [m]')

subplot(7,1,[5,6]);
plot(time,aircraft_state_array(:,3),'LineStyle',linestyle,'Color', col);
hold on;
xlabel('Time')
ylabel('Z Position [m]')
set(gcf,'position',[200 330 710 450])
if x == 0
    lgd = legend(uncontrolled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',4,'Orientation','horizontal')
else
    lgd = legend(controlled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',3,'Orientation','horizontal')
end

%% Euler Angles
figure(2+6*x)
subplot(7,1,[1,2]);
plot(time,aircraft_state_array(:,4),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('Roll [deg]')
title('Euler Angles vs Time')

subplot(7,1,[3,4]);
plot(time,aircraft_state_array(:,5),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('Pitch [deg]')

subplot(7,1,[5,6]);
plot(time,aircraft_state_array(:,6),'LineStyle',linestyle,'Color', col);
hold on;
xlabel('Time')
ylabel('Yaw [deg]')
set(gcf,'position',[720 330 710 450])

if x == 0
    lgd = legend(uncontrolled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',4,'Orientation','horizontal')
else
    lgd = legend(controlled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',3,'Orientation','horizontal')
end
%% Velocity versus Time
figure(3+6*x)
subplot(7,1,[1,2]);
plot(time,aircraft_state_array(:,7),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('u^E [m/s]')
title('Velocity vs Time')

subplot(7,1,[3,4]);
plot(time,aircraft_state_array(:,8),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('v^E [m/s]')

subplot(7,1,[5,6]);
plot(time,aircraft_state_array(:,9),'LineStyle',linestyle,'Color', col);
hold on;
xlabel('Time')
ylabel('w^E [m/s]')
set(gcf,'position',[200 35 710 450])

if x == 0
    lgd = legend(uncontrolled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',4,'Orientation','horizontal')
else
    lgd = legend(controlled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',3,'Orientation','horizontal')
end
%% Angular Velocity versus Time
figure(4+6*x)
subplot(7,1,[1,2]);
plot(time,aircraft_state_array(:,10),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('p [rad/s]')
title('Angular Velocity vs Time')

subplot(7,1,[3,4]);
plot(time,aircraft_state_array(:,11),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('q [rad/s]')

subplot(7,1,[5,6]);
plot(time,aircraft_state_array(:,12),'LineStyle',linestyle,'Color', col);
hold on;
xlabel('Time')
ylabel('r [rad/s]')
set(gcf,'position',[720 35 710 450])
if x == 0
    lgd = legend(uncontrolled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',4,'Orientation','horizontal')
else
    lgd = legend(controlled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',3,'Orientation','horizontal')
end
%% Control forces
figure(5+6*x)
subplot(9, 1, [1,2]);
plot(time,control_input_array(:,1),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('f1 (N)')
title('Control Forces vs. Time')

subplot(9, 1, [3,4]);
plot(time,control_input_array(:,2),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('f2 (N)')

subplot(9, 1, [5,6]);
plot(time,control_input_array(:,3),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('f3 (N)')

subplot(9, 1, [7,8]);
plot(time,control_input_array(:,4),'LineStyle',linestyle,'Color', col);
hold on;
ylabel('f4 (N)')
xlabel('Time')
if x == 0
    lgd = legend(uncontrolled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',4,'Orientation','horizontal')
    set(gcf,'Position',[400,300,710,450])
else
    lgd = legend(controlled);
    set(lgd,'Position',[0.3 0.05 0.4 0.05],'NumColumns',3,'Orientation','horizontal')
end
%% 3D flight path plot
figure(6+6*x)

plot3(aircraft_state_array(:, 1), aircraft_state_array(:, 2), aircraft_state_array(:, 3),'LineStyle',linestyle,'Color', col,'LineWidth',1.25);
hold on;
grid on;
start = plot3(aircraft_state_array(1, 1), aircraft_state_array(1, 2), aircraft_state_array(1, 3), 'go','MarkerFaceColor','g');
stop = plot3(aircraft_state_array(end, 1), aircraft_state_array(end, 2), aircraft_state_array(end, 3), 'ro');
ylabel('y_E [m]')
xlabel('x_E [m]')
zlabel('z_E [m]')
title('3D Flight Path')
ax = gca;
prev_lines = ax.Children(end:-3:1);
if x == 0
    legend([prev_lines;start;stop],uncontrolled_3D);
else
    legend([prev_lines;start;stop],controlled_3D);
end
end

