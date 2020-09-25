%% problem 2
%part 2
air = [23 5 0]; %set both z components to 0 to find beta
body = [1 0 0];
beta = rad2deg(atan2(norm(cross(air,body)),dot(air,body)))
%part 3
acc = [5 0 -2]';
acc_body = findDCM(rad2deg(0.25),0,rad2deg(-1.9),'321')*acc
%part 4
acc = [0 0 0]';
acc_body = findDCM(rad2deg(0.25),0,rad2deg(-1.9),'321')*acc