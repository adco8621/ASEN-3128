function DCM=findDCM(x,y,z,order)

Rx=[1    0           0;
    0    cosd(x)   sind(x);
    0    -sind(x)  cosd(x)];

Ry=[cosd(y)      0       -sind(y);
    0            1       0;
    sind(y)      0       cosd(y)];

Rz=[cosd(z)        sind(z)        0;
    -sind(z)       cosd(z)        0;
    0              0              1];

order = fliplr(order);

if order(1) == '1'
    DCM = Rx;
elseif order(1) == '2'
    DCM = Ry;
else
    DCM = Rz;
end

for i = 2:3
    
    if order(i) == '1'
        DCM = DCM*Rx;
    elseif order(i) == '2'
        DCM = DCM*Ry;
    else
        DCM = DCM*Rz;
    end
    
end



