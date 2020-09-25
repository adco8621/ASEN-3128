function dy = odefun(t,y)

    dy = zeros(3,1);
    dy(1) = y(1) + 2*y(2) + y(3);
    dy(2) = y(1) - 5*(3);
    dy(3) = y(1) * y(2) - (y(2))^2 + 3 * (y(3))^3;
    
end