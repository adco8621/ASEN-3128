function motor_forces = ComputeMotorForces(Zc, Lc, Mc, Nc, R, km)
    % Compute the individual motor forces given the control moments and Z
    % force. Based on page 1-2 of the lab document.
    mom2fs = [-1, -1, -1, -1;
        -R/sqrt(2), -R/sqrt(2), R/sqrt(2), R/sqrt(2); 
        R/sqrt(2), -R/sqrt(2), -R/sqrt(2), R/sqrt(2); 
        km, -km, km, -km];
    allforces = [Zc, Lc, Mc, Nc];
    motor_forces = allforces\mom2fs;

end