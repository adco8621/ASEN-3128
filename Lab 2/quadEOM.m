function rates = quadEOM(t,state,const)

    %rates = zeros(12,1);
    rates = sym(zeros(12,1));
    Zc = -const.m*const.g;
    Lc = 0;
    Mc = 0;
    Nc = 0;
    
    phi = state(4);
    theta = state(5);
    psi = state(6);
    u = state(7);
    v = state(8);
    w = state(9);
    p = state(10);
    q = state(11);
    r = state(12);
    
    
    XYZ = (-const.v*norm([u,v,w]))*[u,v,w];
    LMN = (-const.mu*norm([p,q,r]))*[p,q,r];
    X = XYZ(1);
    Y = XYZ(2);
    Z = XYZ(3);
    L = LMN(1);
    M = LMN(2);
    N = LMN(3);
    
    rates(1:3) = findDCM(phi,theta,psi,'321')*([phi; theta; psi]);
    
    rates(4:6) = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta);...
                  0    cosd(phi)             -sind(phi);...
                  0 sind(phi)*secd(theta) cosd(phi)*secd(theta)]...
                  * [p;q;r];
              
    rates(7:9) = [r*v-q*w; p*w-r*u; q*u-p*v]...
        +const.g*[-sind(theta); cosd(theta)*sind(phi); cosd(theta)*cosd(phi)]...
        +1/const.m*[X;Y;Z]...
        +1/const.m*[0;0;Zc];
    
    rates(10:12) = [(const.Iy-const.Iz)*q*r/const.Ix;...
                    (const.Iz-const.Ix)*q*r/const.Iy;...
                    (const.Ix-const.Iy)*q*r/const.Iz;]...
                 + [(L+Lc)/const.Ix; (M+Mc)/const.Iy; (N+Nc)/const.Iz];


end

