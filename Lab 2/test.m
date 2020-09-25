syms x y z phi theta psi u v w p q r
t=0;
state1 = [x y z phi theta psi u v w p q r];
eqs = quadEOM(t, state1, const)

tot = solve(eqs,phi)