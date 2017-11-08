syms v_x v_y  u2 omega radius

u1 = 4;
u3 = 0;

radius = 1;

v_x = (r*2/3) * (-u1*cos(0) + u2*cos(pi/3-0) + u3*cos(pi/3+0));
v_y = (r*2/3) * (u1*sin(0) + u2*sin(pi/3-0) - u3*sin(pi/3+0));
omega = r/(3*L) * (u1+u2+u3);
    
eqn1 = sqrt(v_x ^2 + v_y^2)/omega - radius

fprintf ('%.15ld',double(solve (eqn1, u2)));