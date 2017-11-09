%% linearizing g(x_t-1, u_t)
syms x1 x2 x3 u1 u2 u3 

x_arr = [x1;x2;x3];

v_x = (r*2/3) * (-u1*cos(x3) + u2*cos(pi/3-x3) + u3*cos(pi/3+x3));
v_y = (r*2/3) * (u1*sin(x3) + u2*sin(pi/3-x3) - u3*sin(pi/3+x3));
omega = r/(3*L) * (u1+u2+u3);

g = x_arr + [ v_x * dt; v_y * dt; omega * dt];

G = jacobian(g, [x1; x2; x3]);