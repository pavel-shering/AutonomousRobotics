syms r l x3 u1 u2 u3  x1 x2 x3
dt = 0.1

x = [x1;x2;x3];
u = [u1;u2;u3];

J = [-1/3  cosd(60)/3 cosd(60)/3;
        0 sind(-60)/2 sind(60)/2;
  1/(3*l)     1/(3*l)     1/(3*l)] .* (dt * r);

K = [cos(x3) cos(x3 + pi/2) 0;
     sin(x3) sin(x3 + pi/2) 0;
         0        0         1];

x_new = x + K*J*u
simplify(x_new)
             
             

v_x = (r*2/3) * (-u1*cos(x3) + u2*cos(pi/2+x3) + u3*cos(pi/2+x3));
v_y = (r*2/3) * (u1*sin(x3)  - u2*sin(pi/2+x3) + u3*sin(pi/2+x3));
omega = r/(3*l) * (u1+u2+u3);

n_new = x + [ v_x * dt; v_y * dt; omega * dt]