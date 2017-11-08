%% linearizing g(x_t-1, u_t)
syms x1 x2 x3 u1 u2 u3 

% % local frame velocities
% lv_x = (r/3) * (u1*cos(pi) + u2*cos(-pi/3) + u3*cos(pi/3));
% lv_y = (r/2) *  (u2*sin(-pi/3) + u3*sin(pi/3));
% omega = r/(3*L) * (u1 + u2 + u3);
% 
% local_m = [lv_x lv_y omega]'*dt; 
% rotation_m = [cos(x3) -sin(x3) 0;
%               sin(x3)  cos(x3) 0;
%                 0         0    1];
%             
% global_m = rotation_m * local_m;
% 
% % global fram velocities 
% % v_x = lv_x * cos(x3) + lv_y * cos(x3 + pi/2);
% % v_y = lv_x * sin(x3) + lv_y * sin(x3 + pi/2);
% 
x_arr = [x1;x2;x3];
% 
% % g = x_arr + [ v_x * dt; v_y * dt; omega * dt];
% g = x_arr + global_m;
% 
% G = jacobian(g, [x1; x2; x3]);

v_x = (r*2/3) * (-u1*cos(x3) + u2*cos(pi/3-x3) + u3*cos(pi/3+x3));
v_y = (r*2/3) * (u1*sin(x3) + u2*sin(pi/3-x3) - u3*sin(pi/3+x3));
omega = r/(3*L) * (u1+u2+u3);

g = x_arr + [ v_x * dt; v_y * dt; omega * dt]

G = jacobian(g, [x1; x2; x3])