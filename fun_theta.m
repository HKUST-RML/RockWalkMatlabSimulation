function f_theta=fun_theta(x,h)
load('robot');
f_theta=sin(x)*r-sin(x)*OH-cos(x)*AH-h;


