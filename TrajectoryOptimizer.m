clc
clear all
close all

load('robot');
r_P_end=[2;3;0];
amplitut_psi=20*pi/180;
period_psi=2.242;
Init_psi=0*pi/180;
Init_theta=120*pi/180;
Init_phi=0*pi/180;

MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
Init_r_O=[5;0;0]+MatR1z*MatR2y*[-r;0;0];

Init_x_O=Init_r_O(1,1);
Init_y_O=Init_r_O(2,1);
Init_z_O=Init_r_O(3,1);

Init_d_psi=0;
Init_d_theta=0;
Init_d_phi=0;

[Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(1,6));

depend_vel_var=-inv(Mat_a_const(1:3,1:3))*Mat_a_const(1:3,4:6)*[Init_d_psi;Init_d_theta;Init_d_phi];
Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);


initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %

halfcyclecond=1;
if halfcyclecond==1
    t_span=[0:period_psi/300:3*period_psi/4];
else
    t_span=[period_psi/2:period_psi/300:5*period_psi/4];
end
opt_fun=@(amplitut_psi)funhalfcycle(amplitut_psi,period_psi,t_span,initial_cond,halfcyclecond,r_P_end);
opt_amplitut_psi= fminbnd(opt_fun,-pi/6,pi/6);
[objfun_i,vect_t_i,Var_i,vect_te_i,Vare_i,ie_i,min_ie_i]=funhalfcycle(opt_amplitut_psi,period_psi,t_span,initial_cond,halfcyclecond,r_P_end);
[diff_t_ie_perid min_ie_i_vect_te_i]=min(abs(vect_t_i-vect_te_i(min_ie_i,1)));
objfun_tot=[objfun_i];
vect_t_tot=vect_t_i(1:min_ie_i_vect_te_i,1);
Var_tot=Var_i(1:min_ie_i_vect_te_i,:);
if halfcyclecond==1
    period_psi=2*vect_te_i(min_ie_i,1);
else
    period_psi=vect_te_i(min_ie_i,1);
end


for itopt=1:5
        
    halfcyclecond=-1*halfcyclecond;
    if halfcyclecond==1
        t_span=[0 period_psi];
    else
        t_span=[period_psi/2 3*period_psi/2];
    end
    initial_cond=Var_tot(end,:);
    
    opt_fun=@(amplitut_psi)funhalfcycle(amplitut_psi,period_psi,t_span,initial_cond,halfcyclecond,r_P_end);
    opt_amplitut_psi= fminbnd(opt_fun,-pi/6,pi/6);
    [objfun_i,vect_t_i,Var_i,vect_te_i,Vare_i,ie_i,min_ie_i]=funhalfcycle(opt_amplitut_psi,period_psi,t_span,initial_cond,halfcyclecond,r_P_end);
    [diff_t_ie_perid min_ie_i_vect_te_i]=min(abs(vect_t_i-vect_te_i(min_ie_i,1)));   
    objfun_tot=[objfun_tot;objfun_i]
    if halfcyclecond==1
        vect_t_tot=[vect_t_tot;(vect_t_tot(end,1))+vect_t_i(3:min_ie_i_vect_te_i,1)];
    else
        vect_t_tot=[vect_t_tot;(vect_t_tot(end,1)-period_psi/2)+vect_t_i(3:min_ie_i_vect_te_i,1)];
    end
    Var_tot=[Var_tot;Var_i(3:min_ie_i_vect_te_i,:)];
    
    if halfcyclecond==1
        period_psi=2*vect_te_i(min_ie_i,1);
    else
        period_psi=vect_te_i(min_ie_i,1);
    end   
end

scaled_vect_t_tot=[vect_t_tot(1):(vect_t_tot(end)-vect_t_tot(1))/300:vect_t_tot(end)]';
scaled_x_O=interp1(vect_t_tot,Var_tot(:,1),scaled_vect_t_tot);
scaled_y_O=interp1(vect_t_tot,Var_tot(:,2),scaled_vect_t_tot);
scaled_z_O=interp1(vect_t_tot,Var_tot(:,3),scaled_vect_t_tot);
scaled_psi=interp1(vect_t_tot,Var_tot(:,4),scaled_vect_t_tot);
scaled_theta=interp1(vect_t_tot,Var_tot(:,5),scaled_vect_t_tot);
scaled_phi=interp1(vect_t_tot,Var_tot(:,6),scaled_vect_t_tot);
scaled_d_x_O=interp1(vect_t_tot,Var_tot(:,7),scaled_vect_t_tot);
scaled_d_y_O=interp1(vect_t_tot,Var_tot(:,8),scaled_vect_t_tot);
scaled_d_z_O=interp1(vect_t_tot,Var_tot(:,9),scaled_vect_t_tot);
scaled_d_psi=interp1(vect_t_tot,Var_tot(:,10),scaled_vect_t_tot);
scaled_d_theta=interp1(vect_t_tot,Var_tot(:,11),scaled_vect_t_tot);
scaled_d_phi=interp1(vect_t_tot,Var_tot(:,12),scaled_vect_t_tot);

scaled_Var_tot=[scaled_x_O,scaled_y_O,scaled_z_O,scaled_psi,scaled_theta,scaled_phi,scaled_d_x_O,scaled_d_y_O,scaled_d_z_O,scaled_d_psi,scaled_d_theta,scaled_d_phi];

[glob_coor_xyz,glob_coor_A]=fun_coorG(scaled_Var_tot);

figure
plot(scaled_vect_t_tot(:,1),scaled_psi,'b')
hold on
plot(scaled_vect_t_tot(:,1),scaled_phi(:,1),'r')
xlabel('t')
ylabel('rad')
legend('psi','phi')

figure
plot(glob_coor_xyz(:,1),glob_coor_xyz(:,2))
figure
plot(glob_coor_A(:,1),glob_coor_A(:,2))
figure
mat_frame=PlotCone(scaled_Var_tot,glob_coor_xyz);



v_cone = VideoWriter('TrajZigzagCone.avi');
open(v_cone);
writeVideo(v_cone,mat_frame);
close(v_cone);
movie(mat_frame,1,20)

