

clc
clear all
close all

workdir1=pwd;
conresdir=exist('sim_par.mat');
if conresdir~=0
    delete 'sim_par.mat'
end
conresdir=exist('robot.mat');
if conresdir~=0
    delete 'robot.mat'
end



% AC=sqrt(1.5^2+(0.5+0.5/2)^2);AB=sqrt(1.5^2+(0.5/2)^2);r=0.5;
% AH=sqrt(-(1/16)*(AC^4-2*AC^2*AB^2-8*AC^2*r^2+AB^4-8*AB^2*r^2+16*r^4)/r^2);
% OH=(1/4)*(AC^2-AB^2)/r;
% MatA=[0,2,3;0,-2,3;3,3,0;3,-3,0]';
% MatB_local=[-AH,0,-OH;-AH,0,-OH;0,r*cos(45*pi/180),r*sin(45*pi/180);0,r*cos(135*pi/180),r*sin(135*pi/180)]';
% r_CM_O=[0.0625;0;-0.375];
% Mat_I=[0.95712,0,-0.11044;0,0.97561,0;-0.11044,0,0.60739];
% m=7.85383;
% g=9.81;


AC=sqrt(1.5^2+(0.35+0.35)^2);AB=sqrt(1.5^2+(0.35-0.35)^2);r=0.35;
AH=sqrt(-(1/16)*(AC^4-2*AC^2*AB^2-8*AC^2*r^2+AB^4-8*AB^2*r^2+16*r^4)/r^2);
OH=(1/4)*(AC^2-AB^2)/r;
MatA=[0,2,3;0,-2,3;3,3,0;3,-3,0]';
MatB_local=[-AH,0,-OH;-AH,0,-OH;0,r*cos(45*pi/180),r*sin(45*pi/180);0,r*cos(135*pi/180),r*sin(135*pi/180)]';
r_CM_O=[0.15;0;-0.29];
% Mat_I=[0,-1,0;-1,0,0;0,0,-1]*([0.21,0,0;0,0.2,0.05;0,0.05,0.09]*[0,-1,0;-1,0,0;0,0,-1]');
Mat_I=zeros(3,3);
m=1.04;
g=9.81;



save('robot','AC','AB','r','AH','OH','MatA','MatB_local','r_CM_O','Mat_I','m','g');

%q_1: x_O
%q_2: y_O
%q_3: z_O
%q_4: psi
%q_5: theta
%q_6: phi
%q_7: d_x_O
%q_8: d_y_O
%q_9: d_z_O
%q_10: d_psi
%q_11: d_theta
%q_12: d_phi







sim_num=3;

%------------------------------------------------------
%kinematic fixed apex point
if sim_num==1
    
    
    Init_psi=0*pi/180;
    Init_theta=120*pi/180;
    Init_phi=0*pi/180;
    
    alpha_min=atan(AH/(r-OH));
    if alpha_min<0
        alpha_min=alpha_min+pi;
    end
    min_radi=AB*cos(Init_theta-alpha_min); %for the case of fixed apex point
    
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[min_radi;0;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
     
    
    Init_d_phi=1*pi/6;
    isterminal = 0;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
    save('eventopt','isterminal','direction');
    save('sim_par','sim_num','Init_d_phi');
    opts = odeset('Events',@EventsFcn);
    
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi],zeros(6,1));
    depend_vel_var=-inv(Mat_a_const(1:5,1:5))*Mat_a_const(1:5,6)*Init_d_phi;
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);Init_d_psi=depend_vel_var(4,1);Init_d_theta=depend_vel_var(5,1);
    
    t_end=100;
    t_span=0:t_end/500:t_end;
    initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %
    [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
    [glob_coor_xyz,glob_coor_A]=fun_coorG(Var);
    % This part is for inner/outer circles and trajectory with fixed apex point
    for it=1:length(t_span)
        radi(it)=norm(glob_coor_xyz(it,1:2));
    end
    figure
    plot(glob_coor_xyz(:,1),glob_coor_xyz(:,2),'k')
    hold on
    angspan=0:2*pi/100:2*pi;
    plot(max(radi)*cos(angspan),max(radi)*sin(angspan),'r');
    plot(min(radi)*cos(angspan),min(radi)*sin(angspan),'b');
    
    
    
%----------------------------------------------------------
%kinematic non-fixe apex point
elseif sim_num==2
    
    period_psi=2.3;
    period_theta=2.3;
    period_phi=2.3;   
    omega_psi=2*pi/period_psi;
    omega_theta=2*pi/period_theta;
    omega_phi=2*pi/period_phi;
    max_amplitude_phi=60*pi/180;
    coeff_t_phi=max_amplitude_phi/(5*period_phi);
    amplitut_psi=30*pi/180;
    amplitut_theta=0*pi/180;
    diff_phase_psi=3*pi/2;
    diff_phase_theta=0;
    diff_phase_phi=0;
    save('sim_par','sim_num','period_psi','period_theta','period_phi','amplitut_psi','amplitut_theta','max_amplitude_phi','diff_phase_psi','diff_phase_theta','diff_phase_phi');
    
    
    
    Init_psi=-amplitut_psi;
    Init_theta=150*pi/180;
    Init_phi=0*pi/180;
  
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[5;0;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
  
    
    

    Init_d_psi=amplitut_psi*omega_psi*cos(omega_psi*0+diff_phase_psi);
    Init_d_theta=amplitut_theta*omega_theta*cos(omega_theta*0+diff_phase_theta);
    Init_d_phi=(coeff_t_phi*0)*omega_phi*cos(omega_phi*0+diff_phase_phi);
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(6,1));
    depend_vel_var=-inv(Mat_a_const(1:3,1:3))*Mat_a_const(1:3,4:6)*[Init_d_psi;Init_d_theta;Init_d_phi];
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);
    t_end=12;
    t_span=0:t_end/500:t_end;
    initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %

    isterminal = 0;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
    save('eventopt','isterminal','direction');
    opts = odeset('Events',@EventsFcn);
    [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
    
    
    %--------------------------------------------------
    %dynamic fixed apex point
elseif sim_num==3
    save('sim_par','sim_num');
    Init_psi=0*pi/180;
    Init_theta=120*pi/180;
    Init_phi=30*pi/180;
    
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[5;0;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
    
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(1,6));
    Init_d_phi=0*pi/180;
    depend_vel_var=-inv(Mat_a_const(1:5,1:5))*Mat_a_const(1:5,6)*Init_d_phi;
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);Init_d_psi=depend_vel_var(4,1);Init_d_theta=depend_vel_var(5,1);
    isterminal = 0;  % Halt integration
    direction = 0;   % The zero can be approached from either direction
    save('eventopt','isterminal','direction');
    opts = odeset('Events',@EventsFcn);
    t_end=10;
    t_span=0:t_end/500:t_end;
    initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(initial_cond(1:6,1),initial_cond(7:12,1));
    Mat_a_const*initial_cond(7:12,1)
    
    [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
    
    
    
%---------------------------------------------
%dynamic with Euler-Angles-Velocity-constrained
elseif sim_num==4
    
    
    period_psi=1.3184;
    period_theta=1.3184;
    omega_psi=2*pi/period_psi;
    omega_theta=2*pi/period_theta;
    amplitut_psi=10*pi/180;
    amplitut_theta=5*pi/180;
    diff_phase_psi=0;
    diff_phase_theta=0;
    save('sim_par','sim_num','period_psi','period_theta','amplitut_psi','amplitut_theta','diff_phase_psi','diff_phase_theta');

    
    Init_psi=90*pi/180;
    Init_theta=120*pi/180;
    Init_phi=0*pi/180;
    
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[0;5;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
    
    Init_d_psi=amplitut_psi*omega_psi*cos(omega_psi*0+diff_phase_psi);
    Init_d_theta=amplitut_theta*omega_theta*cos(omega_theta*0+diff_phase_theta);
    Init_d_phi=0;%150*pi/180;
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(1,6));

    depend_vel_var=-inv(Mat_a_const(1:3,1:3))*Mat_a_const(1:3,4:6)*[Init_d_psi;Init_d_theta;Init_d_phi];
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);
    isterminal = 0;  % Halt integration
    direction = 0;   % The zero can be approached from either direction
    save('eventopt','isterminal','direction');
    opts = odeset('Events',@EventsFcn);
    t_end=20;
    t_span=0:t_end/500:t_end;
    initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %
    [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
    
    
    
    
    
%---------------------------------------------
%dynamic with velocity-constrained of apex point
elseif sim_num==5
    
    period_y=2.4;
    omega_y=2*pi/period_y;
    amplitut_y=0.1;
    diff_phase_y=-pi/2;
    save('sim_par','sim_num','period_y','amplitut_y','diff_phase_y');

    fun = @fun_theta; 
    Init_theta = fzero(fun,150*pi/180,[],0.7);
    if Init_theta<0
        Init_theta=2*pi+Init_theta;
    end
    Init_psi=asin(-amplitut_y/(-r*cos(Init_theta)+OH*cos(Init_theta)-AH*sin(Init_theta)));
    Init_phi=0*pi/180;
    
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[5;0;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
    
 
    Init_d_y=amplitut_y*omega_y*cos(omega_y*0+diff_phase_y);   
    Init_d_phi=0*pi/180;
    
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(1,6));

    depend_vel_var=-inv(Mat_a_const(1:5,1:5))*(Mat_a_const(1:5,6)*Init_d_phi+[0;0;0;Init_d_y;0]);
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);Init_d_psi=depend_vel_var(4,1);Init_d_theta=depend_vel_var(5,1);
    isterminal = 0;  % Halt integration
    direction = 0;   % The zero can be approached from either direction
    save('eventopt','isterminal','direction');
    opts = odeset('Events',@EventsFcn);
    t_end=100;
    t_span=0:t_end/500:t_end;
    initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %

    
    [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
    
    
    
end



[glob_coor_xyz,glob_coor_A]=fun_coorG(Var);









% % This part is to plot A_theta*sin(omega_theta*t)
% figure
% plot(vect_t,Var(:,6),'-b')
% freq(1)=vect_te(length(vect_te))-vect_te(length(vect_te)-2);
% freq(2)=vect_te(length(vect_te)-2)-vect_te(length(vect_te)-4);
% freq(3)=vect_te(length(vect_te)-4)-vect_te(length(vect_te)-6);
% vect_freq=2*pi/mean(freq);
% vect_max_phi=max(Var(:,6));
% hold on
% plot(vect_t,vect_max_phi*sin(vect_freq*vect_t),'r')



% %%This part is to plot the trajectory and the points with phi=0
% figure
% plot(glob_coor_xyz(:,1),glob_coor_xyz(:,2),'k');
% xlabel('x(m)')
% ylabel('y(m)')
% axis([4 6 -1 1])
% hold on
% [glob_coor_xyz_e,glob_coor_A_e]=fun_coorG(Vare);
% for itie=1:length(ie)
%     plot(glob_coor_xyz_e(itie,1),glob_coor_xyz_e(itie,2),'bo');
% end
% plot(glob_coor_xyz_e(:,1),glob_coor_xyz_e(:,2),'b-.')
%


figure
plot(vect_t(:,1),Var(:,6),'b')

figure
plot(vect_t(:,1),Var(:,4),'b')
hold on
plot(vect_t(:,1),Var(:,5),'r')
plot(vect_t(:,1),Var(:,6),'g')
axis([0 20 -1.5 3])
% xlabel('t')
% ylabel('rad')
% legend('\psi','\theta','\phi')

figure
plot(glob_coor_xyz(1:175,1),glob_coor_xyz(1:175,2),'b');
hold on
plot(glob_coor_xyz(176:349,1),glob_coor_xyz(176:349,2),'r');



figure
plot(glob_coor_A(:,1),glob_coor_A(:,2))
figure
mat_frame=PlotCone(Var,glob_coor_xyz);



v_cone = VideoWriter('TrajZigzagCone.avi');
open(v_cone);
writeVideo(v_cone,mat_frame);
close(v_cone);
movie(mat_frame,1,20)


