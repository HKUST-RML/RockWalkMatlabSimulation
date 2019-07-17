

clc
clear all
close all

load('robot');
sim_num=3;
save('sim_par','sim_num');
t_end=50;
t_span=0:t_end/30:t_end;



% minvphi=1*pi/180;
% maxvphi=90*pi/180;
% spanvphi=minvphi:(maxvphi-minvphi)/6:maxvphi;
% for itfreqtest=1:length(spanvphi)
%     
%     itfreqtest
%     Init_psi=0*pi/180;
%     Init_theta=120*pi/180;
%     Init_phi=0*pi/180;
%     
%     MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
%     MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
%     Init_r_O=[5;0;0]+MatR1z*MatR2y*[-r;0;0];
%     
%     Init_x_O=Init_r_O(1,1);
%     Init_y_O=Init_r_O(2,1);
%     Init_z_O=Init_r_O(3,1);
%     
%     
%     [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(1,6));
%     Init_d_phi=spanvphi(itfreqtest);
%     depend_vel_var=-inv(Mat_a_const(1:5,1:5))*Mat_a_const(1:5,6)*Init_d_phi;
%     Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);Init_d_psi=depend_vel_var(4,1);Init_d_theta=depend_vel_var(5,1);
%     
%     opts = odeset('Events',@EventsFcn);
%     initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %
%     [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
%     
%     
%     freq(1)=vect_te(length(vect_te))-vect_te(length(vect_te)-2);
%     freq(2)=vect_te(length(vect_te)-2)-vect_te(length(vect_te)-4);
%     freq(3)=vect_te(length(vect_te)-4)-vect_te(length(vect_te)-6);
%     vect_period(itfreqtest)=mean(freq);
%     vect_freq(itfreqtest)=2*pi/mean(freq);
%     vect_max_phi(itfreqtest)=max(Var(:,6));
% end








minvalpha=93*pi/180;
maxvalpha=177*pi/180;

nalphastep=15;
spanvalpha=maxvalpha:(minvalpha-maxvalpha)/nalphastep:minvalpha;
spanvalpha_plot=3:(87-3)/nalphastep:87;
for itfreqtest=1:length(spanvalpha)
    
    itfreqtest
    Init_psi=0*pi/180;
    Init_theta=spanvalpha(itfreqtest);
    Init_phi=0*pi/180;
    
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[5;0;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
    
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(1,6));
    Init_d_phi=45*pi/180;
    depend_vel_var=-inv(Mat_a_const(1:5,1:5))*Mat_a_const(1:5,6)*Init_d_phi;
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);Init_d_psi=depend_vel_var(4,1);Init_d_theta=depend_vel_var(5,1);
    
    opts = odeset('Events',@EventsFcn);
    initial_cond=[Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi]; %
    [vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);
    
    
    freq(1)=vect_te(length(vect_te))-vect_te(length(vect_te)-2);
    freq(2)=vect_te(length(vect_te)-2)-vect_te(length(vect_te)-4);
    freq(3)=vect_te(length(vect_te)-4)-vect_te(length(vect_te)-6);
    vect_period(itfreqtest)=mean(freq);
    vect_freq(itfreqtest)=2*pi/mean(freq);
end

% Data_Simulation_ZeroI=[spanvalpha_plot',vect_period']
% dlmwrite('Data_Simulation_ZeroI.txt',Data_Simulation_ZeroI,'delimiter','\t','precision',12)


Data_Simulation_nzeroI=[spanvalpha_plot',vect_period']
dlmwrite('Data_Simulation_nzeroI.txt',Data_Simulation_nzeroI,'delimiter','\t','precision',12)


% figure
% plot(vect_max_phi*180/pi,vect_freq);
% axis([minvphi*180/pi maxvphi*180/pi 0 2.5])


% figure
% plot(vect_max_phi,vect_period);
% axis([minvphi*180/pi maxvphi*180/pi 0 10])


