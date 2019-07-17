
function vect_d_q=SteadyStateFunction(vect_t,initial_cond)


%M*dd_q=CG;
%[a]*d_q=-[b]

load('robot');
load('sim_par')

%------------------------------------------------------------
% %kinematic with fixed apex point
if sim_num==1
    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
    depend_vel_var=-inv(Mat_a_const(1:5,1:5))*Mat_a_const(1:5,6)*Init_d_phi;
    vect_d_q(1:5)=depend_vel_var(1:5);
    vect_d_q(6)=Init_d_phi;
    
    
    
    
    %-----------------------------------------------------------------
    %kinematic with non-fixed apex point
elseif sim_num==2
    
    
    
    
    omega_psi=2*pi/period_psi;
    omega_theta=2*pi/period_theta;
    omega_phi=2*pi/period_phi;
    coeff_t_phi=max_amplitude_phi/(5*period_phi);
    

    if vect_t<0*period_phi
        Dphi=(coeff_t_phi*vect_t)*omega_phi*cos(omega_phi*vect_t+diff_phase_phi);
    else
        Dphi=max_amplitude_phi*omega_phi*cos(omega_phi*vect_t+diff_phase_phi);
    end
    
    Dpsi=amplitut_psi*omega_psi*cos(omega_psi*vect_t+diff_phase_psi);
    Dtheta= amplitut_theta*omega_theta*cos(omega_theta*vect_t+diff_phase_theta);
    
    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
    depend_vel_var=-inv(Mat_a_const(1:3,1:3))*Mat_a_const(1:3,4:6)*[Dpsi;Dtheta;Dphi];
    vect_d_q(1:3)=depend_vel_var(1:3);
    vect_d_q(4:6)=[Dpsi;Dtheta;Dphi];
    
    %-----------------------------------------------------------------
    %dynamic with fixed apex point
    
elseif sim_num==3
    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    val_alpha=20;
    
    [Mat_M,Vect_CG]=dynamiceqpar(vect_q(1:6,1),vect_q(7:12,1));Vect_CG=-Vect_CG;
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
     vect_epsil=Mat_a_const(1:5,:)*vect_q(7:12,1);
    d_Ad_q=Mat_d_a_const(1:5,:)*vect_q(7:12,1);
     d_Ad_q=d_Ad_q+2*val_alpha*vect_epsil;
    
    Mat_M_A=[Mat_M,-Mat_a_const(1:5,:)';-Mat_a_const(1:5,:),zeros(5,5)];
    vect_dd_q_lambda=inv(Mat_M_A)*[Vect_CG;d_Ad_q];
    
    vect_d_q(1:6,1)=vect_q(7:12,1);
    vect_d_q(7:12,1)=vect_dd_q_lambda(1:6,1);
    
    
    
    %-----------------------------------------------------------------
    %dynamic with Euler-Angles-Velocity-constrained
    
elseif sim_num==4
    

    
    omega_psi=2*pi/period_psi;
    omega_theta=2*pi/period_theta;
    
    
%     d_psi=-amplitut_psi*omega_psi*sin(omega_psi*vect_t)+(amplitut_psi/3)*omega_psi*sin(3*omega_psi*vect_t);
%     d_theta=0;%-amplitut_theta*omega_theta*sin(omega_theta*vect_t)+(amplitut_theta/3)*omega_theta*sin(3*omega_theta*vect_t);
%     dd_psi=-amplitut_psi*omega_psi^2*cos(omega_psi*vect_t)+amplitut_psi*omega_psi^2*cos(3*omega_psi*vect_t);
%     dd_theta=0;%-amplitut_theta*omega_theta^2*cos(omega_theta*vect_t)+amplitut_theta*omega_theta^2*cos(3*omega_theta*vect_t);
    
    d_psi=amplitut_psi*omega_psi*cos(omega_psi*vect_t+diff_phase_psi);
    d_theta=amplitut_theta*omega_theta*cos(omega_theta*vect_t+diff_phase_theta);
    dd_psi=-amplitut_psi*omega_psi^2*sin(omega_psi*vect_t+diff_phase_psi);
    dd_theta=-amplitut_theta*omega_theta^2*sin(omega_theta*vect_t+diff_phase_theta);
    
    
    
    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    val_alpha=20;
    
    [Mat_M,Vect_CG]=dynamiceqpar(vect_q(1:6,1),vect_q(7:12,1));Vect_CG=-Vect_CG;
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
    Mat_a_const(6,:)=[];Mat_d_a_const(6,:)=[];
    Mat_a_const(4:5,:)=[0,0,0,1,0,0;0,0,0,0,1,0];Mat_d_a_const(4:5,:)=zeros(2,6);
    Mat_b_const=-[0;0;0;d_psi;d_theta];Mat_d_b_const=-[0;0;0;dd_psi;dd_theta];
    Mat_Q=fun_Mat_Q_Force(vect_q(1:6,1));
    % vect_epsil=Mat_a_const(1:5,:)*vect_q(7:12,1)+Mat_b_const;
    d_Ad_qPd_b=Mat_d_a_const(1:5,:)*vect_q(7:12,1)+Mat_d_b_const;
    % d_Ad_qPd_b=d_Ad_qPd_b+2*val_alpha*vect_epsil;
    
    Mat_M_A=[Mat_M,-Mat_a_const(1:5,:)';-Mat_a_const(1:5,:),zeros(5,5)];
    vect_dd_q_lambda=inv(Mat_M_A)*[Vect_CG;d_Ad_qPd_b];
    
    vect_d_q(1:6,1)=vect_q(7:12,1);
    vect_d_q(7:12,1)=vect_dd_q_lambda(1:6,1);
    
    %-----------------------------------------------------------------
    %dynamic with apex-point-Velocity-constrained
    
elseif sim_num==5
 
    omega_y=2*pi/period_y;

    
    y=amplitut_y*sin(omega_y*vect_t-pi/2);
    d_y=amplitut_y*omega_y*cos(omega_y*vect_t+diff_phase_y);
    dd_y=-amplitut_y*omega_y^2*sin(omega_y*vect_t+diff_phase_y);
    
    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    val_alpha=20;
    
    [Mat_M,Vect_CG]=dynamiceqpar(vect_q(1:6,1),vect_q(7:12,1));Vect_CG=-Vect_CG;
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
    Mat_a_const(4,:)=[];Mat_d_a_const(4,:)=[];
    Mat_b_const=-[0;0;0;d_y;0];Mat_d_b_const=-[0;0;0;dd_y;0];
    vect_epsil=Mat_a_const(1:5,:)*vect_q(7:12,1)+Mat_b_const;
    d_Ad_qPd_b=Mat_d_a_const(1:5,:)*vect_q(7:12,1)+Mat_d_b_const;
    d_Ad_qPd_b=d_Ad_qPd_b+2*val_alpha*vect_epsil;
    
    Mat_M_A=[Mat_M,[-Mat_a_const(1:3,:)',-Mat_Q(:,1:2)];-Mat_a_const(1:5,:),zeros(5,5)];
%   Mat_M_A=[Mat_M,-Mat_a_const(1:5,:)';-Mat_a_const(1:5,:),zeros(5,5)];
    vect_dd_q_lambda=inv(Mat_M_A)*[Vect_CG;d_Ad_qPd_b];
    
    vect_d_q(1:6,1)=vect_q(7:12,1);
    vect_d_q(7:12,1)=vect_dd_q_lambda(1:6,1);
    
    %-----------------------------------------------------------------
    %dynamic with Euler-Angles-Velocity-constrained for tajectory planning
    
elseif sim_num==6

    omega_psi=2*pi/period_psi;
    if halfcyclecond==1  

        if vect_t<period_psi/2
            d_psi=-amplitut_psi*omega_psi*sin(omega_psi*vect_t)+(amplitut_psi/3)*omega_psi*sin(3*omega_psi*vect_t);
            d_theta=0;
            dd_psi=-amplitut_psi*omega_psi^2*cos(omega_psi*vect_t)+amplitut_psi*omega_psi^2*cos(3*omega_psi*vect_t);
            dd_theta=0;
        else
            d_psi=0;
            d_theta=0;
            dd_psi=0;
            dd_theta=0;
        end
    else
        if vect_t<period_psi
            d_psi=-amplitut_psi*omega_psi*sin(omega_psi*vect_t)+(amplitut_psi/3)*omega_psi*sin(3*omega_psi*vect_t);
            d_theta=0;
            dd_psi=-amplitut_psi*omega_psi^2*cos(omega_psi*vect_t)+amplitut_psi*omega_psi^2*cos(3*omega_psi*vect_t);
            dd_theta=0;
        else
            d_psi=0;
            d_theta=0;
            dd_psi=0;
            dd_theta=0;
        end
    end



    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    val_alpha=20;
    
    [Mat_M,Vect_CG]=dynamiceqpar(vect_q(1:6,1),vect_q(7:12,1));Vect_CG=-Vect_CG;
    
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
    Mat_a_const(6,:)=[];Mat_d_a_const(6,:)=[];
    Mat_a_const(4:5,:)=[0,0,0,1,0,0;0,0,0,0,1,0];Mat_d_a_const(4:5,:)=zeros(2,6);
    Mat_b_const=-[0;0;0;d_psi;d_theta];Mat_d_b_const=-[0;0;0;dd_psi;dd_theta];
    Mat_Q=fun_Mat_Q_Force(vect_q(1:6,1));
    % vect_epsil=Mat_a_const(1:5,:)*vect_q(7:12,1)+Mat_b_const;
    d_Ad_qPd_b=Mat_d_a_const(1:5,:)*vect_q(7:12,1)+Mat_d_b_const;
    % d_Ad_qPd_b=d_Ad_qPd_b+2*val_alpha*vect_epsil;
    
    Mat_M_A=[Mat_M,[-Mat_a_const(1:3,:)',-Mat_Q(:,1:2)];-Mat_a_const(1:5,:),zeros(5,5)];
%     Mat_M_A=[Mat_M,-Mat_a_const(1:5,:)';-Mat_a_const(1:5,:),zeros(5,5)];
    vect_dd_q_lambda=inv(Mat_M_A)*[Vect_CG;d_Ad_qPd_b];
    vect_d_q(1:6,1)=vect_q(7:12,1);
    vect_d_q(7:12,1)=vect_dd_q_lambda(1:6,1);
       
end









