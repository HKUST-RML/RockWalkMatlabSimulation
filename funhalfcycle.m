

function [objfun,vect_t,Var,vect_te,Vare,ie,min_ie]=funhalfcycle(amplitut_psi,period_psi,t_span,initial_cond,halfcyclecond,r_P_end)
load('robot');
sim_num=6;
save('sim_par','sim_num','period_psi','amplitut_psi','halfcyclecond');
isterminal = 0;  % Halt integration
direction = 0;   % The zero can be approached from either direction
save('eventopt','isterminal','direction');
opts = odeset('Events',@EventsFcn);

[vect_t,Var,vect_te,Vare,ie]=ode45(@SteadyStateFunction,t_span,initial_cond,opts);

nzero=size(vect_te,2);
if nzero==0
    nzero=1
    ie=1;
    if halfcyclecond==1
        vect_te=[period_psi/2];
        [valmint nrowmint]=min(abs(vect_t-vect_te));
        Vare=Var(nrowmint,:);
    else
        vect_te=[period_psi];
        [valmint nrowmint]=min(abs(vect_t-vect_te));
        Vare=Var(nrowmint,:);
    end
end

if halfcyclecond==1
    [minval min_ie]=min(abs((period_psi/2)*ones(nzero,1)-vect_te));
else
    [minval min_ie]=min(abs((period_psi)*ones(nzero,1)-vect_te));
end


MatR1z = [cos(Vare(min_ie,4)) -sin(Vare(min_ie,4)) 0; sin(Vare(min_ie,4)) cos(Vare(min_ie,4)) 0; 0 0 1;];
MatR2y = [cos(Vare(min_ie,5)) 0 sin(Vare(min_ie,5)); 0 1 0; -sin(Vare(min_ie,5)) 0 cos(Vare(min_ie,5));];
r_P=[Vare(min_ie,1);Vare(min_ie,2);Vare(min_ie,3)]+MatR1z*MatR2y*[r;0;0];
objfun=(r_P(1)-r_P_end(1))^2+(r_P(2)-r_P_end(2))^2;





