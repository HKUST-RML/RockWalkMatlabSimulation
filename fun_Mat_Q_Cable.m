
function [Mat_Q]=fun_Mat_Q_Cable(q)

load('robot');
ncable=size(MatA,2);

for itcable=1:ncable
    
    f11=MatA(1,itcable);
    f12=MatA(2,itcable);
    f13=MatA(3,itcable);
    
    k11=MatB_local(1,itcable);
    k12=MatB_local(2,itcable);
    k13=MatB_local(3,itcable);
    
    rho_j1 = f11 - (cos(q(4)) * cos(q(5)) * cos(q(6)) - sin(q(4)) * sin(q(6))) * k11 - (-cos(q(4)) * cos(q(5)) * sin(q(6)) - sin(q(4)) * cos(q(6))) * k12 - cos(q(4)) * sin(q(5)) * k13 - q(1);
    rho_j2 = f12 - (sin(q(4)) * cos(q(5)) * cos(q(6)) + cos(q(4)) * sin(q(6))) * k11 - (-sin(q(4)) * cos(q(5)) * sin(q(6)) + cos(q(4)) * cos(q(6))) * k12 - sin(q(4)) * sin(q(5)) * k13 - q(2);
    rho_j3 = f13 + sin(q(5)) * cos(q(6)) * k11 - sin(q(5)) * sin(q(6)) * k12 - cos(q(5)) * k13 - q(3);
    rho_j=sqrt(rho_j1^2+rho_j2^2+rho_j3^2);
       
    Q1j = -cos(q(4)) * (cos(q(6)) * k11 - sin(q(6)) * k12) * cos(q(5)) + cos(q(6)) * sin(q(4)) * k12 - cos(q(4)) * sin(q(5)) * k13 + sin(q(6)) * sin(q(4)) * k11 - q(1) + f11;
    Q2j = -sin(q(4)) * (cos(q(6)) * k11 - sin(q(6)) * k12) * cos(q(5)) - cos(q(6)) * cos(q(4)) * k12 - sin(q(6)) * cos(q(4)) * k11 - sin(q(4)) * sin(q(5)) * k13 - q(2) + f12;
    Q3j = f13 + sin(q(5)) * cos(q(6)) * k11 - sin(q(5)) * sin(q(6)) * k12 - cos(q(5)) * k13 - q(3);
    Q4j = ((-q(2) + f12) * cos(q(4)) + sin(q(4)) * (q(1) - f11)) * (cos(q(6)) * k11 - sin(q(6)) * k12) * cos(q(5)) + k12 * ((q(1) - f11) * cos(q(4)) + sin(q(4)) * (q(2) - f12)) * cos(q(6)) + (k11 * (q(1) - f11) * sin(q(6)) - sin(q(5)) * k13 * (q(2) - f12)) * cos(q(4)) + (k11 * (q(2) - f12) * sin(q(6)) + sin(q(5)) * k13 * (q(1) - f11)) * sin(q(4));
    Q5j = (k11 * (q(3) - f13) * cos(q(6)) - k13 * (q(1) - f11) * cos(q(4)) - k12 * (q(3) - f13) * sin(q(6)) - sin(q(4)) * k13 * (q(2) - f12)) * cos(q(5)) + sin(q(5)) * (k11 * ((q(1) - f11) * cos(q(4)) + sin(q(4)) * (q(2) - f12)) * cos(q(6)) - sin(q(6)) * k12 * (q(1) - f11) * cos(q(4)) - sin(q(4)) * k12 * (q(2) - f12) * sin(q(6)) + k13 * (q(3) - f13));
    Q6j = (sin(q(6)) * k11 + k12 * cos(q(6))) * ((q(1) - f11) * cos(q(4)) + sin(q(4)) * (q(2) - f12)) * cos(q(5)) + (-k11 * (q(2) - f12) * cos(q(4)) + k11 * (q(1) - f11) * sin(q(4)) - sin(q(5)) * k12 * (q(3) - f13)) * cos(q(6)) - sin(q(6)) * (-k12 * (q(2) - f12) * cos(q(4)) + k12 * (q(1) - f11) * sin(q(4)) + sin(q(5)) * k11 * (q(3) - f13));
    Mat_Q(:,itcable)=([Q1j,Q2j,Q3j,Q4j,Q5j,Q6j]')/rho_j;
    
end








