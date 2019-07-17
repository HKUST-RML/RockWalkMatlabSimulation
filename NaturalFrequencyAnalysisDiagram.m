clc
clear all
close all

load('robot');
nalphastep=20;
spanvalpha_simple_eq=0:90/nalphastep:90;
for it=1:length(spanvalpha_simple_eq)
    currentalpha=spanvalpha_simple_eq(it)*pi/180;
    vect_period_simple_eq(it)=2*pi*sqrt((r-r_CM_O(1))^2/(r_CM_O(1)*g*sin(currentalpha)));
end

figure
plot(spanvalpha_simple_eq,vect_period_simple_eq,'r');
hold on
Data_experiment=dlmread('Data_experiment.txt');
Data_Simulation_ZeroI=dlmread('Data_Simulation_ZeroI.txt');
Data_Simulation_nZeroI=dlmread('Data_Simulation_nZeroI.txt');
for it=1:size(Data_experiment,1)
    plot(Data_experiment(it,1),Data_experiment(it,2),'*k');
end
plot(Data_Simulation_ZeroI(:,1),Data_Simulation_ZeroI(:,2),'g');
plot(Data_Simulation_nZeroI(:,1),Data_Simulation_nZeroI(:,2),'b');

axis([0 90 0 4]);
% legend('Experimenta data','Simplified dynamic','Elaborated dynamic zero moment of inertia','Elaborate dynamic with non-zero moment of inertia');