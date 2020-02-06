function [M_f]=parallelHybrid(tVec,SOC_start,SOC_final);

% costVector=parallelHybrid(t_vec,SOC_start,SOC_final);
%
% Function for calculating the all arc-costs from one node to all other possible nodes.
% 
% This function is called from dynProg1D.m
%  
% Inputs:
%  t_vec     - 1x2 matrix, with the start and stop time for the interval.
%  SOC_start - A single start value for SOC during the interval.
%  SOC_final - Vector (from the dicretization) with all possible final values for the
%             interval.
%
% Output:
%  costVector - Vector with the costs for all arcs from SOC_start. 
  
% Version 1.0,  2008-06-30 Lars Eriksson


% Implement your parallel hybrid model and calculate the arc costs below.
%%Here the goal is to model all the components in the parallel hybrid
%electric vehicle
load('EUDC_MAN_DDP','G_z');
load('EUDC_MAN_DDP','V_z');
load('EUDC_MAN_DDP','T_z');
%  tVec =T_z;
% SOC_start=0.5;
% SOC_final=linspace(0.4,0.6,21);
Q_o = 6.5   %25.036; ; % Battery capacity (Ah)
U_oc = 300;  %351.5; %300; % open circuit voltage (V);
I_max = 200;%167.85;   % 200 ; % max charging or discharging current can be represented by a sign(A)
I_min = -200; %-167.85;  %= -200; 
R_i= 0.65;  %0.6 ; % Inner resistance (ohms)
M_battery = 45; %120.20;    %45; % weight of the battery (kgs)
P_emmax =50; % Max power of the motor or generator(-VE sign indicates the generator)
C_d=0.32;  % Drag coefficient
C_r= 0.015; % Rolling resistance coefficient+pl
r_w= 0.3; %vehicle radius (m)
J_w =0.6; % Inertia of the wheels (kgm^2)
A_f=2.31; %Frontal area (m^2)
rho_a= 1.18; % Air density (kgm^3)
M=1500; %1569.312;    %1500; % Mass of vehicle (kgs)
M_em= 1.5; % Mass of the electric motor (kg/kw)
M_e=1.2; % Mass of the engine (kg/kw)
Total_mass= M+M_em+M_e+M_battery;  % (Total mass of the vehicle)
g=9.81; % Acceleration due to gravity(m/s^2)
J_e=0.2;% Engine inertia (kgm^2)
J_m=0.3; % Motor inertia
pme_o=0.1; %  mean effective pressure 
V_d= 1.497*10^-3; %1.2*10^-3; % Engine displacement (m^3)
eff_motor=0.9; % Efficiency of the motor or generator
H_l = 44.6e6; %Lower heating value (J/kg)
e= 0.4;
efficiency_gearbox =0.98;
Mass_wheel = 6.6;
G_z(G_z==0)=0;
G_z(G_z==1)=9.97; %13.0529;
G_z(G_z==2)=5.86; %8.1595;
G_z(G_z==3)=3.84; %5.6651;
G_z(G_z==4)=2.68; %4.2555;
G_z(G_z==5)=2.14; %3.2623;
%%  SPEED ,ACCLERATION
Average_speed =mean(V_z(tVec)); % Average speed at the specified time
Average_accleration =V_z(tVec(2))-V_z(tVec(1)); % Average acceleration at the specified
speed=Average_speed/r_w;  % Angular speed
acceleration =Average_accleration/r_w; % Angular accleration
gear_ratio =G_z(tVec(1));   % Gear ratios at the specfied time
%% TORQUE
Force_aero=0.5*rho_a*C_d*A_f*(Average_speed)^2;
Force_acc=(M+Mass_wheel)*Average_accleration;
Force_roll=M*g*C_r;
torque_wheel=(Force_roll+Force_aero+Force_acc)*r_w;
torque_gearbox=(torque_wheel/gear_ratio)*(1/(efficiency_gearbox^sign(torque_wheel)));
speed_engine= speed*gear_ratio;
acceleration_engine =acceleration.*gear_ratio;
%% Battery
%I_battery = (U_oc-sqrt(U_oc^2-4*R_i*P_em))/(2*R_i); % Battery current
I_battery= -((SOC_final-SOC_start).*(Q_o*3600)); % Discharge current or charge current from Battery sate of charge

P_bat= (U_oc.*I_battery)-(R_i*(I_battery.^2));  % Battery power consumption
P_em = (0.9.^sign(I_battery)).*P_bat; % Power of the electric motor
%P_bat= (U_oc.*I_battery)-(R_i*(I_battery.^2));  % Battery power consumption
%% ENGINE
Torque_em=(P_em./speed_engine);
Torque_engine = torque_gearbox-Torque_em;
xx=(0.1e6*V_d/(4*pi))
yy=(J_e*acceleration_engine);
M_f =(speed_engine/(e*H_l)).*(Torque_engine+xx+yy);  %Fuel power consumption
%% Limitations 
M_f(M_f<0) =0;
M_f(I_battery>I_max |I_battery<I_min)=inf;
M_f(gear_ratio ==0 & I_battery==0)=0; 
M_f(gear_ratio ==0 & abs(I_battery>0))=inf;
 M_f(Torque_em>400|Torque_em<-400)=inf;
M_f(Torque_engine>110)=inf;



