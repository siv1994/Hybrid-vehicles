function [M_f]=seriesHybrid(t_vec,SOC_start,SOC_final,N_start,N_final);

% costVector=seriesHybrid(t_vec,SOC_start,SOC_final,Ne_start,Ne_final);
% 
% Function for calculating all the arc-costs during one time interval, from one node to
% all other possible nodes.
% 
% This function is called from dynProg2D.m with the following argument list
%    funName([tVec(t) tVec(t+1)],discX(ii),discX,discY(jj),discY)
%  
% Inputs:
%  t_vec      - 1x2 matrix, with the start and stop time for the interval.
%  SOC_start  - A single start value for SOC during the interval.
%  SOC_final  - Vector (from the dicretization) with all possible final values for the
%               interval.
%  Ne_start   - A single start value for engine speed during the interval.
%  Ne_final   - Vector (from the dicretization) with all possible final values for the
%               interval.
%
% Output:
%  costMatrix - Matrix with the costs for all arcs from SOC_start. 
%               The size of the returned cost-matrix should have the following
%               size:  length(SOC_final) x length(Ne_final)
  
% Version 1.0,  2008-06-30 Lars Eriksson
 
% load('city_MAN_DDP','G_z');
% load('city_MAN_DDP','V_z');
% load('city_MAN_DDP','T_z');
load('EUDC_MAN_DDP','G_z');
load('EUDC_MAN_DDP','V_z');
load('EUDC_MAN_DDP','T_z');
% t_vec =[15,159];
% SOC_start=50*10^-2;
%SOC_final=50*10^-2;      % SECOND CASE
%SOC_final=[0.49,0.498,0.50,0.501,0.51];   % FIRST CASE
% SOC_final =[49.9;50].*10^-2;   % THIRD CASE
% N_start = 0;
% N_final =3e3;
% N_final=[0,2e3,3e3,5e3]; % .*((2*pi)/60);  % SECOND CASE
% N_final = [0,8e2,20e2].*((2*pi)/60);  % THIRD CASE
Q_o = 6.5; % Battery capacity (Ah)
U_oc = 300; % open circuit voltage (V);
I_max = 200 ; % max charging or discharging current can be represented by a sign(A)
I_min =-200; % Generator
R_i= 0.65 ; % Inner resistance (ohms)
M_battery = 45; % weight of the battery (kgs)
P_emmax =50; % Max power of the motor or generator(-VE sign indicates the generator)
C_d=0.32;  % Drag coefficient
C_r= 0.015; % Rolling resistance coefficient+pl
r_w= 0.3; %vehicle radius (m)
J_w =0.6; % Inertia of the wheels (kgm^2)
A_f=2.31; %Frontal area (m^2)
rho_a= 1.18; % Air density (kgm^3)
M=1500; % Mass of vehicle (kgs)
M_em= 1.5; % Mass of the electric motor (kg/kw)
M_e=1.2; % Mass of the engine (kg/kw)
Total_mass= M+M_em+M_e+M_battery;  % (Total mass of the vehicle)
g=9.81; % Acceleration due to gravity(m/s^2)
J_e=0.2;% Engine inertia (kgm^2)
J_m=0.3; % Motor inertia
pme_o=0.1; %  mean effective pressure 
V_d= 1.497*10^-3; % Engine displacement (m^3)
eff_motor=0.9; % Efficiency of the motor or generator
H_l = 44.6e6; %Lower heating value (J/kg)
E= 0.4;
efficiency_gearbox =0.98;
Mass_wheel = 6.6; % From I =m*r^2 
G_z(G_z==0)=0;       % which allocates the value of 0 at gear 0
G_z(G_z==1)=13.0529;  % which allocates the value of 13.0529 at gear 1
G_z(G_z==2)=8.1595;   % which allocates the value of 8.1595 at gear 2
G_z(G_z==3)=5.6651;   % which allocates the value of 5.6651 at gear 3
% G_z(G_z==4)=4.2555;
% G_z(G_z==5)=3.2623;

%%  SPEED ,ACCLERATION
Average_speed =mean(V_z(t_vec)); % Average speed at the specified time
Average_acceleration =V_z(t_vec(2))-V_z(t_vec(1)); % Average acceleration at the specified time
speed=(Average_speed/r_w);  % Angular speed
acceleraton =Average_acceleration/r_w; % Angular accleration
gear_ratio =G_z(t_vec(1));   % Gear ratios at the specfied time
%% TORQUE
Force_aero=0.5*rho_a*C_d*A_f*(Average_speed)^2;
Force_acc=(M+Mass_wheel)*Average_acceleration; 
Force_roll=M*g*C_r;
torque_wheel=(Force_roll+Force_aero+Force_acc)*r_w;
Torque_em = (torque_wheel)*(1/(eff_motor^sign(torque_wheel)));
%torque_gearbox=(torque_wheel/gear_ratio)*(1/(efficiency_gearbox^sign(torque_wheel)));
speed_engine= ((N_start+N_final)/2);    %  average speed of the engine
acceleration_engine =(N_final-N_start); % average accleration of the engine
speed_engine(N_start==0 & N_final>83.77)=inf;

%% Battery
%I_battery = (U_oc-sqrt(U_oc^2-4*R_i*P_em))/(2*R_i); % Battery current
I_battery= -(transpose(SOC_final-SOC_start).*(Q_o*3600)); % Discharge current or charge current from Battery sate of charge
P_em = (Torque_em*speed); % Power of the electric motor
P_bat= (U_oc.*I_battery)-(R_i*(I_battery.^2));  % Battery power consumption
P_gen = P_em-P_bat; % In series hybrid the power of the generator
%% ENGINE
Torque_engine=P_gen./(speed_engine.*eff_motor);  
Torque_engine(Torque_engine<=0)=0;% %for[15,16], N_start==0 condition at third case
Torque_engine(N_start==0 & Torque_engine>0)=inf; %for[15,16] condition at third case
Torque_engine(N_start>0 & N_final==0)=0;
Torque_engine(speed_engine==0 & P_gen ==0)=inf; %for[4,5] condition at third case
Torque_engine(speed_engine==0 & P_gen<=0)=0; %for[4,5] condition at third case
 %...................................

M_f =(speed_engine./(E*H_l)).*(Torque_engine+(0.1e6*V_d/(4*pi))+(J_e*acceleration_engine));  %Fuel power consumption
%% Constraints according to the given parameters
M_f(M_f<0) =0;
M_f(I_battery>I_max |I_battery<I_min)=inf;
M_f(Torque_em>400|Torque_em<-400)=inf;
M_f(Torque_engine>115)=inf;
M_f(I_battery==0 & Average_speed>=0 & N_final==0& N_start>N_final)=inf; 
M_f(I_battery==0 & Average_speed==0 & (N_final==0) & acceleration_engine>0)=inf; 
M_f((N_start==0 & speed_engine>83.77)&I_battery>=0)=inf; 








