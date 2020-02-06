function [u]=parallelhybrid_ECMS(x)
global data
%Given inputs from the driving cycle and controller this script should return
%the torques on the electric motor and combustion engine.
w_ice= x(1);
dw_ice=x(2);
T_req= x(3);  
lambda=x(4); 
%% Your code here..........
Q_o = 6.5; % Battery capacity (Ah)
U_oc = 300; % open circuit voltage (V);
I_max = 200 ; % max charging or discharging current can be represented by a sign(A)
I_min = -200; % Generator
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
e= 0.4;
efficiency_gearbox =0.98;
Mass_wheel = 6.6;
% SOC_initial =0.5;
% SOC_final =[0.49 0.498 0.5 0.501 0.51];
%% TORQUE
%T_req=(torque_wheel/gear_ratio)*(1/(efficiency_gearbox^sign(torque_wheel)));
%% Battery
%I_battery= -((SOC_final-SOC_initial).*(Q_o*3600)); % Discharge current or charge current from Battery sate of charge
I_battery= linspace(I_min,I_max,40000);
P_bat= (U_oc.*I_battery)-(R_i*(I_battery.^2));  % Battery power consumption
P_em = (0.9.^sign(I_battery)).*P_bat; % Power of the electric motor;
%% ENGINE
T_em=(P_em./w_ice);
T_ice = T_req-T_em;
M_f =(w_ice/(e*H_l)).*(T_ice+(0.1e6*V_d/(4*pi))+(J_e*dw_ice));  %Fuel power consumption
%% Hamiltonian
P_f = H_l.*M_f;
P_f(M_f<0)=0;
P_ech= I_battery.*U_oc;
H = P_f +(lambda.*P_ech);
%P_f(M_f<0)=0;
if w_ice==0
    T_ice=0;
    T_em=0;
    u=[T_ice;T_em];
else
%H(T_ice<0)=inf;
H(T_em>400)=inf;
H(T_em<-400)=inf;
H(P_em>50000)=inf;
H(P_em<-50000)=inf;
H((T_ice+dw_ice*J_e)>115)=inf;
[r,c]=min(H);
T_em=T_em(c);
T_ice=T_ice(c);
u=[T_ice;T_em];
end

