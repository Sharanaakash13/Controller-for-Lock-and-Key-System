%% Lock and Key minor project:
clear all 
close all 
clc
%% Difining the parameters:
e = 12;                                                                     % Voltage Step input in [volt] 
R = 3.69;                                                                   % Resistance in [ohm]
L = 0.231e-3;                                                               % inductance in [H]
D = 0.0000325;                                                              % Damping coefficient in [N.m.s/rad] 
Kt = 18.4e-3;                                                               % Torque constant in [Nm/A]
Kb = 18.48e-3;                                                              % back emf constan in [V.s/rad] 
r = 4e-3;                                                                   % radius of the pinion in [m]
m_k = 0.014;                                                         % Mass of the key shaft in [Kg]
Jkr = 5.55e-7;                                                          % inertia of the rotor in the key motor in [Kg.m^2] 
Jkg = 0.591e-7;                                                         % Inertia of the gears in the key motor in [Kg.m^2] 
Jkey = m_k*r^2;
Jk_eq = Jkg+Jkr+Jkey
m_pi = 2.144e-3;                                                         % Mass of the pinion in [kg] 
J_pi = m_pi*r^2;                                                      % Inertia of the pinion [Kg.m^2]
Js_r = 5.55e-7;                                                          % inertia of the rotor in the slid motor in [Kg.m^2] 
Js_g = 0.501e-7;                                                         % Inertia of the gears in the slid motor in [Kg.m^2]
Js_eq = Js_g+Js_r+J_pi 
m = 300e-3;
%% The state space matrices of the key motor:
A_k = [-R/L -Kb/L;Kt/Jk_eq -D/Jk_eq]; 
B_k = [1/L;0];
C_k = [0 1];
D_k = [0];
%% The state space matrices of the slid motor:
A_s = [-R/L -Kb/L;Kt/(m*r^2+Js_eq) -D/(m*r^2+Js_eq)]; 
B_s = [1/L;0];
C_s = [0 1];
D_s = [0];
sim('Sim_lk')
%% Plotting result of Rotational speed vs time for Key motor                %//
figure('name','Rotational speed for Key motor')                        
plot (rots_key(:,1),rots_key(:,2),'r','linewidth',1) 
title('Rotational speed Vs time (Key Motor)')
grid on
xlabel('Time (sec)') 
ylabel('RPM (speed)')
%% Plotting result of Rotational speed vs time for slide motor              %//
figure('name','Rotational speed for slide motor')
plot (rots_slide(:,1),rots_slide(:,2),'blue','linewidth',1) 
title('Rotational speed Vs time (Slide Motor)')
grid on 
xlabel('Time(sec)') 
ylabel('RPM (speed)')
%% Verification                                                             %//
figure('name', 'Verification of Key motor')
plot (rots_key(:,1),rots_key(:,2),'y','linewidth',3) 
hold on
grid on
plot (rots_key_tf(:,1),rots_key_tf(:,2),'black x','linewidth',1) 
plot (rots_key_ss(:,1),rots_key_ss(:,2),'r--','linewidth',2)
legend('Differential equation Model_key', 'Transfer function_key', 'state space_key')
xlabel('Time(sec)') 
ylabel('RPM (speed)')
figure('name', 'Verification of slide motor')
plot (rots_slide(:,1),rots_slide(:,2),'y','linewidth',3) 
hold on
grid on
plot (rots_slide_tf(:,1),rots_slide_tf(:,2),'black x','linewidth',1) 
plot (rots_slide_ss(:,1),rots_slide_ss(:,2),'r--','linewidth',2) 
legend('Differential equation Model_slide', 'Transfer function_slide', 'state space_slide') 
xlabel('Time(sec)')
ylabel('RPM (speed)')
%% Root locus compensator of the key motor 
num = Kt;
den = [L*Jk_eq L*D+R*Jk_eq R*D+Kt*Kb];
G_k = tf(num,den);                                                           % The transfer function of the key motor
T_k = feedback(G_k,1);                                                        % The closed loop system of the key motor 
figure(5)
rlocus(G_k) 
z = 0.69;
sgrid(z,0)
figure(6)
step(T_k)                                                                    % The step response of the uncompensated system
Gpd_k = zpk([-33563.37],[],1);                                               % the PD compenstator of the key motor 
figure(7)
rlocus(Gpd_k*G_k) 
z = 0.69;
sgrid(z,0)
Gpd_k = zpk([-33563.37],[],0.000161);                                        % the PD compenstator of the key motor with gain 
Gpi_k = zpk([-0.01],[0],1)                                                   % the PD compenstator of the key motor
figure(8)
Tpd_k = feedback(Gpd_k*G_k,1); 
step(Tpd_k,T_k)
legend('compensated system','Uncompensated system')
figure(9) 
rlocus(Gpi_k*Gpd_k*G_k) 
z = 0.69;
sgrid(z,0)
Gpd_k = zpk([-33563.37],[],0.000161)                                         % the PD compenstator of the key motor with gain 
Tpid_k = feedback(Gpi_k*Gpd_k*G_k,1);
figure(10) 
step(Tpid_k)
%rltool(Gpdk*Gpik*Gk)                                                       % the root locus and the step response of the compensated system
%% %% Root locus compensator of the slide motor 
num_s = Kt;
den_s = [((m*r^2+Js_eq)*L) (L*D+(m*r^2+Js_eq)*R) (R*D+Kt*Kb)]; 
G_s = tf(num_s,den_s);                                                         % The transfer function of the slid motor
T_s = feedback(G_s,1);                                                        % The closed loop system of the slid motor 
figure(11)
rlocus(G_s) 
z = 0.69;
sgrid(z,0)
ylim([-10000 10000])
figure(12)
step(T_s)                                                                    % The step response of the uncompensated system
Gpd_s = zpk([-33621.47],[],1);                                               % the PD compenstator of the slid motor 
figure(13)
rlocus(Gpd_s*G_s) 
z = 0.69;
sgrid(z,0)
Gpd_s = zpk([-33621.47],[],0.00109);
Gpi_s = zpk([-0.01],[0],1)                                                   % the PI compenstator of the slid motor
figure(14)
Tpd_s = feedback(Gpd_s*G_s,1); 
step(Tpd_s,T_s)
legend('compensated system','Uncompensated system')
figure(15) 
rlocus(Gpi_s*Gpd_s*G_s) 
z = 0.69;
sgrid(z,0)
Gpd_s = zpk([-33621.47],[],0.000109);
Tpid_s = feedback(Gpi_s*Gpd_s*G_s,1); 
figure(16)
step(Tpid_s)
% rltool(Gpds*Gpis*Gs)                                                      % the root locus and the step response of the compensated system
% z = 0.69;
% sgrid(z,0);
% Gpds = zpk([-33621.47],[],0.00109)
