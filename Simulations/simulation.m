%% Paramètres du système
clc
close all

%Vélo
Mv = 331/1000;            %Masse du vélo [kg]
Lv = 4/100;                  %Distance sol-CM [m]
Jv = 224468.05/(1000^3);     %Inertie du vélo [Kg*m^2]

%Roue inertielle
Mp = 17/1000;         %Masse de la roue en plastique [kg]
Jp = 14826/(1000^3);    %Inertie du plastique[Kg*m^2]

Mm = 74/1000;   %Masse du washer en métal [kg]   
Rm = 32/1000;   %Rayon total du washer [m]
rm = 14/1000;   %Rayon intérieur du washer [m]
Jm = Mm/2*(Rm^2 + rm^2);

Mr = 165;
Lr = 9/100;             %Distance sol-CM roue [m]
Jr = 2*Jm+Jp;    %Inertie de la roue [Kg*m^2]

%Moteur

% % M1: 6V avec poids
% Km = 252.5; %Gain statique
% Tau_m = 0.031;

% M2: 6V avec poids 
Km = 1311.7; %Gain statique
Tau_m = 0.205;

%Autres
Theta = 0;  %Angle du vélo par rapport à la vertical
Phi = 0;    %Angle de la roue inertielle
Tau = 0;    %Couple du moteur
g = 9.81;   %Accélération gravitationnel [m/s^2]

% Fonctions de transfert
s = tf('s');
H = Km/ ( Tau_m*s + 1 ); %phi_dot selon U EN RAD

G = (-Jr*s)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot(s)  => Angle du vélo selon vitesse roue inertielle
G_a = (-Jr)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot_dot(s)  => Angle du vélo selon accel de la roue

%% ===================
%% ====== Moteur =====
%% ===================

%% ----- Identification -----
%% Step Response 5V
clc
close all
load(fullfile("PythonData", 'step5V'))
simTime = Time(end);
stepVal = str2double(StepAmplitude);
sim("Simulink/Moteur_BO")

% plot_func('Moteur en BF', 'Temps (s)', '', dataList, dataOpt);

figure
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Vitesse exp')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse théorique')
legend
title("Réponse à l'échellon")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

figure
hold on
plot(Time, MotorAngle, 'r', 'DisplayName', 'Angle exp')
plot(Phi(:, 1), Phi(:, 2), 'b', 'DisplayName', 'Angle théorique')
legend
title("Réponse à l'échellon")
xlabel("Temps (sec)")
ylabel("Vitesse [deg]")
grid on
hold off

%% Step Response 6V
clc
close all
load(fullfile("PythonData", 'step6V_25'))
simTime = Time(end);
stepVal = str2double(StepAmplitude);
sim("Simulink/Moteur_BO")

figure
suptitle("Réponse à l'échellon")

subplot(1, 2, 1)
hold on
plot(Time, MotorAngle, 'r', 'DisplayName', 'Angle exp')
plot(Phi(:, 1), Phi(:, 2), 'b', 'DisplayName', 'Angle théorique')
legend
title("Position")
xlabel("Temps (sec)")
ylabel("Position [deg]")
grid on
hold off

subplot(1, 2, 2)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Vitesse exp')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse théorique')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off


%% ----- Contrôleur du moteur -----
%% Designeur de Matalab
[num_m, den_m] = tfdata(Cm, 'v');
Kp_m = num_m(1);
Kd_m = 0; %num_m(1);
Ki_m = num_m(2);

%% Pôles alignés
close all
clc

p = 100;
p1 = s+p+p*1i;
p2 = s+p-p*1i;
poles = p1*p2;

[num_m, den_m] = tfdata(poles, 'v');
Kp_m = (num_m(2) - 1)/Km;
Ki_m = num_m(3)/Km;
Kd_m = (1-Tau_m)/Km;

fprintf("---- Gains du moteur ----\n");
fprintf("\tKp: %4.4f\n", Kp_m)
fprintf("\tKi: %4.4f\n", Ki_m)
fprintf("\tKd: %4.4f\n", Kd_m)  

H2 = feedback(H, Kd_m*s);
H_bf = minreal(feedback(H2*(Kp_m + Ki_m/s), 1));

ramp = 20000;
simTime = 5;
sim("Simulink/Moteur_BF")
dataList = {Phi_dot_des 'r' 'Commande [deg/sec]'; 
            Phi_dot 'b' 'Vitesse [deg/s]'};
            %Phi 'g' 'Position [deg]'};
dataOpt = {U 'k' 'Tension [V]'};

plot_func('Moteur en BF', 'Temps (s)', '', dataList, dataOpt);

%% PD
close all
clc

p = 15; %Pôle
a = 30; %Relation Kp Kd

syms Kps Kds

Kd_m = (p*Tau_m - 1)/((a-p)*Km);
Kp_m = a*Kd_m;
Ki_m = 0;


fprintf("---- Gains du moteur ----\n");
fprintf("\tKp: %4.4f\n", Kp_m)
fprintf("\tKi: %4.4f\n", Ki_m)
fprintf("\tKd: %4.4f\n", Kd_m)  

H2 = feedback(H, Kd_m*s);
H_bf = minreal(feedback(H2*(Kp_m + Ki_m/s), 1));

fprintf("\n---- Caratéristiques en BF ----\n");
H_bf
disp("Pôles: ")
disp(pole(H_bf))
disp("Gain statique: ")
disp(dcgain(H_bf))
disp("Zéro: ")
disp(zero(H_bf))

load(fullfile("PythonData", 'PD_Exp'))
simTime = Time(end);
targetAccel = 20000;
targetSpeed = 5000;
sim("Simulink/Moteur_BF")

figure
suptitle("Réponse à l'échellon")
subplot(1, 2, 1)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Expérimental')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Théorique')
plot(Time, TargetSpeed, 'k', 'DisplayName', 'Target Speed')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

subplot(1, 2, 2)
hold on
plot(Time, Command, 'r', 'DisplayName', 'Expérimental')
plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Théorique')
legend
title("Tension")
xlabel("Temps (sec)")
ylabel("Tension [V]")
grid on
hold off

%% PID 1
close all
clc

p = 15;

syms Kps Kis Kds

eqn1 = 2*p == Kps*Km+1;
eqn2 = 2*p^2 == Kis*Km;
eqn3 = 1 == Km*Kds+Tau_m;

sol = solve([eqn1, eqn2, eqn3], [Kps, Kis, Kds]);

i = 1;
Kp_m = double(sol.Kps(i));
Ki_m = double(sol.Kis(i));
Kd_m = double(sol.Kds(i));

H2 = feedback(H, Kd_m*s);
H_bf = minreal(feedback(H2*(Kp_m + Ki_m/s), 1));

fprintf("---- Caratéristiques en BF ----\n");
H_bf
disp("Pôles: ")
disp(pole(H_bf))
disp("Gain statique: ")
disp(dcgain(H_bf))
disp("Zéro: ")
disp(zero(H_bf))

fprintf("\n---- Gains du moteur ----\n");
fprintf("\tKp: %4.4f\n", Kp_m)
fprintf("\tKi: %4.4f\n", Ki_m)
fprintf("\tKd: %4.4f\n", Kd_m)  

TargetAccel = 20000;
TargetSpeed = 5000;
simTime = 5;

sim("Simulink/Moteur_BF")
dataList = {Phi_dot_des 'r' 'Commande [deg/sec]'; 
            Phi_dot 'b' 'Vitesse [deg/s]'};
            %Phi 'g' 'Position [deg]'};
dataOpt = {U 'k' 'Tension [V]'};

plot_func('Moteur en BF', 'Temps (s)', '', dataList);

%% PID 2
close all
clc

p = 50;
a = 60;

syms Kps Kis Kds

eqn1 = 2*p == (Kps*Km+1)/(Km*Kds + Tau_m);
eqn2 = 2*p^2 == (Kis*Km)/(Km*Kds + Tau_m);
eqn3 = a == Kis/Kps;

sol = solve([eqn1, eqn2, eqn3], [Kps, Kis, Kds]);

i = 1;
Kp_m = double(sol.Kps(i));
Ki_m = double(sol.Kis(i));
Kd_m = double(sol.Kds(i));

H2 = feedback(H, Kd_m*s);
H_bf = minreal(feedback(H2*(Kp_m + Ki_m/s), 1));

fprintf("---- Caratéristiques en BF ----\n");
H_bf
disp("Pôles: ")
disp(pole(H_bf))
disp("Gain statique: ")
disp(dcgain(H_bf))
disp("Zéro: ")
disp(zero(H_bf))

fprintf("\n---- Gains du moteur ----\n");
fprintf("\tKp: %4.6f\n", Kp_m)
fprintf("\tKi: %4.6f\n", Ki_m)
fprintf("\tKd: %4.6f\n", Kd_m)  

%% Réponse à un échellon
load(fullfile("PythonData", 'PID2_Ramp'))
simTime = Time(end);
targetAccel = 20000;
targetSpeed = 5000;
sim("Simulink/Moteur_BF")

figure
suptitle("PID")
subplot(1, 2, 1)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Expérimental')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Théorique')
plot(Time, TargetSpeed, 'k', 'DisplayName', 'Target Speed')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

subplot(1, 2, 2)
hold on
plot(Time, Command, 'r', 'DisplayName', 'Expérimental')
plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Théorique')
legend
title("Tension")
xlabel("Temps (sec)")
ylabel("Tension [V]")
grid on
hold off

%% Réponse à une Rampe
load(fullfile("PythonData", 'PID_01_Ramp_25000'))
simTime = Time(end);
targetAccel = 25000;
targetSpeed = 5000;
sim("Simulink/Moteur_BF")

figure
suptitle("PID")
subplot(1, 2, 1)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Expérimental')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Théorique')
% plot(Time, TargetSpeed, 'k', 'DisplayName', 'Target Speed (exp)')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'k', 'DisplayName', 'Target speed')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

subplot(1, 2, 2)
hold on
plot(Time, Command, 'r', 'DisplayName', 'Expérimental')
plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Théorique')
legend
title("Tension")
xlabel("Temps (sec)")
ylabel("Tension [V]")
grid on
hold off





%% ===================
%% ======= Vélo ======
%% ===================

%% ----- Identification -----
clc
close all
load(fullfile("PythonData", 'Velo_BO'))
simTime = 0.2;
startVal = 20;
nVal = 8;

AngleInitial = BikeAngle(startVal);
sim("Simulink/Velo_BO")


figure
hold on
plot(Time(1:nVal+1), BikeAngle(startVal:startVal+nVal), 'r', 'DisplayName', 'Angle (exp)')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle (théo)')
legend
title("Vélo - Réponse à l'échellon")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off


%% ----- Contrôleur du vélo ----- 
%% Pôles alignés en accel
close all
clc
fprintf("Méthode par pôles alignés\n");
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

p_r = 3;
p_i = 3;
p1 = s+p_r+p_i*1i;
p2 = s+p_r-p_i*1i;
poles = p1*p2*(s+p_r);

[num_v, den_v] = tfdata(poles, 'v');
a_v = num_v(2);
b_v = num_v(3);
c_v = num_v(4);

Kp_v = -((Jv+Jr+A)*b_v+B)/Jr;
Ki_v = -(Jv+Jr+A)*c_v/Jr;
Kd_v = -(Jv+Jr+A)*a_v/Jr;

fprintf("---- Gains du vélo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G_a, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Caratéristiques en BF ----\n");
G_bf
disp("Pôles: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Zéro: ")
disp(zero(G_bf))

%% Simplification pôles/zéro
close all
clc
fprintf("Méthode par pôles alignés\n");
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

syms Kps Kis Kds

a = 1.5;

eqn1 = 2*a + Kis/Kps == -(Jr*Kds)/(Jv+Jr+A);
eqn2 = 2*a^2 + 2*a*Kis/Kps == -(B+Jr*Kps)/(Jv+Jr+A);
eqn3 = (2*a^2*Kis)/Kps == -(Jr*Kis)/(Jv+Jr+A);

sol = solve([eqn1, eqn2, eqn3], [Kps, Kis, Kds]);

i = 2;
Kp_v = double(sol.Kps(i));
Ki_v = double(sol.Kis(i));
Kd_v = double(sol.Kds(i));

fprintf("---- Gains du vélo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G_a, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Caratéristiques en BF ----\n");
G_bf
disp("Pôles: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Zéro: ")
disp(zero(G_bf))


%% Simulation Vélo, sans moteur
clc
close all
AngleInitial = 5;
simTime = 7;

sim("Simulink/Velo_BF")
dataList = {Theta 'r' 'Angle vélo [deg]'};
            
dataOpt = {Phi_dot_des 'k' 'Vitesse des [deg/sec]';
           Phi_dot_dot_des 'b' 'Accel des [deg^2/sec]'};

plot_func('Velo BF', 'Temps (s)', '', dataList, dataOpt);

fprintf("---- Maximum ----\n");
fprintf("\tVitesse max [deg/sec]: %6.2f\n", max(Phi_dot_des(:, 2)))
fprintf("\tAccel max [deg/sec]: %6.2f\n", max(Phi_dot_dot_des(:, 2)))


%% Simulation Vélo complet
clc
close all
AngleInitial = 5;
simTime = 7;

sim("Simulink/Velo_Complet_BF")
dataList = {Theta 'r' 'Angle vélo [deg]'};
            
dataOpt = {Phi_dot_des 'k' 'Vitesse des [deg/sec]';
           Phi_dot_dot_des 'b' 'Accel des [deg^2/sec]'};

plot_func('Velo BF', 'Temps (s)', '', dataList, dataOpt);

fprintf("---- Maximum ----\n");
fprintf("\tVitesse max [deg/sec]: %6.2f\n", max(Phi_dot_des(:, 2)))
fprintf("\tAccel max [deg/sec]: %6.2f\n", max(Phi_dot_dot_des(:, 2)))

%%
hold on
plot(Theta_1(:, 1), Theta_1(:, 2), 'r', 'DisplayName', '0')
plot(Theta_2(:, 1), Theta_2(:, 2), 'b', 'DisplayName', '35000')
legend
grid on
hold off
