% Fichier pour la conception du contrôleur du vélo
% Auteur: Charles Sirois
% Date: 21/04/2020
%
% Guide d'utilisation:
% 1 - Exécuter le fichier parametres.m pour load les params du moteur
% 2 - Exécuter les sections désirées:
%       - Velo BO: Comparaison de la modélisation mathématique et la
%                           réponse exp du vélo
%       - Conception du contrôleur: Calcul les gains au pôle p spécifié
%       - Simulation Vélo complet: Simule le contrôleur. Possible de
%                                 comparer avec la réponse exp
%       - Comparaison Algo: Compare la réponse avec et sans
%                           l'algorithme de limitation de vitesse de 
%                           la roue inertielle

%% ======= Vélo ======
%% ----- Identification -----
%% Velo BO
clc
close all

% Sans accelération
load(fullfile("PythonData", 'Velo_BO'))
simTime = 0.2;
startVal = 20;
nVal = 8;


AngleInitial = BikeAngle(startVal);
AccelVal = 0;
sim("Simulink/Velo_BO")

figure
suptitle("Vélo en BO")
subplot(1,2, 1)
hold on
plot(Time(1:nVal+1), BikeAngle(startVal:startVal+nVal), 'r', 'DisplayName', 'Angle expérimentale')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle théorique')
legend
title("Sans accélération")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Avec accélération
load(fullfile("PythonData", 'Velo_BO_Ramp_20000'))
endVal = 46;
simTime = Time(endVal);

AngleInitial = BikeAngle(1);
AccelVal = -20000;

sim("Simulink/Velo_BO")

subplot(1, 2, 2)
hold on
plot(Time(1:endVal), BikeAngle(1:endVal), 'r', 'DisplayName', 'Angle (exp)')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle (théo)')
legend
title("Avec accélération")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

%% ----- Contrôleur du vélo ----- 
%% Conception du contrôleur
close all
clc
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

p = 8;

syms Kps Kds

eqn1 = 2*p == -(Kds*Jr)/(Jr+Jv+A);
eqn2 = 2*p^2 == -(B+Kps*Jr)/(Jr+Jv+A);

sol = solve([eqn1, eqn2], [Kps, Kds]);

i = 1;
Kp_v = double(sol.Kps(i));
Kd_v = double(sol.Kds(i));
Ki_v = 0;

G2 = feedback(G, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("---- Caratéristiques en BF ----\n");
G_bf
disp("Pôles: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Zéro: ")
disp(zero(G_bf))
disp("Temps de réponse (2%): ")
disp(4/p);

fprintf("\n---- Gains du vélo ----\n");
fprintf("\tKp: %4.6f\n", Kp_v)
fprintf("\tKd: %4.6f\n", Kd_v)

clear p A B eqn1 eqn2 sol
clear Kps Kds i G2

%% Simulation Vélo complet
clc
close all

plotExp = 0;
nData = 75;
p = 0;

if plotExp == true
    load(fullfile("PythonData", 'Velo_PD_03'))
    simTime = Time(nData);
    AngleInitial = BikeAngle(1);
else
    simTime = 1;
    AngleInitial = 8;
end

sim("Simulink/Velo_Complet_BF")
sim("Simulink/Velo_BF")

% Plot results
figure
suptitle("Vélo Complet")
% Angle
subplot(2, 1, 1)
hold on
%   Exp data
if plotExp == true
    plot(Time(1:nData), BikeAngle(1:nData), 'r', 'DisplayName', 'Expérimental')
end
%   Théo data
plot(Theta_BF(:, 1), Theta_BF(:, 2), 'r', 'DisplayName', 'Théorique')
% plot(Theta_BF_nm(:, 1), Theta_BF_nm(:, 2), 'k', 'DisplayName', 'Sans moteur')
legend
title("Position")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% % Vitesse flywheel
% subplot(2, 1, 2)
% hold on
% %   Exp data
% if plotExp == true
%     plot(Time(1:nData), CurrentSpeed(1:nData), 'r', 'DisplayName', 'Vitesse (exp)')
%     plot(Time(1:nData), TargetSpeed(1:nData), 'r--', 'DisplayName', 'Vitesse désirée (ex)')
% end
% %   Theo Data
% plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse actuelle (theo)')
% plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'b--', 'DisplayName', 'Vitesse désirée (theo)')
% % plot(Phi_dot_des_nm(:, 1), Phi_dot_des_nm(:, 2), 'k--', 'DisplayName', 'Vitesse désirée sans moteur') % Simu parfaite
% legend
% title("Vitesse")
% xlabel("Temps (sec)")
% ylabel("Vitessse [deg/sec]")
% grid on
% hold off

fprintf("---- Maximum ----\n");
fprintf("\tVitesse max [deg/sec]: %6.2f\n", max(abs(Phi_dot_des(:, 2))))
fprintf("\tAccel max [deg/sec]: %6.2f\n", max(abs(Phi_dot_dot_des(:, 2))))

clear plotExp p nData

%% Comparaison Algo
clc
close all

n1 = "noAlgo_01";
n2 = "5min";
nData1 = 550;
nData2 = 700;

load(fullfile("PythonData", 'Velo_'+n1))
Bike_Angle = -Bike_Angle;
FW_Speed = -FW_Speed;
% Plot results
figure
% suptitle("Sans l'algorithme")
% Angle
subplot(2, 1, 1)
hold on
plot(Time(1:nData1), Bike_Angle(1:nData1), 'r', 'DisplayName', 'Angle [deg]')
legend
% title("Position")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
xlim([0 5.5])
grid on
hold off
% Vitesse flywheel
subplot(2, 1, 2)
hold on
plot(Time(1:nData1), FW_Speed(1:nData1), 'b', 'DisplayName', 'Vitesse [deg/sec]')
% plot(Time(1:nData), FW_Target_Speed(1:nData), 'r--', 'DisplayName', 'Vitesse désirée (exp 1)')
legend
% title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitessse [deg/sec]")
xlim([0 5.5])
grid on
hold off

load(fullfile("PythonData", 'Velo_'+n2))

% Plot results
figure
% suptitle("Avec l'algorithme")
% Angle
subplot(2, 1, 1)
hold on
plot(Time(1:nData2), Bike_Angle(1:nData2), 'r', 'DisplayName', 'Angle [deg]')
legend
% title("Position")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off
% Vitesse flywheel
subplot(2, 1, 2)
hold on
plot(Time(1:nData2), FW_Speed(1:nData2), 'b', 'DisplayName', 'Vitesse [deg/sec]')
% plot(Time(1:nData), FW_Target_Speed(1:nData), 'r--', 'DisplayName', 'Vitesse désirée (exp 1)')
legend
% title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitessse [deg/sec]")
grid on
hold off


