% Fichier pour la conception du contrôleur du moteur
% Auteur: Charles Sirois
% Date: 21/04/2020
%
% Guide d'utilisation:
% 1 - Exécuter le fichier parametres.m pour load les params du moteur
% 2 - Exécuter les sections désirées:
%       - Réponse en BO: Comparaison de la modélisation mathématique et la
%                           réponse exp
%       - Contrôleur du moteur: Calcul les gains au pôle p spécifié
%       - Réponse à une rampe: Affiche la réponse du moteur en BF à une rampe

%% ====== Moteur =====
%% ----- Identification -----
%% Réponse en BO à un échelon de 6V
clc
close all
load(fullfile("PythonData", 'step6V'))
simTime = Time(end);
stepVal = str2double(StepAmplitude);
sim("Simulink/Moteur_BO")

figure
suptitle("Réponse à un échelon de 6V")

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

clear StepAmplitude Time MotorAngle Phi CurrentSpeed Angle BikeAngle
clear Command Phi_dot targetAccek targetSpeed TargetSpeed tout targetAccel
clear stepVal

%% ----- Contrôleur du moteur -----
%% Contrôleur du moteur
close all
clc

p = 40;

syms Kps Kis

eqn1 = 2*p == (Km*Kps+1)/Tau_m;
eqn2 = 2*p^2 == (Kis*Km)/Tau_m;

sol = solve([eqn1, eqn2], [Kps, Kis]);

i = 1;
Kp_m = double(sol.Kps(i));
Ki_m = double(sol.Kis(i));
Kd_m = 0;

H2 = feedback(H, Kp_m);
H_bf = minreal(feedback(H2*(Ki_m/s), 1));

fprintf("---- Caratéristiques en BF ----\n");
H_bf
disp("Pôles: ")
disp(pole(H_bf))
disp("Gain statique: ")
disp(dcgain(H_bf))
disp("Zéro: ")
disp(zero(H_bf))
disp("Temps de réponse (2%): ")
disp(4/p);
disp("Erreur: ")
disp(1/(Kp_m*Km+1));

fprintf("\n---- Gains du moteur ----\n");
fprintf("\tKp: %4.6f\n", Kp_m)
fprintf("\tKi: %4.6f\n", Ki_m)

% Clean workspace
clear p
clear eqn1 eqn2 sol i
clear Kps Kis
clear H2


%% Réponse à une Rampe, TODO corrigé
clc
close all
plotExp = 0;

if plotExp == true
    load(fullfile("PythonData", 'Moteur_BF_C3_15000'))
    simTime = Time(end);
end

targetAccel = 15000;
targetSpeed = 8000;
sim("Simulink/Moteur_BF")

figure
% suptitle("PID")
% subplot(1, 2, 1)
hold on
if plotExp == true
    plot(Time, FW_Speed, 'r', 'DisplayName', 'Expérimental')
%     plot(Time, FW_Target_Speed, 'k', 'DisplayName', 'Target Speed')
end
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Théorique')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'k', 'DisplayName', 'Vitesse désiré')
legend
title("Comparaison de la réponse théorique et expérimentale")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
xlim([0 0.7])
grid on
hold off

% subplot(1, 2, 2)
% hold on
% if plotExp == true
%     plot(Time, FW_Command, 'r', 'DisplayName', 'Expérimental')
% end
% plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Théorique')
% legend
% title("Tension")
% xlabel("Temps (sec)")
% ylabel("Tension [V]")
% grid on
% hold off

clear plotExp