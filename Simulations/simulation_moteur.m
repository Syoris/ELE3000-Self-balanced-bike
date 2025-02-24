% Fichier pour la conception du contr�leur du moteur
% Auteur: Charles Sirois
% Date: 21/04/2020
%
% Guide d'utilisation:
% 1 - Ex�cuter le fichier parametres.m pour load les params du moteur
% 2 - Ex�cuter les sections d�sir�es:
%       - R�ponse en BO: Comparaison de la mod�lisation math�matique et la
%                           r�ponse exp
%       - Contr�leur du moteur: Calcul les gains au p�le p sp�cifi�
%       - R�ponse � une rampe: Affiche la r�ponse du moteur en BF � une rampe

%% ====== Moteur =====
%% ----- Identification -----
%% R�ponse en BO � un �chelon de 6V
clc
close all
load(fullfile("PythonData", 'step6V'))
simTime = Time(end);
stepVal = str2double(StepAmplitude);
sim("Simulink/Moteur_BO")

figure
suptitle("R�ponse � un �chelon de 6V")

subplot(1, 2, 1)
hold on
plot(Time, MotorAngle, 'r', 'DisplayName', 'Angle exp')
plot(Phi(:, 1), Phi(:, 2), 'b', 'DisplayName', 'Angle th�orique')
legend
title("Position")
xlabel("Temps (sec)")
ylabel("Position [deg]")
grid on
hold off

subplot(1, 2, 2)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Vitesse exp')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse th�orique')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

clear StepAmplitude Time MotorAngle Phi CurrentSpeed Angle BikeAngle
clear Command Phi_dot targetAccek targetSpeed TargetSpeed tout targetAccel
clear stepVal

%% ----- Contr�leur du moteur -----
%% Contr�leur du moteur
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

fprintf("---- Carat�ristiques en BF ----\n");
H_bf
disp("P�les: ")
disp(pole(H_bf))
disp("Gain statique: ")
disp(dcgain(H_bf))
disp("Z�ro: ")
disp(zero(H_bf))
disp("Temps de r�ponse (2%): ")
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


%% R�ponse � une Rampe, TODO corrig�
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
    plot(Time, FW_Speed, 'r', 'DisplayName', 'Exp�rimental')
%     plot(Time, FW_Target_Speed, 'k', 'DisplayName', 'Target Speed')
end
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Th�orique')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'k', 'DisplayName', 'Vitesse d�sir�')
legend
title("Comparaison de la r�ponse th�orique et exp�rimentale")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
xlim([0 0.7])
grid on
hold off

% subplot(1, 2, 2)
% hold on
% if plotExp == true
%     plot(Time, FW_Command, 'r', 'DisplayName', 'Exp�rimental')
% end
% plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Th�orique')
% legend
% title("Tension")
% xlabel("Temps (sec)")
% ylabel("Tension [V]")
% grid on
% hold off

clear plotExp