% Fichier pour tester le syst�me en simulation
% Auteur: Charles Sirois
% Date: 21/04/2020
%
% Guide d'utilisation:
% 1 - Ex�cuter le fichier parametres.m pour load les params du moteur
% 2 - Ex�cuter les sections d�sir�es:
%       - Perturbations: Test la capacti� du contr�leur � rejeter des
%                         perturbations
%       - Angle de d�part: Test l'angle de d�part maximal

%% Test de validation - Perturbations
clc
close all

simTime = 1.5;
AngleInitial = 0;

% p = 0.0052;   %1 deg  
% p = 0.0155;   %3 deg
% p = 0.0255;   %5 deg
% p = 0.029;    %5.8 deg

sim("Simulink/Velo_Complet_BF")

% Plot results
figure
hold on
plot(Theta_BF(:, 1), Theta_BF(:, 2), 'r')
title("R�ponse � une perturbation de 3 degr�s")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

%% Test de validation - Angle de d�part
clc
close all

simTime = 1;
p = 0;

% AngleInitial = 1;
% AngleInitial = 3;
% AngleInitial = 5;
% AngleInitial = 8;
% AngleInitial = 8.4;
% AngleInitial = 8.8;
% AngleInitial = 9;

sim("Simulink/Velo_Complet_BF")

% Plot results
figure
hold on
plot(Theta_BF(:, 1), Theta_BF(:, 2), 'r')
title("R�ponse � un angle de d�part de 8.4 degr�s")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off