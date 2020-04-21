% Fichier pour tester le système en simulation
% Auteur: Charles Sirois
% Date: 21/04/2020
%
% Guide d'utilisation:
% 1 - Exécuter le fichier parametres.m pour load les params du moteur
% 2 - Exécuter les sections désirées:
%       - Perturbations: Test la capactié du contrôleur à rejeter des
%                         perturbations
%       - Angle de départ: Test l'angle de départ maximal

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
title("Réponse à une perturbation de 3 degrés")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

%% Test de validation - Angle de départ
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
title("Réponse à un angle de départ de 8.4 degrés")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off