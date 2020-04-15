%% Test de validation
clc
close all

simTime = 1.5;
AngleInitial = 0;

% p = 0.0052;   %1 deg  
% p = 0.0155;   %3 deg
% p = 0.0255;   %5 deg
p = 0.029;    %5.8 deg

sim("Simulink/Velo_Complet_BF")

% Plot results
figure
hold on
plot(Theta_BF(:, 1), Theta_BF(:, 2), 'b')
title("Réponse à une perturbation de 5,8 degrés")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off