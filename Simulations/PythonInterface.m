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
load(fullfile("PythonData", 'step6V'))
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

%% Contrôleur PD