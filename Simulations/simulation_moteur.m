%% ====== Moteur =====
%% ----- Identification -----
%% Step Response 6V
clc
close all
load(fullfile("PythonData", 'step6V_25'))
simTime = Time(end);
stepVal = str2double(StepAmplitude);
sim("Simulink/Moteur_BO")

figure
suptitle("Réponse à un échellon de 6V")

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
%% PID v2 - Contrôleur choisi
close all
clc

% C1: p = 10
% C2: p = 25

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

%% Réponse à un échellon
load(fullfile("PythonData", 'Moteur_BF_C2_S_5000'))
simTime = Time(end);
targetAccel = 20000;
targetSpeed = 5000;
sim("Simulink/Moteur_BF_v2")

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
clc
close all

load(fullfile("PythonData", 'Moteur_BF_C3_15000'))
simTime = Time(end);
targetAccel = 15000;
targetSpeed = 8000;
sim("Simulink/Moteur_BF_v2")

figure
suptitle("PID")
subplot(1, 2, 1)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Expérimental')
% plot(Time, TargetSpeed, 'k', 'DisplayName', 'Target Speed')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Théorique')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'k', 'DisplayName', 'Vitesse désiré')
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