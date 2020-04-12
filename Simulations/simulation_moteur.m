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
suptitle("R�ponse � un �chellon de 6V")

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

%% ----- Contr�leur du moteur -----
%% PID v2 - Contr�leur choisi
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

%% R�ponse � un �chellon
load(fullfile("PythonData", 'Moteur_BF_C2_S_5000'))
simTime = Time(end);
targetAccel = 20000;
targetSpeed = 5000;
sim("Simulink/Moteur_BF_v2")

figure
suptitle("PID")
subplot(1, 2, 1)
hold on
plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Exp�rimental')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Th�orique')
plot(Time, TargetSpeed, 'k', 'DisplayName', 'Target Speed')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

subplot(1, 2, 2)
hold on
plot(Time, Command, 'r', 'DisplayName', 'Exp�rimental')
plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Th�orique')
legend
title("Tension")
xlabel("Temps (sec)")
ylabel("Tension [V]")
grid on
hold off

%% R�ponse � une Rampe
clc
close all
plotExp = 1;

if plotExp == true
    load(fullfile("PythonData", 'Moteur_BF_C3_15000'))
    simTime = Time(end);
end

targetAccel = 15000;
targetSpeed = 8000;
sim("Simulink/Moteur_BF_v2")

figure
suptitle("PID")
subplot(1, 2, 1)
hold on
if plotExp == true
    plot(Time, CurrentSpeed, 'r', 'DisplayName', 'Exp�rimental')
    % plot(Time, TargetSpeed, 'k', 'DisplayName', 'Target Speed')
end
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Th�orique')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'k', 'DisplayName', 'Vitesse d�sir�')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitesse [deg/sec]")
grid on
hold off

subplot(1, 2, 2)
hold on
if plotExp == true
    plot(Time, Command, 'r', 'DisplayName', 'Exp�rimental')
end
plot(U(:, 1), U(:, 2), 'b', 'DisplayName', 'Th�orique')
legend
title("Tension")
xlabel("Temps (sec)")
ylabel("Tension [V]")
grid on
hold off