%% ======= V�lo ======
%% ----- Identification -----
%% Velo BO_01
clc
close all

% Sans accel�ration
load(fullfile("PythonData", 'Velo_BO'))
simTime = 0.2;
startVal = 20;
nVal = 8;


AngleInitial = BikeAngle(startVal);
AccelVal = 0;
sim("Simulink/Velo_BO")

figure
suptitle("V�lo en BO")
subplot(1,2, 1)
hold on
plot(Time(1:nVal+1), BikeAngle(startVal:startVal+nVal), 'r', 'DisplayName', 'Angle exp�rimentale')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle th�orique')
legend
title("Sans acc�l�ration")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Avec acc�l�ration
load(fullfile("PythonData", 'Velo_BO_Ramp_20000'))
endVal = 46;
simTime = Time(endVal);

AngleInitial = BikeAngle(1);
AccelVal = -20000;

sim("Simulink/Velo_BO")

subplot(1, 2, 2)
hold on
plot(Time(1:endVal), BikeAngle(1:endVal), 'r', 'DisplayName', 'Angle (exp)')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle (th�o)')
legend
title("Avec acc�l�ration")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

%% Velo BO_02
clc
close all

% Sans accel�ration
load(fullfile("PythonData", 'Velo_BO_02'))

startVal = 11;
nVal = 15;
simTime = Time(nVal+1);

AngleInitial = BikeAngle(startVal);
AccelVal = 0;
sim("Simulink/Velo_BO")

figure
suptitle("V�lo en BO")
subplot(1, 3, 1)
hold on
plot(Time(1:nVal+1), BikeAngle(startVal:startVal+nVal), 'r', 'DisplayName', 'Angle exp�rimentale')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle th�orique')
legend
title("Sans acc�l�ration")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Avec acc�l�ration
load(fullfile("PythonData", 'Velo_BO_Mot_02'))
startVal = 1;
nVal = 22;
simTime = Time(nVal+1);

AngleInitial = BikeAngle(startVal);
AccelVal = -20000;

sim("Simulink/Velo_BO_Moteur")

subplot(1, 3, 2)
hold on
plot(Time(1:nVal+1), BikeAngle(startVal:startVal+nVal), 'r', 'DisplayName', 'Angle (exp)')
plot(Theta_BO(:, 1), Theta_BO(:, 2), 'b', 'DisplayName', 'Angle (th�o)')
legend
title("Avec acc�l�ration")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Comparaison des vitesses
subplot(1, 3, 3)
hold on
plot(Time(1:nVal+1), CurrentSpeed(startVal:startVal+nVal), 'r', 'DisplayName', 'Vitesse (exp)')
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse (th�o)')
legend
title("Comparaison des vitesses du moteur")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off


%% ----- Contr�leur du v�lo ----- 
%% P�les align�s en accel
close all
clc
fprintf("M�thode par p�les align�s\n");
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

p_r = 5;
p_i = 5;
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

G2 = feedback(G_a, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Carat�ristiques en BF ----\n");
G_bf
disp("P�les: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Z�ro: ")
disp(zero(G_bf))

fprintf("---- Gains du v�lo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

%% Simplification p�les/z�ro
close all
clc
fprintf("M�thode par p�les align�s\n");
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

syms Kps Kis Kds

a = 10;

eqn1 = 2*a + Kis/Kps == -(Jr*Kds)/(Jv+Jr+A);
eqn2 = 2*a^2 + 2*a*Kis/Kps == -(B+Jr*Kps)/(Jv+Jr+A);
eqn3 = (2*a^2*Kis)/Kps == -(Jr*Kis)/(Jv+Jr+A);

sol = solve([eqn1, eqn2, eqn3], [Kps, Kis, Kds]);

i = 2;
Kp_v = double(sol.Kps(i));
Ki_v = double(sol.Kis(i));
Kd_v = double(sol.Kds(i));

fprintf("---- Gains du v�lo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G_a, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Carat�ristiques en BF ----\n");
G_bf
disp("P�les: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Z�ro: ")
disp(zero(G_bf))

%% PD
close all
clc
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

% C1: p =
% C2: p =

p = 5;

syms Kps Kds

eqn1 = 2*p == -(Kds*Jr)/(Jr+Jv+A);
eqn2 = 2*p^2 == -(B+Kps*Jr)/(Jr+Jv+A);

sol = solve([eqn1, eqn2], [Kps, Kds]);

i = 1;
Kp_v = double(sol.Kps(i));
Kd_v = double(sol.Kds(i));
Ki_v = 0;

G2 = feedback(G_a, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("---- Carat�ristiques en BF ----\n");
G_bf
disp("P�les: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Z�ro: ")
disp(zero(G_bf))
disp("Temps de r�ponse (2%): ")
disp(4/p);

fprintf("\n---- Gains du v�lo ----\n");
fprintf("\tKp: %4.6f\n", Kp_v)
fprintf("\tKd: %4.6f\n", Kd_v)

%% Simulation V�lo complet
clc
close all

plotExp = 0;
nData = 75;

if plotExp == true
    load(fullfile("PythonData", 'Velo_PD_03'))
    simTime = Time(nData);
    AngleInitial = BikeAngle(1);
else
    simTime = 2;
    AngleInitial = 7.5;
end



sim("Simulink/Velo_Complet_BF")
sim("Simulink/Velo_BF")

% Plot results
figure
suptitle("V�lo Complet")
% Angle
subplot(3, 1, 1)
hold on
%   Exp data
if plotExp == true
    plot(Time(1:nData), BikeAngle(1:nData), 'r', 'DisplayName', 'Exp�rimental')
end
%   Th�o data
plot(Theta_BF(:, 1), Theta_BF(:, 2), 'b', 'DisplayName', 'Th�orique')
% plot(Theta_BF_nm(:, 1), Theta_BF_nm(:, 2), 'k', 'DisplayName', 'Sans moteur')
legend
title("Position")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Vitesse flywheel
subplot(3, 1, 2)
hold on
%   Exp data
if plotExp == true
    plot(Time(1:nData), CurrentSpeed(1:nData), 'r', 'DisplayName', 'Vitesse (exp)')
    plot(Time(1:nData), TargetSpeed(1:nData), 'r--', 'DisplayName', 'Vitesse d�sir�e (ex)')
end
%   Theo Data
plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse actuelle (theo)')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'b--', 'DisplayName', 'Vitesse d�sir�e (theo)')
% plot(Phi_dot_des_nm(:, 1), Phi_dot_des_nm(:, 2), 'k--', 'DisplayName', 'Vitesse d�sir�e sans moteur') % Simu parfaite
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitessse [deg/sec]")
grid on
hold off

% Vitesse v�lo
subplot(3, 1, 3)
hold on
%   Exp data
if plotExp == true
    plot(Time(1:nData), AngularVel(1:nData), 'r', 'DisplayName', 'Vitesse (exp)')
end
%   Theo Data
plot(Theta_dot_BF(:, 1), Theta_dot_BF(:, 2), 'b', 'DisplayName', 'Vitesse avec moteur')
% plot(Theta_dot_BF_nm(:, 1), Theta_dot_BF_nm(:, 2), 'k', 'DisplayName', 'Vitesse sans moteur')
legend
title("Vitesse du v�lo")
xlabel("Temps (sec)")
ylabel("Vitessse [deg/sec]")
grid on
hold off

fprintf("---- Maximum ----\n");
fprintf("\tVitesse max [deg/sec]: %6.2f\n", max(abs(Phi_dot_des(:, 2))))
fprintf("\tAccel max [deg/sec]: %6.2f\n", max(abs(Phi_dot_dot_des(:, 2))))

%% Comparaison exp�rimental
clc
close all

n1 = "07";
n2 = "08";
nData = 100;

% Plot results
figure
suptitle("V�lo Complet")
% Angle
subplot(2, 1, 1)
hold on
load(fullfile("PythonData", 'Velo_Exp_'+n1))
plot(Time(1:nData), Bike_Angle(1:nData), 'r', 'DisplayName', 'Angle Exp 1')
load(fullfile("PythonData", 'Velo_Exp_'+n2))
plot(Time(1:nData), Bike_Angle(1:nData), 'b', 'DisplayName', 'Angle Exp 2')
legend
title("Position")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Vitesse flywheel
subplot(2, 1, 2)
hold on
%   Exp data
load(fullfile("PythonData", 'Velo_Exp_'+n1))
plot(Time(1:nData), FW_Speed(1:nData), 'r', 'DisplayName', 'Vitesse (exp 1 )')
plot(Time(1:nData), FW_Target_Speed(1:nData), 'r--', 'DisplayName', 'Vitesse d�sir�e (exp 1)')
load(fullfile("PythonData", 'Velo_Exp_'+n2))
plot(Time(1:nData), FW_Speed(1:nData), 'b', 'DisplayName', 'Vitesse (exp 1 )')
plot(Time(1:nData), FW_Target_Speed(1:nData), 'b--', 'DisplayName', 'Vitesse d�sir�e (exp 1)')
legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitessse [deg/sec]")
grid on
hold off


%% Comparaison exp�rimental, th�orique
clc
close all

n = "11";
nData = 400;
load(fullfile("PythonData", 'Velo_Exp_'+n))

Kp_v = -4300;
Ki_v = 0;
Kd_v = -320;

AngleInitial = Bike_Angle(1);
simTime = Time(nData);
sim("Simulink/Velo_Complet_BF")

% Plot results
figure
suptitle("V�lo Complet")
% Angle
subplot(2, 1, 1)
hold on
plot(Time(1:nData), Bike_Angle(1:nData), 'r', 'DisplayName', 'Angle Exp 1')
plot(Theta_BF(:, 1), Theta_BF(:, 2), 'b', 'DisplayName', 'Th�orique')
legend
title("Position")
xlabel("Temps (sec)")
ylabel("Angle [deg]")
grid on
hold off

% Vitesse flywheel
subplot(2, 1, 2)
hold on
%   Exp data
plot(Time(1:nData), FW_Speed(1:nData), 'r', 'DisplayName', 'Vitesse (exp 1 )')
plot(Time(1:nData), FW_Target_Speed(1:nData), 'r--', 'DisplayName', 'Vitesse d�sir�e (exp 1)')

plot(Phi_dot(:, 1), Phi_dot(:, 2), 'b', 'DisplayName', 'Vitesse actuelle (theo)')
plot(Phi_dot_des(:, 1), Phi_dot_des(:, 2), 'b--', 'DisplayName', 'Vitesse d�sir�e (theo)')

legend
title("Vitesse")
xlabel("Temps (sec)")
ylabel("Vitessse [deg/sec]")
grid on
hold off