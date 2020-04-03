%% Param�tres du syst�me
clc
close all

%V�lo
Mv = 331/1000;            %Masse du v�lo [kg]
Lv = 4/100;                  %Distance sol-CM [m]
Jv = 224468.05/(1000^3);     %Inertie du v�lo [Kg*m^2]

%Roue inertielle
Mp = 17/1000;         %Masse de la roue en plastique [kg]
Jp = 14826/(1000^3);    %Inertie du plastique[Kg*m^2]

Mm = 74/1000;   %Masse du washer en m�tal [kg]   
Rm = 32/1000;   %Rayon total du washer [m]
rm = 14/1000;   %Rayon int�rieur du washer [m]
Jm = Mm/2*(Rm^2 + rm^2);

Mr = 165;
Lr = 9/100;             %Distance sol-CM roue [m]
Jr = 2*Jm+Jp;    %Inertie de la roue [Kg*m^2]

%Moteur

% % M1: 6V avec poids
% Km = 252.5; %Gain statique
% Tau_m = 0.031;

% M2: 6V avec poids 
Km = 1353; %Gain statique
Tau_m = 0.274;

%Autres
Theta = 0;  %Angle du v�lo par rapport �la vertical
Phi = 0;    %Angle de la roue inertielle
Tau = 0;    %Couple du moteur
g = 9.81;   %Acc�l�ration gravitationnel [m/s^2]

% Fonctions de transfert
s = tf('s');
H = Km/ ( Tau_m*s + 1 ); %phi_dot selon U
G = (-Jr*s)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot(s)  => Angle du v�lo selon vitesse roue inertielle
G_a = (-Jr)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot_dot(s)  => Angle du v�lo selon accel de la roue

%% ----- Contr�leur du moteur -----
%% Designeur de Matalab
[num_m, den_m] = tfdata(Cm, 'v');
Kp_m = num_m(1);
Kd_m = 0; %num_m(1);
Ki_m = num_m(2);

%% P�les align�s
close all
clc

p = 100;
p1 = s+p+p*1i;
p2 = s+p-p*1i;
poles = p1*p2;

[num_m, den_m] = tfdata(poles, 'v');
Kp_m = (num_m(2) - 1)/Km;
Ki_m = num_m(3)/Km;
Kd_m = (1-Tau_m)/Km;

fprintf("---- Gains du moteur ----\n");
fprintf("\tKp: %4.4f\n", Kp_m)
fprintf("\tKi: %4.4f\n", Ki_m)
fprintf("\tKd: %4.4f\n", Kd_m)  

ramp = 35000;
sim("Simulink/Moteur_BF")
dataList = {Phi_dot_des 'r' 'Commande [deg/sec]'; 
            Phi_dot 'b' 'Vitesse [deg/s]'};
            %Phi 'g' 'Position [deg]'};
dataOpt = {U 'k' 'Tension [V]'};

plot_func('Moteur en BF', 'Temps (s)', '', dataList, dataOpt);



%% ----- Contr�leur du v�lo ----- 
%% Designeur de Matlab
clc 
close all
fprintf("Par Matlab\n");
load Cv

[num_v, ~] = tfdata(Cv, 'v');
Kp_v = num_v(2);
Kd_v = num_v(1);
Ki_v = num_v(3);

fprintf("---- Gains du v�lo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G, Kd_v*s);
G_bf_mt = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Carat�ristiques en BF ----\n");
G_bf_mt
disp("P�les: ")
disp(pole(G_bf_mt))
disp("Gain statique: ")
disp(dcgain(G_bf_mt))
disp("Z�ro: ")
disp(zero(G_bf_mt))

%% P�les align�s
close all
clc
fprintf("M�thode par p�les align�s\n");
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

% p_r = 97.1572;
% p_i = 45.5004;
p_r = 5;
p_i = 5;
p1 = s+p_r+p_i*1i;
p2 = s+p_r-p_i*1i;
poles = p1*p2;

[num_v, den_v] = tfdata(poles, 'v');
a_v = num_v(2);
b_v = num_v(3);

Kp_v = a_v/Jr;
Ki_v = (b_v-B)/Jr;
Kd_v = (1+A+Jv)/Jr - 1;

fprintf("---- Gains du v�lo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Carat�ristiques en BF ----\n");
G_bf
disp("P�les: ")
disp(pole(G_bf))
disp("Gain statique: ")
disp(dcgain(G_bf))
disp("Z�ro: ")
disp(zero(G_bf))

% G_bf2 = (Jr*(Kp_v*s+Ki_v))/( -(Jv + Jr*(1-Kd_v)+A)*s^2 + Kp_v*Jr*s + (Ki_v*Jr+B) );

% sim("Simulink/Moteur_BF")
% dataList = {Phi_dot_des 'r' 'Commande [deg/sec]'; 
%             Phi_dot 'b' 'Vitesse [deg/s]'};
%             %Phi 'g' 'Position [deg]'};
% dataOpt = {U 'k' 'Tension [V]'};
% 
% plot_func('Moteur en BF', 'Temps (s)', '', dataList, dataOpt);

%% Matlab en accel
clc 
close all
fprintf("Par Matlab\n");
load C_accel

[num_v, ~] = tfdata(C_accel2, 'v');
Kp_v = num_v(2);
Kd_v = num_v(1);
Ki_v = num_v(3);

fprintf("---- Gains du v�lo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G_a, Kd_v*s);
G_bf_mt = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

fprintf("\n---- Carat�ristiques en BF ----\n");
G_bf_mt
disp("P�les: ")
disp(pole(G_bf_mt))
disp("Gain statique: ")
disp(dcgain(G_bf_mt))
disp("Z�ro: ")
disp(zero(G_bf_mt))

%% P�les align�s en accel
close all
clc
fprintf("M�thode par p�les align�s\n");
A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

p_r = 0.1;
p_i = 0.1;
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

fprintf("---- Gains du v�lo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G_a, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));

% fprintf("\n---- Carat�ristiques en BF ----\n");
% G_bf
% disp("P�les: ")
% disp(pole(G_bf))
% disp("Gain statique: ")
% disp(dcgain(G_bf))
% disp("Z�ro: ")
% disp(zero(G_bf))

%% Simulation V�lo, sans moteur.
clc
close all
sim("Simulink/Velo_BF")
dataList = {Theta 'r' 'Angle v�lo [deg]'};
            
dataOpt = {Phi_dot_des 'k' 'Vitesse des [deg/sec]';
           Phi_dot_dot_des 'b' 'Accel des [deg^2/sec]'};

plot_func('Velo BF', 'Temps (s)', '', dataList, dataOpt);

fprintf("---- Maximum ----\n");
fprintf("\tVitesse max [deg/sec]: %6.2f\n", max(Phi_dot_des(:, 2)))
fprintf("\tAccel max [deg/sec]: %6.2f\n", max(Phi_dot_dot_des(:, 2)))




