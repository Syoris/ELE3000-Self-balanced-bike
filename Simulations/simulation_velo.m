%% Paramètres du système
clc
close all

%Vélo
Mv = 265.41/1000;            %Masse du vélo [kg]
Lv = 4/100;                  %Distance sol-CM [m]
Jv = 224468.05/(1000^3);     %Inertie du vélo [Kg*m^2]

%Roue inertielle
Mr = 15.73/100;         %Masse de la roue [kg]
Lr = 6/100;             %Distance sol-CM roue [m]
Jr = 14826/(1000^3);    %Inertie de la roue [Kg*m^2]

%Moteur
Km = 282.7054; %Gain statique
Tau_m = 0.04;

%Autres
Theta = 0;  %Angle du vélo par rapport à la vertical
Phi = 0;    %Angle de la roue inertielle
Tau = 0;    %Couple du moteur
g = 9.81;   %Accélération gravitationnel [m/s^2]

% Fonctions de transfert
s = tf('s');
H = Km/ ( Tau_m*s + 1 ); %phi_dot selon U
G = (-Jr*s)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot(s)  => Angle du vélo selon vitesse roue inertielle

% ----- Contrôleur du vélo ----- 
% Pôles alignés
clc
close all

A = Mv*Lv^2 + Mr*Lr^2;
B = (Mv*Lv + Mr*Lr)*g;

p = 10; % Pôles à -10+/-10j
p1 = s+p+p*1i;
p2 = s+p-p*1i;
poles = p1*p2;

[num_v, den_v] = tfdata(poles, 'v');
a_v = num_v(2);
b_v = num_v(3);

Kp_v = a_v/Jr;
Ki_v = (b_v-B)/Jr;
Kd_v = (1+A+Jv)/Jr - 1;

fprintf("---- Gains du vélo ----\n");
fprintf("\tKp: %4.2f\n", Kp_v)
fprintf("\tKi: %4.2f\n", Ki_v)
fprintf("\tKd: %4.2f\n", Kd_v)

G2 = feedback(G, Kd_v*s);
G_bf = minreal(feedback(G2*(Kp_v + Ki_v/s), 1));


