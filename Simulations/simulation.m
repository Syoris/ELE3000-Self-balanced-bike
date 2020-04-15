%% Paramètres du système
clc
close all

%Vélo
Mv = 331/1000;            %Masse du vélo [kg]
Lv = 4.8/100;                  %Distance sol-CM [m]
Jv_cm = 371561.95/(1000^3);     %Inertie du vélo [Kg*m^2]
Jv = Jv_cm + Mv*Lv^2;
Jvi = Jv;
Jv = 0.0003;

disp("Jv: ")
disp(Jv)

%Roue inertielle
Mp = 17/1000;         %Masse de la roue en plastique [kg]
Jp = 14826/(1000^3);    %Inertie du plastique[Kg*m^2]


Mm = 74/1000;   %Masse du washer en métal [kg]   
Rm = 32/1000;   %Rayon total du washer [m]
rm = 14/1000;   %Rayon intérieur du washer [m]
Jm = Mm/2*(Rm^2 + rm^2);

Mr = 165/1000;  %Masse de la roue [kg]   
Lr = 9/100;     %Distance sol-CM roue [m]
Jr = 2*Jm+Jp;   %Inertie de la roue [Kg*m^2]
Jri = Jr;

% À partir de Velo_BO_Mot_0x data
Jr2 = 0.000091;
Jr3 = 0.0000865; 
Jr4 = 0.0000855;
Jr5 = 0.0000855;
Jr6 = 0.000083;
Jr_exp = mean([Jr2 Jr3 Jr4 Jr5 Jr6]);
% Jr = Jr_exp;
Jr = 1.485e-04;

disp("Jr: ")
disp(Jr)


%Moteur

% % M1: 6V avec poids
% Km = 252.5; %Gain statique
% Tau_m = 0.031;

% M2: 6V avec poids
Km = 1311.7; %Gain statique
Tau_m = 0.205;

g = 9.81;   %Accélération gravitationnel [m/s^2]

% Fonctions de transfert
s = tf('s');
H = Km/ ( Tau_m*s + 1 ); %phi_dot selon U [deg]

G = (-Jr*s)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot(s)  => Angle du vélo selon vitesse roue inertielle
G_a = (-Jr)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot_dot(s)  => Angle du vélo selon accel de la roue



