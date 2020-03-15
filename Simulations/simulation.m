%% Param�tres du syst�me
clc
close all

%V�lo
Mv = 265.41/1000;            %Masse du v�lo [kg]
Lv = 4/100;                  %Distance sol-CM [m]
Jv = 224468.05/(1000^3);     %Inertie du v�lo [Kg*m^2]

%Roue inertielle
Mr = 15.73/100;         %Masse de la roue [kg]
Lr = 6/100;             %Distance sol-CM roue [m]
Jr = 14826/(1000^3);    %Inertie de la roue [Kg*m^2]

%Moteur
K_m = 282.7054; %Gain statique
Tau_m = 0.04;

%Autres
Theta = 0;  %Angle du v�lo par rapport �la vertical
Phi = 0;    %Angle de la roue inertielle
Tau = 0;    %Couple du moteur
g = 9.81;   %Acc�l�ration gravitationnel [m/s^2]

% Moteur - TF
s = tf('s');
H = K_m/ ( Tau_m*s + 1 ); %phi_dot selon U

[num_m, den_m] = tfdata(Cm, 'v');
Kp_m = num_m(1);
Kd_m = 0;%num_m(1);
Ki_m = num_m(2);

% V�lo - TF
G = (-Jr*s)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot(s)  => Angle du v�lo selon vitesse roue inertielle
load Cv

[num_v, den_v] = tfdata(Cv, 'v');
Kp_v = num_v(2);
Kd_v = num_v(1);
Ki_v = num_v(3);

