% Fichier contenant les param�tres du syst�me de v�lo auto�quilibr�
% Auteur: Charles Sirois
% Date: 21/04/2020

%% Param�tres du syst�me
clc
close all

%V�lo
Mv = 331/1000;              %Masse du v�lo [kg]
Lv = 4.8/100;               %Distance sol-CM [m]
Jv_cm = 371561.95/(1000^3); %Inertie du v�lo [Kg*m^2]
Jv = Jv_cm + Mv*Lv^2;       
Jv = 0.0003;                %Inertie obtenue de fa�on exp�rimentale                

disp("Jv: ")
disp(Jv)

%Roue inertielle
Mp = 17/1000;         %Masse de la roue en plastique [kg]
Jp = 14826/(1000^3);    %Inertie du plastique[Kg*m^2]


Mm = 74/1000;   %Masse du washer en m�tal [kg]   
Rm = 32/1000;   %Rayon total du washer [m]
rm = 14/1000;   %Rayon int�rieur du washer [m]
Jm = Mm/2*(Rm^2 + rm^2);

Mr = 165/1000;  %Masse de la roue [kg]   
Lr = 9/100;     %Distance sol-CM roue [m]
Jr = 2*Jm+Jp;   %Inertie de la roue [Kg*m^2], pas la bonne valeur
Jr = 1.485e-04; %Inertie obtenue de fa�on exp�rimentale

disp("Jr: ")
disp(Jr)


%Moteur
Km = 1311.7; %Gain statique
Tau_m = 0.205;

g = 9.81;   %Acc�l�ration gravitationnel [m/s^2]

% Fonctions de transfert
s = tf('s');
H = Km/ ( Tau_m*s + 1 ); %phi_dot selon U [deg]
G = (-Jr)/( (Jv+Jr+Mv*Lv^2+Mr*Lr^2)*s^2 - (Mv*Lv+Mr*Lr)*g); %Theta(s)/Phi_dot_dot(s)  => Angle du v�lo selon accel de la roue

% Clean workspace
clear Jm Jp Jv_cm Mm Mp rm Rm 


