clc
theta_max = 15*pi/180;

%Vélo
Mv = 265.41/1000;            %Masse du vélo [kg]
Lv = 4/100;                  %Distance sol-CM [m]
Jv = 224468.05/(1000^3);     %Inertie du vélo [Kg*m^2]

%Roue inertielle
Mr = 76.4/1000;         %Masse de la roue [kg]
Lr = 9/100;             %Distance sol-CM roue [m]
Jr = 228418/(1000^3);    %Inertie de la roue [Kg*m^2]

couple_max = (Mv*Lv + Mr*Lr)*g*sin(theta_max);
couple_max = couple_max*100/g; %[kg*cm]

disp("Couple max [kg*cm]: ")
disp(couple_max)
