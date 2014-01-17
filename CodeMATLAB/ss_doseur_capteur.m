

ss_capteur_temperature = ss(tf([1],[tth 1]));
A_capteur_temperature = -1.2500;
B_capteur_temperature = 1.2500;
C_capteur_temperature = 1;
D_capteur_temperature = 0;

ss_capteur_temperature = ss(A_capteur_temperature, B_capteur_temperature, C_capteur_temperature, D_capteur_temperature);

ss_doseur = ss(tf([1],[tdos 1]));
A_doseur = -18.5185;
B_doseur = 18.5185;
C_doseur = 1;
D_doseur = 0;

% [ss_doseur,T] = canon(ss_doseur,'companion');
% 
% ss_doseur = ss(A_doseur, B_doseur, C_doseur, D_doseur);