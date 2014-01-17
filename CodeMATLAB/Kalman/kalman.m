clear all;
close all;
clc;

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('%%%                                      %%%');
disp('%%%    SCRIPT MATLAB DU BUREAU D ETUDES  %%%');
disp('%%%                                      %%%');
disp('%%%         -- Filtre de Kalman --       %%%');
disp('%%%          -- Prof. Léa Cot --         %%%');
disp('%%%               -- V.02 --             %%%');
disp('%%%                                      %%%');
disp('%%% Auteurs:                             %%%');   
disp('%%%          Martin PETROV               %%%');
disp('%%%          Julien MAFFRE               %%%');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');


%% Estimation d'une constante aléatoire (3.2) :
disp('-');
disp('-');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('%%%                                      %%%');
disp('%%%   Filtre de Kalman discret linéaire  %%%');
disp('%%%                                      %%%');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

% -- Generate random constant (mesure) : 
Z = [] ;
mean_Z = 0 ; 
for i = 1:50
    Z = [Z normrnd(mean_Z,0.1)] ;    
end

% Système :
n = 1 ;         % Dimension de l'état
A = eye(n) ;    % Matrice dynamique
B = 0 ;         % Matrice d'entrée
C = 1 ;         % Matrice de sortie

% Bruits : 
Q = 1e-5 ;
X_0 = 0 ;
P_0 = 1 ;
R_table = [0.01 1 0.0001] ; 

for R = R_table 
    % Iterative Kalman : 
    K = [] ;
    X = [] ;
    P = [] ;

    X_before = X_0 ;
    P_before = P_0 ;
    for k = 1:50
        X_next = A*X_before + Q ;                   % Prediction
        X = [X X_next] ;

        P_next = A*P_before*A' + Q ;                % Variance de l'erreur
        I_k = Z(k) - C*X_before ;                   % Innovation
        K_next = P_next*C'*inv(C*P_next*C' + R) ;   % Calcul du gain
        X_next = X_next + K_next * (I_k) ;          % Estimation
        P_next = (eye(n) - K_next*C) * P_next ;     % Valeur de l'erreur d'estimation

        X_before = X_next ; 
        P_before = P_next ; 
        K = [K K_next] ;
        P = [P P_next] ;
    end

    % Tirage de la valeur exacte (en gros, que des 0) : 
    valeur_exacte = [] ; 
    for j = 1:50
       valeur_exacte = [valeur_exacte mean_Z] ; 
    end

    figure('Name' , '3.2 : Estimation d une constante aléatoire','NumberTitle','off');
        subplot(1,3,1), hold on, plot(valeur_exacte, 'g', 'linewidth', 2), plot(X, 'r', 'linewidth', 2), plot(Z, 'b--'), xlabel('# échantillon'), ylabel('Valeur tirée'),  title('Evolution de l état estimé par rapport à la constante recherchée') ; 
        legend('Valeur exacte',  'Etat', 'Mesures') ;
        subplot(1,3,2), hold on, plot(Z-X, 'b'), plot(3*sqrt(P), 'm','linewidth', 2), plot(-3*sqrt(P), 'm','linewidth', 2) , xlabel('# échantillon'), title('Evolution de l erreur d estimation') ; 
        legend('Evolution de l erreur d estimation') ; 
        subplot(1,3,3), hold on, plot(K, 'b', 'linewidth', 2),  xlabel('# échantillon'), title('Evolution du gain du filtre de Kalman') ; 
        legend('Gain K du filtre') 
end
    
%% Estimation d'une récurrence (3.3) : 

% Xn_1 = F(Xn) = 1 - 2*Xn 

X_0 = 0 ;
X = [X_0] ;
X_before = X_0 ;
for i = 1:50
    X_next = 1-2*X_before ; 
    X = [X X_next] ;
    X_before = X_next ;
end 

figure('Name' , '3.3 : Estimation d une récurence','NumberTitle','off');
    plot(X)
% Commentaires : système instable (prévisible car le seul pôle du système
% discret est -2 (qui n'appartient pas au cercle de stabilité discret).

%% Filtre de Kalman étendu (EKF) : 
disp('-');
disp('-');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('%%%                                      %%%');
disp('%%%      Filtre de Kalman étendu         %%%');
disp('%%%                                      %%%');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

% Suite des itérés : 
eps1 = 1e-14 ;
eps2 = 2*eps1 ;

x1 = [] ;
x2 = [] ;
x1_0 = 0.5 ;
x2_0 = 0.5 ;
x1_before = x1_0 ;
x2_before = x2_0 ;

for i=1:50
    x1_next = (1-eps1)*(1-2*(x1_before)^4) + eps1*(1-2*(x2_before)^4) ; 
    x2_next = eps2*(1-2*(x1_before)^4) + (1-eps2)*(1-2*(x1_before)^4) ;

    x1_before = x1_next ;
    x2_before = x2_next ; 
    
    x1 = [x1 x1_next] ;
    x2 = [x2 x2_next] ;
end 

figure('Name' , '4.1 : Algorithme et programme (EKF)','NumberTitle','off');
hold on ;
plot(x1, 'b') ;

%% Filtre de Kalman Etendu : Estimation du système
n = 2 ;

% Bruits : 
W = 1e-5 * [1 ; 1] ;
Q = 1e-5 * eye(n) ; 

R_table = [0.01 1 0.0001] ; 

for R_i = R_table 

R = R_i * eye(n) ;
P_0 = eye(2) ;

% Kalman étendu : 
K = [] ;
X = [] ;
P = [] ;
Z= [] ; 

% Système : 
% % % % Xk_hat = g(Xk-1_hat, Uk-1) + K(Yk - h(Xk_hat)
% % % % Yk = C*Xh + Vk
C = eye(2) ; 

X_before = [x1_0 ; x2_0] ;
P_before = P_0 ;
for k = 1:50
    % Calcul de la Jacobienne : 
    x1 = X_before(1) ;
    x2 = X_before(2) ;
    Jac_11 = -8*x1^3*(1-eps1) ; 
    Jac_12 = -8*x2^3*(eps1) ;
    Jac_21 = -8*x1^3*(eps2) ;
    Jac_22 = -8*x2^3*(1-eps2) ;
    J_A = [Jac_11 Jac_12 ; Jac_21 Jac_22] ;

    X_next = J_A*X_before + W ;                     % Prediction
    P_next = J_A*P_before*J_A' + Q ;                % Variance de l'erreur
    Z_mesure = normrnd(0,0.1) ;

    I_k = Z_mesure - C*X_next ;                     % Innovation
    K_next = P_next * C'*inv(C*P_next*C' + R) ;     % Calcul du gain
    X_next = X_next + K_next * (I_k) ;              % Estimation
    P_next = (eye(n) - K_next*C) * P_next ;         % Valeur de l'erreur d'estimation

    X_before = X_next ; 
    P_before = P_next ; 

    X = [X X_next(2,1)] ;
    Z = [Z Z_mesure] ;
    K = [K K_next(1,1)] ;
    P = [P P_next(1,1)] ;
end

% Tirage de la valeur exacte (en gros, que des 0) : 
valeur_exacte = [] ; 
for j = 1:50
   valeur_exacte = [valeur_exacte 0] ; 
end

figure('Name' , '4.2 :Estimation d une récurrence','NumberTitle','off');
    subplot(1,3,1), hold on, plot(valeur_exacte, 'g', 'linewidth', 2), plot(X, 'r', 'linewidth', 2), plot(Z, 'b--'), xlabel('# échantillon'), ylabel('Valeur tirée'),  title('Evolution de l état estimé par rapport à la mesure') ; 
    legend('Valeur exacte',  'Etat', 'Mesures') ;
    subplot(1,3,2), hold on, plot(P, 'b'), xlabel('# échantillon'), title('Evolution de l erreur d estimation') ; 
    legend('Evolution de l erreur d estimation') ; 
    subplot(1,3,3), hold on, plot(K, 'b', 'linewidth', 2),  xlabel('# échantillon'), title('Evolution du gain du filtre de Kalman') ; 
    legend('Gain K du filtre') 

end