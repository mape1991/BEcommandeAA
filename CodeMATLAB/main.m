% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - Séance 1 
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all ; 
clc ; 

VERBOSE = 1 ;

%% Chargement des données : 
lookup_table ; 

%% Conversion en unités S.I. :
% Remarque : Coefficient 10 car daN

Ig_norm = ig * (2*pi)/(60*10) ;                 
Itot_norm = itot *(2*pi)/(60*10) ;

%% Transformation TF en modèle d'état : 
ss_doseur_capteur ; 

%% Détermination du point d'équilibre : 
point_equilibre ; 

%% Linearisation au point d'équilibre : 
linearisation ; 

%% Kalman (Estimation de Wf)
% Remarque : Fait ici pour pouvoir lancer la simulation 'sim_system_kalman'

close all;

% QN = E{ww'} [bruit sur l'état], [n,n] n=3    
% RN = E{vv'} [bruis sur la mesure], [m,m] m=3
% NN = E{wv'}=0 car decorrélées

Precision_Ng = 0.001;
Precision_P3 = 0.02;
Precision_T45 = 0.02;
Precision_Ntl = 0.001;

QN = 1e-5 ;             % Confiance au modèle (grand : peu de confiance)

RN = [0.001 0 0 0 ;     % Confiance en la mesure (grand : peu de confiance)
      0 0.001 0 0 ;
      0 0 0.001 0 ;
      0 0 0 0.001] ;

SYS_lin_K = ss(A_lin, B_lin(:,1), C_lin, 0);

[KEST_lin, L_lin, P_lin] = kalman(SYS_lin_K, QN, RN, 0) ;

K_kalman = ss(A_lin-L_lin*C_lin, [B_lin(:,1) L_lin], eye(3), 0) ;

%% Simulations (Système Linéaire VS Système complet) : 
close all ; 
t_sim = 20, sim('sim_system_kalman', t_sim) ; 

% Système Linéaire en rouge et Système Initial en bleu
figure('name', 'Système initial VS système linéarisé', 'units','normalized','outerposition',[0 0 1 1])
subplot(221), hold on, grid on, plot(Ng, 'b'), plot(D_Ng, 'r'), title('Evolution de Ng'), legend('Non-linéaire (initial)','Linéaire') , 
subplot(222), hold on, grid on, plot(P3, 'b'), plot(D_P3, 'r'), title('Evolution de P3') ;
subplot(223), hold on, grid on, plot(T45, 'b'), plot(D_T45, 'r'), title('Evolution de T45') ;
subplot(224), hold on, grid on, plot(Ntl, 'b'), plot(D_Ntl, 'r'), title('Evolution de Ntl') ; 


%% Simulations (Système Linéaire VS Kalman) :
t_sim = 20 ; sim('sim_system_kalman', t_sim) ; 
 
% Système Linéaire en rouge et Kalman en vert
figure('name', 'Système linéarisé avec un filtre de Kalman', 'units','normalized','outerposition',[0 0 1 1])
subplot(221), hold on, grid on, plot(D_Wf, 'r'), plot(K_Wf, 'g', 'linewidth', 2), title('Evolutions de Wf'), legend('Linéaire', 'Kalman') ,
subplot(222), hold on, grid on, plot(D_Ng, 'r'), plot(K_Ng, 'g', 'linewidth', 2), title('Evolution de Ng') ;
subplot(223), hold on, grid on, plot(D_Ntl, 'r'), plot(K_Ntl, 'g', 'linewidth', 2), title('Evolution de Ntl') ; 

% Remarque : Forcément, Wf est exactement le même puisque Wf n'était pas
% bruité


%% Performance du système linearisé
close all;

figure('name', 'Bode : Système linéarisé', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on;
bode(SYS_lin);
figure('name', 'Nyquist : Système linéarisé', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on;
nichols(SYS_lin);
figure; 
hold on, grid on;
step(SYS_lin) ;

% Poles du système lineaire : 
lambda = eig(A_lin);

% Trace des poles du système lineaire dans le plan complexe :
figure('name', 'Plan complexe : Poles du Système linéarisé', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on;
pzmap(SYS_lin);

%% Retour d'état : 
% Remarque : On repart du système linéarisé initial (sans observateur) en
% admettant que tous les états sont connus.

% Performances : régulation de Ntl (erreur statique) avec un dépassement
% maximum de 7,5%
d_goal = 0.05 ;        % Dépassement objectif 

% Calcul de l'amortissement en conséquence : 
% D(%) = exp( (-E*pi) / (sqrt(1 - E^2)) ) ;
% Soit, inversé : E = sqrt( (log(D)^2) / (pi^2 + log(D)^2) ) ;
% Calcul du cas limite : 
E_lim = sqrt( (log(d_goal)^2) / (pi^2 + log(d_goal)^2) ) ; 

% Remarque : On doit donc prendre E tel que E_goal > E_lim (voir explication
% notes).
E_goal = E_lim + 0.1 ; 

% Pour calculer Wn, on prendra un temps de réponse objectif de 10s (pour
% garder la dynamique du système initial) : 
Wn_goal = 4.5/(E_goal*1) ; 

% Les poles à atteindre dépendent donc de E_goal et Wn_goal. Le troisième
% pole sera placé à l'infini pour qu'il soit négligé dans la dynamique du
% système bouclé
poles_goal = [ -E_goal*Wn_goal + 1i*Wn_goal*sqrt(1-E_goal^2) ;
               -E_goal*Wn_goal - 1i*Wn_goal*sqrt(1-E_goal^2) ; 
               -5 ] ;
           
% Fonction place pour déterminer le retour d'état : 
K_feedback_1 = place(A_lin, B_lin(:,1), poles_goal) ; 


% Simulations (Système bouclé : placement de poles) : 
t_simu = 50 ; 
t_step = t_simu / 4 ; 
sim('sim_system_lin_retour_etat') ; 

% close all ; 
% Système Linéaire en rouge et Système Initial en bleu
figure('name', 'Système linéarisé en boucle fermée (placement de poles)', 'units','normalized','outerposition',[0 0 1 1])
subplot(223), hold on, grid on,  plot(D_Ng, 'r'), plot(D_P3, 'b'), title('Evolution de Ng et P3'),legend('Ng','P3') ; 
subplot(224), hold on, grid on,  plot(D_Wf_comm, 'b'), title('Evolution de la commande Wf*') ;
subplot(221), hold on, grid on,  plot(D_Wf, 'r'), title('Evolution de Wf') ;
subplot(222), hold on, grid on, plot(D_Ntl, 'r'), title('Evolution de Ntl') ;


% REMARQUE POUR LA PROCHAINE FOIS : LES CALCULS DES POLES SONT LAMES, IL
% VAUT MIEUX UTILISER LES POLES :  [-9.5 - 3i ;  -9.5 + 3i ] (voir
% pourquoi)

%% Correction erreur statique (Retour d'etat)
close all;
% Faire une augmentation du modele afin d'ajouter un integrateur dans le
% modèle. (Ajouter l'etat integral(Ntl))
A_lin_a = [A_lin [0 0 0]' ; 0 0 1 0] ;
B_lin_a = [B_lin ; 0 0] ;           
C_lin_a = [C_lin [0 0 0 0]' ; 0 0 0 1] ;
% Ajouter une ligne dans C donc une ligne de plus dans D [une sortie en plus]
D_lin_a = [D_lin ; 0 0];

% affiche poles
eig(A_lin_a)

% Remarque : On repart du système linéarisé initial (sans observateur) en
% admettant que tous les états sont connus.

% Performances : régulation de Ntl (erreur statique) avec un dépassement
% maximum de 7,5%
d_goal = 0.075 ;        % Dépassement objectif 
d_reel = 0.075;         % Dépassement spécifié   

% Calcul de l'amortissement en conséquence : 
% D(%) = exp( (-E*pi) / (sqrt(1 - E^2)) ) ;
% Soit, inversé : E = sqrt( (log(D)^2) / (pi^2 + log(D)^2) ) ;
% Calcul du cas limite : 
E_lim = sqrt( (log(d_reel)^2) / (pi^2 + log(d_reel)^2) ) ; 

% Remarque : On doit donc prendre E tel que E_goal > E_lim (voir explication
% notes).
E_goal = E_lim + 0.1 ; 

% Pour calculer Wn, on prendra un temps de réponse objectif de 10s (pour
% garder la dynamique du système initial) : 
Wn_goal = 4.5/(E_goal*20) ; 

% Les poles à atteindre dépendent donc de E_goal et Wn_goal. Le troisième
% pole sera placé à l'infini pour qu'il soit négligé dans la dynamique du
% système bouclé
%poles_goal_aug = [ -E_goal*Wn_goal + i*Wn_goal*sqrt(1-E_goal^2) ;
%               -E_goal*Wn_goal - i*Wn_goal*sqrt(1-E_goal^2) ; 
%                -E_goal*Wn_goal*50 ; -E_goal*Wn_goal*40] ;
           
poles_goal_aug = [ -1 + i*0.2 ;
                     -1 - i*0.2 ;
               -5;
               -6] ;               

% Fonction place pour déterminer le retour d'état : 
K_feedback_2_aug = place(A_lin_a, B_lin_a(:,1), poles_goal_aug) ; 

% -- DEBUT SATURATION -- %
dWf_max = 100 ;                 % [L/h/s] 

% Saturateur sur Wf
Wf_min = 60 - Wf_0 ;                   % [L/h]
Wf_max = f_wfmax(P3_0) - Wf_0 ;        % [L/h]

% Détermination du pire échelon de charge : 
C_charge_min = f_ctl(Wf_min + Wf_0, Ng_0) - C_charge_0 ; 
C_charge_max = f_ctl(Wf_max + Wf_0, Ng_0) - C_charge_0 ;

% Simulations (Système bouclé ==> placement de poles) : 
t_simu = 100 ; 
t_step = t_simu / 2 ; 
sim('sim_system_lin_retour_etat_aug') ; 

% Système Linéaire en rouge et Système Initial en bleu
figure('name', 'Système linéarisé en boucle fermée (placement de poles)', 'units','normalized','outerposition',[0 0 1 1])
subplot(221), hold on, grid on,  plot(D_Wf, 'b'), title('Evolution de Wf') ;
subplot(222), hold on, grid on, plot(D_Ntl, 'b'), line([0 t_simu], [(d_goal)*Ntl_0 (d_goal)*Ntl_0], 'Color', 'r'),  line([0 t_simu], [-(d_goal)*Ntl_0 -(d_goal)*Ntl_0], 'Color', 'r'), title('Evolution de Ntl') ;
subplot(223), hold on, grid on,  plot(D_dWf, 'b'), line([0 t_simu], [dWf_max dWf_max], 'Color', 'r'), line([0 t_simu], [-dWf_max -dWf_max], 'Color', 'r'), title('Evolutions de dWf/dt') ; 
subplot(224), hold on, grid on,  plot(D_Wf_comm, 'b'), line([0 t_simu], [Wf_max Wf_max], 'Color', 'r'), line([0 t_simu], [Wf_min Wf_min], 'Color', 'r'), title('Evolution de la commande Wf*') ;


% Creation de la matrice A-BK
Asta=A_lin_a-B_lin_a(:,1)*K_feedback_2_aug; % retour d'état stable  u = -K X


%% vérification de la stabilité avec Lyapunov
% bon c'est stupide car il suffit de regarder les valeurs propres
% données par eig ci-dessus mais ça introduit les LMI et Yalmip

% on cherche une fonction de Lyapunov quadratique v(X(t))=X(t)'.P.X(t) 
% où P est définie positive et défini la fonction de Lyapunov candidate
% le systeme dX/dt = A.X + B.u est stable s'il existe P telle que
% dv/dt<0 or dv/dt= dX/dt'P + P'.dX/dt
% dv/dt= X'.A'.P + P'.A.X <0 soit  A'.P + P'A définie négative

% définition des variables

P = sdpvar(3,3); %P est une variable de dimension 3x3 symétrique (par defaut)

% définition des contraintes

F = set([P>0],'P positive');                 % P est définie positive
F = F + set([Asta'*P + P'*Asta<0],'Lyap decroit');   % dv/dt <0 

% appel du solveur de LMI

diagnostic = solvesdp(F);

% Yalmip a appelé SeDuMi qui a résolu ou pas la LMI
% diagnostic.problem indique si ça a marché ou non

if (diagnostic.problem==0)
    disp('====================== ');
    disp('Asta est stable')
    disp('====================== ');

else
    disp('====================== ');
    disp('LMI infaisable ou problème numérique');
    disp('====================== ');
end
% On extrait la solution numerique de P avec double()
disp('on extrait avec double(P) la valeur numérique de la solution')
P_feasible = double(P)

% Vérification des contraintes
%eig(P_feasible) % est-elle définie positive strictement ?
%eig(Asta'*P_feasible+P_feasible'*Asta) % v est-elle décroissante ?
%checkset(F) % chekset fait les même  tests et donne le min(eig( de chaque contrainte))

disp('Il existe donc une fonction de Lyapunov pour astab')
disp('Il ne devrait pas en exister pour Ainst')
pause




















 




