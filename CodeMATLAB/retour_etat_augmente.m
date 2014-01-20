close all;
% Faire une augmentation du modele afin d'ajouter un integrateur dans le
% mod?le. (Ajouter l'etat integral(Ntl))
disp('Augmentation du modele d etat avec un etat supplementaire');
A_lin_a = [A_lin [0 0 0]' ; 0 0 1 0] 
B_lin_a = [B_lin ; 0 0]            
C_lin_a = [C_lin [0 0 0 0]' ; 0 0 0 1] 
% Ajouter une ligne dans C donc une ligne de plus dans D [une sortie en plus]
D_lin_a = [D_lin ; 0 0]

% affiche poles
eig(A_lin_a)

% Remarque : On repart du syst?me lin?aris? initial (sans observateur) en
% admettant que tous les ?tats sont connus.

% Performances : r?gulation de Ntl (erreur statique) avec un d?passement
% maximum de 7,5%
d_goal = 0.075 ;        % D?passement objectif 
d_reel = 0.075;         % D?passement sp?cifi?   

% Calcul de l'amortissement en cons?quence : 
% D(%) = exp( (-E*pi) / (sqrt(1 - E^2)) ) ;
% Soit, invers? : E = sqrt( (log(D)^2) / (pi^2 + log(D)^2) ) ;
% Calcul du cas limite : 
E_lim = sqrt( (log(d_reel)^2) / (pi^2 + log(d_reel)^2) ) ; 

% Remarque : On doit donc prendre E tel que E_goal > E_lim (voir explication
% notes).
E_goal = E_lim + 0.1 ; 

% Pour calculer Wn, on prendra un temps de r?ponse objectif de 10s (pour
% garder la dynamique du syst?me initial) : 
Wn_goal = 4.5/(E_goal*20) ; 

% Les poles ? atteindre d?pendent donc de E_goal et Wn_goal. Le troisi?me
% pole sera plac? ? l'infini pour qu'il soit n?glig? dans la dynamique du
% syst?me boucl?

% poles_goal_aug = [ -E_goal*Wn_goal + i*Wn_goal*sqrt(1-E_goal^2) ;
%               -E_goal*Wn_goal - i*Wn_goal*sqrt(1-E_goal^2) ; 
%                -E_goal*Wn_goal*50 ; -E_goal*Wn_goal*40] ;
           
poles_goal_aug = [ -0.3 + i*0.2 ;
                     -0.3 - i*0.2 ;
               -11;
               -13] ;               

% Fonction place pour d?terminer le retour d'?tat : 
K_feedback_2_aug = place(A_lin_a, B_lin_a(:,1), poles_goal_aug) ; 
%K_feedback_2_aug = K ; 

% -- DEBUT SATURATION -- %
dWf_max = 100 ;                 % [L/h/s] 

% Saturateur sur Wf
Wf_min = 60 - Wf_0                    % [L/h]
Wf_max = f_wfmax(P3_0) - Wf_0         % [L/h]

% D?termination du pire ?chelon de charge : 
C_charge_min = f_ctl(Wf_min + Wf_0, Ng_0) - C_charge_0  
C_charge_max = f_ctl(Wf_max + Wf_0, Ng_0) - C_charge_0 

% Simulations (Syst?me boucl? ==> placement de poles) : 
t_simu = 100 ; 
t_step = t_simu / 2 ; 
sim('sim_system_lin_retour_etat_aug') ; 

figure('name', 'Pire echelon de charge', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on,  plot(D_Ccharge.time, D_Ccharge.signals.values, 'b'), title('Pire echelon de charge') ;

% Syst?me Lin?aire en rouge et Syst?me Initial en bleu
figure('name', 'Syst?me lin?aris? en boucle ferm?e (placement de poles)', 'units','normalized','outerposition',[0 0 1 1])
subplot(221), hold on, grid on,  plot(D_Wf.time, D_Wf.signals.values, 'b'), title('Evolution de Wf') ;
subplot(222), hold on, grid on, plot(D_Ntl.time, D_Ntl.signals.values, 'b'), line([0 t_simu], [(d_goal)*Ntl_0 (d_goal)*Ntl_0], 'Color', 'r'),  line([0 t_simu], [-(d_goal)*Ntl_0 -(d_goal)*Ntl_0], 'Color', 'r'), title('Evolution de Ntl') ;
subplot(223), hold on, grid on,  plot(D_dWf.time, D_dWf.signals.values, 'b'), line([0 t_simu], [dWf_max dWf_max], 'Color', 'r'), line([0 t_simu], [-dWf_max -dWf_max], 'Color', 'r'), title('Evolutions de dWf/dt') ; 
subplot(224), hold on, grid on,  plot(D_Wf_comm.time, D_Wf_comm.signals.values, 'b'), line([0 t_simu], [Wf_max Wf_max], 'Color', 'r'), line([0 t_simu], [Wf_min Wf_min], 'Color', 'r'), title('Evolution de la commande Wf*') ;


% Creation de la matrice A-BK
Asta=A_lin_a-B_lin_a(:,1)*K_feedback_2_aug; % retour d'?tat stable  u = -K X