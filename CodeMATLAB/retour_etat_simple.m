% Remarque : On repart du syst?me lin?aris? initial (sans observateur) en
% admettant que tous les ?tats sont connus.

% Performances : r?gulation de Ntl (erreur statique) avec un d?passement
% maximum de 7,5%
d_goal = 0.05 ;        % D?passement objectif 

% Calcul de l'amortissement en cons?quence : 
% D(%) = exp( (-E*pi) / (sqrt(1 - E^2)) ) ;
% Soit, invers? : E = sqrt( (log(D)^2) / (pi^2 + log(D)^2) ) ;
% Calcul du cas limite : 
E_lim = sqrt( (log(d_goal)^2) / (pi^2 + log(d_goal)^2) ) ; 

% Remarque : On doit donc prendre E tel que E_goal > E_lim (voir explication
% notes).
E_goal = E_lim + 0.1 ; 

% Pour calculer Wn, on prendra un temps de r?ponse objectif de 10s (pour
% garder la dynamique du syst?me initial) : 
Wn_goal = 4.5/(E_goal*1) ; 

% Les poles ? atteindre d?pendent donc de E_goal et Wn_goal. Le troisi?me
% pole sera plac? ? l'infini pour qu'il soit n?glig? dans la dynamique du
% syst?me boucl?
poles_goal = [ -E_goal*Wn_goal + 1i*Wn_goal*sqrt(1-E_goal^2) ;
               -E_goal*Wn_goal - 1i*Wn_goal*sqrt(1-E_goal^2) ; 
               -5 ] ;
           
% Fonction place pour d?terminer le retour d'?tat : 
K_feedback_1 = place(A_lin, B_lin(:,1), poles_goal) ; 


% Simulations (Syst?me boucl? : placement de poles) : 
t_simu = 50 ; 
t_step = t_simu / 4 ; 
sim('sim_system_lin_retour_etat') ; 

% close all ; 
% Syst?me Lin?aire en rouge et Syst?me Initial en bleu
figure('name', 'Syst?me lin?aris? en boucle ferm?e (placement de poles)', 'units','normalized','outerposition',[0 0 1 1])
subplot(223), hold on, grid on,  plot(D_Ng, 'r'), plot(D_P3, 'b'), title('Evolution de Ng et P3'),legend('Ng','P3') ; 
subplot(224), hold on, grid on,  plot(D_Wf_comm, 'b'), title('Evolution de la commande Wf*') ;
subplot(221), hold on, grid on,  plot(D_Wf, 'r'), title('Evolution de Wf') ;
subplot(222), hold on, grid on, plot(D_Ntl, 'r'), title('Evolution de Ntl') ;
