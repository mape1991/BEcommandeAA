% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - Linearisation
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%

% Voir énoncé page 13

%% Calcul de la Jacobienne : 

pas_ng = 1000 ; 
pas_wf = 40 ; 

% J1
J11 = (-1/tdos) ;                    % Coef. pour passer en unité SI
J12 = 0 ; 
J13 = 0 ;

% J21 : 
% Ng = Ng_0 ;
% tcg_21 = tcg(:,8) ;             % Ng = Ng_0 ; 
% figure ; 
% plot(isowf, tcg_21) ;
% xlabel('W_f'), ylabel('C_g(W_f, Ng_0)') ; 
% 
% J21 = (60/(2*pi))/ig * (tcg_21(7) - tcg_21(6)) / (isowf(7) - isowf(6))
J21 = 1/Ig_norm * (f_cg(Wf_0 + pas_wf, Ng_0) - f_cg(Wf_0 - pas_wf, Ng_0)) / (2*pas_wf) ;

% J22 :
J22 = 1/Ig_norm * (f_cg(Wf_0, Ng_0 + pas_ng) - f_cg(Wf_0, Ng_0 - pas_ng)) / (2*pas_ng) ;

% J23 : 
J23 = 0 ; 

% J31 : 
J31 = 1/Itot_norm * (f_ctl(Wf_0 + pas_wf, Ng_0) - f_ctl(Wf_0 - pas_wf, Ng_0)) / (2*pas_wf) ;

% J32 :
J32 = 1/Itot_norm * (f_ctl(Wf_0, Ng_0 + pas_ng) - f_ctl(Wf_0, Ng_0 - pas_ng)) / (2*pas_ng) ;

% J33 : 
J33 = 1/Itot_norm * f_dctl_dntl(Ng_0) ;

% Matrice Jacobienne : 
J = [J11 J12 J13 ; 
     J21 J22 J23 ;
     J31 J32 J33] ;
 
%% Creation du modèle

A_lin = J ;

B_lin_Wf = [1/tdos ; 0 ; 0];
B_lin_Ccharge = [0 ; 0 ; -1/Itot_norm];

B_lin = [B_lin_Wf B_lin_Ccharge] ;

% linearization de C (matrice de sortie)
C21 =  (f_p3(Wf_0 + pas_wf, Ng_0) - f_p3(Wf_0 - pas_wf, Ng_0)) / (2*pas_wf) ;
C22 =  (f_p3(Wf_0, Ng_0 + pas_ng) - f_p3(Wf_0, Ng_0 - pas_ng)) / (2*pas_ng) ; 

C31 =  (f_t45(Wf_0 + pas_wf, Ng_0) - f_t45(Wf_0 - pas_wf, Ng_0)) / (2*pas_wf) ;
C32 =  (f_t45(Wf_0, Ng_0 + pas_ng) - f_t45(Wf_0, Ng_0 - pas_ng)) / (2*pas_ng) ; 

C_lin = [0 1 0;
         C21 C22 0;
         C31 C32 0;
         0 0 1];
     
D_lin = [0 0 ;
         0 0 ;
         0 0 ;
         0 0] ;
     
SYS_lin = ss(A_lin, B_lin, C_lin, D_lin);


