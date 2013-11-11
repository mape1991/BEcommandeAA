% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - Lookup table
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%


global isong isowf tcg tp3 tt4 tctl100 tdctldntl xwfp3 ywfp3 f_cg ;
load donnees_turbine_v2007a ;
% Toutes les constantes et tables sont importées
 
%% Definition de fonctions de lecture
f_cg = @(wf,ng)interp2(isong,isowf,tcg,ng,wf);                  % fonction inline cg(wf,ng)
f_wfmax = @(p3) interp1(xwfp3,ywfp3,p3);                        % fonction inline wfmax(P3)


%% Affichage des courbes 3D : 
% close all ; 
figure ; 
[Xisong, Xisowf] = meshgrid(isong, isowf) ;
mesh(Xisong, Xisowf, (tcg)) ; 
xlabel('N_g'), ylabel('W_f'), zlabel('Cg(Ng,Wf)') ; 
title('Evolution du Couple Résulant C_g en fonction de N_g et W_f') ; 

figure ; 
[Xisong, Xisowf] = meshgrid(isong, isowf) ;
mesh(Xisong, Xisowf, abs(tcg)) ; 
xlabel('N_g'), ylabel('W_f'), zlabel('abs(Cg(Ng,Wf))') ; 
title('Evolution de la valeur absolue Couple Résulant C_g en fonction de N_g et W_f') ; 

 
%% Exemples de lecture
 
% Debit Wf à ne pas dépasser pour une pression P3= 3,3 Bar
% à cause de la limite de pompage
wf_lim_ambiant = interp1(xwfp3,ywfp3,3.3)                       % lecture directe 1D
 
wf_lim_ambiant = f_wfmax(3.3)                                   % pareil mais avec une focntion inline
 
% Couple Cg  (Wf,Ng) à la limite du pompage (pour 3.3 bar) avec NG=25000 tr/min
cg_lim_pomp=interp2(isong,isowf,tcg,25000,wf_lim_ambiant)       % lecture directe 2D
cg_lim_pomp=f_cg(wf_lim_ambiant,25000)                          % avec la fonction inline
cg_lim_pomp=f_cg(f_wfmax(3.3),25000)                            % avec fonctions inline imbriquées



