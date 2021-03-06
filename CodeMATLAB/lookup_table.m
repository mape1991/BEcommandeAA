% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - Lookup table
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%


global isong isowf tcg tp3 tt4 tctl100 tdctldntl xwfp3 ywfp3 f_cg ;
load donnees_turbine_v2007a ;
% Toutes les constantes et tables sont import�es
 
%% Definition de fonctions de lecture
f_cg = @(wf,ng)interp2(isong,isowf,tcg,ng,wf);                  % fonction inline cg(wf,ng)
f_wfmax = @(p3) interp1(xwfp3,ywfp3,p3);                        % fonction inline wfmax(P3)
f_ctl = @(wf,ng) interp2(isong, isowf, tctl100, ng, wf) ;       % fonction inline ctl(wf, ng)
f_dctl_dntl = @(ng) interp1(isong,tdctldntl,ng);                % fonction inline dctl/dNntl(P3)
f_p3 = @(wf,ng) interp2(isong, isowf, tp3, ng, wf) ;            % fonction inline ctl(wf, ng)
f_t45 = @(wf,ng) interp2(isong, isowf, tt4, ng, wf) ;           % fonction inline ctl(wf, ng)

%% Affichage des courbes 3D : 
close all ; 
    [Xisong, Xisowf] = meshgrid(isong, isowf) ;
    
if (VERBOSE)
    figure ; 
    subplot(221), hold all, grid on, mesh(Xisong, Xisowf, (tcg)), xlabel('N_g'), ylabel('W_f'), zlabel('Cg(Ng,Wf)'), title('C_g=f(N_g, W_f)') ; 
    subplot(222), hold all, grid on, mesh(Xisong, Xisowf, abs(tcg)), xlabel('N_g'), ylabel('W_f'), zlabel('abs(Cg(Ng,Wf))'), title('C_g=f(N_g, W_f)') ; 
    subplot(223), hold all, grid on, mesh(Xisong, Xisowf, (tctl100)), xlabel('N_g'), ylabel('W_f'), zlabel('Cftl(Ng,Wf)'), title('C_{tl}=f(N_g, W_f)') ;
    subplot(224), hold all, grid on, mesh(Xisong, Xisowf, (tt4)), xlabel('N_g'), ylabel('W_f'), zlabel('T45(Ng,Wf)'), title('T_{45}=(N_g, W_f)') ;
    
    figure ; 
    subplot(121), hold all, grid on, mesh(Xisong, Xisowf, (tctl100)), xlabel('N_g'), ylabel('W_f'), zlabel('C_{tl})'), title('C_{tl} = f(N_g, W_f)') ; 
    subplot(122), hold all, grid on, plot(Xisong, (tdctldntl)), xlabel('N_g'), ylabel('dC_{tl}/dN_{tl}'), title('dC_{tl}/{dN_{tl}} = f(N_g)') ; 
end 



 
%% Exemples de lecture
 
% Debit Wf � ne pas d�passer pour une pression P3= 3,3 Bar
% � cause de la limite de pompage
wf_lim_ambiant = interp1(xwfp3,ywfp3,3.3)                       % lecture directe 1D
 
wf_lim_ambiant = f_wfmax(3.3)                                   % pareil mais avec une focntion inline
 
% Couple Cg  (Wf,Ng) � la limite du pompage (pour 3.3 bar) avec NG=25000 tr/min
cg_lim_pomp=interp2(isong,isowf,tcg,25000,wf_lim_ambiant)       % lecture directe 2D
cg_lim_pomp=f_cg(wf_lim_ambiant,25000)                          % avec la fonction inline
cg_lim_pomp=f_cg(f_wfmax(3.3),25000)                            % avec fonctions inline imbriqu�es



