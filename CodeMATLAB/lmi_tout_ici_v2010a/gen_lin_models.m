%% script qui génère les modèles linéarisés pour tout Ng de isong
% toutes les données sont stockées dans le fichier
% donnees_et_modeles_lineaires
%_____
% Ac{i},Bc{i},Cc{i},Dc{i} les modèles d'états continus linéarisés
% A{i},B{i},C{i},D{i} les modèles d'états discrétisés ('zoh')
% ATTENTION!!{} et pas () car ce sont des "cell" de matrice et non
%     des matrices de matrices...
%
% twf0(i), tp30(i), tt40(i), tc_charge0(i), tdctldntl0(i)
%   les vecteur 1D contenant les valeurs correspondantes à l'équilibre
%_____
% i va de 1 à 16 et indique pour quel ng de isong les linéarisation
% sont faites.
%_______
% Modèle d'état à temps continu considéré 
%
% dX/dt = A.X + B.U  
%   Y   = C.X + D.U 
%   
%X'= [Wfr       Ng       Ntl     ] 
%U'= [Wf   Ccharge]
%Y'=[ Ng  P3  Ntl]
%

clear all;
close all;
global isong isowf tcg tp3 tt4 tctl100
global f_acg_2_ng_wf f_cg f_p3 f_t4 f_ctl f_wf0_2_ng f_actl_2_ng_wf f_dctldntl


%% chargements des tables de donnees 
load donnees_turbine_v2007a;
ign = ig *2*pi/60/10;
itotn = itot *2*pi/60/10;

%% Fonction de lecture dans les tables
f_cg = @(ng,wf)interp2(isong,isowf,tcg,ng,wf);
f_acg_2_ng_wf = @(ng,wf)abs(interp2(isong,isowf,tcg,ng,wf));
f_wf0_2_ng= @(ng) fminsearch(@(wf) f_acg_2_ng_wf(ng,wf),mean(isowf));
f_ng0_2_wf= @(wf) fminsearch(@(ng) f_acg_2_ng_wf(ng,wf),mean(isong));
f_t4 = @(ng,wf)interp2(isong,isowf,tt4,ng,wf);
f_p3 = @(ng,wf)interp2(isong,isowf,tp3,ng,wf);
f_ctl = @(ng,wf)interp2(isong,isowf,tctl100,ng,wf);
f_dctldntl = @(ng)interp1(isong,tdctldntl,ng);


%% On linéarise le système pour tout les points de isong
ntl0=ntlnom;
N=length(isong);

%matrices d'état du système continu
Ac=cell(1,N);
Bc=cell(1,N);
Cc=cell(1,N);
Dc=cell(1,N);

%matrice d'état du système discret
A=cell(1,N);
B=cell(1,N);
C=cell(1,N);
D=cell(1,N);

for i=1:N
    ng0=isong(i);
    wf0= f_wf0_2_ng(ng0);
    twf0(i)=wf0;
    tp30(i)= f_p3(ng0,wf0);
    tt40(i)= f_t4(ng0,wf0);
    tc_charge0(i)=f_ctl(ng0,wf0);
    dctldntl0=f_dctldntl(ng0);
    tdctldntl0(i)=dctldntl0;

    %On crée le modèle linéaire
    % calcul des dérivées partieles des tables
    [dcgdwf,dcgdng,dp3dwf,dp3dng,dt4dwf,dt4dng,dctldwf,dctldng]=d_p_turbine(ng0,wf0);
    dwfdng = dcgdng / dcgdwf;

    % Modèle d'état à temps continu 
    %
    % dX/dt = A.X + B.U  
    %   Y   = C.X + D.U 
    %   
    %X'= [Wfr       Ng       Ntl     ] 
    %U'= [Wf   Ccharge]
    %Y'=[ Ng  P3  Ntl]
 
    %X'= [Wfr       Ng          Ntl     ] 
    nx=3; %3 variales d'état
    ny=3; %3 sorties
    nu=2; %2 entrées
    Ac{i}=[
        -1/tdos      0             0       ;
        dcgdwf/ign   dcgdng/ign     0       ;
        dctldwf/itotn  dctldng/itotn dctldntl0/itotn 
        ];

    %U'= [Wf   Ccharge]
    %X'= [Wfr        Ng       Ntl     ] 
    Bc{i}=[  1/tdos       0        0 ;
            0           0        -1/itotn ]';

    %Y'=[ Ng  P3  Ntl]
    %X'= [Wfr       Ng         Ntl     ] 
    Cc{i}=  [  0        1           0   ;
            dp3dwf  dp3dng        0   ;
            0         0           1   ];
    Dc{i} = zeros(ny,nu);
    
    syscont=ss(Ac{i},Bc{i},Cc{i},Dc{i});
    sysdisc=c2d(syscont,te,'zoh');
    [A{i},B{i},C{i},D{i}]=ssdata(sysdisc);
end

save donnees_et_modeles_lineaires