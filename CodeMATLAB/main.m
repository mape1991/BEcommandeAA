% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - S?ance 1 
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all ; 
clc ; 

VERBOSE = 0 ;

%% Chargement des donn?es : 
lookup_table ; 

%% Conversion en unit?s S.I. :
% Remarque : Coefficient 10 car daN

Ig_norm = ig * (2*pi)/(60*10) ;                 
Itot_norm = itot *(2*pi)/(60*10) ;

%% Transformation TF en mod?le d'?tat : 
ss_doseur_capteur ; 

%% D?termination du point d'?quilibre : 
point_equilibre ; 

%% Linearisation au point d'?quilibre : 
linearisation ; 

%% Kalman (Estimation de Wf)
obs_kalman;

%% Performance du syst?me linearis?
performances_bo ; 

%% Retour d'?tat : 
retour_etat_simple ; 

%% Correction erreur statique (Retour d'etat)
retour_etat_augmente ; 

%% Vérification stabilité LMI : 
% Penser à charger 'donnees_et_modeles_lineaire.mat'
load('donnees_et_modeles_lineaires.mat')
stabilite_lmi ; 















 




