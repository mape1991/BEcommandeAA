% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - Séance 1 
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%

close all ; 
clc ; 

%% Chargement des données : 
lookup_table ; 

%% Détermination du point d'équilibre : 

Ng_0 = 28000 ;              % tr/min
Ntl_0 = ntlnom ;            % 

% Tracer les évolutions de Cg et abs(Cg) pour bien comprendre le problème
Wf_init = 500 ; % La valeur de départ de Wf_start (doit appartenir à [0..600])

Wf_eq = fminsearch(@(wf) abs(f_cg(wf,28000)), Wf_init) ;       % Wf_eq = 259.6451 (L/h)

