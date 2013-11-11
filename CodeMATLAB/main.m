% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     Be Commande AA - S�ance 1 
% 
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%

close all ; 
clc ; 

%% Chargement des donn�es : 
lookup_table ; 

%% D�termination du point d'�quilibre : 

Ng_0 = 28000 ;              % tr/min
Ntl_0 = ntlnom ;            % 

% Tracer les �volutions de Cg et abs(Cg) pour bien comprendre le probl�me
Wf_init = 500 ; % La valeur de d�part de Wf_start (doit appartenir � [0..600])

Wf_eq = fminsearch(@(wf) abs(f_cg(wf,28000)), Wf_init) ;       % Wf_eq = 259.6451 (L/h)

