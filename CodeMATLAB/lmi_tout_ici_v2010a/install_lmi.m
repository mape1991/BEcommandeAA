%% script qui installe SeDuMi et Yalmip
% version 2008a
% pascal Acco

y_a_eu_install=0;

%% installe le solver Sedumi
if exist('sedumi')==2
    disp('sedumi est déjà installé : vous n''avez pas à exécuter ce script');
else
    addpath(genpath('./SeDuMi_1_1'));
    y_a_eu_install=1;
end
    
%% installe Yalmip
if exist('yalmiptest')==2
    disp('Yalmip est déjà installé : vous n''avez pas à exécuter ce script');
else
    addpath(genpath('./yalmip'));
    y_a_eu_install=1;
end 

%% mets à jour le path de matlab
rehash

%% autotest de Yalmip
if (y_a_eu_install==1)
    yalmiptest
end