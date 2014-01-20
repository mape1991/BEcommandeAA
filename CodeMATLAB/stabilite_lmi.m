%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                               %
%           LMI Basique : retour d'état         %
%                                               %
%  Note : - Working (validated with place())    %
%         - /!\ Dynamic Matrix is now (A+B*K)   %
%                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clc ; 

%% Calcul d'un gain de retour d'état stabilisant : 
% -------- Data -------- %
A = A_lin_a ;
B = B_lin_a(:,1) ;

n = size(A,1) ;
ncon = size(B,2) ;

% -------- LMI -------- %
setlmis([]) ;

P = lmivar(1, [n 1]) ;
L = lmivar(2, [ncon n]) ;

% 1st : 
lmiterm([1,1,1,P],-1,1) ;

% 2nd : 
lmiterm([2,1,1,P],1,A','s') ;  
lmiterm([2,1,1,L],B,1,'s') ;     
lmisys = getlmis ; 
[tmin,xfeas] = feasp(lmisys) ; 
L1 = dec2mat(lmisys,xfeas,L) ; 
P1 = dec2mat(lmisys,xfeas,P) ; 

K = L1/P1 ; 
A_stable = A+B*K ;

disp('--------------')
disp('Open-loop Poles')
eig(A)
disp('--------------')
disp('Closed-loop Poles')
eig(A_stable) 
disp('for K = ')
disp(K)
disp('--------------')

%% Vérification de la stabilité aux points d'équilibre : 
% Remarque : Charger d'abord "donnees_et_modeles_lineaires.mat".

% -------- Data -------- %
B = B_lin_a(:,1) ;

A6 = [cell2mat(Ac(6)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;
A7 = [cell2mat(Ac(7)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;
A8 = [cell2mat(Ac(8)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;
A9 = [cell2mat(Ac(9)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;
A10 = [cell2mat(Ac(10)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;
A11 = [cell2mat(Ac(11)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;
A12 = [cell2mat(Ac(12)) [0 0 0]' ; 0 0 1 0]  - B*K_feedback_2_aug;

n = size(A6,1) ;
ncon = size(B,2) ;

% -------- LMI -------- %
setlmis([]) ;

P = lmivar(1, [n 1]) ;

% 1st : 
lmiterm([1,1,1,P],-1,1) ;

% 2nd : 
lmiterm([2,1,1,P],1,A6','s') ; 
lmiterm([3,1,1,P],1,A7','s') ; 
lmiterm([4,1,1,P],1,A8','s') ; 
lmiterm([5,1,1,P],1,A9','s') ; 
lmiterm([6,1,1,P],1,A10','s') ; 
lmiterm([7,1,1,P],1,A11','s') ; 
lmiterm([8,1,1,P],1, A12','s') ; 
lmisys = getlmis ; 
[tmin,xfeas] = feasp(lmisys) ; 

P1 = dec2mat(lmisys,xfeas,P) ; 








    
    