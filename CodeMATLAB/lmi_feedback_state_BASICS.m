%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                               %
%           LMI Basique : retour d'état         %
%                                               %
%  Note : - Working (validated with place())    %
%         - /!\ Dynamic Matrix is now (A+B*K)   %
%                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc ; 

% -------- Data -------- %
A = [0 1 ; 5 -4] ;
B = [0 ; 1] ;

% -------- LMI -------- %
setlmis([]) ;

P = lmivar(1, [2,1]) ;
L = lmivar(2, [1,2]) ;

%1st : 
lmiterm([1,1,1,P],-1,1) ;

%2nd : 
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





    
    