%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                               %
%           LMI Basique : retour d'�tat         %
%                                               %
%  Note : - Working (validated with place())    %
%         - /!\ Dynamic Matrix is now (A+B*K)   %
%                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clc ; 

% -------- Data -------- %
A = A_lin_a ;
B = B_lin_a(:,1) ;

n = size(A,1) ;
ncon = size(B,2) ;

% -------- LMI -------- %
setlmis([]) ;

P = lmivar(1, [n 1]) ;
L = lmivar(2, [ncon n]) ;

%1st : 
lmiterm([1,1,1,P],-1,1) ;

%2nd : 
disp('couc1') 
lmiterm([2,1,1,P],1,A','s') ; 
lmiterm([2,1,1,L],B,1,'s') ; 
region = lmireg ; 

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





    
    