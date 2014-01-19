echo off
clear all;
close all;
clc
global isong isowf tcg tp3 tt4 tctl100
global f_acg_2_ng_wf f_cg f_p3 f_t4 f_ctl f_wf0_2_ng f_actl_2_ng_wf f_dctldntl

%% installation du solver SeDuMi et de Yalmip
% commentez cette ligne une fois l'install effectu�e avec succ�s
% install_lmi 

%% chargements les tables de donnees et les mod�les lin�airis�s
% lorsqu'elles existent, sinon on les cr�es
if (~exist('donnees_et_modeles_lineaires.mat'))
    disp('Pas de mod�les stock�s : on les reg�n�re donc'); 
    gen_lin_models;
else
    disp('Chargement des mod�les lin�aires continus et discrets'); 
    load donnees_et_modeles_lineaires;
end


disp(' LISEZ BIEN l''ent�te de gen_lin_models.m pour voir les donn�es cr��es ');
pause

%% calcul d'un placement de pole 
% le gain K est calcul� pour le syst�me continu lin�aris� en milieu de
% plage isong : i=8 => ng0=isong(8)


% cr�e les matrices d'entr�e Bwf et Bch pour les entr�e de Wfc et Ccharge
for i = 1:N
   Bwf{i} = Bc{i}(:,1);
   Bch{i} = Bc{i}(:,2);
end

% on appelle place pour calculer K

[K,PREC,MESSAGE]=place(Ac{8},Bwf{8},[-30.0 -10+10*j -10-10*j]);
disp('Le retour d''�tat K est calcul� pour A=A{8} en milieu de plage');
disp('on v�rifie que A-B.K est stable avec >>Racines_Asta = eig(A-B.K)');
Asta=Ac{8}-Bwf{8}*K; % retour d'�tat stable  u = -K X
Racines_Asta=eig(Asta)




disp('On va se poser les m�me problemes sous forme de LMI...');
pause

%% v�rification de la stabilit� avec Lyapunov
% bon c'est stupide car il suffit de regarder les valeurs propres
% donn�es par eig ci-dessus mais �a introduit les LMI et Yalmip

% on cherche une fonction de Lyapunov quadratique v(X(t))=X(t)'.P.X(t) 
% o� P est d�finie positive et d�fini la fonction de Lyapunov candidate
% le systeme dX/dt = A.X + B.u est stable s'il existe P telle que
% dv/dt<0 or dv/dt= dX/dt'P + P'.dX/dt
% dv/dt= X'.A'.P + P'.A.X <0 soit  A'.P + P'A d�finie n�gative

% d�finition des variables

P = sdpvar(3,3); %P est une variable de dimension 3x3 sym�trique (par defaut)

% d�finition des contraintes

F = set([P>0],'P positive');                 % P est d�finie positive
F = F + set([Asta'*P + P'*Asta<0],'Lyap decroit');   % dv/dt <0 

% appel du solveur de LMI

diagnostic = solvesdp(F);

% Yalmip a appel� SeDuMi qui a r�solu ou pas la LMI
% diagnostic.problem indique si �a a march� ou non

if (diagnostic.problem==0)
    disp('====================== ');
    disp('Asta est stable')
    disp('====================== ');

else
    disp('====================== ');
    disp('LMI infaisable ou probl�me num�rique');
    disp('====================== ');
end
% On extrait la solution numerique de P avec double()
disp('on extrait avec double(P) la valeur num�rique de la solution')
P_feasible = double(P)

% V�rification des contraintes
%eig(P_feasible) % est-elle d�finie positive strictement ?
%eig(Asta'*P_feasible+P_feasible'*Asta) % v est-elle d�croissante ?
%checkset(F) % chekset fait les m�me  tests et donne le min(eig( de chaque contrainte))

disp('Il existe donc une fonction de Lyapunov pour astab')
disp('Il ne devrait pas en exister pour Ainst')
pause

%% m�me v�rification avec le retour d'�tat instable
% on efface les LMI de F et on recommence
clear F
disp('on v�rifie que A+B.K est instable avec >>Racines_Ains = eig(A-B.K)');
Ains=Ac{8}+Bwf{8}*K; % retour d'�tat instable u= K X
Racines_Ains=eig(Ains)


% red�finition des contraintes
F = set([P>0],'P positive');                 % P est d�finie positive
F = F + set([Ains'*P + P'*Ains <0],'Lyap decroit');   % dv/dt <0 


% on refait le calcul

diagnostic = solvesdp(F);

if (diagnostic.problem==0)
    disp('====================== ');
    disp('LMI faisable   !!!');
    disp('====================== ');

else
    disp('====================== ');
    disp('LMI infaisable ou probl�me num�rique');
    disp('====================== ');
end

% si la LMI est faisable c'est qu'il y a probleme

% Extract numerical solution
P_feasible = double(P);

% V�rification des contraintes

Valeur_Propres_P=eig(P_feasible) % est-elle d�finie positive strictement ?!
Valeur_Propres_LMI_Lyapunov=eig(Ains'*P_feasible+P_feasible'*Ains) % v est-elle d�croissante ?
checkset(F) % chekset fait les m�me  tests et donne le min(eig( de chaque contrainte))

disp('La LMI est dite faisable alors que A+B.K est instable ! Que dis checkset ?');
pause

%% Affiche le lieu des poles de la turbine r�gul�e avec u=-K X
% lorsque le point statique varie sur toute la gamme de Ng

for i=1:N
    B2{i} = Bc{i}(:,1);
   poles_b{i}=eig(Ac{i}-B2{i}*K);
    plot(real(poles_b{i}),imag(poles_b{i}),'+'); hold on;
end

plot(real(poles_b{8}),imag(poles_b{8}),'r.');

title('Les diff�rents poles de Ai-Bi.K pour tous les isong');
