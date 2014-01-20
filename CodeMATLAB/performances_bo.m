close all;

figure('name', 'Bode : Syst?me lin?aris?', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on;
bode(SYS_lin);
figure('name', 'Nyquist : Syst?me lin?aris?', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on;
nichols(SYS_lin);
figure; 
hold on, grid on;
step(SYS_lin) ;

disp('Valeurs propres du systeme lineaire')
% Poles du syst?me lineaire : 
lambda = eig(A_lin)

% Trace des poles du syst?me lineaire dans le plan complexe :
figure('name', 'Plan complexe : Poles du Syst?me lin?aris?', 'units','normalized','outerposition',[0 0 1 1])
hold on, grid on;
pzmap(SYS_lin);