%	function [dcgdwf,dwfdng]=derivees_partielles(ng0,wf0);
% 		calcule les dérivées partielles dcg/dwf et dwf/dng au point (ng0,wf0) donné.
% 		créé par pascal
function [dcgdwf,dcgdng,dp3dwf,dp3dng,dt4dwf,dt4dng,dctldwf,dctldng]=d_p_turbine(ng0,wf0)
global isong isowf tcg tp3 tt4 tctl
global f_acg_2_ng_wf f_cg f_p3 f_t4 f_ctl 

%calcul de dcg/dwf par approximation sur variation de +/-20 l/h
ecart_wf = isowf(2)-isowf(1);
wf0p = min(wf0 + ecart_wf/2, isowf(end));
wf0m = max(wf0 - ecart_wf/2, isowf(1));
dcgdwf=(f_cg(ng0,wf0p)-f_cg(ng0,wf0m))/ecart_wf;
dp3dwf=(f_p3(ng0,wf0p)-f_p3(ng0,wf0m))/ecart_wf;
dt4dwf=(f_t4(ng0,wf0p)-f_t4(ng0,wf0m))/ecart_wf;
dctldwf=(f_ctl(ng0,wf0p)-f_ctl(ng0,wf0m))/ecart_wf;

%calcul de dcg/dng par approximation sur variation de +/- 400tr/mn
ecart_ng = isong(2)- isong(1);
ng0p = min(ng0 + ecart_ng/2, isong(end));
ng0m = max(ng0 - ecart_ng/2, isong(1));
dcgdng=(f_cg(ng0p,wf0)-f_cg(ng0m,wf0))/ecart_ng;
dp3dng=(f_p3(ng0p,wf0)-f_p3(ng0m,wf0))/ecart_ng;
dt4dng=(f_t4(ng0p,wf0)-f_t4(ng0m,wf0))/ecart_ng;
dctldng=(f_ctl(ng0p,wf0)-f_ctl(ng0m,wf0))/ecart_ng;

