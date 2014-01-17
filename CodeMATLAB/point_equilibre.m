Ng_0 = 28000 ;              % tr/min
Ntl_0 = ntlnom ;            % 

% Tracer les �volutions de Cg et abs(Cg) pour bien comprendre le probl�me
Wf_init = 500 ; % La valeur de d�part de Wf_start (doit appartenir � [0..600])

Wf_0 = fminsearch(@(wf) abs(f_cg(wf,28000)), Wf_init) ;       % Wf_0 = 259.6451 (L/h)
T45_0 = f_t45(Wf_0, Ng_0) ; % �C
P3_0 = f_p3(Wf_0, Ng_0) ;
C_charge_0 = f_ctl(Wf_0, Ng_0) ;           % m.daN
