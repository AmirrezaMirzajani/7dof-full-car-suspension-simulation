function out=suspension_Ks(t14,phi_dot,theta_dot,Z_dot,zufl_dot,zufr_dot,zurl_dot,zurr_dot,phi,theta,zufl,zufr,zurl,zurr,Z,Ks)
%% 
   A1=-0.02;
A2=-0.03;
A3=-0.01;
A4=0.02;
A5=0.03;
w1=3;
w2=5;
w3=8;
w4=5;
w5=6;
t=t14;
R_harmonic = A1*sin(w1*t)+A2*cos(w2*t)-A3*sin(w3*t)+A4*cos(w4*t);
R_harmonic_dot = A1*w1*cos(w1*t)-A2*w2*sin(w2*t)-A3*w3*cos(w3*t)-A4*w4*sin(w4*t);

zrfl=R_harmonic;
zrfr=R_harmonic;
zrrl=R_harmonic;
zrrr=R_harmonic;

zrfl_dot=R_harmonic_dot;
zrfr_dot=R_harmonic_dot;
zrrl_dot=R_harmonic_dot;
zrrr_dot=R_harmonic_dot;
%% parameters


Lf=1; %longitudinal distance from c.g to front tires
Lr=1.5; %longitudinal distance from c.g to rear tires
Lsf=0.75; % half tracks of the front axles
Lsr=0.75; % half tracks of the rear axles
% linear spring force coefficients (left front wheel)
Ks1fl=Ks; 
%linear damping force coefficients (left front wheel)
Cs1fl=1385;

%Tyre spring and damping coefficients (left front whel)
Ktfl=190000;
Ctfl=70;
% linear spring force coefficients (right front wheel)
Ks1fr=Ks;
%linear damping force coefficients (right front wheel)
Cs1fr=1385;
%Tyre spring and damping coefficients (right front whel)
Ktfr=190000;
Ctfr=70;
% linear spring force coefficients (left rear wheel)
Ks1rl=Ks;
%linear damping force coefficients (left rear wheel)
Cs1rl=1385;
%Tyre spring and damping coefficients (left rear whel)
Ktrl=190000;
Ctrl=70;
% linear spring force coefficients (right rear wheel)
Ks1rr=Ks;
%linear damping force coefficients (right rear wheel)
Cs1rr=1385;
%Tyre spring and damping coefficients (right rear whel)
Ktrr=190000;
Ctrr=70;

%%
% ufl=0;
% ufr=0;
% url=0;
% urr=0;
% Fxfl=0;
% Fxfr=0;
% Fxrl=0;
% Fxrr=0;
% Fyfl=0;
% Fyfr=0;
% Fyrl=0;
% Fyrr=0;
%%
E=10^-6;
%% normal force on tire
% Tyre deflection in each station
ztfl_dot = zufl_dot - zrfl_dot+E; %zufl_dot=y(7) 
ztfr_dot = zufr_dot - zrfr_dot+E; %zufr_dot=y(8) 
ztrl_dot = zurl_dot - zrrl_dot+E;  %zurl_dot=y(9) 
ztrr_dot = zurr_dot - zrrr_dot+E;  %zurr_dot=y(10) 

ztfl=zufl-zrfl+E;  %zufl=y(18) 
ztfr=zufr-zrfr+E;   %zufr=y(19) 
ztrl=zurl-zrrl+E;  %zurl=y(20) 
ztrr=zurr-zrrr+E;  %zurr=y(21) 

% The tyre elastic and damping forces
fstfl=Ktfl*ztfl;
fstfr=Ktfr*ztfr;
fstrl=Ktrl*ztrl;
fstrr=Ktrr*ztrr;

fdtfl=Ctfl*ztfl_dot;
fdtfr=Ctfr*ztfr_dot;
fdtrl=Ctrl*ztrl_dot;
fdtrr=Ctrr*ztrr_dot;

Fzfl=fstfl+fdtfl;
Fzfr=fstfr+fdtfr;
Fzrl=fstrl+fdtrl;
Fzrr=fstrr+fdtrr;
%% Suspension vertical force

%Vertical displacement of sprung mass in each station
zsfl=Z+Lsf*phi-Lf*theta+E;
zsfr=Z-Lsf*phi-Lf*theta+E;
zsrl=Z+Lsr*phi+Lr*theta+E;
zsrr=Z-Lsr*phi+Lr*theta+E;

zsfl_dot=Z_dot+Lsf*phi_dot-Lf*theta_dot;  %Z_dot=y(6)
zsfr_dot=Z_dot-Lsf*phi_dot-Lf*theta_dot;
zsrl_dot=Z_dot+Lsr*phi_dot+Lr*theta_dot;
zsrr_dot=Z_dot-Lsr*phi_dot+Lr*theta_dot;

% Suspension deflection in each station
zwfl=zsfl-zufl+E; 
zwfr=zsfr-zufr+E;
zwrl=zsrl-zurl+E;
zwrr=zsrr-zurr+E;

zwfl_dot=zsfl_dot-zufl_dot;
zwfr_dot=zsfr_dot-zufr_dot;
zwrl_dot=zsrl_dot-zurl_dot;
zwrr_dot=zsrr_dot-zurr_dot;

%  nonlinear functions of suspension deflection (fsi & fdi)
fsfl=Ks1fl*zwfl;
fsfr=Ks1fr*zwfr;
fsrl=Ks1rl*zwrl;
fsrr=Ks1rr*zwrr;

fdfl=Cs1fl*zwfl_dot;
fdfr=Cs1fr*zwfr_dot;
fdrl=Cs1rl*zwrl_dot;
fdrr=Cs1rr*zwrr_dot;

Fsfl=fsfl+fdfl;
Fsfr=fsfr+fdfr;
Fsrl=fsrl+fdrl;
Fsrr=fsrr+fdrr;

out=[ztfl,ztfr,ztrl,ztrr,...
    ztfl_dot,ztfr_dot,ztrl_dot,ztrr_dot,...
    Fzfl,Fzfr,Fzrl,Fzrr,...
    zsfl,zsfr,zsrl,zsrr,...
    zsfl_dot,zsfr_dot,zsrl_dot,zsrr_dot,...
    zwfl,zwfr,zwrl,zwrr,...
    zwfl_dot,zwfr_dot,zwrl_dot,zwrr_dot,...
    Fsfl,Fsfr,Fsrl,Fsrr];


end

