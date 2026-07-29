function out=suspension_random(t14,phi_dot,theta_dot,Z_dot,zufl_dot,zufr_dot,zurl_dot,zurr_dot,phi,theta,zufl,...
zufr,zurl,zurr,Z,zrfl, zrfr, zrrl, zrrr, zrfl_dot, zrfr_dot, zrrl_dot, zrrr_dot)
%% 
Vy=0;
%% parameters
Vx=15; 
Mt=1030;
Mb=910;
Ms =900; %sprung mass of the vehicle
Lf=1; %longitudinal distance from c.g to front tires
Lr=1.5; %longitudinal distance from c.g to rear tires
Lsf=0.75; % half tracks of the front axles
Lsr=0.75; % half tracks of the rear axles
g=9.81;
Ixx=300; %Roll moment of interia
Iyy=1058; %pitch moment of interia
Izz=1088; %yaw moment of interia
hg=0.3; % the height of c.g. of the vehicle
hd=0.3;
hp=0.2;  %Height of the pitch
hc=0.1; % Height of the roll centre
musfl=30; 
musfr=30; 
musrl=30; 
musrr=30;
% linear spring force coefficients (left front wheel)
Ks1fl=50000; 
%linear damping force coefficients (left front wheel)
Cs1fl=1385;

%Tyre spring and damping coefficients (left front whel)
Ktfl=190000;
Ctfl=70;
% linear spring force coefficients (right front wheel)
Ks1fr=50000;
%linear damping force coefficients (right front wheel)
Cs1fr=1385;
%Tyre spring and damping coefficients (right front whel)
Ktfr=190000;
Ctfr=70;
% linear spring force coefficients (left rear wheel)
Ks1rl=50000;
%linear damping force coefficients (left rear wheel)
Cs1rl=1385;
%Tyre spring and damping coefficients (left rear whel)
Ktrl=190000;
Ctrl=70;
% linear spring force coefficients (right rear wheel)
Ks1rr=50000;
%linear damping force coefficients (right rear wheel)
Cs1rr=1385;
%Tyre spring and damping coefficients (right rear whel)
Ktrr=190000;
Ctrr=70;

%%
ufl=0;
ufr=0;
url=0;
urr=0;
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

z2dot_sfl=1/musfl*(Fsfl-Fzfl-musfl*g-ufl+E); %zufl_dot
z2dot_sfr=1/musfr*(Fsfr-Fzfr-musfr*g-ufr+E); %zufr_dot
z2dot_srl=1/musrl*(Fsrl-Fzrl-musrl*g-url+E); %zurl_dot     
z2dot_srr=1/musrr*(Fsrr-Fzrr-musrr*g-urr+E); %zurr_dot
Z2dot=1/Mb*((Mb*Vy*phi_dot+Mb*Vx*theta_dot-((Fsfl-ufl)+(Fsfr-ufr)+(Fsrl-url)+(Fsrr-urr))-Mb*g)+E); 

out=[ztfl,ztfr,ztrl,ztrr,...
    ztfl_dot,ztfr_dot,ztrl_dot,ztrr_dot,...
    Fzfl,Fzfr,Fzrl,Fzrr,...
    zsfl,zsfr,zsrl,zsrr,...
    zsfl_dot,zsfr_dot,zsrl_dot,zsrr_dot,...
    zwfl,zwfr,zwrl,zwrr,...
    zwfl_dot,zwfr_dot,zwrl_dot,zwrr_dot,...
    Fsfl,Fsfr,Fsrl,Fsrr,...
    z2dot_sfl,z2dot_sfr,z2dot_srl,z2dot_srr,...
    ufl,ufr,url,urr,Z2dot];


end

