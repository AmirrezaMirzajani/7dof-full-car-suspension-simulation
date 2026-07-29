function dydt=Zeta_comparison_suspension(t14,y14,zeta)
%% 
delta=0;
Vy=0;
Vx=15;
% random=input('random: ');
random=0;
%% road profile
if random==0
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
elseif random==1
    %%
% % save('random_road_profile', 's' ,'zr','zr_dot');
% % load('random_road_profile', 's' ,'zr','zr_dot');
% Data=load('random_road_profile.mat');
% 
% s=Data.s;
% zr=Data.zr;
% zr_dot=Data.zr_dot;
% 
% s=s';
% zr=zr'*0.001;
% zr_dot=zr_dot'*0.001;
% 
% % N=X/300;
% % X_road=X-floor(N)*X;
% 
% X_road=300;
% 
% zrfl=interp1(s,zr,X_road);
% zrfr=interp1(s,zr,X_road);
% zrrl=interp1(s,zr,X_road);
% zrrr=interp1(s,zr,X_road);
% 
% zrfl_dot=interp1(s,zr_dot,X_road);
% zrfr_dot=interp1(s,zr_dot,X_road);
% zrrl_dot=interp1(s,zr_dot,X_road);
% zrrr_dot=interp1(s,zr_dot,X_road);

%%

n=2000;
Omega0=1;
Omega1=0.02*pi;
OmegaN=6*pi;
%% road class
RC=1;
if RC==1 % road class: A(very good)
    sigma=2*10^(-3);
    phi_Omega0=1*10^(-6);
    alpha=0.127;
elseif RC==2 % road class: B(good)
    sigma=4*10^(-3);
    phi_Omega0=4*10^(-6);
    alpha=0.127;
elseif RC==3 % road class: C(average)
     sigma=8*10^(-3);
     phi_Omega0=16*10^(-6);
     alpha=0.127;
elseif RC==4 % road class: D(poor)
     sigma=16*10^(-3);
     phi_Omega0=64*10^(-6);
     alpha=0.127;
elseif RC==5 % road class: E(very poor)
     sigma=32*10^(-3);
     phi_Omega0=256*10^(-6);
     alpha=0.127;
end

N=n;
delta_Omega=(OmegaN-Omega1)/(N-1);
Omega=Omega1:delta_Omega:OmegaN;
%% Phi(omega)
phi_Omega=zeros(1,length(Omega));
for i=1:length(Omega)
if Omega(i)>=0 && Omega(i)<=Omega1
    phi_Omega(i) = phi_Omega0*Omega1^(-2); 
elseif Omega(i)>Omega1 && Omega(i)<=OmegaN
    phi_Omega(i) = phi_Omega0*(Omega(i)/Omega0)^(-2); 
elseif Omega(i)>OmegaN
    phi_Omega(i)=0;
end
end
%%
% delta_Omega=(2*pi)/(L);
% omega0=V*delta_Omega;
% A=zeros(1,length(Omega));
% zr1=zeros(1,length(Omega));
phi=2*pi*randn(1,length(Omega));
% L0=0;
% LN=300;
L0=0;
LN=20;
s=L0:LN/N:LN-LN/N;
zr=zeros(1,length(Omega));
zr_dot=zeros(1,length(Omega));
% for i=1:length(Omega)
% A(i)=sqrt(phi_Omega(i)*(delta_Omega/pi));
% % zr1(i)=A(i)*sin(Omega(i)*s(i)-phi(i));
% % zr(i)=sum(zr1(i));
% end
A=sqrt(phi_Omega.*(delta_Omega/pi));
for i=1:length(Omega)
%     zr(i)=sum(A.*sin(Omega(i)*s(i)-phi));
%     zr(i)=sum(A.*sin(Omega(i)*s(i)+phi));
    zr(i)=sum(A.*cos(2*pi*Omega*s(i)+ phi));
    zr_dot(i)=sum(-2*pi*A.*sin(2*pi*Omega*s(i)+ phi));
end


end
%% parameters

Ks = 50000;
ms=220;
C_cr=2*sqrt(ms*Ks);
Cs=zeta*C_cr;

% Vx=15; 
Mb=910;
Lf=1; %longitudinal distance from c.g to front tires
Lr=1.5; %longitudinal distance from c.g to rear tires
Lsf=0.75; % half tracks of the front axles
Lsr=0.75; % half tracks of the rear axles
g=9.81;
Ixx=300; %Roll moment of interia
Iyy=1058; %pitch moment of interia
Izz=1088; %yaw moment of interia
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
Cs1fl=Cs;

%Tyre spring and damping coefficients (left front whel)
Ktfl=190000;
Ctfl=70;
% linear spring force coefficients (right front wheel)
Ks1fr=50000;
%linear damping force coefficients (right front wheel)
Cs1fr=Cs;
%Tyre spring and damping coefficients (right front whel)
Ktfr=190000;
Ctfr=70;
% linear spring force coefficients (left rear wheel)
Ks1rl=50000;
%linear damping force coefficients (left rear wheel)
Cs1rl=Cs;
%Tyre spring and damping coefficients (left rear whel)
Ktrl=190000;
Ctrl=70;
% linear spring force coefficients (right rear wheel)
Ks1rr=50000;
%linear damping force coefficients (right rear wheel)
Cs1rr=Cs;
%Tyre spring and damping coefficients (right rear whel)
Ktrr=190000;
Ctrr=70;


%%
dydt=zeros(size(y14));

phi_dot=y14(1); % Roll rate
theta_dot=y14(2); % pitch rate
Z_dot=y14(3); % Vertical speed
zufl_dot=y14(4); %Vertial speed of unsprung mass (front-left)
zufr_dot=y14(5); %Vertial speed of unsprung mass (front-right)
zurl_dot=y14(6); %Vertial speed of unsprung mass (rear-left)
zurr_dot=y14(7); %Vertial speed of unsprung mass (rear-right)
phi=y14(8); % Roll angle
theta=y14(9); % pitch angle
zufl=y14(10); %Vertial displacement of unsprung mass (front-lef)
zufr=y14(11); %Vertial displacement of unsprung mass (front-right)
zurl=y14(12); %Vertial displacement of unsprung mass (rear-left)
zurr=y14(13); %Vertial displacement of unsprung mass (rear-right)
Z=y14(14); % Vertical displacement

%%
ufl=0;
ufr=0;
url=0;
urr=0;
Fxfl=0;
Fxfr=0;
Fxrl=0;
Fxrr=0;
Fyfl=0;
Fyfr=0;
Fyrl=0;
Fyrr=0;
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


%%
E=10^-6;
dydt(1)=1/Ixx*((-Fsfl+Fsfr+ufl-ufr)*Lsf+(-Fsrl+Fsrr+url-urr)*Lsr+ ...
    ((Fxfl+Fxfr)*sin(delta)+(Fyfl+Fyfr)*cos(delta)+Fyrl+Fyrr)*hc+Mb*g*hd*sin(phi)+E); %phi_dot

dydt(2)=1/Iyy*((Fsfl+Fsfr-ufl-ufr)*Lf-(Fsrl+Fsrr-url-urr)*Lr+ ...
    (-(Fxfl+Fxfr)*cos(delta)+(Fyfl+Fyfr)*sin(delta)-Fxrl-Fxrr)*hp+Mb*g*hd*sin(theta)+E); %theta_dot

dydt(3)=1/Mb*((Mb*Vy*phi_dot+Mb*Vx*theta_dot-((Fsfl-ufl)+(Fsfr-ufr)+(Fsrl-url)+(Fsrr-urr))-Mb*g)+E); %zeta_dot

dydt(4)=1/musfl*(Fsfl-Fzfl-musfl*g-ufl+E); %zufl_dot
dydt(5)=1/musfr*(Fsfr-Fzfr-musfr*g-ufr+E); %zufr_dot
dydt(6)=1/musrl*(Fsrl-Fzrl-musrl*g-url+E); %zurl_dot     
dydt(7)=1/musrr*(Fsrr-Fzrr-musrr*g-urr+E); %zurr_dot

dydt(8)=phi_dot+E; % phi
dydt(9)=theta_dot+E;  %theta
dydt(10)=zufl_dot+E; %zufl
dydt(11)=zufr_dot+E; %zufr
dydt(12)=zurl_dot+E; %zurl
dydt(13)=zurr_dot+E; %zurr
dydt(14)=Z_dot+E; %Z


end

