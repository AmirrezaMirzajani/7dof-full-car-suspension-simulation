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



tf=10;
t_zr=linspace(0,tf,length(zr));
plot(t_zr,zr)
grid on

zr_interp = interp1(t_zr, zr, t14, 'linear', 'extrap');
zr_dot_interp = interp1(t_zr, zr_dot, t14, 'linear', 'extrap');
