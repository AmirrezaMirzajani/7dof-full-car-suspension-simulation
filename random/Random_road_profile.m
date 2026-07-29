function out=Random_road_profile()

n=10001;
% clc
% clear
% close all
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




% %%
% plot(s,zr)
% grid on
% 
% if RC==1
%    title('Grade A')
% elseif RC==2
%    title('Grade B')
% elseif RC==3
%    title('Grade C')
% elseif RC==4
%    title('Grade D')
% elseif RC==5
%    title('Grade E')
% end
% 
% xlabel('road slongitudinal axis (m)');
% ylabel('z_{r}(mm)');
% 
% interp1(s,zr,200)
data=zeros(3,length(Omega));
data(1,:)=s;
data(2,:)=zr;
data(3,:)=zr_dot;
% out=[s,zr,zr_dot];
out=[data(1,:);data(2,:);data(3,:)];
end
