clc;
clear;
close all;
Vx=30;
RC=1;
%% Initialization
phi_odt_0=0.01; %4
theta_odt_0=0; %5
z_dot_0=0; %6
zufl_dot_0=0; %7
zufr_dot_0=0; %8
zurl_dot_0=0; %9
zurr_dot_0=0; %10
phi0=0; %16
theta0=0; %17
zufl0=0.01; %18
zufr0=0.01; %19
zurl0=0; %20
zurr0=0; %21
Z0=0; %22


h = 0.01;
t0 = 0;
tF = 5;

t14 = t0:h:tF;
n = length(t14);

hold_on_flag = 1;
colors = {'r', 'b', 'g',[0.5 0 0.5],'k'};
lineStyles = {'--', '-', ':','-.','-'};


%% Define zeta values
zeta_values=[0.2,0.3,0.4,0.5];
Fsfl_all = zeros(length(zeta_values), n); % To store Fsfl for each zeta
y14(1,:)=[phi_odt_0 theta_odt_0 z_dot_0 zufl_dot_0 zufr_dot_0 zurl_dot_0  ...
    zurr_dot_0 phi0 theta0 zufl0 zufr0 zurl0 zurr0 Z0];
n_RK5=length(t14)-1;

for j = 1:length(zeta_values)
    [zr_interp, zr_dot_interp] = generate_random_profile(t14, Vx,RC);


counter=0;
Per=0;
for i=1:(n_RK5)
    counter=counter+1;
    n_RK5_100=n_RK5/100;
    if counter==n_RK5_100
        Per=Per+1
        counter=0;
    end
   
            zrfl = zr_interp(i);
        zrfr = zr_interp(i);
        zrrl = zr_interp(i);
        zrrr = zr_interp(i);
        
        zrfl_dot = zr_dot_interp(i);
        zrfr_dot = zr_dot_interp(i);
        zrrl_dot = zr_dot_interp(i);
        zrrr_dot = zr_dot_interp(i);
    kx1=Zeta_comparison_suspension(t14(i),y14(i,:),zeta_values(j),Vx,zrfl,zrfr,zrrl,zrrr,zrfl_dot,zrfr_dot,zrrl_dot,zrrr_dot);
    kx2=Zeta_comparison_suspension(t14(i)+0.25*h,y14(i,:)+(1/4)*h*kx1,zeta_values(j),Vx,zrfl,zrfr,zrrl,zrrr,zrfl_dot,zrfr_dot,zrrl_dot,zrrr_dot);
    kx3=Zeta_comparison_suspension(t14(i)+0.25*h,y14(i,:)+((1/8)*kx1+(1/8)*kx2)*h,zeta_values(j),Vx,zrfl,zrfr,zrrl,zrrr,zrfl_dot,zrfr_dot,zrrl_dot,zrrr_dot);
    kx4=Zeta_comparison_suspension(t14(i)+0.5*h,y14(i,:)+(-0.5*kx2+kx3)*h,zeta_values(j),Vx,zrfl,zrfr,zrrl,zrrr,zrfl_dot,zrfr_dot,zrrl_dot,zrrr_dot);
    kx5=Zeta_comparison_suspension(t14(i)+(3/4)*h,y14(i,:)+((3/16)*kx1+(9/16)*kx4)*h,zeta_values(j),Vx,zrfl,zrfr,zrrl,zrrr,zrfl_dot,zrfr_dot,zrrl_dot,zrrr_dot);
    kx6=Zeta_comparison_suspension(t14(i)+h,y14(i,:)+((-3/7)*kx1+(2/7)*kx2+(12/7)*kx3-(12/7)*kx4+(8/7)*kx5)*h,zeta_values(j),Vx,zrfl,zrfr,zrrl,zrrr,zrfl_dot,zrfr_dot,zrrl_dot,zrrr_dot);
    y14(i+1,:)=y14(i,:)+(7*kx1+32*kx3+12*kx4+32*kx5+7*kx6)*h/90;
end

%%
phi_dot=y14(:,1); % Roll rate
theta_dot=y14(:,2); % pitch rate
Z_dot=y14(:,3); % Vertical speed
zufl_dot=y14(:,4); %Vertial speed of unsprung mass (front-left)
zufr_dot=y14(:,5); %Vertial speed of unsprung mass (front-right)
zurl_dot=y14(:,6); %Vertial speed of unsprung mass (rear-left)
zurr_dot=y14(:,7); %Vertial speed of unsprung mass (rear-right)
phi=y14(:,8); % Roll angle
theta=y14(:,9); % pitch angle
zufl=y14(:,10); %Vertial displacement of unsprung mass (front-lef)
zufr=y14(:,11); %Vertial displacement of unsprung mass (front-right)
zurl=y14(:,12); %Vertial displacement of unsprung mass (rear-left)
zurr=y14(:,13); %Vertial displacement of unsprung mass (rear-right)
Z=y14(:,14); % Vertical displacement


%%
n=length(t14);
Suspention=zeros(n,41);
zeta=zeta_values(j);
for i=1:n
    Suspention(i,:)=suspension_Zeta(t14(i),phi_dot(i),theta_dot(i),Z_dot(i),zufl_dot(i),zufr_dot(i)...
        ,zurl_dot(i),zurr_dot(i),phi(i),theta(i),zufl(i),zufr(i),zurl(i),zurr(i),...
        Z(i),zeta,zrfl, zrfr, zrrl, zrrr, zrfl_dot, zrfr_dot, zrrl_dot, zrrr_dot);
end

ztfl=Suspention(:,1);
ztfr=Suspention(:,2);
ztrl=Suspention(:,3);
ztrr=Suspention(:,4);
ztfl_dot=Suspention(:,5);
ztfr_dot=Suspention(:,6);
ztrl_dot=Suspention(:,7);
ztrr_dot=Suspention(:,8);
Fzfl=Suspention(:,9);
Fzfr=Suspention(:,10);
Fzrl=Suspention(:,11);
Fzrr=Suspention(:,12);
zsfl=Suspention(:,13);
zsfr=Suspention(:,14);
zsrl=Suspention(:,15);
zsrr=Suspention(:,16);
zsfl_dot=Suspention(:,17);
zsfr_dot=Suspention(:,18);
zsrl_dot=Suspention(:,19);
zsrr_dot=Suspention(:,20);
zwfl=Suspention(:,21);
zwfr=Suspention(:,22);
zwrl=Suspention(:,23);
zwrr=Suspention(:,24);
zwfl_dot=Suspention(:,25);
zwfr_dot=Suspention(:,26);
zwrl_dot=Suspention(:,27);
zwrr_dot=Suspention(:,28);
Fsfl=Suspention(:,29);
Fsfr=Suspention(:,30);
Fsrl=Suspention(:,31);
Fsrr=Suspention(:,32);
z2dot_sfl=Suspention(:,33);
z2dot_sfr=Suspention(:,34);
z2dot_srl=Suspention(:,35);   
z2dot_srr=Suspention(:,36);
ufl=Suspention(:,37);
ufr=Suspention(:,38);
url=Suspention(:,39);   
urr=Suspention(:,40);
Z2dot=Suspention(:,41);

%% plot
fig_num = 1;
figure(fig_num);
plot_function_comparison(t14,{phi_dot},{'$\dot{phi}$'}, zeta_values, j, colors, lineStyles);
fig_num = fig_num + 1;
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{theta_dot},{'$\dot{theta}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Z_dot},{'$\dot{Z}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zufl_dot,zufr_dot,zurl_dot,zurr_dot},{'$\dot{z}_{ufl}$','$\dot{z}_{ufr}$','$\dot{z}_{url}$','$\dot{z}_{urr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{phi},{'${phi}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{theta},{'${theta}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zufl,zufr,zurl,zurr},{'$z_{ufl}$','$z_{ufr}$','$z_{url}$','$z_{urr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Z},{'$Z$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{ztfl,ztfr,ztrl,ztrr},{'$z_{tfl}$','$z_{tfr}$','$z_{trl}$','$z_{trr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{ztfl_dot,ztfr_dot,ztrl_dot,ztrr_dot},{'$\dot{z}_{tfl}$','$\dot{z}_{tfr}$','$\dot{z}_{trl}$','$\dot{z}_{trr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Fzfl,Fzfr,Fzrl,Fzrr},{'$F_{zfl}$','$F_{zfr}$','$F_{zrl}$','$F_{zrr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zsfl,zsfr,zsrl,zsrr},{'$z_{sfl}$','$z_{sfr}$','$z_{srl}$','$z_{srr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zsfl_dot,zsfr_dot,zsrl_dot,zsrr_dot},{'$\dot{z}_{sfl}$','$\dot{z}_{sfr}$','$\dot{z}_{srl}$','$\dot{z}_{srr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zwfl,zwfr,zwrl,zwrr},{'$z_{wfl}$','$z_{wfr}$','$z_{wrl}$','$z_{wrr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zwfl_dot,zwfr_dot,zwrl_dot,zwrr_dot},{'$\dot{z}_{wfl}$','$\dot{z}_{wfr}$','$\dot{z}_{wrl}$','$\dot{z}_{wrr}$'}, zeta_values, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Fsfl,Fsfr,Fsrl,Fsrr},{'$F_{sfl}$','$F_{sfr}$','$F_{srl}$','$F_{srr}$'}, zeta_values, j, colors, lineStyles );
end



figure(1)
legend({['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))]},'Interpreter','latex','Location','best')
figure(2)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(3)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(4)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(5)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(6)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(7)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(8)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(9)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(10)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(11)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(12)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(13)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(14)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(15)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')
figure(16)
legend(['$\zeta$=',num2str(zeta_values(1))],['$\zeta$=',num2str(zeta_values(2))],['$\zeta$=',num2str(zeta_values(3))],['$\zeta$=',num2str(zeta_values(4))],'Interpreter','latex','Location','best')



  figHandles = findall(0, 'Type', 'figure'); for i = 1:length(figHandles) set(figHandles(i), 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); end
