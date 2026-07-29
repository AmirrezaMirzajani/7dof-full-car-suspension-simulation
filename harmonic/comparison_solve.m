clc
clc 
close all
Vx0=[20 50 80]/3.6;
%%
random = input('random= ');
phi_odt_0=zeros(1,length(Vx0)); 
theta_odt_0=zeros(1,length(Vx0)); 
z_dot_0=zeros(1,length(Vx0)); 
zufl_dot_0=zeros(1,length(Vx0));
zufr_dot_0=zeros(1,length(Vx0)); 
zurl_dot_0=zeros(1,length(Vx0)); 
zurr_dot_0=zeros(1,length(Vx0));
phi0=zeros(1,length(Vx0)); 
theta0=zeros(1,length(Vx0)); 
zufl0=zeros(1,length(Vx0)); 
zufr0=zeros(1,length(Vx0)); 
zurl0=zeros(1,length(Vx0)); 
zurr0=zeros(1,length(Vx0));
Z0=zeros(1,length(Vx0)); 

%%
h=0.001;
t0=0;
tF=3;

t14=t0:h:tF;


y14=zeros(length(t14),14);

for j=1:length(Vx0)
y14(1,:)=[phi_odt_0(j) theta_odt_0(j) z_dot_0(j) zufl_dot_0(j) zufr_dot_0(j) zurl_dot_0(j)  ...
    zurr_dot_0(j) phi0(j) theta0(j) zufl0(j) zufr0(j) zurl0(j) zurr0(j) Z0(j)];
n_RK5=length(t14)-1;
counter=0;
Per=0;
for i=1:(n_RK5)
    counter=counter+1;
    n_RK5_100=n_RK5/100;
    if counter==n_RK5_100
        Per=Per+1
        counter=0;
    end
    kx1=vehicle_suspension(t14(i),y14(i,:),Vx0(j));
    kx2=vehicle_suspension(t14(i)+0.25*h,y14(i,:)+(1/4)*h*kx1,Vx0(j));
    kx3=vehicle_suspension(t14(i)+0.25*h,y14(i,:)+((1/8)*kx1+(1/8)*kx2)*h,Vx0(j));
    kx4=vehicle_suspension(t14(i)+0.5*h,y14(i,:)+(-0.5*kx2+kx3)*h,Vx0(j));
    kx5=vehicle_suspension(t14(i)+(3/4)*h,y14(i,:)+((3/16)*kx1+(9/16)*kx4)*h,Vx0(j));
    kx6=vehicle_suspension(t14(i)+h,y14(i,:)+((-3/7)*kx1+(2/7)*kx2+(12/7)*kx3-(12/7)*kx4+(8/7)*kx5)*h,Vx0(j));
    y14(i+1,:)=y14(i,:)+(7*kx1+32*kx3+12*kx4+32*kx5+7*kx6)*h/90;
end
if j==1
    a='--';
    b=1;
elseif j==2
    b=1;
    a='-';
elseif j==3
    a=':';
    b=2;
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

for i=1:n
    Suspention(i,:)=suspension(random,t14(i),phi_dot(i),theta_dot(i),Z_dot(i),zufl_dot(i),zufr_dot(i)...
        ,zurl_dot(i),zurr_dot(i),phi(i),theta(i),zufl(i),zufr(i),zurl(i),zurr(i),Z(i));
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

%%
% fig1=figure(1);
% p1=plot(t14,Fsfl ,'Color', 'k','LineStyle',a,'linewidth',b);
% grid on
% hold on
% title('Fsfl')
% xlabel('\fontsize{9}  t' );
% ylabel('\fontsize{9} Fsfl');
fig_num = 1;
figure(fig_num);
plot_function_comparison(t14,{phi_dot},{'$\dot{phi}$'}, Vx0, j, colors, lineStyles);
fig_num = fig_num + 1;
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{theta_dot},{'$\dot{theta}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Z_dot},{'$\dot{Z}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zufl_dot,zufr_dot,zurl_dot,zurr_dot},{'$\dot{z}_{ufl}$','$\dot{z}_{ufr}$','$\dot{z}_{url}$','$\dot{z}_{urr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{phi},{'${phi}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{theta},{'${theta}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zufl,zufr,zurl,zurr},{'$z_{ufl}$','$z_{ufr}$','$z_{url}$','$z_{urr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Z},{'$Z$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{ztfl,ztfr,ztrl,ztrr},{'$z_{tfl}$','$z_{tfr}$','$z_{trl}$','$z_{trr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{ztfl_dot,ztfr_dot,ztrl_dot,ztrr_dot},{'$\dot{z}_{tfl}$','$\dot{z}_{tfr}$','$\dot{z}_{trl}$','$\dot{z}_{trr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Fzfl,Fzfr,Fzrl,Fzrr},{'$F_{zfl}$','$F_{zfr}$','$F_{zrl}$','$F_{zrr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zsfl,zsfr,zsrl,zsrr},{'$z_{sfl}$','$z_{sfr}$','$z_{srl}$','$z_{srr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zsfl_dot,zsfr_dot,zsrl_dot,zsrr_dot},{'$\dot{z}_{sfl}$','$\dot{z}_{sfr}$','$\dot{z}_{srl}$','$\dot{z}_{srr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zwfl,zwfr,zwrl,zwrr},{'$z_{wfl}$','$z_{wfr}$','$z_{wrl}$','$z_{wrr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{zwfl_dot,zwfr_dot,zwrl_dot,zwrr_dot},{'$\dot{z}_{wfl}$','$\dot{z}_{wfr}$','$\dot{z}_{wrl}$','$\dot{z}_{wrr}$'}, Vx0, j, colors, lineStyles );
figure(fig_num);
fig_num = fig_num + 1;
plot_function_comparison(t14,{Fsfl,Fsfr,Fsrl,Fsrr},{'$F_{sfl}$','$F_{sfr}$','$F_{srl}$','$F_{srr}$'}, Vx0, j, colors, lineStyles );
end

Vx0=[20 50 80];

figure(1)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(2)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(3)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(4)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(5)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(6)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(7)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(8)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(9)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(10)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(11)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(12)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(13)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(14)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(15)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best')
figure(16)
legend(['$V_{x}$=',num2str(Vx0(1))],['$V_{x}$=',num2str(Vx0(2))],['$V_{x}$=',num2str(Vx0(3))],'Interpreter','latex','Location','best','Interpreter','latex','Location','best')

  figHandles = findall(0, 'Type', 'figure'); for i = 1:length(figHandles) set(figHandles(i), 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); end