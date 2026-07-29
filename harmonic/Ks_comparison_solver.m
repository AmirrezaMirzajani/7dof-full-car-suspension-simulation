clc;
clear;
close all;

%% Initialization
random = input('random= '); 
phi_odt_0 = 0; 
theta_odt_0 = 0; 
z_dot_0 = 0; 
zufl_dot_0 = 0; 
zufr_dot_0 = 0; 
zurl_dot_0 = 0; 
zurr_dot_0 = 0;
phi0 = 0; 
theta0 = 0; 
zufl0 = 0; 
zufr0 = 0; 
zurl0 = 0; 
zurr0 = 0;
Z0 = 0;

h = 0.01;
t0 = 0;
tF = 2;

t14 = t0:h:tF;
n = length(t14);

% Define Ks as a vector for the range
Ks = linspace(50000, 60000, n);
zeta=linspace(0.1, 0.4, n);
ms=220;
C_cr=2*sqrt(ms*Ks);
Cs=zeta.*C_cr;
%% Harmonic inputs
A1 = -0.02;
A2 = -0.03;
A3 = -0.01;
A4 = 0.02;
A5 = 0.03;
w1 = 3;
w2 = 5;
w3 = 8;
w4 = 5;
w5 = 6;
t = t14;
R_harmonic = A1*sin(w1*t) + A2*cos(w2*t) - A3*sin(w3*t) + A4*cos(w4*t);
R_harmonic_dot = A1*w1*cos(w1*t) - A2*w2*sin(w2*t) - A3*w3*cos(w3*t) - A4*w4*sin(w4*t);

zrfl = R_harmonic;
zrfr = R_harmonic;
zrrl = R_harmonic;
zrrr = R_harmonic;

zrfl_dot = R_harmonic_dot;
zrfr_dot = R_harmonic_dot;
zrrl_dot = R_harmonic_dot;
zrrr_dot = R_harmonic_dot;

%% Vehicle parameters
Lf = 1; % longitudinal distance from c.g to front tires
Lr = 1.5; % longitudinal distance from c.g to rear tires
Lsf = 0.75; % half tracks of the front axles
Lsr = 0.75; % half tracks of the rear axles

% Linear spring and damping force coefficients for tires
Ks1fl = Ks; 
Cs1fl = Cs;

Ktfl = 190000;
Ctfl = 70;

Ks1fr = Ks;
Cs1fr = Cs;
Ktfr = 190000;
Ctfr = 70;

Ks1rl = Ks;
Cs1rl = Cs;
Ktrl = 190000;
Ctrl = 70;

Ks1rr = Ks;
Cs1rr = Cs;
Ktrr = 190000;
Ctrr = 70;
%%
ztfl_dot = zeros(1, n); 
ztfr_dot = zeros(1, n); 
ztrl_dot = zeros(1, n); 
ztrr_dot = zeros(1, n); 

ztfl = zeros(1, n); 
ztfr = zeros(1, n); 
ztrl = zeros(1, n); 
ztrr = zeros(1, n); 

fstfl = zeros(1, n); 
fstfr = zeros(1, n); 
fstrl = zeros(1, n); 
fstrr = zeros(1, n); 

fdtfl = zeros(1, n); 
fdtfr = zeros(1, n); 
fdtrl = zeros(1, n); 
fdtrr = zeros(1, n); 

Fzfl = zeros(1, n); 
Fzfr = zeros(1, n); 
Fzrl = zeros(1, n); 
Fzrr = zeros(1, n); 

% Suspension vertical force
zsfl = zeros(1, n); 
zsfr = zeros(1, n); 
zsrl = zeros(1, n); 
zsrr = zeros(1, n); 

zsfl_dot = zeros(1, n); 
zsfr_dot = zeros(1, n); 
zsrl_dot = zeros(1, n); 
zsrr_dot = zeros(1, n); 

zwfl = zeros(1, n); 
zwfr = zeros(1, n); 
zwrl = zeros(1, n); 
zwrr = zeros(1, n); 

zwfl_dot = zeros(1, n); 
zwfr_dot = zeros(1, n); 
zwrl_dot = zeros(1, n); 
zwrr_dot = zeros(1, n); 

fsfl = zeros(1, n); 
fsfr = zeros(1, n); 
fsrl = zeros(1, n); 
fsrr = zeros(1, n); 

fdfl = zeros(1, n); 
fdfr = zeros(1, n); 
fdrl = zeros(1, n); 
fdrr = zeros(1, n); 

Fsfl = zeros(1, n); 
Fsfr = zeros(1, n); 
Fsrl = zeros(1, n); 
Fsrr = zeros(1, n); 

%%
%% Initial conditions and main loop for Runge-Kutta
for j = 1:n
    % Set initial conditions for state vector y14
    y14 = zeros(length(t14), 14);
    y14(1,:) = [phi_odt_0, theta_odt_0, z_dot_0, zufl_dot_0, zufr_dot_0, ...
                zurl_dot_0, zurr_dot_0, phi0, theta0, zufl0, zufr0, ...
                zurl0, zurr0, Z0];
    
    % Runge-Kutta 5th order integration
    n_RK5 = length(t14) - 1;
    counter = 0;
    Per = 0;
    for i = 1:n_RK5
        counter = counter + 1;
        n_RK5_100 = n_RK5 / 100;
        if counter == n_RK5_100
            Per = Per + 1
            counter = 0;
        end

        kx1 = vehicle_suspension_Ks(t14(i), y14(i,:), Ks(j),Cs(j));
        kx2 = vehicle_suspension_Ks(t14(i) + 0.25 * h, y14(i,:) + (1/4) * h * kx1, Ks(j),Cs(j));
        kx3 = vehicle_suspension_Ks(t14(i) + 0.25 * h, y14(i,:) + ((1/8) * kx1 + (1/8) * kx2) * h, Ks(j),Cs(j));
        kx4 = vehicle_suspension_Ks(t14(i) + 0.5 * h, y14(i,:) + (-0.5 * kx2 + kx3) * h, Ks(j),Cs(j));
        kx5 = vehicle_suspension_Ks(t14(i) + (3/4) * h, y14(i,:) + ((3/16) * kx1 + (9/16) * kx4) * h, Ks(j),Cs(j));
        kx6 = vehicle_suspension_Ks(t14(i) + h, y14(i,:) + ((-3/7) * kx1 + (2/7) * kx2 + (12/7) * kx3 - (12/7) * kx4 + (8/7) * kx5) * h, Ks(j),Cs(j));
        
        y14(i + 1, :) = y14(i, :) + (7 * kx1 + 32 * kx3 + 12 * kx4 + 32 * kx5 + 7 * kx6) * h / 90;
    end

    phi_dot = y14(:,1); % Roll rate
    theta_dot = y14(:,2); % Pitch rate
    Z_dot = y14(:,3); % Vertical speed
    zufl_dot = y14(:,4); % Vertical speed of unsprung mass (front-left)
    zufr_dot = y14(:,5); % Vertical speed of unsprung mass (front-right)
    zurl_dot = y14(:,6); % Vertical speed of unsprung mass (rear-left)
    zurr_dot = y14(:,7); % Vertical speed of unsprung mass (rear-right)
    phi = y14(:,8); % Roll angle
    theta = y14(:,9); % Pitch angle
    zufl = y14(:,10); % Vertical displacement of unsprung mass (front-left)
    zufr = y14(:,11); % Vertical displacement of unsprung mass (front-right)
    zurl = y14(:,12); % Vertical displacement of unsprung mass (rear-left)
    zurr = y14(:,13); % Vertical displacement of unsprung mass (rear-right)
    Z = y14(:,14); % Vertical displacement

    %% Compute suspension forces (Fsfl)
    E = 10^-6;

    % Calculate deflections and forces
ztfl_dot(j) = zufl_dot(j) - zrfl_dot(j) + E;
ztfr_dot(j) = zufr_dot(j) - zrfr_dot(j) + E;
ztrl_dot(j) = zurl_dot(j) - zrrl_dot(j) + E;
ztrr_dot(j) = zurr_dot(j) - zrrr_dot(j) + E;

ztfl(j) = zufl(j) - zrfl(j) + E;
ztfr(j) = zufr(j) - zrfr(j) + E;
ztrl(j) = zurl(j) - zrrl(j) + E;
ztrr(j) = zurr(j) - zrrr(j) + E;

fstfl(j) = Ktfl * ztfl(j);
fstfr(j) = Ktfr * ztfr(j);
fstrl(j) = Ktrl * ztrl(j);
fstrr(j) = Ktrr * ztrr(j);

fdtfl(j) = Ctfl * ztfl_dot(j);
fdtfr(j) = Ctfr * ztfr_dot(j);
fdtrl(j) = Ctrl * ztrl_dot(j);
fdtrr(j) = Ctrr * ztrr_dot(j);

Fzfl(j) = fstfl(j) + fdtfl(j);
Fzfr(j) = fstfr(j) + fdtfr(j);
Fzrl(j) = fstrl(j) + fdtrl(j);
Fzrr(j) = fstrr(j) + fdtrr(j);

%% Suspension vertical force
zsfl(j) = Z(j) + Lsf * phi(j) - Lf * theta(j) + E;
zsfr(j) = Z(j) - Lsf * phi(j) - Lf * theta(j) + E;
zsrl(j) = Z(j) + Lsr * phi(j) + Lr * theta(j) + E;
zsrr(j) = Z(j) - Lsr * phi(j) + Lr * theta(j) + E;

zsfl_dot(j) = Z_dot(j) + Lsf * phi_dot(j) - Lf * theta_dot(j);
zsfr_dot(j) = Z_dot(j) - Lsf * phi_dot(j) - Lf * theta_dot(j);
zsrl_dot(j) = Z_dot(j) + Lsr * phi_dot(j) + Lr * theta_dot(j);
zsrr_dot(j) = Z_dot(j) - Lsr * phi_dot(j) + Lr * theta_dot(j);

zwfl(j) = zsfl(j) - zufl(j) + E;
zwfr(j) = zsfr(j) - zufr(j) + E;
zwrl(j) = zsrl(j) - zurl(j) + E;
zwrr(j) = zsrr(j) - zurr(j) + E;

zwfl_dot(j) = zsfl_dot(j) - zufl_dot(j);
zwfr_dot(j) = zsfr_dot(j) - zufr_dot(j);
zwrl_dot(j) = zsrl_dot(j) - zurl_dot(j);
zwrr_dot(j) = zsrr_dot(j) - zurr_dot(j);

% Nonlinear suspension forces with (j) for correct access to stiffness values
fsfl(j) = Ks1fl(j) * zwfl(j);
fsfr(j) = Ks1fr(j) * zwfr(j);
fsrl(j) = Ks1rl(j) * zwrl(j);
fsrr(j) = Ks1rr(j) * zwrr(j);

fdfl(j) = Cs1fl(j) * zwfl_dot(j);
fdfr(j) = Cs1fr(j) * zwfr_dot(j);
fdrl(j) = Cs1rl(j) * zwrl_dot(j);
fdrr(j) = Cs1rr(j) * zwrr_dot(j);

Fsfl(j) = fsfl(j) + fdfl(j);
Fsfr(j) = fsfr(j) + fdfr(j);
Fsrl(j) = fsrl(j) + fdrl(j);
Fsrr(j) = fsrr(j) + fdrr(j);


end

%% fl
% Create 2x2 subplot layout
figure;

% First subplot (3D Surface Plot)
subplot(2, 2, 1); % 2 rows, 2 columns, 1st plot
[T, K] = meshgrid(t14, Ks);
Fsfl_matrix = repmat(Fsfl, length(Ks), 1);
surf(K, T, Fsfl_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsfl)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Second subplot (3D Mesh Plot)
subplot(2, 2, 2); % 2 rows, 2 columns, 2nd plot
meshz(K, T, Fsfl_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsfl)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Third subplot (2D Contour Plot)
subplot(2, 2, 3); % 2 rows, 2 columns, 3rd plot
contourf(K, T, Fsfl_matrix); % Plot 2D contour
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('2D Contour Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Fourth subplot (Periodogram)
subplot(2, 2, 4); % 2 rows, 2 columns, 4th plot
Fs = 50; % Sampling frequency (Hz)
T = 1/Fs; % Sampling period (s)
L = length(Fsfl_matrix); % Length of the signal
t = (0:L-1)*T; % Time vector

% Compute the periodogram
[pxx, f] = periodogram(Fsfl_matrix, [], [], Fs);
plot(f, 10*log10(pxx)); % Plot the periodogram
title('Periodogram of Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
grid on;




%% %%%%%%%%%%% fr
% Create 2x2 subplot layout
figure;

% First subplot (3D Surface Plot)
subplot(2, 2, 1); % 2 rows, 2 columns, 1st plot
[T, K] = meshgrid(t14, Ks);
Fsfr_matrix = repmat(Fsfr, length(Ks), 1);
surf(K, T, Fsfr_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsfr)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Second subplot (3D Mesh Plot)
subplot(2, 2, 2); % 2 rows, 2 columns, 2nd plot
meshz(K, T, Fsfr_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsfr)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Third subplot (2D Contour Plot)
subplot(2, 2, 3); % 2 rows, 2 columns, 3rd plot
contourf(K, T, Fsfr_matrix); % Plot 2D contour
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('2D Contour Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Fourth subplot (Periodogram)
subplot(2, 2, 4); % 2 rows, 2 columns, 4th plot
Fs = 50; % Sampling frequency (Hz)
T = 1/Fs; % Sampling period (s)
L = length(Fsfr_matrix); % Length of the signal
t = (0:L-1)*T; % Time vector

% Compute the periodogram
[pxx, f] = periodogram(Fsfr_matrix, [], [], Fs);
plot(f, 10*log10(pxx)); % Plot the periodogram
title('Periodogram of Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
grid on;

%% %%%%%%%%%%%%% rl
% Create 2x2 subplot layout
figure;

% First subplot (3D Surface Plot)
subplot(2, 2, 1); % 2 rows, 2 columns, 1st plot
[T, K] = meshgrid(t14, Ks);
Fsrl_matrix = repmat(Fsrl, length(Ks), 1);
surf(K, T, Fsrl_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsrl)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Second subplot (3D Mesh Plot)
subplot(2, 2, 2); % 2 rows, 2 columns, 2nd plot
meshz(K, T, Fsrl_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsrl)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Third subplot (2D Contour Plot)
subplot(2, 2, 3); % 2 rows, 2 columns, 3rd plot
contourf(K, T, Fsrl_matrix); % Plot 2D contour
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('2D Contour Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Fourth subplot (Periodogram)
subplot(2, 2, 4); % 2 rows, 2 columns, 4th plot
Fs = 50; % Sampling frequency (Hz)
T = 1/Fs; % Sampling period (s)
L = length(Fsrl_matrix); % Length of the signal
t = (0:L-1)*T; % Time vector

% Compute the periodogram
[pxx, f] = periodogram(Fsrl_matrix, [], [], Fs);
plot(f, 10*log10(pxx)); % Plot the periodogram
title('Periodogram of Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
grid on;

%% %%%%%%%%%%%%%%%%%%%%%

% Create 2x2 subplot layout
figure;

% First subplot (3D Surface Plot)
subplot(2, 2, 1); % 2 rows, 2 columns, 1st plot
[T, K] = meshgrid(t14, Ks);
Fsrr_matrix = repmat(Fsrr, length(Ks), 1);
surf(K, T, Fsrr_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsrr)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Second subplot (3D Mesh Plot)
subplot(2, 2, 2); % 2 rows, 2 columns, 2nd plot
meshz(K, T, Fsrr_matrix); % Plot 3D surface
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
zlabel('Suspension Force (Fsrr)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('3D Surface Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Third subplot (2D Contour Plot)
subplot(2, 2, 3); % 2 rows, 2 columns, 3rd plot
contourf(K, T, Fsrr_matrix); % Plot 2D contour
xlabel('Stiffness (Ks)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Time(s)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
title('2D Contour Plot of Suspension Force vs. Time and Stiffness', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
colorbar;
grid on;

% Fourth subplot (Periodogram)
subplot(2, 2, 4); % 2 rows, 2 columns, 4th plot
Fs = 50; % Sampling frequency (Hz)
T = 1/Fs; % Sampling period (s)
L = length(Fsrr_matrix); % Length of the signal
t = (0:L-1)*T; % Time vector

% Compute the periodogram
[pxx, f] = periodogram(Fsrr_matrix, [], [], Fs);
plot(f, 10*log10(pxx)); % Plot the periodogram
title('Periodogram of Signal', 'FontSize', 14, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('Frequency (Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
ylabel('Power/Frequency (dB/Hz)', 'FontSize', 12, 'FontName', 'Arial', 'Interpreter', 'latex');
grid on;

