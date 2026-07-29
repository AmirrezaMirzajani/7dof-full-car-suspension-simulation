function [zr_interp, zr_dot_interp] = generate_random_profile(t, Vx,RC)
    n = 10001;

    Omega0 = 1;
    Omega1 = 0.02 * pi;
    OmegaN = 6 * pi;
    
    %% road class

    if RC == 1 % road class: A(very good)
        sigma = 2*10^(-3);
        phi_Omega0 = 1*10^(-6);
        alpha = 0.127;
    elseif RC == 2 % road class: B(good)
        sigma = 4*10^(-3);
        phi_Omega0 = 4*10^(-6);
        alpha = 0.127;
    elseif RC == 3 % road class: C(average)
        sigma = 8*10^(-3);
        phi_Omega0 = 16*10^(-6);
        alpha = 0.127;
    elseif RC == 4 % road class: D(poor)
        sigma = 16*10^(-3);
        phi_Omega0 = 64*10^(-6);
        alpha = 0.127;
    elseif RC == 5 % road class: E(very poor)
        sigma = 32*10^(-3);
        phi_Omega0 = 256*10^(-6);
        alpha = 0.127;
    end

    N = n;
    delta_Omega = (OmegaN - Omega1) / (N - 1);
    Omega = Omega1:delta_Omega:OmegaN;
    
    % Phi(omega) computation
    phi_Omega = zeros(1, length(Omega));
    for i = 1:length(Omega)
        if Omega(i) >= 0 && Omega(i) <= Omega1
            phi_Omega(i) = phi_Omega0 * Omega1^(-2);
        elseif Omega(i) > Omega1 && Omega(i) <= OmegaN
            phi_Omega(i) = phi_Omega0 * (Omega(i) / Omega0)^(-2);
        elseif Omega(i) > OmegaN
            phi_Omega(i) = 0;
        end
    end

    phi = 2 * pi * randn(1, length(Omega));

    L0 = 0;
    LN = 20;
    s = L0:LN / N:LN - LN / N;
    
    zr = zeros(1, length(t));
    zr_dot = zeros(1, length(t));
    
    A = sqrt(phi_Omega .* (delta_Omega / pi));
    for i = 1:length(t)
        % Compute zr and zr_dot as functions of time
        zr(i) = sum(A .* cos(2 * pi * Omega * (Vx * t(i)) + phi));
        zr_dot(i) = sum(-2 * pi * A .* sin(2 * pi * Omega * (Vx * t(i)) + phi));
    end
    
    zr_interp = zr;
    zr_dot_interp = zr_dot;
end
