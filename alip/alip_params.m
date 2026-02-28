function params = alip_params()
    params.m = 32;          % masa [kg] segun paper
    params.H = 1.0;         % altura del CoM [m]
    params.g = 9.81;        % gravedad
    params.T = 0.35;        % duración del paso [s]

    params.alpha = 0.3;     % ganancia del controlador (0 = deadbeat)
end

