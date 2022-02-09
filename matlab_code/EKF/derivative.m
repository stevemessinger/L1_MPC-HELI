function xdot = derivative(x, y, u, t)

    % Our control gains
    Kpsibyx = gains(1);
    Kpsi = gains(2);
    Kv = gains(3);

    % Now, assembling the derivatives of each of the states.
    % Note, this is expanded out for readability -- can be done in a more
    % compact, but perhaps less-readable way too!

    % derivative of x, with V converted from knots to ft per sec
    xderivative = x(3) * sind(x(4)) * 6076/3600;

    % derivative of y, with V converted from knots to ft per sec
    yderivative = x(3) * cosd(x(4)) * 6076/3600;

    % derivative of V - a simple proportional control law on error in V
    if strcmp(model, 'simulation')
        Vderivative = Kv * (u(2) - x(3));
    elseif strcmp(model, 'data')
        Vderivative = Kv * (u(2) - y(3));
    end

    % derivative of psi - a simple proportional control law on error in x & psi
    if strcmp(model, 'simulation')
        psiderivative = Kpsibyx * (u(1) - x(1)) + Kpsi * (u(3) - x(4));
    elseif strcmp(model, 'data')
        psiderivative = Kpsibyx * (u(1) - y(1)) + Kpsi * (u(3) - y(4));
    end

    if psiderivative > 3
        psiderivative = 3;
    elseif psiderivative < -3
        psiderivative = -3;
    end

    xdot = [xderivative yderivative Vderivative psiderivative]';
end

