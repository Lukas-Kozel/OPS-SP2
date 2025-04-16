function dxlam = duocopterOptimalOde(t, xlam, g, m1, m2, d)

    % Rozdělení stavových a costate proměnných
    x = xlam(1:6);
    lambda = xlam(7:12);

    % Získání řízení pomocí optimalU
    u = optimalU(d, lambda(4), lambda(5), lambda(6), m1, m2, x(3));

    % Stavová dynamika
    dx = duocopter_ode(t, x, g, m1, m2, d, u(1), u(2));

    % Costate dynamika
    dlambda = duocopterCostate(x, lambda, u(1), u(2), g, m1, m2, d);

    dxlam = [dx; dlambda];
end
