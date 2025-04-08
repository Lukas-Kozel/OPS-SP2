function dx = duocopter_ode(t, x, g, m1, m2, d, u1, u2)

    dx = zeros(6,1);
    
    dx(1) = x(4);
    dx(2) = x(5);
    dx(3) = x(6);
    dx(4) = ( sin(x(3))*(u1 + u2) )/(m1 + m2);
    dx(5) = - g + ( cos(x(3))*(u1 + u2) )/(m1 + m2);
    dx(6) = ( u1/(m1 * d) ) - ( u2/(m2 * d) );
end