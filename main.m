clc;
clear;
close all;

[g, m, d, t0, x0, tf] = ops_zadani_2_2025_data(2);
m1 = m(1);
m2 = m(2);

%% Výpis symbolických rovnic modelu
% Model duokoptéry dle zadání:
%   ẋ₁ = x₄,
%   ẋ₂ = x₅,
%   ẋ₃ = x₆,
%   ẋ₄ = ( sin(x₃)*(u₁ + u₂) )/(m₁ + m₂),
%   ẋ₅ = - g + ( cos(x₃)*(u₁ + u₂) )/(m₁ + m₂),
%   ẋ₆ = ( u₁/(m₁*d) ) - ( u₂/(m₂*d) )
syms x1 x2 x3 x4 x5 x6 u1_sym u2_sym
dx1_sym = x4;
dx2_sym = x5;
dx3_sym = x6;
dx4_sym = ( sin(x3)*(u1_sym+u2_sym) )/(m1+m2);
dx5_sym = - g + ( cos(x3)*(u1_sym+u2_sym) )/(m1+m2);
dx6_sym = ( u1_sym/(m1*d) ) - ( u2_sym/(m2*d) );
disp('Symbolické rovnice modelu:');
disp(dx1_sym);
disp(dx2_sym);
disp(dx3_sym);
disp(dx4_sym);
disp(dx5_sym);
disp(dx6_sym);

ukol2(g, m1, m2, d, t0, tf, x0);
