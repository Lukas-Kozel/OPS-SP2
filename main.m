clc;
clear;
close all;

[g, m, d, t0, x0, tf] = ops_zadani_2_2025_data(2);
m1 = m(1);
m2 = m(2);
% m2 = 5;
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
dx4_sym = -( sin(x3)*(u1_sym+u2_sym) )/(m1+m2);
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

[xf, end_conds] = compute_end_state(d/2);

%% Linearizace okolo rovnovážného bodu
% Rovnovážný stav (x_e) a vstup (u_e)
x_eq = [0; 0; 0; 0; 0; 0];             
u_eq = [m1*g; m2*g];           
M = m1 + m2;

% Matice A = ∂f/∂x (linearizovaná dynamika)
% f = [ x4;
%       x5;
%       x6;
%      - sin(x3)*(u1+u2)/M;
%      - g + cos(x3)*(u1+u2)/M;
%       u1/(m1*d) - u2/(m2*d) ]
% Linearizace v bodě: x3 = 0, u = u_eq (tedy u1+u2 = M*g)
A = [ 0   0   0   1   0   0;
      0   0   0   0   1   0;
      0   0   0   0   0   1;
      0   0  -g   0   0   0;
      0   0   0   0   0   0;
      0   0   0   0   0   0];

% Matice B = ∂f/∂u
% f₄: ∂f₄/∂u1 = - sin(x3)/M, při x3=0 → 0; analogicky pro u2.
% f₅: ∂f₅/∂u1 = cos(x3)/M, u2 analogicky; při x3 = 0 → 1/M.
% f₆: ∂f₆/∂u1 = 1/(m1*d), ∂f₆/∂u2 = -1/(m2*d)
B = [ 0         0;
      0         0;
      0         0;
      0         0;
      1/M       1/M;
      1/(m1*d) -1/(m2*d)];

%% Definice referenčního bodu
x_ref = [1; 1.5; 0; 0; 0; 0];  
% feedforward vstup,  A*x_ref+B*u_ref = 0.
u_ref = pinv(B)*(-A*x_ref);

%% Návrh regulátoru - pole placement 
desired_poles = [-2 -2.5 -3 -3.5 -4 -4.5];
K = place(A, B, desired_poles);
disp('Vypočítaná matice K:');
disp(K);

%% regulace
%  u = u_ref - K*(x - x_ref)
cl_sys_tracking = @(t, x) A*x + B*(u_ref - K*(x - x_ref));

[t, x] = ode45(cl_sys_tracking, [t0 tf], x0);


u = zeros(length(t),2);
for i=1:length(t)
    u(i,:) = (u_ref - K*(x(i,:)'-x_ref))';
end


figure;
plot(t, x, 'LineWidth',1.5);
xlabel('Čas [s]');
ylabel('Stavové veličiny');
legend('x1','x2','x3','x4','x5','x6','Location','Best');
grid on;
title('Uzavřený systém – tracking (pole placement)');

figure;
plot(t, u, 'LineWidth',1.5);
xlabel('Čas [s]');
ylabel('Řízení [N]');
grid on;
legend('u1','u2','Location','Best');
title('Řídicí signály – tracking (pole placement)');

% Volitelně: trajektorie v rovině (x1,x2)
figure;
plot(x(:,1), x(:,2), 'b-', 'LineWidth',2); hold on;
plot(x0(1), x0(2), 'ro', 'MarkerSize',8, 'LineWidth',2);
plot(x_ref(1), x_ref(2), 'gx', 'MarkerSize',10, 'LineWidth',2);
grid on;
xlabel('x1'); ylabel('x2');
title('Trajektorie duokoptéry – sledování referenčního bodu');


%% Úkol 5 - stanovení optimálního řízení
% Symbolicka priprava
clc
close all
syms x [6 1] real
syms m1_syms m2_syms d_syms g_syms real
syms u1_sym u2_sym real
syms lambda_sym [6 1] real

% Dynamika
dx1 = x(4);
dx2 = x(5);
dx3 = x(6);
dx4 = -( sin(x(3))*(u1_sym+u2_sym) )/(m1_syms+m2_syms);
dx5 = -g_syms + ( cos(x(3))*(u1_sym+u2_sym) )/(m1_syms+m2_syms);
dx6 = ( u1_sym/(m1_syms*d_syms) ) - ( u2_sym/(m2_syms*d_syms) );
dx_sym = [dx1; dx2; dx3; dx4; dx5; dx6];

% Hamiltonian
u_sym = [u1_sym; u2_sym];
Hamil = 0.5 * (u1_sym^2 + u2_sym^2) + lambda_sym' * dx_sym;

% Derivace Hamiltonianu podle x
H_x = -jacobian(Hamil, x).';

%matlabFunction(H_x, 'Vars', {x, lambda_sym, u1_sym, u2_sym, g_syms, m1_syms, m2_syms, d_syms}, 'File', 'duocopterCostate');


% Boundary conditions
[xf, ~] = compute_end_state(d*(0.5-m1/(m1+m2)),m1,m2);
bcfun = @(ya, yb) [ya(1:6) - x0; yb(1:6) - xf];

% Initial guess - lambda dělá problémy
lambda0 = [0.01; 0.2; -1; -0.03; -4; 0.22]%4*rand(6,1)-2%[0.01; 0.2; -1; -0.03; -4; 0.22];
%lambda0(5) = 1;
solinit = bvpinit(linspace(t0, tf, 40), @(t) [x0; lambda0]);

options = bvpset('RelTol', 1e-6, 'AbsTol', 1e-6, 'NMax', 2e4, 'Stats', 'on');
sol = bvp4c(@(t,y) duocopterOptimalOde(t,y,g,m1,m2,d), ...
            bcfun, ...
            solinit,options);

t = sol.x;
y = sol.y;
x = y(1:6, :);
dx = sol.yp(1:6,:);
lambda = y(7:12, :);

% Compute control
u = zeros(2, length(t));
for i = 1:length(t)
    u(:, i) = optimalU(d, lambda(4,i), lambda(5,i), lambda(6,i), m1, m2, x(3,i));
end

Hamilton = 0.5*sum(u.^2,1)+sum(lambda .* dx, 1);
H = (Hamilton - Hamilton(1))/Hamilton(1);

xT = x';
n =  round(0.01*length(xT));
plotTrajectory(xT(1:n:end,:), t(1:n:end), m1, m2, d, 0.01)

figure(OuterPosition=[50, 700, 600, 500]);
plot(t,x)
title('Stavy');
legend('1','2','3','4','5','6')

figure(OuterPosition=[50, 200, 600, 500]);
plot(t,lambda)
title('Lagrangeovy multiplikátory');
legend('1','2','3','4','5','6')

figure(OuterPosition=[1250, 700, 600, 500]);
plot(t,H)
title('Hamilton');

figure(OuterPosition=[1250, 200, 600, 500]);
plot(t, u');
xlabel('t'); ylabel('u');
title('Optimální řízení');
legend('u_1','u_2');

%funkce byla upravena - jen jeden výstupní vektor u, jinak to matlab mentálně nezvládne
%matlabFunction(solve(jacobian(Hamil, u_sym),u).u1_sym, solve(jacobian(Hamil, u_sym),u).u2_sym,"File","optimalU") 