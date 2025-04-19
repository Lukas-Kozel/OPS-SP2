function ukol2(g, m1, m2, d, t0, tf, x0)
    % ukol2 - Simulační ověření modelu duokoptéry podle zadání
    %
    % Vstupní argumenty:
    %   g  - tíhové zrychlení
    %   m1 - hmotnost motoru a vrtule 1
    %   m2 - hmotnost motoru a vrtule 2
    %   d  - vzdálenost mezi vrtulemi (nebo jiný rozměr)
    %   t0 - počáteční čas
    %   tf - koncový čas simulace
    %   x0 - počáteční stav (6x1 vektor)
    %
    % Funkce provede simulaci modelu duokoptéry pro dva testovací signály:
    %   utest,1 - nulový vstup: [0;0]
    %   utest,2 - vstupní signál volený tak, aby (ideálně) udržel konstantní
    %              rychlost těžiště a konstantní úhlovou rychlost;
    
    %% Testovací signál utest,1 (nulový vstup)
    u1_1 = 0;
    u2_1 = 0;
    [t_sim1, X_sim1] = ode45(@(t, x) duocopter_ode(t, x, g, m1, m2, d, u1_1, u2_1), [t0 tf], x0);
    
%     figure;
%     subplot(3,1,1);
%     plot(t_sim1, X_sim1(:,1), 'LineWidth', 1.5);
%     xlabel('Čas t'); ylabel('x_1');
%     title('TC1 (Nulový vstup): Časový průběh x_1');
%     
%     subplot(3,1,2);
%     plot(t_sim1, X_sim1(:,3), 'LineWidth', 1.5);
%     xlabel('Čas t'); ylabel('x_3');
%     title('TC1 (Nulový vstup): Časový průběh x_3');
%     
%     subplot(3,1,3);
%     plot(t_sim1, X_sim1(:,6), 'LineWidth', 1.5);
%     xlabel('Čas t'); ylabel('x_6');
%     title('TC1 (Nulový vstup): Časový průběh x_6');
    figure;
    subplot(6,1,1);
    plot(t_sim1, X_sim1(:,1), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_1 [m]');
    title(' Časový průběh x_1');
    
    subplot(6,1,2);
    plot(t_sim1, X_sim1(:,2), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_2 [m]');
    title(' Časový průběh x_2');
    
    subplot(6,1,3);
    plot(t_sim1, X_sim1(:,3), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_3 [rad]');
    title(' Časový průběh x_3');
    
    subplot(6,1,4);
    plot(t_sim1, X_sim1(:,4), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_4 [m/s]');
    title(' Časový průběh x_4');
    
    subplot(6,1,5);
    plot(t_sim1, X_sim1(:,5), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_5 [m/s]');
    title(' Časový průběh x_5');
    
    subplot(6,1,6);
    plot(t_sim1, X_sim1(:,6), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_6 [rad/s]');
    title(' Časový průběh x_6');

    figure;
    plot(X_sim1(:,1), X_sim1(:,2), 'LineWidth', 1.5);
    xlabel('Horizontální poloha (x_1)');
    ylabel('Vertikální poloha (x_2)');
    title(' Trajektorie duokoptéry');
    grid on;
    
    %% Testovací signál utest,2
    % Chceme, aby rychlost těžiště (x4) a úhlová rychlost (x6) byly konstantní.
    % Rovnice modelu dávají:
    %   dx4 = ( sin(x3)*(u1+u2) )/(m1+m2)  a  dx6 = (u1/(m1*d)) - (u2/(m2*d)).
    % Pokud v počátečním stavu platí sin(x0(3)) ≈ 0, lze volit u2 = (m2/m1)*u1.
    % Jinak "ideální" signál neexistuje a volíme např. utest,2 = [1; 1].

    syms u1 u2 real
    eq1 = -sin(x0(3))*(u1 + u2)/(m1 + m2)      == 0;
    eq2 = -g   + cos(x0(3))*(u1 + u2)/(m1 + m2) == 0;
    eq3 = u1/(m1*d) - u2/(m2*d)               == 0;
    S = solve([eq1,eq2,eq3],[u1,u2],'IgnoreProperties',true);
    
    u1_2 = double(S.u1);
    disp(u1_2);
    u2_2 = double(S.u2);
    disp(u2_2);
    utest2 = [u1_2; u2_2];
    disp('Testovací signál utest,2 = ');
    disp(utest2);
    
    [t_sim2, X_sim2] = ode45(@(t, x) duocopter_ode(t, x, g, m1, m2, d, u1_2, u2_2), [t0 tf], x0);
    
%     figure;
%     subplot(3,1,1);
%     plot(t_sim2, X_sim2(:,1), 'LineWidth', 1.5);
%     xlabel('Čas t'); ylabel('x_1');
%     title(' Časový průběh x_1');
%     
%     subplot(3,1,2);
%     plot(t_sim2, X_sim2(:,3), 'LineWidth', 1.5);
%     xlabel('Čas t'); ylabel('x_3');
%     title(' Časový průběh x_3');
%     
%     subplot(3,1,3);
%     plot(t_sim2, X_sim2(:,6), 'LineWidth', 1.5);
%     xlabel('Čas t'); ylabel('x_6');
%     title(' Časový průběh x_6');
    figure;
    title('nenulový testovací signál')
    subplot(6,1,1);
    plot(t_sim2, X_sim2(:,1), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_1 [m]');
    title(' Časový průběh x_1');
    
    subplot(6,1,2);
    plot(t_sim2, X_sim2(:,2), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_2 [m]');
    title(' Časový průběh x_2');
    
    subplot(6,1,3);
    plot(t_sim2, X_sim2(:,3), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_3 [rad]');
    title(' Časový průběh x_3');
    
    subplot(6,1,4);
    plot(t_sim2, X_sim2(:,4), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_4 [m/s]');
    title(' Časový průběh x_4');
    
    subplot(6,1,5);
    plot(t_sim2, X_sim2(:,5), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_5 [m/s]');
    title(' Časový průběh x_5');
    
    subplot(6,1,6);
    plot(t_sim2, X_sim2(:,6), 'LineWidth', 1.5);
    xlabel('Čas [s]'); ylabel('x_6 [rad/s]');
    title(' Časový průběh x_6');

    figure;
    plot(X_sim2(:,1), X_sim2(:,2), 'LineWidth', 1.5);
    xlabel('Horizontální poloha (x_1)');
    ylabel('Vertikální poloha (x_2)');
    title(' Trajektorie duokoptéry');
    grid on;
    
    hold off;
    figure;
%     plotTrajectory(X_sim2,t_sim2,m1,m2,d)
end