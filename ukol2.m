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
    %              pokud není ideální signál dostupný, použije se [1;1].
    %
    % Výstupem jsou vykreslené grafy časových průběhů stavů (x1, x3, x6)
    % a trajektorií polohy (x1 vs. x3) pro oba testovací případy.
    
    %% Testovací signál utest,1 (nulový vstup)
    u1_1 = 0;
    u2_1 = 0;
    [t_sim1, X_sim1] = ode45(@(t, x) duocopter_ode(t, x, g, m1, m2, d, u1_1, u2_1), [t0 tf], x0);
    
    figure;
    subplot(3,1,1);
    plot(t_sim1, X_sim1(:,1), 'LineWidth', 1.5);
    xlabel('Čas t'); ylabel('x_1');
    title('TC1 (Nulový vstup): Časový průběh x_1');
    
    subplot(3,1,2);
    plot(t_sim1, X_sim1(:,3), 'LineWidth', 1.5);
    xlabel('Čas t'); ylabel('x_3');
    title('TC1 (Nulový vstup): Časový průběh x_3');
    
    subplot(3,1,3);
    plot(t_sim1, X_sim1(:,6), 'LineWidth', 1.5);
    xlabel('Čas t'); ylabel('x_6');
    title('TC1 (Nulový vstup): Časový průběh x_6');
    
    figure;
    plot(X_sim1(:,1), X_sim1(:,3), 'LineWidth', 1.5);
    xlabel('Horizontální poloha (x_1)');
    ylabel('Vertikální poloha (x_3)');
    title('TC1: Trajektorie duokoptéry');
    grid on;
    
    %% Testovací signál utest,2
    % Chceme, aby rychlost těžiště (x4) a úhlová rychlost (x6) byly konstantní.
    % Rovnice modelu dávají:
    %   dx4 = ( sin(x3)*(u1+u2) )/(m1+m2)  a  dx6 = (u1/(m1*d)) - (u2/(m2*d)).
    % Pokud v počátečním stavu platí sin(x0(3)) ≈ 0, lze volit u2 = (m2/m1)*u1.
    % Jinak "ideální" signál neexistuje a volíme např. utest,2 = [1; 1].
    if abs(sin(x0(3))) < 1e-6
        u1_2 = 1;
        u2_2 = (m2/m1)*u1_2;
    else
        u1_2 = 1;
        u2_2 = 1;
    end
    utest2 = [u1_2; u2_2];
    disp('Testovací signál utest,2 = ');
    disp(utest2);
    
    [t_sim2, X_sim2] = ode45(@(t, x) duocopter_ode(t, x, g, m1, m2, d, u1_2, u2_2), [t0 tf], x0);
    
    figure;
    subplot(3,1,1);
    plot(t_sim2, X_sim2(:,1), 'LineWidth', 1.5);
    xlabel('Čas t'); ylabel('x_1');
    title('TC2: Časový průběh x_1');
    
    subplot(3,1,2);
    plot(t_sim2, X_sim2(:,3), 'LineWidth', 1.5);
    xlabel('Čas t'); ylabel('x_3');
    title('TC2: Časový průběh x_3');
    
    subplot(3,1,3);
    plot(t_sim2, X_sim2(:,6), 'LineWidth', 1.5);
    xlabel('Čas t'); ylabel('x_6');
    title('TC2: Časový průběh x_6');
    
    figure;
    plot(X_sim2(:,1), X_sim2(:,3), 'LineWidth', 1.5);
    xlabel('Horizontální poloha (x_1)');
    ylabel('Vertikální poloha (x_3)');
    title('TC2: Trajektorie duokoptéry');
    grid on;
end