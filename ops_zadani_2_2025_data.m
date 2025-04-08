function [g,m,d,t0,x0,tf] = ops_zadani_2_2025_data(i)
%UNTITLED Paramery druhe semestralni prace z OPS
%   [g,m,d,t0,x0,tf] = ops_zadani_2_2025_data(i) rpo zadani cislo 'i' vrati
%   gravitacni zrychleni 'g', vektor dvou hmotnosti 'm', vzdalenost mezi
%   motory 'd', pocatecni cas 't0', pocatecni stav 'x0' a koncovy cas 'tf'

switch i
    case 1
        g = 9.81;
        m = [0.5 0.51];
        d = 0.2;
        t0 = 0;
        x0 = [0
            1.5
            0
            0
            0
            0];
        tf = 6;
        
    case 2
        g = 9.81;
        m = [0.53 0.5];
        d = 0.3;
        t0 = 0;
        x0 = [0
            1.5
            0
            0
            0
            0];
        tf = 6;
        
    case 3
        
        g = 9.81;
        m = [0.5 0.5];
        d = 0.3;
        t0 = 0;
        x0 = [0
            1.5
            0
            0
            0
            0];
        tf = 6;
        
    case 4
        g = 9.81;
        m = [0.5 0.55];
        d = 0.2;
        t0 = 0;
        x0 = [0
            1.5
            0
            0
            0
            0];
        tf = 6;
        
    case 5
        g = 9.81;
        m = [0.5 0.55];
        d = 0.2;
        t0 = 0;
        x0 = [0
            1.5
            0
            0
            0
            0];
        tf = 1;
        
end


