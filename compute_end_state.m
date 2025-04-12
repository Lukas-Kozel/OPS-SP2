function [xf, end_conds] = compute_end_state(r)
%
% Vstupní argumenty:
%   r  - (scalar) Vzdálenost mezi těžištěm a geometrickým středem duokoptéry [m].
%        Tato hodnota je dána konstrukčními parametry modelu.

xC_des = [-1; 2];

speed_des = 0.14;
flight_angle_deg = 135;
flight_angle_rad = deg2rad(flight_angle_deg);

% Složky požadované rychlosti geometrického středu (v_C_des)
vC_des = [speed_des * cos(flight_angle_rad);   
          speed_des * sin(flight_angle_rad)];  
% Požadovaná orientace a úhlová rychlost:
x3_des = flight_angle_rad; 
x3_dot_des = 0;   % zmena orientace ma byt nulova        

%% 2. Přepočet terminálních podmínek na stav těžiště
% Transformace mezi těžištěm (T) a geometrickým středem (C):
%   x_C = x_T + r*[cos(x3); sin(x3)]
% Proto inverzní vztah je:
%   x_T = x_C - r*[cos(x3); sin(x3)]
xT_des = xC_des - r * [cos(x3_des); sin(x3_des)];

% Předpokládáme, že rychlost těžiště je stejná jako rychlost geometrického středu.
vT_des = vC_des;

% Stavový vektor se skládá z:
%   - Pozice těžiště: xT_des (2x1)
%   - Orientace: x3_des (skalár)
%   - Rychlost těžiště: vT_des (2x1)
%   - Úhlová rychlost: x3_dot_des (skalár)
xf = [xT_des; x3_des; vT_des; x3_dot_des];

end_conds = struct;
end_conds.xC_des     = xC_des;      % Požadovaná pozice geometrického středu [m]
end_conds.vC_des     = vC_des;      % Požadovaná rychlost geometrického středu [m/s]
end_conds.x3_des     = x3_des;      % Požadovaná orientace duokoptéry [rad]
end_conds.x3_dot_des = x3_dot_des;  % Požadovaná úhlová rychlost [rad/s]
end_conds.xT_des     = xT_des;      % Vypočítaná pozice těžiště [m]
end_conds.vT_des     = vT_des;      % Vypočítaná rychlost těžiště [m/s]
end_conds.tf         = tf;          % Konečný čas [s]
end
