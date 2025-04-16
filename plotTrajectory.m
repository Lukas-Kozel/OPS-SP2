function plotTrajectory(X, t, m1, m2, d, dt)
%funkce pro vykreslení trajektorie
%t ... vektor času
%X ... matice stavů

%chtělo by to ještě poladit velikost plochy, musí být čtverec
    figure
    ylim([-2 3])
    axis equal
    for i = 1:length(t)
        %a - pozice jedné vrtule
        %b - pozice druhé vrtule
        a = X(i,1:2)' + m1/(m1+m2)*d*[cos(X(i,3)) ; sin(X(i,3))];
        b = X(i,1:2)' - m2/(m1+m2)*d*[cos(X(i,3)) ; sin(X(i,3))];

        plot(a(1),a(2),'Color','green',Marker='x',MarkerFaceColor='green',MarkerSize=5)
            ylim([-2 3])
    axis equal
        hold on
        line([a(1) b(1)],[a(2) b(2)],'color','black')
        plot(b(1),b(2),'Color','blue',Marker='x',MarkerFaceColor='blue',MarkerSize=5)
        pause(dt)
        hold off
    end
end
