function simulate_flight(timeFactor, scaleFactor)
% timeFactor - set to ~100 for good simulation speed, then uses every 100th
%    datapoint for the visualization.
% scaleFactor - set to 0.1 in this case, used to scale the quiverplot.
%
% Comment: run with axis('equal to better view the coordinate system')
load('simulation_results.mat');
x = result.Data(:,1);
dx = max(x) - min(x);
y = result.Data(:,2);
dy = max(y) - min(y);
z = result.Data(:,3);
dz = max(z) - min(z);

phi = result.Data(:,4);
theta = result.Data(:,5);
psi =result.Data(:,6);

close all
figure('Name','Fligt','Position',[0 0 800 600]);
copterHandle = plot3(x(1),y(1),z(1),'ko','MarkerSize',8,...
                                         'MarkerFaceColor','r');
axis([min(x)-dx/5, max(x)+dx/5,...
      min(y)-dy/5, max(y)+dy/5,...
      min(z)-dz/5, max(z)+dz/5])
%axis('equal')

hold on;
pathHandle = plot3(x(1),y(1),z(1),'k--');
nv = scaleFactor*eye(3);
vv = [1;1;1];
pathVectors = quiver3(x(1).*vv,y(1).*vv,z(1).*vv,...
                      nv(:,1),nv(:,2),nv(:,3),'b');
xlabel('x');ylabel('y');zlabel('z');
drawnow

for ii = 2:length(result.Time)
    if mod(ii,timeFactor) == 0
        set(copterHandle,'XData',x(ii));
        set(copterHandle,'YData',y(ii));
        set(copterHandle,'ZData',z(ii));
        set(pathHandle,'XData',x(1:ii));
        set(pathHandle,'YData',y(1:ii));
        set(pathHandle,'ZData',z(1:ii));
        set(pathVectors,'XData',x(ii).*vv);
        set(pathVectors,'YData',y(ii).*vv);
        set(pathVectors,'ZData',z(ii).*vv);
        
        cphi = cos(phi(ii));
        sphi = sin(phi(ii));
        ctheta = cos(theta(ii));
        stheta = sin(theta(ii));
        cpsi = cos(psi(ii));
        spsi = sin(psi(ii));
        
        R = [cphi*cpsi-ctheta*sphi*spsi,...
             -cpsi*sphi-cphi*ctheta*spsi,...
             stheta*spsi;...
             ctheta*cpsi*sphi+ctheta*spsi,...
             cphi*ctheta*cpsi-sphi*spsi,...
             -cpsi*stheta;...
             sphi*stheta,cphi*stheta,ctheta];
         
        vec = R*nv;
        set(pathVectors,'UData',vec(:,1));
        set(pathVectors,'VData',vec(:,2));
        set(pathVectors,'WData',vec(:,3));
        drawnow
    end
end
end

