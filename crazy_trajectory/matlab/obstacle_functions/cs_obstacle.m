function [ xout, yout ] = cs_obstacle( x, y , d)
    k = convhull(x, y);
    xhat = x(k);
    yhat = y(k);
    xout = [];
    yout = [];
    for ii = 1:length(xhat)-1
        xout = [xout, xhat(ii)];
        yout = [yout, yhat(ii)];
        dist = sqrt((xhat(ii) - xhat(ii + 1))^2 + (yhat(ii) - yhat(ii + 1))^2);
        if dist > d
            % Compute intermediary points
            dir = [xhat(ii + 1) - xhat(ii); yhat(ii+1) - yhat(ii)];
            %cla;
            %plot([0,dir(1)], [0,dir(2)],'gx-')
            dir = dir./norm(dir);
            %hold on;plot([0,dir(1)*dist], [0,dir(2)*dist],'r+-')
            xnew = xhat(ii) + dist / ceil(dist/d) * dir(1)*(1:ceil(dist/d));
            ynew = yhat(ii) + dist / ceil(dist/d) * dir(2)*(1:ceil(dist/d));
            xout = [xout, xnew];
            yout = [yout, ynew];
        end 
    end
end

