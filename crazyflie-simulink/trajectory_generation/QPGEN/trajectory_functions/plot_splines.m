function plot_splines( P , points, times , derivative)
    nS = length(times) - 1;
    nCoeff = length(P)/nS;
    handles = zeros(1,length(derivative));
    for ii = 1:length(derivative)
        type = derivative{ii};
        for jj = 1:nS
            pcoeff = P((jj-1)*nCoeff+1:1:jj*nCoeff);
            pcoeff = pcoeff(length(pcoeff):-1:1);
            tt = linspace(times(jj),times(jj+1),100);
            tp = linspace(0,times(jj+1)-times(jj),100);
            switch type
                case 'position'
                    hold on; handles(ii) = plot(tt, polyval(pcoeff,tp),'b');
                case 'velocity'
                    pcoeff(end) = [];
                    pcoeff = (length(pcoeff):-1:1)'.*pcoeff;
                    hold on; handles(ii) = plot(tt, polyval(pcoeff,tp),'g');
                case 'acceleration'
                    pcoeff(end-1:end) = [];
                    pcoeff = (length(pcoeff)+1:-1:2)'.*(length(pcoeff):-1:1)'.*pcoeff;
                    hold on; handles(ii) = plot(tt, polyval(pcoeff,tp),'r');
                case 'jerk'
                    pcoeff(end-2:end) = [];
                    pcoeff = (length(pcoeff)+2:-1:3)'.*(length(pcoeff)+1:-1:2)'.*(length(pcoeff):-1:1)'.*pcoeff;
                    hold on; handles(ii) = plot(tt, polyval(pcoeff,tp),'c');
                otherwise
                    error('Invalid option '+ type +' in plot_splines().')
            end
        end
    end
    legend(handles, derivative)
    for ii = 1:length(points(1,:))
        plot(times(ii), points(1,ii), '*k')
        txt1 = ['     p',num2str(ii)];
        text(times(ii), points(1,ii), txt1)
    end
end

