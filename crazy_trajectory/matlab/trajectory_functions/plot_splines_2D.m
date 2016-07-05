function plot_splines_2D( PX , PY, pointsX, pointsY, times , derivative)
    nS = length(times) - 1;
    nCoeff = length(PX)/nS;
    handles = zeros(1,length(derivative));
    for ii = 1:length(derivative)
        type = derivative{ii};
        for jj = 1:nS
            pcoeffx = PX((jj-1)*nCoeff+1:1:jj*nCoeff);
            pcoeffx = pcoeffx(length(pcoeffx):-1:1);
            pcoeffy = PY((jj-1)*nCoeff+1:1:jj*nCoeff);
            pcoeffy = pcoeffy(length(pcoeffy):-1:1);
            tt = linspace(times(jj),times(jj+1),100);
            tp = linspace(0,times(jj+1)-times(jj),100);
            switch type
                case 'position'
                    hold on; handles(ii) = plot(polyval(pcoeffx,tp), polyval(pcoeffy,tp),'b');
                case 'velocity'
                    pcoeffx(end) = [];
                    pcoeffx = (length(pcoeffx):-1:1)'.*pcoeffx;
                    pcoeffy(end) = [];
                    pcoeffy = (length(pcoeffy):-1:1)'.*pcoeffy;
                    hold on; handles(ii) = plot(polyval(pcoeffx,tp), polyval(pcoeffy,tp),'g');
                case 'acceleration'
                    pcoeffx(end-1:end) = [];
                    pcoeffx = (length(pcoeffx)+1:-1:2)'.*(length(pcoeffx):-1:1)'.*pcoeffx;
                    hold on; handles(ii) = plot(tt, polyval(pcoeffx,tp),'r');
                case 'jerk'
                    pcoeffx(end-2:end) = [];
                    pcoeffx = (length(pcoeffx)+2:-1:3)'.*(length(pcoeffx)+1:-1:2)'.*(length(pcoeffx):-1:1)'.*pcoeffx;
                    hold on; handles(ii) = plot(tt, polyval(pcoeffx,tp),'c');
                otherwise
                    error('Invalid option '+ type +' in plot_splines().')
            end
        end
    end
    legend(handles, derivative)
    if derivative{1} == 'position'
        for ii = 1:length(pointsX(1,:))
            plot(pointsX(1,ii), pointsY(1,ii), '*k')
            txt1 = ['     p',num2str(ii)];
            text(pointsX(1,ii), pointsY(1,ii), txt1)
        end
        for ii = 1:length(pointsX(1,:))-1
            plot([pointsX(1,ii), pointsX(1,ii+1)], [pointsY(1,ii), pointsY(1,ii+1)], 'k--')
        end
    end
end

