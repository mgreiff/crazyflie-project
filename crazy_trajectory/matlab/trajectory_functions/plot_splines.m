function plot_splines( P , times , option)
    nS = length(times) - 1;
    nCoeff = length(P)/nS;
    for ii = 1:length(option)
        type = option{ii};
        for jj = 1:nS
            pcoeff = P((jj-1)*nCoeff+1:1:jj*nCoeff);
            pcoeff = pcoeff(length(pcoeff):-1:1);
            tt = linspace(times(jj),times(jj+1),100);
            tp = linspace(0,times(jj+1)-times(jj),100);
            switch type
                case 'position'
                    hold on; plot(tt, polyval(pcoeff,tp),'b')
                case 'velocity'
                    pcoeff(end) = [];
                    pcoeff = (length(pcoeff):-1:1)'.*pcoeff;
                    hold on; plot(tt, polyval(pcoeff,tp),'g')
                case 'acceleration'
                    pcoeff(end-1:end) = [];
                    pcoeff = (length(pcoeff)+1:-1:2)'.*(length(pcoeff):-1:1)'.*pcoeff;
                    hold on; plot(tt, polyval(pcoeff,tp),'r')
                case 'jerk'
                    pcoeff(end-2:end) = [];
                    pcoeff = (length(pcoeff)+2:-1:3)'.*(length(pcoeff)+1:-1:2)'.*(length(pcoeff):-1:1)'.*pcoeff;
                    hold on; plot(tt, polyval(pcoeff,tp),'c')
                otherwise
                    error('Invalid option '+ type +' in plot_splines().')
            end
        end
    end
    legend(option)
end

