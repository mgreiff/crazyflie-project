function plot_splines( P , times , option)
    nS = length(times) - 1;
    for ii = 1:nS
        pcoeff = P(ii:nS:end);
        pcoeff = pcoeff(length(pcoeff):-1:1);
        tt = linspace(times(ii),times(ii+1),100);
        tp = linspace(0,times(ii+1)-times(ii),100);
        switch option
            case 'position'
                hold on; plot(tt, polyval(pcoeff,tp),'b')
                         plot(tt(1), polyval(pcoeff,tp(1)),'k+')
                         plot(tt(end), polyval(pcoeff,tp(end)),'kx')

            case 'velocity'
                pcoeff(end) = [];
                pcoeff = (length(pcoeff):-1:1)'.*pcoeff;
                hold on; plot(tt, polyval(pcoeff,tp),'g')
                         plot(tt(1), polyval(pcoeff,tp(1)),'k+')
                         plot(tt(end), polyval(pcoeff,tp(end)),'kx')

            case 'acceleration'
                pcoeff(end-1:end) = [];
                pcoeff = (length(pcoeff)+1:-1:2)'.*(length(pcoeff):-1:1)'.*pcoeff;
                hold on; plot(tt, polyval(pcoeff,tp),'r')
                         plot(tt(1), polyval(pcoeff,tp(1)),'k+')
                         plot(tt(end), polyval(pcoeff,tp(end)),'kx')

            otherwise
                error('Invalid option '+ option +' in plot_splines().')
        end
    end
end

