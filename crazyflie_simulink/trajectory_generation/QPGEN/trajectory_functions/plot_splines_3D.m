function plot_splines_3D( P, points, times , derivative)
    nS = length(times) - 1;
    nCoeff = length(P{1})/nS;
    handles = zeros(1,length(derivative));
    for ii = 1:length(derivative)
        type = derivative{ii};
        for jj = 1:nS
            for kk =1:length(P)
                % Extracts and reverses the spline polynomial coefficients
                pci = P{kk}((jj-1)*nCoeff+1:1:jj*nCoeff);
                pcoeff{kk} = pci(length(pci):-1:1);
            end
            tt = linspace(times(jj),times(jj+1),1000);
            tp = linspace(0,times(jj+1)-times(jj),1000);
            switch type
                case 'position'
                    hold on;
                    if length(P) == 1
                        handles(ii) = plot(tt, polyval(pcoeff{1},tp),'b.');
                    elseif length(P) == 2
                        handles(ii) = plot(polyval(pcoeff{1},tp),...
                                           polyval(pcoeff{2},tp),'b.');
                    elseif length(P) == 3
                        handles(ii) = plot3(polyval(pcoeff{1},tp),...
                                            polyval(pcoeff{2},tp),...
                                            polyval(pcoeff{3},tp),'k.');
                    end
                otherwise
                    error('Invalid option '+ type +' in plot_splines().')
            end
        end
    end
    legend(handles, derivative)
    for ii = 1:length(points{1}(1,:))
        if length(P) == 1
            plot(times(ii), points{1}(1,ii),'kx');
        elseif length(P) == 2
            plot(points{1}(1,ii), points{2}(1,ii),'kx');
        elseif length(P) == 3
            plot3(points{1}(1,ii), points{2}(1,ii), points{3}(1,ii),'kx');
        end
        %txt1 = ['     p',num2str(ii)];
        %text(pointsX(1,ii), pointsY(1,ii), txt1)
    end
    %for ii = 1:length(pointsX(1,:))-1
    %    plot([pointsX(1,ii), pointsX(1,ii+1)], [pointsY(1,ii), pointsY(1,ii+1)], 'k--')
    %end
end

