function plotBag( filename , varargin)
    % Exatracts and plots data from .bag files where the optional
    % arguments can be set wit plotting settings. Example usage
    %
    % plotBag('kalmanTest.bag', {{'b','r','g'},{'k','k','k'}})
    
    bag = rosbag(filename);
    figure;
    topics = bag.AvailableTopics.Properties.RowNames(end:-1:1);
    for ii  = 1:length(topics)
        timeSeries = timeseries(select(bag,'Topic',topics(ii)));
        data = timeSeries.Data;
        time = timeSeries.Time - timeSeries.TimeInfo.Start;
        switch nargin
            case 1
                hold on;
                plot(time, data)
            case 2
                for jj = 1:size(data,2)
                    subplot(size(data,2),1,jj);
                    hold on;
                    plot(time, data(:,jj), varargin{1}{ii}{jj})
                end
        end
    end
end

