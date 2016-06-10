% Serial Data Logger
% Yu Hin Hau
% 7/9/2013
% **CLOSE PLOT TO END SESSION
 
clear
clc

fclose(instrfind)

%User Defined Properties 
serialPort = 'COM11';            % define COM port #
plotTitle = 'Serial Data Log';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Data';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -2;                     % set y-min
max = 2;                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .001;                    % make sure sample faster than resolution
 
%Define Function Variables
time = 0;
data = 0;
count = 0;
 
%Set up Plot
plotGraph = plot(time,data,'-mo',...
                'LineWidth',2,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor',[.49 1 .63],...
                'MarkerSize',1);
             
title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);
 
%Open Serial COM Port
s = serial(serialPort, 'BaudRate',115200)
disp('Close Plot to End Session');
fopen(s);
 
tic
 
while ishandle(plotGraph) %Loop when Plot is Active
     
    %dat = fscanf(s,'%f'); %Read Data from Serial as Float
    
    str = fgetl(s);
    if strfind(str,'BOUT!')
        str = str(7:end);
    end
    num = textscan(str, '%f', 'Delimiter', ':');
    
    dat = num{1}';
    
    dat = dat(1);
    
    if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
        count = count + 1;    
        time(count) = toc;    %Extract Elapsed Time
        data(count) = dat(1); %Extract 1st Data Element         
        stationary(count) = dat(1) < 0.09;
        %Set Axis according to Scroll Width
        if(scrollWidth > 0)
        set(plotGraph,'XData',time(time > time(count)-scrollWidth),'YData',data(time > time(count)-scrollWidth));
        set(plotGraph,'XData',time(time > time(count)-scrollWidth),'YData',stationary(time > time(count)-scrollWidth));
        axis([time(count)-scrollWidth time(count) min max]);
        else
        set(plotGraph,'XData',time,'YData',data);
        axis([0 time(count) min max]);
        end
         
        %Allow MATLAB to Update Plot
        pause(delay);
    end
end
 
%Close Serial COM Port and Delete useless Variables
fclose(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;
 
 
disp('Session Terminated...');