clc;
clear all;
fclose(instrfind);
% variables
time = 0;
data1 = 0;
data2 = 0;
data3 = 0;
index = 0;
scrollWidth = 10;
delay = 0.001;
min = -3;
max = 3;

graph1 = plot(time, data1, 'm-', 'LineWidth', 2);
hold on;
graph2 = plot(time, data2, 'c-', 'LineWidth', 2);
hold on;
graph3 = plot(time, data3, 'g-', 'LineWidth', 2);

% setup serial port

s = serial('COM11', 'BaudRate' , 115200);
fopen(s);

% start time
tic;

loop = 0;

while ishandle(graph1)
    
   stream = fgetl(s);
   if strfind(stream,'BOUT!')
        stream = stream(7:end);
   end
   
   num = textscan(stream, '%f', 'Delimiter', ':');

   dat = num{1}';
   if(~isempty(dat) && isfloat(dat))
       index = index + 1;
       time(index) = toc;
       
       data1(index) = dat(1);
       data2(index) = dat(2);
       data3(index) = dat(3);
       
       if(scrollWidth > 0)
           selindex = time > time(index) - scrollWidth;
           set(graph1,'XData', time(selindex), 'YData', data1(selindex));
           set(graph2,'XData', time(selindex), 'YData', data2(selindex));
           set(graph3,'XData', time(selindex), 'YData', data3(selindex));
           axis([time(index) - scrollWidth time(index) min max]);
       else
           set(graph1,'XData', time, 'YData', data1);
           set(graph2, 'XData', time, 'YData', data2);
           set(graph3, 'Xdata', time, 'YData', data3);
           axis([0 time(index) min max]);
       end
       
       pause(delay);
   end
end

fclose(s);
delete(s);