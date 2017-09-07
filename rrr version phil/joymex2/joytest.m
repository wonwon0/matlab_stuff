function joytest
%JOYTEST Example which shows to usage of joymex2.
%   JOYTEST Initializes the first gamepad device and shows the axes and
%   button states in a plot.

%   Copyright 2011 Martijn Aben
% 
%   Licensed under the Apache License, Version 2.0 (the "License");
%   you may not use this file except in compliance with the License.
%   You may obtain a copy of the License at
% 
%      http://www.apache.org/licenses/LICENSE-2.0
%
%   Unless required by applicable law or agreed to in writing, software
%   distributed under the License is distributed on an "AS IS" BASIS,
%   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%   See the License for the specific language governing permissions and
%   limitations under the License.

    running = 1;
    
    % Initialize joystick
    joymex2('open',2);
   
    % Create figure and attach close function
    figure('CloseRequestFcn',@onClose);

    % Create plots
    subplot(2,2,1)
    p1=plot([0],[0],'x'); hold on;
    p2=plot([0],[0],'+r');
    p3=plot([0],[0],'og');
    title(sprintf('Device 0\nAxis'))
    axslims = [double(intmin('int16'))-10 double(intmax('int16'))+10];
    set(gca,'xlim',axslims,'ylim',axslims); axis square
    
    subplot(2,2,2)
    h1=plot([0],[0],'x'); 
    title('Hats')
    set(gca,'xlim',[-2 2],'ylim',[-2 2]); axis square
    
    subplot(2,2,[3 4])
    b1=bar(zeros(1,8));
    title('Button States')
    set(gca,'xlim',[0 9],'ylim',[0 1]); axis square
    
    while(running)
        % Query postion and button state of joystick 0
        a = joymex2('query',0);
        
        
        % Update the plots
        
        % Notice the usage of minus signs to invert certain axis values.
        % Depending on the device you may want to change these 
        % (The plots are originally configured for a Xbox 360 Controller)
        set(p1,'Xdata',a.axes(1),'Ydata',-a.axes(2));
        set(p2,'Xdata',a.axes(3),'Ydata',-a.axes(4));        
        set(p3,'Xdata',0,'Ydata',a.axes(3));        
        set(b1,'Ydata',a.buttons);
        set(h1,'Xdata',a.hats.right -  a.hats.left,'Ydata',a.hats.up - a.hats.down);
        
        % Force update of plot
        drawnow
    end
    
    % Clear MEX-file to release joystick
    clear joymex2
    
    function onClose(src,evt)
       % When user tries to close the figure, end the while loop and
       % dispose of the figure
       running = 0;
       delete(src);
    end
end