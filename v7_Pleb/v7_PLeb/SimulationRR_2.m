function varargout = SimulationRR_2(varargin)
% SIMULATIONRR_2 MATLAB code for SimulationRR_2.fig
%      SIMULATIONRR_2, by itself, creates a new SIMULATIONRR_2 or raises the existing
%      singleton*.
%
%      H = SIMULATIONRR_2 returns the handle to a new SIMULATIONRR_2 or the handle to
%      the existing singleton*.
%
%      SIMULATIONRR_2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULATIONRR_2.M with the given input arguments.
%
%      SIMULATIONRR_2('Property','Value',...) creates a new SIMULATIONRR_2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SimulationRR_2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SimulationRR_2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SimulationRR_2

% Last Modified by GUIDE v2.5 12-Oct-2015 09:58:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SimulationRR_2_OpeningFcn, ...
                   'gui_OutputFcn',  @SimulationRR_2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before SimulationRR_2 is made visible.
function SimulationRR_2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SimulationRR_2 (see VARARGIN)

    a_GeneralParams()


    global plotlimit;
    plotlimit=0;
    
    global active_slide1;
    active_slide1 = 1;
    
    global active_slide2;
    active_slide2 = 1;
    
    global active_links;
    active_links = 1;
    
    global active_elbow;
    active_elbow = 1;
    

    global V0;
    global F0;
    global C0;
    global V1;
    global F1;
    global C1;
    global V2;
    global F2;
    global C2;
    global V3;
    global F3;
    global C3;
    global V6;
    global F6;
    global C6;
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename = 'M0_bin.STL';
    [V0, F0, N0, C0, stltitle] = stlread(filename, 0);
    [V0, F0]=patchslim(V0, F0);
    
    sc=1;
    px=-0.05202+0.00177;py=+0.05025+0.00148;pz=+0.1185+0.04;
    
    V0 = V0';
    V0 = [V0(1,:); V0(2,:); V0(3,:); ones(1,length(V0))];
    V0=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V0;
    V0=[0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1]*V0;
    V0=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V0;
    V0=V0';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename = 'M1_bin.STL';
    [V1, F1, N1, C1, stltitle] = stlread(filename, 0);
    [V1, F1]=patchslim(V1, F1);
        
    sc=1;
    px=0.04176;py=0.04271;pz=0.1876;
    
    V1 = V1';
    V1 = [V1(1,:); V1(2,:); V1(3,:); ones(1,length(V1))];
    V1=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V1;
    V1=[0 0 -1 0;0 -1 0 0;-1 0 0 0;0 0 0 1]*V1;
    V1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V1;
    V1=[0 1 0 0; -1 0 0 0; 0 0 1 0;0 0 0 1]*V1;
    V1 = [V1(1,:); V1(2,:); V1(3,:); ones(1,length(V1))];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename = 'M2_bin.STL';
    [V2, F2, N2, C2, stltitle] = stlread(filename, 0);
    [V2, F2]=patchslim(V2, F2);
        
    sc=1;
    px=-0.3313;py=-0.04126;pz=-0.1125;
    
    V2 = V2';
    V2 = [V2(1,:); V2(2,:); V2(3,:); ones(1,length(V2))];
    V2=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V2;
    V2=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]*V2;
%     V2=[0 0 -1 0;0 -1 0 0;-1 0 0 0;0 0 0 1]*V2;
    V2=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V2;
    V2=[-1 0 0 0;0 1 0 0;0 0 -1 0;0 0 0 1]*V2;
    V2=[0 1 0 0;-1 0 0 0; 0 0 1 0;0 0 0 1]*V2;
    V2 = [V2(1,:); V2(2,:); V2(3,:); ones(1,length(V2))];    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename = 'M3_bin.STL';
    [V3, F3, N3, C3, stltitle] = stlread(filename, 0);
    [V3, F3]=patchslim(V3, F3);
        
    sc=1;
    px=-0.04126;py=-0.04126+0.08252;pz=-0.0495;
    
    V3 = V3';
    V3 = [V3(1,:); V3(2,:); V3(3,:); ones(1,length(V3))];
    V3=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V3;
%     V3=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]*V3;
    V3=[0 1 0 0;-1 0 0 0;0 0 1 0;0 0 0 1]*V3;
    V3=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V3;
    V3=[0 1 0 0;-1 0 0 0; 0 0 1 0;0 0 0 1]*V3;
    V3 = [V3(1,:); V3(2,:); V3(3,:); ones(1,length(V3))];    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename = 'M6_bin.STL';
    [V6, F6, N6, C6, stltitle] = stlread(filename, 0);
    [V6, F6]=patchslim(V6, F6);
        
    sc=1;
    px=-0.03155-0.1233-0.1443;py=-0.05962;pz=-0.1761+0.195897+0.00312;
    
    V6 = V6';
    V6 = [V6(1,:); V6(2,:); V6(3,:); ones(1,length(V6))];
    V6=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V6;
%     V6=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]*V6;
    V6=[0 1 0 0;0 0 1 0;1 0 0 0;0 0 0 1]*V6;
    V6=[0 0 1 0;0 1 0 0;-1 0 0 0;0 0 0 1]*V6;
    V6=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V6;
%     V6=[1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1]*V6;
    V6 = [V6(1,:); V6(2,:); V6(3,:); ones(1,length(V6))];    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
    
    set(gcf, 'renderer', 'OpenGL');
    plot3(0,0,0,'x');
    xlim([-1 1]*0.7);
    ylim([-1 1]*0.7);
    zlim([-1 1]*0.7);
    
    axis vis3d;

        px=0;py=0;pz=+0.1185+0.04;
        V1_1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V1;
        V1_1=V1_1';

        V2_1=V2;
        V2_1=V2_1';
        
        V3_1=V3;
        V3_1=V3_1';
        
        V6_1=V6;
        V6_1=V6_1';
    
    global p1;
    global p2;
    global p3;
    global p6;
    
        p0 = patch('faces', F0, 'vertices' ,V0(:,1:3));
%         reducepatch(p0,0.99);
        set(p0, 'facec', 'flat');  
        set(p0, 'EdgeColor','none'); 
        set(p0, 'facec', [1 1 1]);
        
        
        p1 = patch('faces', F1, 'vertices' ,V1_1(:,1:3));
        set(p1, 'facec', 'flat');  
        set(p1, 'EdgeColor','none'); 
        set(p1, 'facec', [1 1 1]);
%                 reducepatch(p1,0.8);

        p2 = patch('faces', F2, 'vertices' ,V2_1(:,1:3));
        set(p2, 'facec', 'flat');  
        set(p2, 'EdgeColor','none'); 
        set(p2, 'facec', [1 1 1]);
%         reducepatch(p2,0.8);

        p3 = patch('faces', F3, 'vertices' ,V3_1(:,1:3));
        set(p3, 'facec', 'flat');  
        set(p3, 'EdgeColor','none'); 
        set(p3, 'facec', [1 1 1]);
        
        p6 = patch('faces', F6, 'vertices' ,V6_1(:,1:3));
        set(p6, 'facec', 'flat');  
        set(p6, 'EdgeColor','none'); 
        set(p6, 'facec', [1 1 1]);
        
        light
         
        view([60 24]);

        grid on;
        xlabel('X');ylabel('Y');zlabel('Z');
        
        drawnow;
%         plotter(handles);
        
%         clear JoyMEX
%         JoyMEX('init',0);
%         JoyMEX('init',1);
        
    theta=[0;0;0];


% Choose default command line output for SimulationRR_2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SimulationRR_2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


GoReady(handles);




% --- Outputs from this function are returned to the command line.
function varargout = SimulationRR_2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function GoReady(handles)
    set(handles.editT1,'String', 0);
    set(handles.editT2,'String', -30);
    set(handles.editT3,'String', -60);
    set(handles.sliderT1,'Value', 0);
    set(handles.sliderT2,'Value', -30);
    set(handles.sliderT3,'Value', -60);
    moveTheta(handles);

function movePosition(handles)
% Receives a position


function moveTheta(handles)
% Receives a position
        plotter(handles);
    theta1=str2num( get(handles.editT1,'String') )*pi/180;
    theta2=str2num( get(handles.editT2,'String') )*pi/180;
    theta3=str2num( get(handles.editT3,'String') )*pi/180;
    theta=[theta1;theta2;theta3];
    
    [T,T1,T2,T3] = a_CinDirecte(theta);
            set(handles.posX,'String', T(1,4));
            set(handles.posY,'String', T(2,4));
            set(handles.posZ,'String', T(3,4));
        
    
function VitCart(handles, vit)
global Cart_Velocity_Max;
global Cart_VelocitySlowZone_Factor;
persistent vitm1;
if isempty(vitm1)
    vitm1=[0;0;0];
end

    % Max Acceleration
        vit000(1) = a_accelmax(vit(1), vitm1(1), 0.2, 3, 0.01);
        vit000(2) = a_accelmax(vit(2), vitm1(2), 0.2, 3, 0.01);
        vit000(3) = a_accelmax(vit(3), vitm1(3), 0.2, 3, 0.01);
    
        
    % Transform vit with Cartesian velocity limitation
        vit00 = a_VitCartMax(vit000,Cart_Velocity_Max);
    
        vitm1 = vit00;
        

    % Get the actual position
        theta1=str2num( get(handles.editT1,'String') )*pi/180;
        theta2=str2num( get(handles.editT2,'String') )*pi/180;
        theta3=str2num( get(handles.editT3,'String') )*pi/180;
        theta=[theta1;theta2;theta3];
        [T,T1,T2,T3] = a_CinDirecte(theta);
            
        
        
    % Find the limitation planes 1
            [Limitation1,alpha1] = a_LimitationPlanes1_3D(theta,T);
            alpha1;
    % Slow zone
                if alpha1>=0.01
                    vit0 = a_VitCartMax(vit00,Cart_Velocity_Max*Cart_VelocitySlowZone_Factor);
                else
                    vit0 = vit00;
                end
        
    % Slide1
            global active_slide1;
            if active_slide1 == 1
                [vit1,LimitationActive1]=a_Slide2_3D(Limitation1, vit0);
                 vit1 = alpha1*vit1 + (1-alpha1)*vit0;
            else
                vit1=vit0;
            end

    % Find the limitation planes 2
                Limitation = a_LimitationPlanes2_3D(theta,T);

    % Slide2
            global active_slide2;
            if active_slide2 == 1
                [vit2,LimitationActive]=a_Slide2_3D(Limitation, vit1);
% % %                 if active_slide1 == 1
% % %                     LimitationActive=LimitationActive1;
% % %                 end
            else
                vit2=vit1; 
            end


    % Plot limitations
            global plotlimit
            if plotlimit==1
                cla
                hold on;
                    x = [-0.14 0.14];
                    y = [0.28 0.45];
                    z = [0.2 0.5];

    
                    plot3([x(1)-0.03;x(1)-0.03],[y(1)-0.03;y(2)+0.03],[z(1)-0.03;z(1)-0.03]);
                    plot3([x(1)-0.03;x(1)-0.03],[y(1)-0.03;y(1)-0.03],[z(1)-0.03;z(2)+0.03]);
                    plot3([x(1)-0.03;x(1)-0.03],[y(1)-0.03;y(2)+0.03],[z(2)+0.03;z(2)+0.03]);
                    plot3([x(1)-0.03;x(1)-0.03],[y(1)-0.03;y(2)+0.03],[z(2)+0.03;z(2)+0.03]);
                    
                    plot3([x(2)+0.03;x(2)+0.03],[y(1)-0.03;y(2)+0.03],[z(1)-0.03;z(1)-0.03]);
                    plot3([x(2)+0.03;x(2)+0.03],[y(1)-0.03;y(1)-0.03],[z(1)-0.03;z(2)+0.03]);
                    plot3([x(2)+0.03;x(2)+0.03],[y(1)-0.03;y(2)+0.03],[z(2)+0.03;z(2)+0.03]);
                    plot3([x(2)+0.03;x(2)+0.03],[y(1)-0.03;y(2)+0.03],[z(2)+0.03;z(2)+0.03]);
                    plot3([x(1)-0.03;x(2)+0.03],[y(1)-0.03;y(1)-0.03],[z(1)-0.03;z(1)-0.03]);
                    plot3([x(1)-0.03;x(2)+0.03],[y(1)-0.03;y(1)-0.03],[z(2)+0.03;z(2)+0.03]);
                    
                    
                hold off;
                
            end
        %         global plotlimit
        %         if plotlimit==1
        %             cla
        %             for i=1:size(LimitationActive,2)   
        %                 hold on;
        %                     plot3([T(1,4)-LimitationActive(2,i);T(1,4)+LimitationActive(2,i)],[T(2,4)+LimitationActive(1,i);T(2,4)-LimitationActive(1,i)],[0;0])
        %                 hold off;
        %             end
        %         end

    % New desired position
%     	vit2 = vit00;
        posDes = [T(1,4)+vit2(1);T(2,4)+vit2(2);T(3,4)+vit2(3)];
    
    % Resolve the inverse kinematics
        [theta,error]=a_InvKinematics(theta,posDes);
        [T,T1,T2,T3] = a_CinDirecte(theta);
        test=[posDes,[T(1,4);T(2,4);T(3,4)]];
        
    % Update the angular command
        if error==0
            set(handles.editT1,'String', theta(1)*180/pi);
            set(handles.editT2,'String', theta(2)*180/pi);
            set(handles.editT3,'String', theta(3)*180/pi);
            refreshTheta(handles);
        end
    
    % Plot
        plotter(handles);
    

function refreshTheta(handles)
            theta1=str2num( get(handles.editT1,'String') )*pi/180;
            theta2=str2num( get(handles.editT2,'String') )*pi/180;
            theta3=str2num( get(handles.editT3,'String') )*pi/180;
            theta=[theta1;theta2;theta3];
            set(handles.sliderT1,'Value',theta1*180/pi);
            set(handles.sliderT2,'Value',theta2*180/pi);
            set(handles.sliderT3,'Value',theta3*180/pi);
            [T,T1,T2,T3] = a_CinDirecte(theta);
            set(handles.posX,'String', T(1,4));
            set(handles.posY,'String', T(2,4));    
            set(handles.posZ,'String', T(3,4));  
        
    
function plotter (handles) 
    theta1=str2num( get(handles.editT1,'String') )*pi/180;
    theta2=str2num( get(handles.editT2,'String') )*pi/180;
    theta3=str2num( get(handles.editT3,'String') )*pi/180;
    theta=[theta1;theta2;theta3];
    
    [T,T1,T2,T3] = a_CinDirecte(theta);
    
    CameraPosition=get(handles.axes1,'CameraPosition');
    CameraTarget=get(handles.axes1,'CameraTarget');
    CameraUpVector=get(handles.axes1,'CameraUpVector');
    CameraViewAngle=get(handles.axes1,'CameraViewAngle');
        
    hold off;

    global V0;
    global F0;
    global C0;
    global V1;
    global F1;
    global C1;
    global V2;
    global F2;
    global C2;
    global V3;
    global F3;
    global C3;
    global V6;
    global F6;
    global C6;
    
    global p1;
    global p2;
    global p3;
    global p6;
    
        px=0;py=0;pz=+0.1185+0.04;
        V1_1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V1;
        V1_1=[cos(theta1-pi) -sin(theta1-pi) 0 0;sin(theta1-pi) cos(theta1-pi) 0 0;0 0 1 0;0 0 0 1]*V1_1;
        V1_1=V1_1';

        V2_1=T1*[cos(theta2-pi) -sin(theta2-pi) 0 0;sin(theta2-pi) cos(theta2-pi) 0 0;0 0 1 0;0 0 0 1]*V2;
        V2_1=V2_1';
        
        V3_1=T1*T2*[cos(theta3-pi) -sin(theta3-pi) 0 0;sin(theta3-pi) cos(theta3-pi) 0 0;0 0 1 0;0 0 0 1]*V3;
        V3_1=V3_1';
        
        V6_1=T1*T2*[cos(theta3-pi) -sin(theta3-pi) 0 0;sin(theta3-pi) cos(theta3-pi) 0 0;0 0 1 0;0 0 0 1]*V6;
        V6_1=V6_1';
        
        global plotlimit;
        if plotlimit==1
            hold on;
%                 ttemp=0:0.05:2*pi;
%                 xtemp=0.185*cos(ttemp);
%                 ytemp=0.185*sin(ttemp);
%                 plot3(xtemp,ytemp,zeros(size(xtemp)));
%                 xtemp=0.56*cos(ttemp);
%                 ytemp=0.56*sin(ttemp);
%                 plot3(xtemp,ytemp,zeros(size(xtemp)));
            hold off;
%             
%             
            hold on;
%                 [Prisms, Cylinders, Spheres, Cones] = a_ProtectionZoneDef();
%                         global Slide2_ProtectionZonePrismGap;
%                         gap = Slide2_ProtectionZonePrismGap;
%             if size(Prisms,1)>0                
%                 for i=1:size(Prisms,3)
%                     P1 = Prisms(1,1:3,i);
%                     P2 = Prisms(2,1:3,i); 
%                     P3 = Prisms(3,1:3,i); 
%                     P4 = Prisms(4,1:3,i); 
%                     plot3([P1(1);P3(1);P2(1);P4(1);P1(1)],[P1(2);P3(2);P2(2);P4(2);P1(2)],[P1(3);P3(3);P2(3);P4(3);P1(3)],'r');
%     
%                     P1 = Prisms(1,1:3,i)+[-gap;gap;0]';
%                     P2 = Prisms(2,1:3,i)+[+gap;-gap;0]';
%                     P3 = Prisms(3,1:3,i)+[gap;gap;0]'; 
%                     P4 = Prisms(4,1:3,i)+[-gap;-gap;0]'; 
%                     plot3([P1(1);P3(1);P2(1);P4(1);P1(1)],[P1(2);P3(2);P2(2);P4(2);P1(2)],[P1(3);P3(3);P2(3);P4(3);P1(3)]);
%                 end
%             end
%             
%             
            hold off;
            

            light
            axis vis3d;

            p0 = patch('faces', F0, 'vertices' ,V0(:,1:3));
            set(p0, 'facec', 'flat');  
            set(p0, 'EdgeColor','none'); 
            set(p0, 'facec', [1 1 1]);

            p1 = patch('faces', F1, 'vertices' ,V1_1(:,1:3));
            set(p1, 'facec', 'flat');  
            set(p1, 'EdgeColor','none'); 
            set(p1, 'facec', [1 1 1]);

            p2 = patch('faces', F2, 'vertices' ,V2_1(:,1:3));
            set(p2, 'facec', 'flat');  
            set(p2, 'EdgeColor','none'); 
            set(p2, 'facec', [1 1 1]);
            
            p3 = patch('faces', F3, 'vertices' ,V3_1(:,1:3));
            set(p3, 'facec', 'flat');  
            set(p3, 'EdgeColor','none'); 
            set(p3, 'facec', [1 1 1]);
            
            p6 = patch('faces', F6, 'vertices' ,V6_1(:,1:3));
            set(p6, 'facec', 'flat');  
            set(p6, 'EdgeColor','none'); 
            set(p6, 'facec', [1 1 1]);
        else
              set(p1,'vertices',V1_1(:,1:3));
              set(p2,'vertices',V2_1(:,1:3));
              set(p3,'vertices',V3_1(:,1:3));
              set(p6,'vertices',V6_1(:,1:3));
        end
          
          
          
          
    campos(CameraPosition);
    camtarget(CameraTarget);
    camup(CameraUpVector);
    camva(CameraViewAngle);
    xlim([-1 1]*1);
    ylim([-1 1]*1);
    zlim([-1 1]*1);   
    
    grid on;
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
 
    
    drawnow;  
    
hold off;

% --- Executes on slider movement.
function SliderX_Callback(hObject, eventdata, handles)
% hObject    handle to SliderX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.SliderX,'Value') ));
        set(handles.editX,'String', num2str(round(sliderValue*100)/100));
%         movePosition(handles);
        
        

% --- Executes during object creation, after setting all properties.
function SliderX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SliderX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function SliderY_Callback(hObject, eventdata, handles)
% hObject    handle to SliderY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.SliderY,'Value') ));
        set(handles.editY,'String', num2str(round(sliderValue*100)/100));
%         movePosition(handles);

% --- Executes during object creation, after setting all properties.
function SliderY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SliderY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function editX_Callback(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editX as text
%        str2double(get(hObject,'String')) returns contents of editX as a double
    textValue = str2num( get(handles.editX,'String') );
    set(handles.SliderX,'Value',textValue);
%     movePosition(handles);






% --- Executes during object creation, after setting all properties.
function editX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editY_Callback(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editY as text
%        str2double(get(hObject,'String')) returns contents of editY as a double
    textValue = str2num( get(handles.editY,'String') );
    set(handles.SliderY,'Value',textValue);
%     movePosition(handles);

% --- Executes during object creation, after setting all properties.
function editY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonPos.
function pushbuttonPos_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slidervitX_Callback(hObject, eventdata, handles)
% hObject    handle to slidervitX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.slidervitX,'Value') ));
        set(handles.editvitX,'String', num2str(round(sliderValue*100)/100));
%         movePosition(handles);



% --- Executes during object creation, after setting all properties.
function slidervitX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slidervitX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slidervitY_Callback(hObject, eventdata, handles)
% hObject    handle to slidervitY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.slidervitY,'Value') ));
        set(handles.editvitY,'String', num2str(round(sliderValue*100)/100));
%         movePosition(handles);

% --- Executes during object creation, after setting all properties.
function slidervitY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slidervitY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function editvitX_Callback(hObject, eventdata, handles)
% hObject    handle to editvitX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editvitX as text
%        str2double(get(hObject,'String')) returns contents of editvitX as a double


% --- Executes during object creation, after setting all properties.
function editvitX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editvitX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editvitY_Callback(hObject, eventdata, handles)
% hObject    handle to editvitY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editvitY as text
%        str2double(get(hObject,'String')) returns contents of editvitY as a double


% --- Executes during object creation, after setting all properties.
function editvitY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editvitY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in togglebuttonVit.
function togglebuttonVit_Callback(hObject, eventdata, handles)
% hObject    handle to togglebuttonVit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebuttonVit
while 1==1
    
    vitX = str2num( get(handles.editvitX,'String') )/50;
    vitY = str2num( get(handles.editvitY,'String') )/50;
    vitZ = str2num( get(handles.editvitZ,'String') )/50;
    vit=[vitX;vitY;vitZ];
    
    VitCart(handles, vit);
        
%     pause(0.1);
    
end






% --- Executes on button press in togglebuttonJoystick.
function togglebuttonJoystick_Callback(hObject, eventdata, handles)
% hObject    handle to togglebuttonJoystick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebuttonJoystick
Onoff=get(handles.togglebuttonJoystick,'Value');
while 1==1
    
%     tic
    
        Onoff=get(handles.togglebuttonJoystick,'Value');
        if Onoff==0
            break;
        end
    [a ab] = JoyMEX(0);
    
    
    if abs(a(2)) > 0.1
        vitX = a(2)/5;
    else
        vitX=0;
    end
    if abs(a(1)) > 0.1
        vitY = a(1)/5;
    else
        vitY=0;
    end
    if abs(a(6)) > 0.1
        vitZ = -a(6)/5;
    else
        vitZ=0;
    end

    
    vit=[vitX;vitY;vitZ];
    
    VitCart(handles, vit);
        
%     pause(0.1);
%     t=toc

end

% --- Executes on slider movement.
function sliderT1_Callback(hObject, eventdata, handles)
% hObject    handle to sliderT1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.sliderT1,'Value') ));
        set(handles.editT1,'String', num2str(round(sliderValue*100)/100));
        moveTheta(handles);

% --- Executes during object creation, after setting all properties.
function sliderT1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderT1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sliderT2_Callback(hObject, eventdata, handles)
% hObject    handle to sliderT2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.sliderT2,'Value') ));
        set(handles.editT2,'String', num2str(round(sliderValue*100)/100));
        moveTheta(handles);

% --- Executes during object creation, after setting all properties.
function sliderT2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderT2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function editT1_Callback(hObject, eventdata, handles)
% hObject    handle to editT1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editT1 as text
%        str2double(get(hObject,'String')) returns contents of editT1 as a double
    textValue = str2num( get(handles.editT1,'String') );
    set(handles.sliderT1,'Value',textValue);
    moveTheta(handles);

% --- Executes during object creation, after setting all properties.
function editT1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editT1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editT2_Callback(hObject, eventdata, handles)
% hObject    handle to editT2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editT2 as text
%        str2double(get(hObject,'String')) returns contents of editT2 as a double
    textValue = str2num( get(handles.editT2,'String') );
    set(handles.sliderT2,'Value',textValue);
    moveTheta(handles);

% --- Executes during object creation, after setting all properties.
function editT2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editT2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SeeLimitations.
function SeeLimitations_Callback(hObject, eventdata, handles)
% hObject    handle to SeeLimitations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SeeLimitations
global plotlimit;
if plotlimit==1
    plotlimit=0;
else
    plotlimit=1;
end



function posX_Callback(hObject, eventdata, handles)
% hObject    handle to posX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posX as text
%        str2double(get(hObject,'String')) returns contents of posX as a double


% --- Executes during object creation, after setting all properties.
function posX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posY_Callback(hObject, eventdata, handles)
% hObject    handle to posY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posY as text
%        str2double(get(hObject,'String')) returns contents of posY as a double


% --- Executes during object creation, after setting all properties.
function posY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on editT1 and none of its controls.
function editT1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to editT1 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in checkbox_slide2.
function checkbox_slide2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_slide2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_slide2
    global active_slide2;
    active_slide2=get(handles.checkbox_slide2, 'Value');


% --- Executes on button press in checkbox_slide1.
function checkbox_slide1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_slide1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_slide1
    global active_slide1;
    active_slide1=get(handles.checkbox_slide1, 'Value');


% --- Executes on button press in checkbox_elbow.
function checkbox_elbow_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_elbow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_elbow
    global active_elbow;
    active_elbow=get(handles.checkbox_elbow, 'Value');


% --- Executes on button press in checkbox_links.
function checkbox_links_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_links (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_links
    global active_links;
    active_links=get(handles.checkbox_links, 'Value');



function editT3_Callback(hObject, eventdata, handles)
% hObject    handle to editT3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editT3 as text
%        str2double(get(hObject,'String')) returns contents of editT3 as a double
    textValue = str2num( get(handles.editT3,'String') );
    set(handles.sliderT3,'Value',textValue);
    moveTheta(handles);

% --- Executes during object creation, after setting all properties.
function editT3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editT3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function sliderT3_Callback(hObject, eventdata, handles)
% hObject    handle to sliderT3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.sliderT3,'Value') ));
        set(handles.editT3,'String', num2str(round(sliderValue*100)/100));
        moveTheta(handles);

% --- Executes during object creation, after setting all properties.
function sliderT3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderT3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function editvitZ_Callback(hObject, eventdata, handles)
% hObject    handle to editvitZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editvitZ as text
%        str2double(get(hObject,'String')) returns contents of editvitZ as a double
        sliderValue = (( get(handles.slidervitZ,'Value') ));
        set(handles.editvitZ,'String', num2str(round(sliderValue*100)/100));
%         movePosition(handles);

% --- Executes during object creation, after setting all properties.
function editvitZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editvitZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slidervitZ_Callback(hObject, eventdata, handles)
% hObject    handle to slidervitZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
        sliderValue = (( get(handles.slidervitZ,'Value') ));
        set(handles.editvitZ,'String', num2str(round(sliderValue*100)/100));

% --- Executes during object creation, after setting all properties.
function slidervitZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slidervitZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function posZ_Callback(hObject, eventdata, handles)
% hObject    handle to posZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posZ as text
%        str2double(get(hObject,'String')) returns contents of posZ as a double


% --- Executes during object creation, after setting all properties.
function posZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
