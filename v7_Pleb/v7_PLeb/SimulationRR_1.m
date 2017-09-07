function varargout = SimulationRR_1(varargin)
% SIMULATIONRR_1 MATLAB code for SimulationRR_1.fig
%      SIMULATIONRR_1, by itself, creates a new SIMULATIONRR_1 or raises the existing
%      singleton*.
%
%      H = SIMULATIONRR_1 returns the handle to a new SIMULATIONRR_1 or the handle to
%      the existing singleton*.
%
%      SIMULATIONRR_1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULATIONRR_1.M with the given input arguments.
%
%      SIMULATIONRR_1('Property','Value',...) creates a new SIMULATIONRR_1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SimulationRR_1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SimulationRR_1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SimulationRR_1

% Last Modified by GUIDE v2.5 25-Sep-2015 12:02:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SimulationRR_1_OpeningFcn, ...
                   'gui_OutputFcn',  @SimulationRR_1_OutputFcn, ...
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


function [D,LimitAng] = DefParam(handles)
L1=0.29;
L2=0.29;
D=[L1;L2];

LimitAng = [-30 240;-150 150]*pi/180;


% --- Executes just before SimulationRR_1 is made visible.
function SimulationRR_1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SimulationRR_1 (see VARARGIN)

    global plotlimit;
    plotlimit=0;

    global V0;
    global F0;
    global C0;
    global V1;
    global F1;
    global C1;
    global V2;
    global F2;
    global C2;
    
    
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
    filename = 'M2_bin.STL';
    [V1, F1, N1, C1, stltitle] = stlread(filename, 0);
    [V1, F1]=patchslim(V1, F1);
        
    sc=1;
    px=-0.041255;py=0.041258;pz=0.1125;
    
    V1 = V1';
    V1 = [V1(1,:); V1(2,:); V1(3,:); ones(1,length(V1))];
    V1=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V1;
    V1=[1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 1]*V1;
    V1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V1;
    V1 = [V1(1,:); V1(2,:); V1(3,:); ones(1,length(V1))];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename = 'M2_bin.STL';
    [V2, F2, N2, C2, stltitle] = stlread(filename, 0);
    [V2, F2]=patchslim(V2, F2);
        
    sc=1;
    px=-0.041255;py=-0.041258;pz=-0.1-0.01238-0.00016;
    
    V2 = V2';
    V2 = [V2(1,:); V2(2,:); V2(3,:); ones(1,length(V2))];
    V2=[sc 0 0 0;0 sc 0 0;0 0 sc 0;0 0 0 1]*V2;
%     V2=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]*V2;
    V2=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V2;
    V2 = [V2(1,:); V2(2,:); V2(3,:); ones(1,length(V2))];    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    set(gcf, 'renderer', 'OpenGL');
    plot3(0,0,0,'x');
    xlim([-1 1]*0.7);
    ylim([-1 1]*0.7);
    zlim([-1 1]*0.7);
    
    axis vis3d;

    px=0;py=0;pz=(+0.1185+0.04);
    V1_1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V1;
    V1_1=V1_1';

    px=0.29;py=0;pz=0.1585;
    V2_1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V2;
    V2_1=V2_1';
    
    global p1;
    global p2;
    
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
        
        light
         
        view([315 24]);
        view([0 90]);

        grid on;
        xlabel('X');ylabel('Y');zlabel('Z');
%         plotter(handles);
        
        clear JoyMEX
        JoyMEX('init',0);
        JoyMEX('init',1);
        
    theta=[0;0];


% Choose default command line output for SimulationRR_1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SimulationRR_1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


GoReady(handles);




% --- Outputs from this function are returned to the command line.
function varargout = SimulationRR_1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function GoReady(handles)
    set(handles.editT1,'String', 90);
    set(handles.editT2,'String', -45);
    set(handles.sliderT1,'Value', 90);
    set(handles.sliderT2,'Value', -45);
    moveTheta(handles);

function movePosition(handles)
% Receives a position


function moveTheta(handles)
% Receives a position
        plotter(handles);
    theta1=str2num( get(handles.editT1,'String') )*pi/180;
    theta2=str2num( get(handles.editT2,'String') )*pi/180;
    theta=[theta1;theta2];
    
    [T,T1,T2] = CinDirecte(handles, theta);
        
    
function VitCart(handles, vit)


    vitX = vit(1);
    vitY = vit(2);
    vitnorm = sqrt(vitX^2+vitY^2);
    
    vitmax = 0.03;
    if vitnorm > vitmax
        vitX=vitX*vitmax/vitnorm;
        vitY=vitY*vitmax/vitnorm;
    end
    
    vit1=[vitX;vitY;0];
    
    theta1=str2num( get(handles.editT1,'String') )*pi/180;
    theta2=str2num( get(handles.editT2,'String') )*pi/180;
    theta=[theta1;theta2];
    
    [T,T1,T2] = CinDirecte(handles, theta);
    
    Limitation = LimitationPlanes2(handles,theta,T);
    [vit2,LimitationActive]=Slide2(handles, Limitation, vit1);
      
    global plotlimit
    if plotlimit==1
        cla
        for i=1:size(LimitationActive,2)   
            hold on;
                plot3([T(1,4)-LimitationActive(2,i);T(1,4)+LimitationActive(2,i)],[T(2,4)+LimitationActive(1,i);T(2,4)-LimitationActive(1,i)],[0;0])
            hold off;
        end
    end

    posDes = [T(1,4)+vit2(1);T(2,4)+vit2(2)];
    

    
    [theta,error]=InvKinematics(handles,posDes);
    
    if error==0
        set(handles.editT1,'String', theta(1)*180/pi);
        set(handles.editT2,'String', theta(2)*180/pi);
    end
    
    plotter(handles);

    
    
    
function [AxeRes,LimitationActive]=Slide2(handles, Limitation, Axe)

        numberLimitActive = 0;  % Number of active limits detected
        LimitationActive=[];    % Active limits detected
        nbCrossOK=0;            % Number of common lines between planes that are not equals.
        nbCrossOK2=0;           % Number of accepted vectors
        VectbGood=1;            % If resulting vector accepted
        Vectb=[];               % Resulting vector
        Vectbkeep=[];           % Final vector
        
        for i=1:size(Limitation,2) 
            if norm(Limitation(:,i))>=0
               Limitation(:,i)=Limitation(:,i)/norm(Limitation(:,i));
               if (Axe'*Limitation(:,i)) < 0
                   numberLimitActive=numberLimitActive+1;
                   LimitationActive=[LimitationActive,Limitation(:,i)/norm(Limitation(:,i))];
               end
            end
        end
        
        if numberLimitActive==0 % If no active limitations  
            AxeRes=Axe; % The resulting vector is the initial vector
        elseif numberLimitActive==1 % If 1 active limitation
            Vecta=(Axe'*LimitationActive)*LimitationActive; % Component along restriceted direction
            AxeRes=Axe-Vecta; % Remove the component from the vector
        else %If several limit active
            for i=1:numberLimitActive % For all combination pairs between limitations planes
                Vecta=(Axe'*LimitationActive(:,i))*LimitationActive(:,i); % Component along restriceted direction
                Vectb(:,i)=Axe-Vecta; % Remove the component from the vector
            end
            
            for i=1:size(Vectb,2) % For each resulting vector
                VectbGood=1;% initialize at 1
                for j=1:size(Limitation,2)% check for all limitations
                    if (Vectb(:,i)'*Limitation(:,j))<-0.000000000001 %if the resulting vector goes in the limitation
                        VectbGood = 0;
                    end
                end
                if VectbGood==1 % If the vector is good
                    Vectbkeep=Vectb(:,i);%Keep the vector as the final vector
                    nbCrossOK2=nbCrossOK2+1;%Increase the number of accepted vector.
                end % we assume here that only one is good since we overwrite in Vectbkeep. nbCrossOK2 can be equal to 2 or more but each vector should be the same. This should be verified more thoroughly.
            end
            if nbCrossOK2>0 %If one vector is accepted.
                AxeRes = Vectbkeep;%we keep the vector
            else
                AxeRes=[0;0;0];%else set to zero
            end
            
        end
        for j=1:size(Limitation,2) % Check that the resulting vector satisfies all limitations.
            if (AxeRes'*Limitation(:,j))<-0.000000000001
                AxeRes = [0;0;0];
            end
        end
            
            



        
function Limitation = LimitationPlanes2(handles,theta,T)

    [D,LimitAng] = DefParam(handles);

    [J, Jinv] = Jacobian(handles, theta);
       
    Limitation = [];
    

    % Theta1 Limitation
        if theta(1) <= LimitAng(1,1)+6*pi/180;
            Limitation = [Limitation,[Jinv(1,1);Jinv(1,2);0]];
        elseif theta(1) >= LimitAng(1,2)-6*pi/180;
            Limitation = -[Limitation,[Jinv(1,1);Jinv(1,2);0]];
        end
       
    % Theta2 Limitation
        if theta(2) <= LimitAng(2,1)+6*pi/180;
            Limitation = [Limitation,[Jinv(2,1);Jinv(2,2);0]];
        elseif theta(2) >= LimitAng(2,2)-6*pi/180;
            Limitation = -[Limitation,[Jinv(2,1);Jinv(2,2);0]];
        end
        
    % Maximum reach limitation
        if sqrt(T(1,4)^2+T(2,4)^2) >= (D(1)+D(2))-0.05
            Limitation = -[Limitation,[T(1,4);T(2,4);0]];
        end
        

function [J, Jinv] = Jacobian(handles, theta)

    [D,LimitAng] = DefParam(handles);
    L1=D(1);
    L2=D(2);
    
    theta1=theta(1);
    theta2=theta(2);

    J=[-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2);
       L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2)];

    Jinv=pinv(J);

    
    
    
function [T,T1,T2] = CinDirecte(handles, theta) 
    theta1=theta(1);
    theta2=theta(2);
    
    [D,LimitAng] = DefParam(handles);
    L1=D(1);
    L2=D(2);

    T1=[cos(theta1) -sin(theta1) 0 L1*cos(theta1);
        sin(theta1) cos(theta1) 0 L1*sin(theta1);
        0 0 1 0;
        0 0 0 1];
    
    T2=[cos(theta2) -sin(theta2) 0 L2*cos(theta2);
    sin(theta2) cos(theta2) 0 L2*sin(theta2);
    0 0 1 0;
    0 0 0 1];

    T=T1*T2;
        
    
function [thetaout,error] = InvKinematics(handles,posDes)
    error=0;

    [D,LimitAng] = DefParam(handles);
    L1=D(1);
    L2=D(2);
    
    cylindreInt=0.05;
    
    
    theta1=str2num( get(handles.editT1,'String') )*pi/180;
    theta2=str2num( get(handles.editT2,'String') )*pi/180;
    thetain=[theta1;theta2];
    
    
    x = posDes(1);
    y = posDes(2);

    ct2 = (x^2+y^2-L1^2-L2^2)/(2*L1*L2);

    
    if 1-ct2^2 < 0
        error=1;
    end
    if sqrt(x^2+y^2) >= (L1+L2)
        error=1;
    end
    if sqrt(x^2+y^2) <= (cylindreInt)
        error=1;
    end
    
    if (error==0)
        st2 = sqrt(1-ct2^2);
        theta2_1 = atan2(st2,ct2);
        theta2_2 = atan2(-st2,ct2);

        K1_1 = L1+L2*cos(theta2_1);
        K2_1 = L2*sin(theta2_1);
        theta1_1 = atan2(y,x)-atan2(K2_1,K1_1);

        K1_2 = L1+L2*cos(theta2_2);
        K2_2 = L2*sin(theta2_2);
        theta1_2 = atan2(y,x)-atan2(K2_2,K1_2);

        % theta1_1
        % theta1_2
        % theta2_1
        % theta2_2

        if theta2_2<-pi
            theta2_2=theta2_2+2*pi;
        end
        if theta2_2>pi
            theta2_2=theta2_2-2*pi;
        end
        
        if theta2_1<-pi
            theta2_1=theta2_1+2*pi;
        end
        if theta2_1>pi
            theta2_1=theta2_1-2*pi;
        end
        
        
        if theta2_2 >= -pi && theta2_2 <= 0
            thetaout1=theta1_2;
            thetaout2=theta2_2;
        else
            thetaout1=theta1_1;
            thetaout2=theta2_1;
        end
    else
        thetaout1=0;
        thetaout2=0;
    end
    
    if thetaout1<-pi/2
        thetaout1=thetaout1+2*pi;
    end
    if thetaout1>3*pi/2
        thetaout1=thetaout1-2*pi;
    end
    
    if thetain(1)<-pi/2
        thetain(1)=thetain(1)+2*pi;
    end
    if thetain(1)>3*pi/2
        thetain(1)=thetain(1)-2*pi;
    end
    
    if thetaout2<-pi
        thetaout2=thetaout2+2*pi;
    end
    if thetaout2>pi
        thetaout2=thetaout2-2*pi;
    end
    
    if thetain(2)<-pi
        thetain(2)=thetain(2)+2*pi;
    end
    if thetain(2)>pi
        thetain(2)=thetain(2)-2*pi;
    end
    
    difftheta=[thetaout1;thetaout2]-thetain;
    maxthetainc=2*pi/180;
    if norm(difftheta)>maxthetainc
        thetaout1 = thetain(1) + difftheta(1)*maxthetainc/norm(difftheta);
        thetaout2 = thetain(2) + difftheta(2)*maxthetainc/norm(difftheta);
    end
    
    [T,T1,T2] = CinDirecte(handles, [thetaout1;thetaout2]);
    

    if sqrt(T(1,4)^2+T(2,4)^2) <= (cylindreInt)
        error=1;
    end
    
    if (thetaout1 < LimitAng(1,1) || thetaout1 > LimitAng(1,2))
        error=1;
    end
    if (thetaout2 < LimitAng(2,1) || thetaout2 > LimitAng(2,2))
        error=1;
    end
    
    if error==1
        thetaout1=0;
        thetaout2=0;
    end
    

    thetaout=[thetaout1;thetaout2];
    


    
    
function plotter (handles) 
    theta1=str2num( get(handles.editT1,'String') )*pi/180;
    theta2=str2num( get(handles.editT2,'String') )*pi/180;
    theta=[theta1;theta2];
    
    [T,T1,T2] = CinDirecte(handles, theta);
    
    


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
    
    global p1;
    global p2;
    

    
        px=0;py=0;pz=+0.1185+0.04;
        V1_1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V1;
        V1_1=[cos(theta1) -sin(theta1) 0 0;sin(theta1) cos(theta1) 0 0;0 0 1 0;0 0 0 1]*V1_1;
        V1_1=V1_1';

        px=0.29*0;py=0;pz=0.1585;
        V2_1=[1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1]*V2;
        V2_1=T1*[cos(theta2) -sin(theta2) 0 0;sin(theta2) cos(theta2) 0 0;0 0 1 0;0 0 0 1]*V2_1;
%  V2_1=V2;
        V2_1=V2_1';

        global plotlimit;
        if plotlimit==1
            hold on;
            ttemp=0:0.05:2*pi;
            xtemp=0.185*cos(ttemp);
            ytemp=0.185*sin(ttemp);
            plot3(xtemp,ytemp,zeros(size(xtemp)));
            xtemp=0.54*cos(ttemp);
            ytemp=0.54*sin(ttemp);
            plot3(xtemp,ytemp,zeros(size(xtemp)));
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
        else
              set(p1,'vertices',V1_1(:,1:3));
              set(p2,'vertices',V2_1(:,1:3));
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
    vit=[vitX;vitY];
    
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
    
    
    if abs(a(1)) > 0.1
        vitX = a(1)/50;
    else
        vitX=0;
    end
    if abs(a(2)) > 0.1
        vitY = -a(2)/50;
    else
        vitY=0;
    end

    
    vit=[vitX;vitY];
    
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
    plotlimite=0;
else
    plotlimit=1;
end