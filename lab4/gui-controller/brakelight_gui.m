function varargout = brakelight_gui(varargin)
% BRAKELIGHT_GUI MATLAB code for brakelight_gui.fig
%      BRAKELIGHT_GUI, by itself, creates a new BRAKELIGHT_GUI or raises the existing
%      singleton*.
%
%      H = BRAKELIGHT_GUI returns the handle to a new BRAKELIGHT_GUI or the handle to
%      the existing singleton*.
%
%      BRAKELIGHT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BRAKELIGHT_GUI.M with the given input arguments.
%
%      BRAKELIGHT_GUI('Property','Value',...) creates a new BRAKELIGHT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before brakelight_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to brakelight_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help brakelight_gui

% Last Modified by GUIDE v2.5 08-Oct-2019 15:44:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @brakelight_gui_OpeningFcn, ...
    'gui_OutputFcn',  @brakelight_gui_OutputFcn, ...
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
%mode
global mode;
mode = 0;
%lcm initialization
global lc;
global aggregator;
global aggregator2;
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('EXAMPLE_int', aggregator);
aggregator2 = lcm.lcm.MessageAggregator();
lc.subscribe('EXAMPLE_ext', aggregator2);
%timer
global t;
t = 0;
global timerstarted;
timerstarted = 0;
global modechanged;
modechanged = 0;
% End initialization code - DO NOT EDIT
end


% --- Executes just before brakelight_gui is made visible.
function brakelight_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to brakelight_gui (see VARARGIN)

% Choose default command line output for brakelight_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes brakelight_gui wait for user response (see UIRESUME)
% uiwait(handles.brakelight);
end

% --- Outputs from this function are returned to the command line.
function varargout = brakelight_gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end


function brakelight_Callback(hObject, eventdata, handles)
% hObject    handle to brakelight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of brakelight as text
%        str2double(get(hObject,'String')) returns contents of brakelight as a double
end

% --- Executes during object creation, after setting all properties.
function brakelight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to brakelight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in mode_one_button.
function mode_one_button_Callback(hObject, eventdata, handles)
% hObject    handle to mode_one_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mode_one_button
global mode;
mode = 1;
global modechanged;
modechanged = 1;
printfcn(handles);
end

% --- Executes on button press in mode_zero_button.
function mode_zero_button_Callback(hObject, eventdata, handles)
% hObject    handle to mode_zero_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mode_zero_button
global mode;
mode = 0;
global modechanged;
modechanged = 1;
printfcn(handles);
end

% --- Executes on button press in mode_two_button.
function mode_two_button_Callback(hObject, eventdata, handles)
% hObject    handle to mode_two_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mode_two_button
global mode;
mode = 2;
global modechanged;
modechanged = 1;
printfcn(handles);
end

function setBrake(handles)
set(handles.brake_light, 'String', "brake");
set(handles.brake_light, 'BackgroundColor', "red");
end

function resetBrake(handles)
set(handles.brake_light, 'String', "no brake");
set(handles.brake_light, 'BackgroundColor', "green");
end

% --- Executes on button press in enable_brakelight.
function enable_brakelight_Callback(hObject, eventdata, handles)
% hObject    handle to enable_brakelight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_brakelight
printfcn(handles);
end

function printfcn(handles)
%this is the function running for lcm when activated
global t;
global timerstarted;
global modechanged;
if(get(handles.enable_brakelight, 'value'))
    global aggregator;
    global aggregator2;
    global lc;
    global mode;
    
    while true
        millis_to_wait = 1000;
        msg = aggregator.getNextMessage(millis_to_wait);
        msg2 = aggregator2.getNextMessage(millis_to_wait);
        if length(msg) > 0
            m = exlcm.detectmsg_t(msg.data);
            sendmsg = exlcm.detectmsg_t();
            if (mode == 0)
                if not(t == 0)
                    stop(t);
                    t = 0;
                end
                if equals(m.type, "alert")
                    setBrake(handles);
                else
                    resetBrake(handles);
                end
            end
            if (mode == 1)
                if equals(m.type, "alert")
                    setBrake(handles);
                    if not(equals(t, 0))
                        stop(t);
                        t = 0;
                    end
                    sendmsg.timestamp = datestr(now,'HH:MM:SS.FFF');
                    sendmsg.type = "alert";
                    sendmsg.mode = 1;
                    lc.publish('EXAMPLE_ext', sendmsg);
                    lc.publish('EXAMPLE_int', sendmsg);
                else
                    resetBrake(handles);
                    %timer for heartbeatmsg
                    if or((modechanged == 1), (timerstarted == 0))
                        if not(equals(t, 0))
                            stop(t);
                            t = 0;
                        end
                        t = timer('ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', sendheartbeat(1));
                        start(t);
                        modechanged = 0;
                        timerstarted = 1;
                    end
                end
            end
        end
        if length(msg2) > 0
            m_ext = exlcm.detectmsg_t(msg.data);
            sendmsg = exlcm.detectmsg_t();
            if (mode == 2)
                if equals(m_ext.type, "alert")
                    setBrake(handles);
                    if not(equals(t, 0))
                        stop(t);
                        t = 0;
                    end
                    sendmsg.timestamp = datestr(now,'HH:MM:SS.FFF');
                    sendmsg.type = "alert";
                    sendmsg.mode = 1;
                    lc.publish('EXAMPLE_ext', sendmsg);
                    lc.publish('EXAMPLE_int', sendmsg);
                else
                    resetBrake(handles);
                    %timer for heartbeatmsg
                    if or((modechanged == 1), (timerstarted == 0))
                        if not(equals(t, 0))
                            stop(t);
                            t = 0;
                        end
                        t = timer('ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', sendheartbeat(2));
                        start(t);
                        modechanged = 0;
                        timerstarted = 1;
                    end
                end
            end
        end
    end
else
    if not(t == 0)
        stop(t);
        t = 0;
    end
end
end

function sendheartbeat(heartbeatmode)
global lc;
sendmsg = exlcm.detectmsg_t();
sendmsg.timestamp = datestr(now,'HH:MM:SS.FFF');
sendmsg.type = "heartbeat";
sendmsg.mode = heartbeatmode;
lc.publish('EXAMPLE', sendmsg);

%%braking
bl = 1;
%setting or resetting brakelight
if (bl == 1)
    setBrake(handles);
else
    resetBrake(handles);
end
end
