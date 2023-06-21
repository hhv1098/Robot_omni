function varargout = test_xulymau4_guide(varargin)
% TEST_XULYMAU4_GUIDE MATLAB code for test_xulymau4_guide.fig
%      TEST_XULYMAU4_GUIDE, by itself, creates a new TEST_XULYMAU4_GUIDE or raises the existing
%      singleton*.
%
%      H = TEST_XULYMAU4_GUIDE returns the handle to a new TEST_XULYMAU4_GUIDE or the handle to
%      the existing singleton*.
%
%      TEST_XULYMAU4_GUIDE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEST_XULYMAU4_GUIDE.M with the given input arguments.
%
%      TEST_XULYMAU4_GUIDE('Property','Value',...) creates a new TEST_XULYMAU4_GUIDE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before test_xulymau4_guide_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to test_xulymau4_guide_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help test_xulymau4_guide
% Last Modified by GUIDE v2.5 23-Mar-2019 11:34:15
% Begin initialization code - DO NOT EDIT

% 200x150; 640x480;
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @test_xulymau4_guide_OpeningFcn, ...
                   'gui_OutputFcn',  @test_xulymau4_guide_OutputFcn, ...
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


% --- Executes just before test_xulymau4_guide is made visible.
function test_xulymau4_guide_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to test_xulymau4_guide (see VARARGIN)

% Choose default command line output for test_xulymau4_guide
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes test_xulymau4_guide wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = test_xulymau4_guide_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Camera.
function Camera_Callback(hObject, eventdata, handles)
% hObject    handle to Camera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Initialization
global s x_mouse y_mouse;
s = serial('COM10');
set(s,'Baudrate', 19200);
fopen(s);

redThresh = 0.2; % Threshold for red detection
greenThresh = 0.05; % Threshold for green detection

vidDevice = imaq.VideoDevice('winvideo',1,'I420_640x480', 'ROI', [1 1 640 480], 'ReturnedColorSpace', 'rgb');
vidInfo = imaqhwinfo(vidDevice); % Acquire input video property
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 200, ...
                                'MaximumBlobArea', 3000, ...
                                'MaximumCount', 10);
hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ... % Set box handling
                                        'Fill', true, ...
                                        'FillColorSource', 'Input port', ...
                                        'Opacity', 0.4);
htextinsRed = vision.TextInserter('Text', 'Red   : %2d', ... % Set text for number of blobs
                                    'Location',  [5 2], ...
                                    'Color', [1 0 0], ... // red color
                                    'Font', 'Courier New', ...
                                    'FontSize', 14);
htextinsGreen = vision.TextInserter('Text', 'Green : %2d', ... % Set text for number of blobs
                                    'Location',  [5 18], ...
                                    'Color', [0 1 0], ... // green color
                                    'Font', 'Courier New', ...
                                    'FontSize', 14);

htextinsCent = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % set text for centroid
                                    'LocationSource', 'Input port', ...
                                    'Color', [1 1 0], ... // yellow color
                                    'Font', 'Courier New', ...
                                    'FontSize', 14);
hVideoIn = vision.VideoPlayer('Name', 'Final Video', ... % Output video player
                                'Position', [100 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);
nFrame = 0; % Frame number initialization

%% Processing Loop
while (nFrame < 200)
    rgbFrame = step(vidDevice); % Acquire single frame
    rgbFrame = flipdim(rgbFrame,1); % obtain the mirror image for displaying
    
    diffFrameRed = imsubtract(rgbFrame(:,:,1), rgb2gray(rgbFrame)); % Get red component of the image
    diffFrameRed = medfilt2(diffFrameRed, [3 3]); % Filter out the noise by using median filter
    binFrameRed = im2bw(diffFrameRed, redThresh); % Convert the image into binary image with the red objects as white
    
    diffFrameGreen = imsubtract(rgbFrame(:,:,2), rgb2gray(rgbFrame)); % Get green component of the image
    diffFrameGreen = medfilt2(diffFrameGreen, [3 3]); % Filter out the noise by using median filter
    binFrameGreen = im2bw(diffFrameGreen, greenThresh); % Convert the image into binary image with the green objects as white
    

    [centroidRed, bboxRed] = step(hblob, binFrameRed); % Get the centroids and bounding boxes of the red blobs
    centroidRed = uint16(centroidRed); % Convert the centroids into Integer for further steps 
    
    [centroidGreen, bboxGreen] = step(hblob, binFrameGreen); % Get the centroids and bounding boxes of the green blobs
    centroidGreen = uint16(centroidGreen); % Convert the centroids into Integer for further steps 

    rgbFrame(1:50,1:90,:) = 0; % put a black region on the output stream
    vidIn = step(hshapeinsBox, rgbFrame, bboxRed, single([1 0 0])); % Instert the red box
    vidIn = step(hshapeinsBox, vidIn, bboxGreen, single([0 1 0])); % Instert the green box

    for object = 1:1:length(bboxRed(:,1)) % Write the corresponding centroids for red
        centXRed = centroidRed(object,1);
        centYRed = centroidRed(object,2);
        vidIn = step(htextinsCent, vidIn, [centXRed centYRed], [centXRed-6 centYRed-9]); 
    end
    
    for object = 1:1:length(bboxGreen(:,1)) % Write the corresponding centroids for green
        centXGreen = centroidGreen(object,1); 
        centYGreen = centroidGreen(object,2);
        vidIn = step(htextinsCent, vidIn, [centXGreen centYGreen], [centXGreen-6 centYGreen-9]); 
    end
     tile=3.2;
     
%     x_Mouse=get(handles.lbl_x, 'string'); % update text for x loc
%     x_Mouse = str2num(x_Mouse);
%     y_Mouse=get(handles.lbl_y, 'string');% update text for y loc 
%     y_Mouse = str2num(y_Mouse);
    mang = [centXRed/tile,centYRed/tile,centXGreen/tile,centYGreen/tile,x_mouse*10,y_mouse*10];
%  mang = [x_Mouse,y_Mouse,centXGreen/tile,centYGreen/tile];
%    length(mang)
   set(handles.x_do, 'string', mang(1));
   set(handles.y_do, 'string', mang(2));
   
   set(handles.x_xanh, 'string', mang(3));
   set(handles.y_xanh, 'string', mang(4));
   
   arrayString='!';
   specCharArray='@#$%^&';
   for  i=1:length(mang)
       arrayString=strcat(arrayString, num2str(mang(i)),specCharArray(i));
   end
   fprintf(s,arrayString);
   %arrayString
   %mang(1)
    vidIn = step(htextinsRed, vidIn, uint8(length(bboxRed(:,1)))); % Count the number of red blobs
    vidIn = step(htextinsGreen, vidIn, uint8(length(bboxGreen(:,1)))); % Count the number of green blobs
   
    x_tam = 100*tile;
    y_tam =75*tile;
    bk=45*tile;

    
    t=0:0.001:1;
    x=x_tam+bk*sin(2*pi*2*t);
    y=y_tam+bk*cos(2*pi*2*t);
    
%      x_8=x_tam+bk*sin(2*pi*2*t);
%      y_8=y_tam+bk*cos(1*pi*2*t);
     %hold on
    plot(x,y, 'b',x_tam, y_tam,'*');%Plotting sin Vs cos
    hold on
    plot(mang(1)*tile, mang(2)*tile,'b--o');
    %axes(handles.axes2); cla;
%     plot(x_8,y_8,x_tam, y_tam,'*');%Plotting sin Vs cos
    axes(handles.axes1);cla;
    imshow(vidIn);   
   % axes(handles.axes2);
    nFrame = nFrame+1;
    if nFrame >100
        nFrame=0;
    end 
end


%% Clearing Memory
release(hVideoIn); % Release all memory and buffer used
release(vidDevice);
clear all;
clc;
fclose(s);
delete(s);
close


% --- Executes on button press in thoat.
function thoat_Callback(hObject, eventdata, handles)
% hObject    handle to thoat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s;

close
fclose(s);
delete(s);
release(hVideoIn); % Release all memory and buffer used
release(vidDevice);

clear all;
clc;



function x_do_Callback(hObject, eventdata, handles)
% hObject    handle to x_do (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_do as text
%        str2double(get(hObject,'String')) returns contents of x_do as a double


% --- Executes during object creation, after setting all properties.
function x_do_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_do (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_do_Callback(hObject, eventdata, handles)
% hObject    handle to y_do (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_do as text
%        str2double(get(hObject,'String')) returns contents of y_do as a double


% --- Executes during object creation, after setting all properties.
function y_do_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_do (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_xanh_Callback(hObject, eventdata, handles)
% hObject    handle to x_xanh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_xanh as text
%        str2double(get(hObject,'String')) returns contents of x_xanh as a double


% --- Executes during object creation, after setting all properties.
function x_xanh_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_xanh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_xanh_Callback(hObject, eventdata, handles)
% hObject    handle to y_xanh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_xanh as text
%        str2double(get(hObject,'String')) returns contents of y_xanh as a double


% --- Executes during object creation, after setting all properties.
function y_xanh_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_xanh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Connect_com.
function Connect_com_Callback(hObject, eventdata, handles)
% hObject    handle to Connect_com (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global s;
s = serial('COM10');
set(s,'Baudrate', 19200);
fopen(s);

% --- Executes on button press in guichuoi.
function guichuoi_Callback(hObject, eventdata, handles)
% hObject    handle to guichuoi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s;
fprintf(s,'1234567E');


% --- Executes on button press in disconnect.
function disconnect_Callback(hObject, eventdata, handles)
% hObject    handle to disconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse motion over figure - except title and menu.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s x_mouse  y_mouse ;
pos = get(hObject, 'currentpoint'); % get mouse location on figure
x_mouse =pos(1); y_mouse = 15- pos(2); % assign locations to x and y

set(handles.lbl_x, 'string', [num2str(10*x_mouse)]); % update text for x loc
set(handles.lbl_y, 'string', [num2str(10*y_mouse)]); % update text for y loc 


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s;
pos = get(hObject, 'currentpoint'); % get mouse location on figure
x_mouse = pos(1); y_mouse = pos(2); % assign locations to x and y
set(handles.lbl_last_action, 'string', [' X: ', num2str(x_mouse), ', Y: ', num2str(y_mouse)]);

% fprintf(s,'1234567E');

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('thien');


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2
