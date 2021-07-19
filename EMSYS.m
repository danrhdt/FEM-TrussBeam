function varargout = EMSYS(varargin)
% EMSYS MATLAB code for EMSYS.fig
%      EMSYS, by itself, creates a new EMSYS or raises the existing
%      singleton*.
%
%      H = EMSYS returns the handle to a new EMSYS or the handle to
%      the existing singleton*.
%
%      EMSYS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in EMSYS.M with the given input arguments.
%
%      EMSYS('Property','Value',...) creates a new EMSYS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before EMSYS_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to EMSYS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help EMSYS

% Last Modified by GUIDE v2.5 17-Dec-2020 15:51:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @EMSYS_OpeningFcn, ...
                   'gui_OutputFcn',  @EMSYS_OutputFcn, ...
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


% --- Executes just before EMSYS is made visible.
function EMSYS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to EMSYS (see VARARGIN)

% Choose default command line output for EMSYS
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes EMSYS wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = EMSYS_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenuBC.
function popupmenuBC_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuBC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(hObject,'Value')
     case 1
         
         set(handles.uipanelBCD,'visible','on');
         set(handles.uipanelBCR,'visible','off');
     case 2
         
         set(handles.uipanelBCD,'visible','off');
         set(handles.uipanelBCR,'visible','on');
     
 end
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuBC contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuBC


% --- Executes during object creation, after setting all properties.
function popupmenuBC_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuBC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushCloseLoad.
function pushCloseLoad_Callback(hObject, eventdata, handles)
% hObject    handle to pushCloseLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.uipanelLoad,'visible','off');

function editEm_Callback(hObject, eventdata, handles)
% hObject    handle to editEm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editEm as text
%        str2double(get(hObject,'String')) returns contents of editEm as a double


% --- Executes during object creation, after setting all properties.
function editEm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editEm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editVP_Callback(hObject, eventdata, handles)
% hObject    handle to editVP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVP as text
%        str2double(get(hObject,'String')) returns contents of editVP as a double


% --- Executes during object creation, after setting all properties.
function editVP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editArea_Callback(hObject, eventdata, handles)
% hObject    handle to editArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editArea as text
%        str2double(get(hObject,'String')) returns contents of editArea as a double


% --- Executes during object creation, after setting all properties.
function editArea_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editJnteror_Callback(hObject, eventdata, handles)
% hObject    handle to editJnteror (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editJnteror as text
%        str2double(get(hObject,'String')) returns contents of editJnteror as a double


% --- Executes during object creation, after setting all properties.
function editJnteror_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editJnteror (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Ok_Material.
function Ok_Material_Callback(hObject, eventdata, handles)
% hObject    handle to Ok_Material (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global E A J v
E = str2double(get(handles.editEm,'string'))
v = str2double(get(handles.editVP,'string'))
A = str2double(get(handles.editArea,'string'))
J = str2double(get(handles.editJnteror,'string'))
set(handles.uipanelMaterial,'visible','off');

function editNode_Callback(hObject, eventdata, handles)
% hObject    handle to editNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editNode as text
%        str2double(get(hObject,'String')) returns contents of editNode as a double


% --- Executes during object creation, after setting all properties.
function editNode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editX_Callback(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editX as text
%        str2double(get(hObject,'String')) returns contents of editX as a double


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



function editElem_Callback(hObject, eventdata, handles)
% hObject    handle to editElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editElem as text
%        str2double(get(hObject,'String')) returns contents of editElem as a double


% --- Executes during object creation, after setting all properties.
function editElem_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editI_Callback(hObject, eventdata, handles)
% hObject    handle to editI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editI as text
%        str2double(get(hObject,'String')) returns contents of editI as a double


% --- Executes during object creation, after setting all properties.
function editI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editJ_Callback(hObject, eventdata, handles)
% hObject    handle to editJ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editJ as text
%        str2double(get(hObject,'String')) returns contents of editJ as a double


% --- Executes during object creation, after setting all properties.
function editJ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editJ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushApplyNode.
function pushApplyNode_Callback(hObject, eventdata, handles)
% hObject    handle to pushApplyNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global toado
axes(handles.axes1);
ID = str2double(get(handles.editNode,'string'));
x = str2double(get(handles.editX,'string'));
y = str2double(get(handles.editY,'string'));
toado(ID,1:2) = [x y]
hold on
plot(x,y,'.-','markersize',25)
axis([min(toado(:))-1 max(toado(:))+1 min(toado(:))-1 max(toado(:))+1])
text(x+0.08,y+0.08,num2str(ID),'BackgroundColor',[0.2 1 1],...
    'FontSize',12,'Color',[0 0 0],'LineWidth',12)
% --- Executes on button press in pushApplyElem.
function pushApplyElem_Callback(hObject, eventdata, handles)
% hObject    handle to pushApplyElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%%%%%---------Ve phan tu
global toado phantu
axes(handles.axes1);
ID = str2double(get(handles.editElem,'string'))
pt1 = str2double(get(handles.editI,'string'));
pt2 = str2double(get(handles.editJ,'string'));
phantu(ID,1:2) = [pt1 pt2]
hold on
a = toado(pt1,1);
b = toado(pt2,1);
c = toado(pt1,2);
d = toado(pt2,2);
plot([a b],[c d], 'linewidth',2)
text((a+b)/2+0.07,(c+d)/2+0.07,num2str(ID),'FontSize',12,'Color',[0.8 0 0.8])
% chuyen tu dang so sang dang chuoi

% --- Executes on button press in pushOkModel.
function pushOkModel_Callback(hObject, eventdata, handles)
% hObject    handle to pushOkModel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%% Conect to plot()
set(handles.uipanel4,'visible','off');
% --- Executes on button press in pushResult.
function pushResult_Callback(hObject, eventdata, handles)
% hObject    handle to pushResult (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.uipanelResult,'visible','on');
% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tyle = str2double(get(handles.tyle,'string'));
global toado phantu Utt type
set(handles.axes1,'visible','off');
set(handles.axes2,'visible','off');
set(handles.axes3,'visible','on');
axes(handles.axes3);
if type ==1
    btd_nut = 2;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
%     tyle = 10000;

    sobuoc = 100;
    for i = 1:sobuoc
        hold off
        for j = 1:sophantu
            nutdau = phantu(j,1);
            nutcuoi = phantu(j,2);
            xi = toado(nutdau,1);
            xj = toado(nutcuoi,1);
            yi = toado(nutdau,2);
            yj = toado(nutcuoi,2);
            mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
            Upt = tinh_Upt(Utt,mt_bool_pt);
            xi_sbd = xi + tyle*Upt(1)*(i/sobuoc);
            xj_sbd = xj + tyle*Upt(3)*(i/sobuoc);
            yi_sbd = yi + tyle*Upt(2)*(i/sobuoc);
            yj_sbd = yj + tyle*Upt(4)*(i/sobuoc);
            plot([xi xj],[yi yj],'-b','linewidth',4);
            hold on 
            plot([xi_sbd xj_sbd],[yi_sbd yj_sbd],'-r','linewidth',4);
        end
        axis([-2.5 2.5 -2.5 2.5]);
        NN = getframe;
    end
elseif type==2
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
%     tyle = 10000;
    sobuoc = 100;
    for i = 1:sobuoc
        hold off
        for j = 1:sophantu
            nutdau = phantu(j,1);
            nutcuoi = phantu(j,2);
            xi = toado(nutdau,1);
            xj = toado(nutcuoi,1);
            yi = toado(nutdau,2);
            yj = toado(nutcuoi,2);
            mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
            Upt = tinh_Upt(Utt,mt_bool_pt);
            xi_sbd = xi + tyle*Upt(1)*(i/sobuoc);
            xj_sbd = xj + tyle*Upt(4)*(i/sobuoc);
            yi_sbd = yi + tyle*Upt(2)*(i/sobuoc);
            yj_sbd = yj + tyle*Upt(5)*(i/sobuoc);
            plot([xi xj],[yi yj],'-b','linewidth',4);
            hold on 
            plot([xi_sbd xj_sbd],[yi_sbd yj_sbd],'-r','linewidth',4);
        end
        axis([-2.5 2.5 -2.5 2.5]);
        NN = getframe;
    end
end

% --- Executes on button press in Ok_BC.
function Ok_BC_Callback(hObject, eventdata, handles)
% hObject    handle to Ok_BC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toado phantu rangbuoc type
ux = get(handles.checkUX,'value');
uy = get(handles.checkUY,'value');
mz = get(handles.checkROTZ,'value');
node = str2double(get(handles.editBCNode,'string'));
if type == 1
    if ux == 1
        x = toado(node,1);
        y = toado(node,2);
        plot(x-0.08,y,'>','markersize',15,'markerfacecolor','g');
        rangbuoc(1,length(rangbuoc)+1) = node*2-1;
    end
    if uy == 1
        x = toado(node,1);
        y = toado(node,2);
        plot(x,y-0.08,'^','markersize',15,'markerfacecolor','g');
        rangbuoc(1,length(rangbuoc)+1) = node*2;
    end
elseif type ==2
    if ux == 1
        x = toado(node,1);
        y = toado(node,2);
        plot(x-0.08,y,'>','markersize',15,'markerfacecolor','g');
        rangbuoc(1,length(rangbuoc)+1) = node*3-2;
    end
    if uy == 1
        x = toado(node,1);
        y = toado(node,2);
        plot(x,y-0.08,'^','markersize',15,'markerfacecolor','g');
        rangbuoc(1,length(rangbuoc)+1) = node*3-1;
    end    
    if mz ==1
        x = toado(node,1);
        y = toado(node,2);
        theta = linspace(pi,2*pi + pi/2);
        plot(0.07*cos(theta) + x,0.07*sin(theta) + y,'lineWidth',3,'color', 'b')
        hold on
        plot(0.07*cos(pi/2) + x,0.07*sin(pi/2) + y,'<','color', 'b')

        rangbuoc(1,length(rangbuoc)+1) = node*3;
    end
end


% --- Executes on button press in pushBC.
function pushBC_Callback(hObject, eventdata, handles)
% hObject    handle to pushBC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.uipanelBC,'visible','on');
% --- Executes on button press in pushLoad.
function pushLoad_Callback(hObject, eventdata, handles)
% hObject    handle to pushLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.uipanelLoad,'visible','on');
% --- Executes on button press in pushSolve.
function pushSolve_Callback(hObject, eventdata, handles)
% hObject    handle to pushSolve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%%%%TINH TOAN
format shortg
global rangbuoc phantu toado luctieplink luctiepbeam lucphap E A v J Ftt Utt R Npt type Mpt
if type == 1
    btd_nut = 2;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    Ktt = zeros(sonut*btd_nut,sonut*btd_nut);
    for i=1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        xi=toado(nutdau,1);
        xj=toado(nutcuoi,1);
        yi=toado(nutdau,2);
        yj=toado(nutcuoi,2);
        L(i) = sqrt((xj-xi)^2 + (yj-yi)^2);
        C(i) = (xj-xi)/L(i);
        S(i) = (yj-yi)/L(i);
        mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
        Kpt = tinh_Kpt(E,L(i),A,S(i),C(i));
        Ktt = tinh_Ktt(Ktt,Kpt,mt_bool_pt);
        luctiep_tt = quydoi_luctieplink(luctieplink(i),S(i),C(i),L(i)); 
        Ftt_td = luctiep_tt;
        Ftt = chuyen_Ftt_td(Ftt,Ftt_td,mt_bool_pt); 
    end
    Ktt_bd = Ktt; 
    Ftt_bd = Ftt; 
    [Ktt,Ftt] = khu_dkb(Ktt,Ftt,rangbuoc);
    Utt = inv(Ktt)*Ftt;
    R = zeros(size(toado,1)*btd_nut,1);
    R = tinh_phan_luc(Ktt_bd,Ftt_bd,Utt);
    for i = 1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
        Upt = tinh_Upt(Utt,mt_bool_pt);
        Spt = (E*A/L(i))*[-C(i) -S(i) C(i) S(i)];
        Npt(i,1) = Spt*Upt;
    end
    
    US = (1/A)*Npt;
    
    cot1 = {'NODE','CHUYEN VI'};
    node1 = transpose((1:size(toado,1)*2));
    dulieu1 = [node1 Utt];
    set(handles.chuyenvi,'ColumnName',cot1,'data',dulieu1);
    cot4 = {'NODE','PHAN LUC'};
    dulieu4 = [node1 R];
    set(handles.phanluc,'ColumnName',cot4,'data',dulieu4);
    cot2 = {'PHANTU', 'NOI LUC'};
    phantu1 = transpose((1:size(phantu,1)));
    dulieu2 = [phantu1 Npt];
    set(handles.noiluc,'ColumnName',cot2,'data',dulieu2);
    cot3 = {'PHANTU', 'UNG SUAT'};
    dulieu3 = [phantu1 US];
    set(handles.ungsuat,'ColumnName',cot3,'data',dulieu3);
    
elseif type==2
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    Ktt = zeros(sonut*btd_nut,sonut*btd_nut);
    for i=1:sophantu 
        nutdau = phantu(i,1);  
        nutcuoi = phantu(i,2); 
        xi=toado(nutdau,1);
        xj=toado(nutcuoi,1);
        yi=toado(nutdau,2);
        yj=toado(nutcuoi,2);
        L(i) = sqrt((xj-xi)^2 + (yj-yi)^2);
        C(i) = (xj-xi)/L(i);
        S(i) = (yj-yi)/L(i);    
        mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
        Kpt = tinh_Kptbeam(E,A,J,L(i),S(i),C(i)); 
        Ktt = tinh_Kttbeam(Ktt,Kpt,mt_bool_pt); 
        lucphap_tt = quydoi_lucphap(lucphap(i),S(i),C(i),L(i));
        luctiep_tt = quydoi_luctiepbeam(luctiepbeam(i),S(i),C(i),L(i)); 
        Ftt_td = lucphap_tt + luctiep_tt;
        Ftt = chuyen_Ftt_td(Ftt,Ftt_td,mt_bool_pt); 
    end 
    Ktt_bd = Ktt; %Luu lai ma tran do cung tong the ban dau 
    Ftt_bd = Ftt; %Luu lai ma tran luc tong the ban dau 
    [Ktt,Ftt] = khu_dkb(Ktt,Ftt,rangbuoc);%khu dieu kien bien
    Utt=inv(Ktt)*Ftt
    R = zeros(size(toado,1)*btd_nut,1);
    R = tinh_phan_luc(Ktt_bd,Ftt_bd,Utt)
    for i = 1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
        Upt = tinh_Upt(Utt,mt_bool_pt);
        Spt = E*J/L(i)^3*[6*L(i)*S(i) -6*L(i)*C(i) -4*L(i)^2 -6*L(i)*S(i) 6*L(i)*C(i) -2*L(i)^2;-6*L(i)*S(i) 6*L(i)*C(i) 2*L(i)^2 6*L(i)*S(i) -6*L(i)*C(i) 4*L(i)^2];
        Mpt(i,1:2) = Spt*Upt;%tinh noi luc tung phan tu luu lai trong cung 1 vector
    end
    
    cot1 = {'NODE','CHUYEN VI'};
    node1 = transpose((1:size(toado,1)*3));
    dulieu1 = [node1 Utt];
    set(handles.chuyenvi,'ColumnName',cot1,'data',dulieu1);
    cot4 = {'NODE','PHAN LUC'};
    dulieu4 = [node1 R];
    set(handles.phanluc,'ColumnName',cot4,'data',dulieu4);
    cot2 = {'PHANTU', 'NOI LUC 1', 'NOI LUC 2'};
    phantu1 = transpose((1:size(phantu,1)));
    dulieu2 = [phantu1 Mpt];
    set(handles.noiluc,'ColumnName',cot2,'data',dulieu2);

end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global type
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1

switch get(hObject,'Value')
     case 1
         type=0
         set(handles.pushbutton19,'visible','on');
         set(handles.pushbutton20,'visible','on');
     case 2
         type=1
         set(handles.pushbutton19,'visible','on');
         set(handles.pushbutton20,'visible','off');
     case 3
         type=2
         set(handles.pushbutton19,'visible','off');
         set(handles.pushbutton20,'visible','on');
     case 4
         type=3
         set(handles.pushbutton19,'visible','off');
         set(handles.pushbutton20,'visible','off');
         %+PlateBending/PlateProblem.m  % a package function
         PlateProblem()
 end
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.uipanel4,'visible','on');

% --- Executes on button press in pushMaterial.
function pushMaterial_Callback(hObject, eventdata, handles)
% hObject    handle to pushMaterial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ftt luctieplink luctiepbeam lucphap rangbuoc type phantu toado
    if type ==1
        btd_nut = 2;
        sophantu = size(phantu,1);
        sonut_pt = size(phantu,2);
        sonut = size(toado,1);
        Ftt = zeros(sonut*btd_nut,1);
        luctieplink = zeros(1,sophantu);
        rangbuoc = [];
    elseif type==2
        btd_nut = 3;
        sophantu = size(phantu,1);
        sonut_pt = size(phantu,2);
        sonut = size(toado,1);
        Ftt = zeros(sonut*btd_nut,1);
        luctiepbeam = zeros(1,sophantu);
        lucphap = zeros(1,sophantu);
        rangbuoc = [];
    end
set(handles.uipanelMaterial,'visible','on');
% --- Executes on button press in pushApplyM.
function pushApplyM_Callback(hObject, eventdata, handles)
% hObject    handle to pushApplyM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%%%% ----------VE LUC MOMENT
axes(handles.axes1);
global type phantu toado Ftt
if type == 2
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    node = str2double(get(handles.editLMNode,'string'));
    mz = str2double(get(handles.editLM,'string'));
    x = toado(node,1);
    y = toado(node,2);
    if mz > 0
        theta = linspace(pi,2*pi + pi/2);
        plot(0.07*cos(theta) + x,0.07*sin(theta) + y,'lineWidth',3,'color', 'red')
        hold on
        plot(0.07*cos(pi/2) + x,0.07*sin(pi/2) + y,'<','color', 'red')
    elseif mz < 0
        theta = linspace(pi/2,2*pi);
        plot(0.07*cos(theta) + x,0.07*sin(theta) + y,'lineWidth',3,'color', 'red')
        hold on
        plot(0.07*cos(pi/2) + x,0.07*sin(pi/2) + y,'>','color', 'red')
    end
    Ftt(node*3,1) = mz;
end


function editLMNode_Callback(hObject, eventdata, handles)
% hObject    handle to editLMNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLMNode as text
%        str2double(get(hObject,'String')) returns contents of editLMNode as a double


% --- Executes during object creation, after setting all properties.
function editLMNode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLMNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editLM_Callback(hObject, eventdata, handles)
% hObject    handle to editLM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLM as text
%        str2double(get(hObject,'String')) returns contents of editLM as a double


% --- Executes during object creation, after setting all properties.
function editLM_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushApplyDP.
function pushApplyDP_Callback(hObject, eventdata, handles)
% hObject    handle to pushApplyDP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%------VE LUC PHAN BO
axes(handles.axes1);
global type toado phantu lucphap
if type == 2
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    element = str2double(get(handles.editLDPElem,'string'));
    q0 = str2double(get(handles.editLDP,'string'));
    
    a = toado(phantu(element,1),1);%1x
    b = toado(phantu(element,1),2);%1y
    c = toado(phantu(element,2),1);%2x
    d = toado(phantu(element,2),2);%2y
    
    drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, 'LineWidth',2,'MaxHeadSize',5, 'color','red'); 
    tdnx = [];
    tdny = [];
    if q0 > 0
        for i = 0:6
            tdnx = [tdnx, (- sin(pi/2))*(b + (i+1)*(d-b)/6 - (b + i*(d-b)/6)) + a + i*(c-a)/6];
            tdny = [tdny, (sin(pi/2))*(a + (i+1)*(c-a)/6 - (a + i*(c-a)/6)) + b + i*(d-b)/6];
            drawArrow([a + i*(c-a)/6 ,(- sin(pi/2))*(b + (i+1)*(d-b)/6 - (b + i*(d-b)/6)) + a + i*(c-a)/6],...
                      [b + i*(d-b)/6 ,(sin(pi/2))*(a + (i+1)*(c-a)/6 - (a + i*(c-a)/6)) + b + i*(d-b)/6]);
            
            hold on
        end
        
        for i = 1: 6
            plot([tdnx(i), tdnx(i+1)], [tdny(i), tdny(i+1)], '-', 'color', 'red');
        end
    elseif q0 < 0
        for i = 0:6
            tdnx = [tdnx, (- sin(pi/2))*(b + (i+1)*(d-b)/6 - (b + i*(d-b)/6)) + a + i*(c-a)/6];
            tdny = [tdny, (sin(pi/2))*(a + (i+1)*(c-a)/6 - (a + i*(c-a)/6)) + b + i*(d-b)/6];
            drawArrow([(- sin(pi/2))*(b + (i+1)*(d-b)/6 - (b + i*(d-b)/6)) + a + i*(c-a)/6, a + i*(c-a)/6 ],...
                      [(sin(pi/2))*(a + (i+1)*(c-a)/6 - (a + i*(c-a)/6)) + b + i*(d-b)/6, b + i*(d-b)/6 ]);
            hold on
        end
        
        for i = 1: 6
            plot([tdnx(i), tdnx(i+1)], [tdny(i), tdny(i+1)], '-', 'color', 'red');
        end
    end
    
    
    lucphap(1,element) = q0;
end

function editLDPElem_Callback(hObject, eventdata, handles)
% hObject    handle to editLDPElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLDPElem as text
%        str2double(get(hObject,'String')) returns contents of editLDPElem as a double


% --- Executes during object creation, after setting all properties.
function editLDPElem_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLDPElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editLDP_Callback(hObject, eventdata, handles)
% hObject    handle to editLDP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLDP as text
%        str2double(get(hObject,'String')) returns contents of editLDP as a double


% --- Executes during object creation, after setting all properties.
function editLDP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLDP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushApplyAF.
function pushApplyAF_Callback(hObject, eventdata, handles)
% hObject    handle to pushApplyAF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%%%%%---VE LUC DOC TRUC
axes(handles.axes1);
global toado rangbuoc phantu luctieplink luctiepbeam type
if type ==1
    btd_nut = 2;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
%     luctiep = zeros(1,sophantu);
  
    element = str2double(get(handles.editLAFElem,'string'));
    p0 = str2double(get(handles.editLAF,'string'));
    a = toado(phantu(element,1),1);%1x
    b = toado(phantu(element,1),2);%1y
    c = toado(phantu(element,2),1);%2x
    d = toado(phantu(element,2),2);%2y
    
    drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, 'LineWidth',2,'MaxHeadSize',5, 'color','red'); 
    if p0 > 0
        for i = 0:5
            drawArrow([a + i*(c-a)/6 ,a + (i+1)*(c-a)/6], [b + i*(d-b)/6 ,b + (i+1)*(d-b)/6]);
            hold on
        end
    elseif p0 < 0
        for i = 0:5
            drawArrow([c - i*(c-a)/6 ,c - (i+1)*(c-a)/6], [d - i*(d-b)/6 ,d - (i+1)*(d-b)/6]);
            hold on
        end
    end
    
    luctieplink(1,element) = p0;
elseif type==2
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
%     luctiep = zeros(1,sophantu);
    element = str2double(get(handles.editLAFElem,'string'));
    p0 = str2double(get(handles.editLAF,'string'));
    a = toado(phantu(element,1),1);%1x
    b = toado(phantu(element,1),2);%1y
    c = toado(phantu(element,2),1);%2x
    d = toado(phantu(element,2),2);%2y
    
    drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, 'LineWidth',2,'MaxHeadSize',5, 'color','red'); 
    if p0 > 0
        for i = 0:5
            drawArrow([a + i*(c-a)/6 ,a + (i+1)*(c-a)/6], [b + i*(d-b)/6 ,b + (i+1)*(d-b)/6]);
            hold on
        end
    elseif p0 < 0
        for i = 0:5
            drawArrow([c - i*(c-a)/6 ,c - (i+1)*(c-a)/6], [d - i*(d-b)/6 ,d - (i+1)*(d-b)/6]);
            hold on
        end
    end
    luctiepbeam(1,element) = p0;
end


function editLAFElem_Callback(hObject, eventdata, handles)
% hObject    handle to editLAFElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLAFElem as text
%        str2double(get(hObject,'String')) returns contents of editLAFElem as a double


% --- Executes during object creation, after setting all properties.
function editLAFElem_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLAFElem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editLAF_Callback(hObject, eventdata, handles)
% hObject    handle to editLAF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLAF as text
%        str2double(get(hObject,'String')) returns contents of editLAF as a double


% --- Executes during object creation, after setting all properties.
function editLAF_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLAF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushApplyF.
function pushApplyF_Callback(hObject, eventdata, handles)
% hObject    handle to pushApplyF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%-----------VE LUC FORCE-----------------
global rangbuoc toado phantu Ftt type
if type == 1
    btd_nut = 2;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
%     Ftt = zeros(sonut*btd_nut,1);
    node = str2double(get(handles.editLForceNode,'string'));
    fx = str2double(get(handles.editLFX,'string'));
    fy = str2double(get(handles.editLFY,'string'));
    Ftt(node*2-1,1) = fx;
    Ftt(node*2,1) = fy;
   if fx>0
         quiver(toado(node,1),toado(node,2),0.4,0,'lineWidth',2,'color',[1 0 0],...
    'autoscalefactor',1);
       %text(toado(node,1),toado(node,2),'\rightarrow','color','r','fontsize',20)
    elseif fx<0
         quiver(toado(node,1),toado(node,2),-0.4,0,'lineWidth',2,'color',[1 0 0],...
    'autoscalefactor',1);
        %text(toado(node,1),toado(node,2),'\leftarrow','color','r','fontsize',20)
    end
    if fy>0
         quiver(toado(node,1),toado(node,2),0,0.4,'lineWidth',2,'color',[1 0 0],...
    'autoscalefactor',1);
       %text(toado(node,1),toado(node,2),'\uparrow','color','r','fontsize',20)
    elseif fy<0
         quiver(toado(node,1),toado(node,2),0,-0.4,'lineWidth',2,'color',[1 0 0],...
    'autoscalefactor',1);
        %text(toado(node,1),toado(node,2),'\downarrow','color','r','fontsize',20)
    end
elseif type == 2
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    Ftt = zeros(sonut*btd_nut,1);
    node = str2double(get(handles.editLForceNode,'string'));
    fx = str2double(get(handles.editLFX,'string'));
    fy = str2double(get(handles.editLFY,'string'));
    Ftt(node*3-2,1) = fx;
    Ftt(node*3-1,1) = fy;
    if fx>0
        quiver(toado(node,1),toado(node,2),0.4,0,'lineWidth',3,'color',[1 0 0],...
    'autoscalefactor',1);
       %text(toado(node,1),toado(node,2),'\rightarrow','color','r','fontsize',20)
    elseif fx<0
        %text(toado(node,1),toado(node,2),'\leftarrow','color','r','fontsize',20)
        quiver(toado(node,1),toado(node,2),-0.4,0,'lineWidth',3,'color',[1 0 0],...
    'autoscalefactor',1);
    end
    if fy>0
       %text(toado(node,1),toado(node,2),'\uparrow','color','r','fontsize',20)
      quiver(toado(node,1),toado(node,2),0,0.4,'lineWidth',3,'color',[1 0 0],...
    'autoscalefactor',1);
    elseif fy<0
        %text(toado(node,1),toado(node,2),'\downarrow','color','r','fontsize',20)
        quiver(toado(node,1),toado(node,2),0,-0.4,'lineWidth',3,'color',[1 0 0],...
    'autoscalefactor',1);
    end
end

function editLForceNode_Callback(hObject, eventdata, handles)
% hObject    handle to editLForceNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLForceNode as text
%        str2double(get(hObject,'String')) returns contents of editLForceNode as a double


% --- Executes during object creation, after setting all properties.
function editLForceNode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLForceNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editLFY_Callback(hObject, eventdata, handles)
% hObject    handle to editLFY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLFY as text
%        str2double(get(hObject,'String')) returns contents of editLFY as a double


% --- Executes during object creation, after setting all properties.
function editLFY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLFY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editLFX_Callback(hObject, eventdata, handles)
% hObject    handle to editLFX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editLFX as text
%        str2double(get(hObject,'String')) returns contents of editLFX as a double


% --- Executes during object creation, after setting all properties.
function editLFX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editLFX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%%%-----VE BIEU DO NOI LUC
global toado phantu Utt luctieplink E A 
set(handles.axes1,'visible','off');
set(handles.axes2,'visible','off');
set(handles.axes3,'visible','on');
axes(handles.axes3);
    btd_nut = 2;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    hold off
    for i = 1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        xi=toado(nutdau,1);
        xj=toado(nutcuoi,1);
        yi=toado(nutdau,2);
        yj=toado(nutcuoi,2);
        L(i) = sqrt((xj-xi)^2 + (yj-yi)^2);
        C(i) = (xj-xi)/L(i);
        S(i) = (yj-yi)/L(i);
        mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
        Upt = tinh_Upt(Utt,mt_bool_pt);
        Spt = (E*A/L(i))*[-C(i) -S(i) C(i) S(i)];
        Npt(i) = Spt*Upt;
    end

    Npt_scale = Npt;
    s = 1;
    while max(abs(Npt_scale))>max(L)/4
        Npt_scale = Npt_scale/1.1;
        s = s/1.1;
    end


    for i = 1:sophantu
        axialforce(i) = luctieplink(1,i)*L(i)/2;
    end
    axialforce_scale = axialforce;

    axialforce_scale = axialforce_scale*s;

    for i = 1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        xi = toado(nutdau,1);
        yi = toado(nutdau,2);
        X = [0 L(i) L(i) 0];
        N = [Npt(i)+axialforce(i) Npt(i)-axialforce(i) Npt(i)-axialforce(i) Npt(i)+axialforce(i)];
        Y = [0 0 Npt_scale(i)-axialforce_scale(i) Npt_scale(i)+axialforce_scale(i)];
        Xalpha = X*C(i)-Y*S(i);
        Yalpha = X*S(i)+Y*C(i);
        XTT = Xalpha + xi;
        YTT = Yalpha + yi;
        fill(XTT,YTT,N)
        colorbar
        hold on
    end


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toado phantu lucphap Utt E J A
set(handles.axes1,'visible','off');
set(handles.axes3,'visible','off');
set(handles.axes2,'visible','on');
axes(handles.axes2);
    btd_nut = 3;
    sophantu = size(phantu,1);
    sonut_pt = size(phantu,2);
    sonut = size(toado,1);
    hold off
    for i = 1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        xi=toado(nutdau,1);
        xj=toado(nutcuoi,1);
        yi=toado(nutdau,2);
        yj=toado(nutcuoi,2);
        L(i) = sqrt((xj-xi)^2 + (yj-yi)^2);
        C(i) = (xj-xi)/L(i);
        S(i) = (yj-yi)/L(i);
        mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
        Upt = tinh_Upt(Utt,mt_bool_pt);
        Spt = E*J/L(i)^3*[6*L(i)*S(i) -6*L(i)*C(i) -4*L(i)^2 -6*L(i)*S(i) 6*L(i)*C(i) -2*L(i)^2;-6*L(i)*S(i) 6*L(i)*C(i) 2*L(i)^2 6*L(i)*S(i) -6*L(i)*C(i) 4*L(i)^2];
        Mpt(1:2,i) = Spt*Upt;
    end

    Mpt_scale = Mpt;
    s = 10;
    while max(abs(Mpt_scale))>max(L)/8
        Mpt_scale = Mpt_scale/1.1;
        s = s/1.1;
    end

    for i = 1:sophantu
        x = linspace(0,L(i),100);
        for j = 1:length(x)
            moment(i,j) = lucphap(i)*L(i)^2/12-lucphap(i)*L(i)*x(j)/2+lucphap(i)*x(j)^2/2;
        end
    end
    moment_scale = moment;

    moment_scale = moment_scale*s;

    for i = 1:sophantu
        nutdau = phantu(i,1);
        nutcuoi = phantu(i,2);
        xi = toado(nutdau,1);
        yi = toado(nutdau,2);
        x = linspace(0,L(i),100);
        x1 = [x fliplr(x)];
        y1 = (Mpt(1,i) - Mpt(2,i))/(0-L(i))*x + Mpt(1,i);
        y2 = (Mpt_scale(1,i) - Mpt_scale(2,i))/(0-L(i))*x + Mpt_scale(1,i);
        y3 = y1+moment(i,1:length(x));
        y4 = y2+moment_scale(i,1:length(x));
        YM = [zeros(1,100) fliplr(y4)];
        MM = [y3 fliplr(y3)];
        Xalpha = x1*C(i)-YM*S(i);
        Yalpha = x1*S(i)+YM*C(i);
        XTT = Xalpha + xi;
        YTT = Yalpha + yi;
        fill(XTT,YTT,MM)
        colorbar
        hold on
        axis equal
        grid on
    end

% --- Executes on button press in checkROTX.
function checkROTX_Callback(hObject, eventdata, handles)
% hObject    handle to checkROTX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkROTX


% --- Executes on button press in checkROTY.
function checkROTY_Callback(hObject, eventdata, handles)
% hObject    handle to checkROTY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkROTY


% --- Executes on button press in checkROTZ.
function checkROTZ_Callback(hObject, eventdata, handles)
% hObject    handle to checkROTZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkROTZ


% --- Executes on button press in checkUX.
function checkUX_Callback(hObject, eventdata, handles)
% hObject    handle to checkUX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkUX


% --- Executes on button press in checkUY.
function checkUY_Callback(hObject, eventdata, handles)
% hObject    handle to checkUY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkUY


% --- Executes on button press in checkUZ.
function checkUZ_Callback(hObject, eventdata, handles)
% hObject    handle to checkUZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkUZ



function editBCNode_Callback(hObject, eventdata, handles)
% hObject    handle to editBCNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editBCNode as text
%        str2double(get(hObject,'String')) returns contents of editBCNode as a double


% --- Executes during object creation, after setting all properties.
function editBCNode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editBCNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editUtt_Callback(hObject, eventdata, handles)
% hObject    handle to editUtt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editUtt as text
%        str2double(get(hObject,'String')) returns contents of editUtt as a double


% --- Executes during object creation, after setting all properties.
function editUtt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editUtt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editReac_Callback(hObject, eventdata, handles)
% hObject    handle to editReac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editReac as text
%        str2double(get(hObject,'String')) returns contents of editReac as a double


% --- Executes during object creation, after setting all properties.
function editReac_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editReac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editNpt_Callback(hObject, eventdata, handles)
% hObject    handle to editNpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editNpt as text
%        str2double(get(hObject,'String')) returns contents of editNpt as a double


% --- Executes during object creation, after setting all properties.
function editNpt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editNpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushCloseResult.
function pushCloseResult_Callback(hObject, eventdata, handles)
% hObject    handle to pushCloseResult (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.uipanelResult,'visible','off');



function editMpt_Callback(hObject, eventdata, handles)
% hObject    handle to editMpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editMpt as text
%        str2double(get(hObject,'String')) returns contents of editMpt as a double


% --- Executes during object creation, after setting all properties.
function editMpt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editMpt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%-----------------------------
%------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Function--------------------------------
function [Upt] = tinh_Upt(Utt,chiso_bool)
Upt = Utt(chiso_bool);
function [phan_luc] = tinh_phan_luc(Ktt_bd,Ftt_bd,Utt)
phan_luc = Ktt_bd*Utt - Ftt_bd;
function [N] = tinh_noiluc(phantu,toado,bactudo,btd_nut,E,F,q)
for i = 1:length(phantu)
    qe = zeros(size(phantu,2)*btd_nut,1);
    l = sqrt((toado(phantu(i,1),1)-toado(phantu(i,2),1))^2+(toado(phantu(i,1),2)-toado(phantu(i,2),2))^2);
    c = (toado(phantu(i,2),1)-toado(phantu(i,1),1))/l;
    s = (toado(phantu(i,2),2)-toado(phantu(i,1),2))/l;
    for j = 1:size(phantu,2)*btd_nut
        qe(j,1) =  q(bactudo(i,j),1);
    end
    N(1,i) = E*F/l*[-c -s c s]*qe;
end
    
for i =1:sophantu
    nutdau = phantu(i,1);
    nutcuoi = phantu(i,2);
    xi=toado(nutdau,1);
    xj=toado(nutcuoi,1);
    yi=toado(nutdau,2);
    yj=toado(nutcuoi,2);
    L(i) = sqrt((xj-xi)^2 + (yj-yi)^2);
    C(i) = (xj-xi)/L(i);
    S(i) = (yj-yi)/L(i);
    mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
    N(mt_bool_pt) = E*F/l*[-C -S C S]*Utt(mt_bool_pt)
end
function [Ktt] = tinh_Kttbeam(Ktt,Kpt,mt_bool_pt)
Ktt(mt_bool_pt,mt_bool_pt) = Ktt(mt_bool_pt,mt_bool_pt)+Kpt;
function [Ktt] = tinh_Ktt(Ktt,Kpt,mt_bool_pt)
Ktt(mt_bool_pt,mt_bool_pt) = Ktt(mt_bool_pt,mt_bool_pt)+Kpt;
function [Kpt] = tinh_Kptbeam(E,A,J,L,S,C) 
B=12*J/(L^2);
Kpt = (E/L)*[A*C^2+B*S^2    (A-B)*C*S        -B*L*S/2    -(A*C^2+B*S^2)  -(A-B)*C*S      -B*L*S/2;
            (A-B)*C*S        A*S^2+B*C^2      B*L*C/2    -(A-B)*C*S      -(A*S^2+B*C^2)   B*L*C/2; 
            -B*L*S/2         B*L*C/2          4*J         B*L*S/2        -B*L*C/2         2*J; 
            -(A*C^2+B*S^2)   -(A-B)*C*S       B*L*S/2     A*C^2+B*S^2    (A-B)*C*S        B*L*S/2; 
            -(A-B)*C*S       -(A*S^2+B*C^2)   -B*L*C/2    (A-B)*C*S       A*S^2+B*C^2     -B*L*C/2;
            -B*L*S/2         B*L*C/2          2*J         B*L*S/2        -B*L*C/2         4*J];
%TTT
function [Kpt] = tinh_Kpt(E,L,A,S,C)
Kpt = ((E*A)/L)*[C^2 C*S -C^2 -C*S;...
C*S S^2 -C*S -S^2;...
-C^2 -C*S C^2 C*S;...
-C*S -S^2 C*S S^2];
function [Ftt] = tinh_Ftt(Ftt,lucdoctruc,toado,bactudo,phantu,sophantu,btd_nut)
for i=1:sophantu
    nutdau = phantu(i,1);
    nutcuoi = phantu(i,2);
    xi=toado(nutdau,1);
    xj=toado(nutcuoi,1);
    yi=toado(nutdau,2);
    yj=toado(nutcuoi,2);
    L(i) = sqrt((xj-xi)^2 + (yj-yi)^2);
    C(i) = (xj-xi)/L(i);
    S(i) = (yj-yi)/L(i);
    mt_bool_pt = tinh_bool([nutdau nutcuoi],btd_nut);
    if phantu(i,1) == lucdoctruc(i,1)|| phantu(i,2) == lucdoctruc(i,2)   
        Ftt(mt_bool_pt) = Ftt(mt_bool_pt) + lucdoctruc(i,3)*L(i)/2*[C(i); S(i); C(i); S(i)];
    elseif phantu(i,1) == lucdoctruc(i,2) || phantu(i,2) == lucdoctruc(i,1) 
        Ftt(mt_bool_pt) = Ftt(mt_bool_pt) - lucdoctruc(i,3)*L(i)/2*[C(i); S(i); C(i); S(i)];
    end
end
function [mt_bool_pt] = tinh_bool(mt_nut_pt,btd_nut)
k = 0;
sonut_pt = size(mt_nut_pt,2);
for i = 1:sonut_pt
    s = (mt_nut_pt(i) - 1)*btd_nut;
    for j = 1:btd_nut
        k=k+1;
        mt_bool_pt(k) = s + j;
    end
end
function luctiep_tt = quydoi_luctieplink(luctiep,S,C,L) 
luctiep_tt(1,1) = (L/2)*(luctiep*C);
luctiep_tt(2,1) = (L/2)*(luctiep*S); 
luctiep_tt(3,1) = (L/2)*(luctiep*C);
luctiep_tt(4,1) = (L/2)*(luctiep*S); 
function luctiep_tt = quydoi_luctiepbeam(luctiep,S,C,L) 
luctiep_tt(1,1) = (L/2)*(luctiep*C);
luctiep_tt(2,1) = (L/2)*(luctiep*S); 
luctiep_tt(3,1) = (L/2)*0;
luctiep_tt(4,1) = (L/2)*(luctiep*C);
luctiep_tt(5,1) = (L/2)*(luctiep*S); 
luctiep_tt(6,1) = (L/2)*0;
function lucphap_tt = quydoi_lucphap(lucphap,S,C,L) 
lucphap_tt(1,1) = (L/2)* (-lucphap*S); 
lucphap_tt(2,1) = (L/2)*  (lucphap*C); 
lucphap_tt(3,1) = (L/2)* (lucphap*L/6); 
lucphap_tt(4,1) = (L/2)* (-lucphap*S); 
lucphap_tt(5,1) = (L/2)* (lucphap*C); 
lucphap_tt(6,1) = (L/2)* (-lucphap*L/6);
function [Ktt,Ftt] = khu_dkb(Ktt,Ftt,rangbuoc)
n = length(rangbuoc);
btdtt = length(Ktt);
for i = 1:n
    c = rangbuoc(i);
    for j=1:btdtt
        Ktt(c,j)=0;%khu hang
        Ktt(j,c)=0;%khu cot
    end
    Ktt(c,c) = 1;
    Ftt(c) = 0;
end
function Ftt = chuyen_Ftt_td(Ftt,Ftt_td,mt_bool_pt)
Ftt(mt_bool_pt,1) = Ftt(mt_bool_pt,1) + Ftt_td;

%%%%----------------------
%%%%----------------------
%%%%----------------------


% --- Executes on button press in pushRefesh.
function pushRefesh_Callback(hObject, eventdata, handles)
% hObject    handle to pushRefesh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
clear all;
cla
% --- Executes on selection change in listboxUtt.
function listboxUtt_Callback(hObject, eventdata, handles)
% hObject    handle to listboxUtt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listboxUtt contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listboxUtt


% --- Executes during object creation, after setting all properties.
function listboxUtt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listboxUtt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushCloseBC.
function pushCloseBC_Callback(hObject, eventdata, handles)
% hObject    handle to pushCloseBC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.uipanelBC,'visible','off');



function tyle_Callback(hObject, eventdata, handles)
% hObject    handle to tyle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tyle as text
%        str2double(get(hObject,'String')) returns contents of tyle as a double


% --- Executes during object creation, after setting all properties.
function tyle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tyle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
