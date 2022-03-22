clear all
close all
clc

%All angle measurements saved in radians***
%All distances saved in millimeters***

%In reality, ...
%D=420mm;
%d=42mm;
%tQ=input;
%tR=input;
%tS=input;
%tT=input;

%In test model, ...
D=40; %in model
d=5; %in model
tQ=25.3*pi/180;
tR=111.7*pi/180;
tS=44.5*pi/180;
tT=67.7*pi/180;

%syms D d tQ tR tS tT

%X1=0; Y1=0; %coordinates of first sweep;
X2=D*cos(90*pi/180-tQ-tR/2); %coordinates of second sweep
Y2=D*sin(90*pi/180-tQ-tR/2);

tA=180*pi/180-tQ-tR;
    tA*180/pi;
tB=tA+tQ+tR/2; %not needed
    tB*180/pi;
tC=180*pi/180-tS-tB; %not needed
    tC*180/pi;
tD=180*pi/180-tR/2-tS-tT;
    tD*180/pi;
tE=tA+tQ; 
    tE*180/pi;
tF=180*pi/180-tR/2-tS; 
    tF*180/pi;
tG=180*pi/180-tF;
    tG*180/pi;
tH=180*pi/180-tE-tG;
    tH*180/pi;
tI=180*pi/180-tT-tS-tR/2;
    tI*180/pi;
tJ=90*pi/180-tR/2;
    tJ*180/pi;
tK=90*pi/180-tJ/2; %not needed
    tK*180/pi;
    
L1=sin(tS)/(sin(tH)/D);
L2=sin(tA+tQ+tR/2)/(sin(tH)/D); %not needed
L3=sin(tR/2)/(sin(tI)/D);
L4=sin(tS+tT)/(sin(tI)/D);
L5=sqrt(L2^2+L3^2-2*L2*L3*cos(tT)); %not needed
L6=sin(tR/2)/(sin(tJ)/d); %not needed
L7=sin(tJ)/(sin(90*pi/180)/L6); %not needed
L8=sin(tR/2)/(sin(tF)/D); %correct
L9=sin(tS)/(sin(tF)/D);   %correct
L10=sin(tT)/(sin(tD)/L8); %correct   
L11=sin(tE)/(sin(tH)/L9); %correct
LL=d/sin(90*pi/180-tH/2);
Lm=d*sin(tH/2)/sin(90*pi/180-tH/2);
LR=d/sin(tI/2);

XL=L1*cos(90*pi/180+tA)+Lm*cos(90*pi/180+tA-180*pi/180)+d*cos(tA-180*pi/180); %correct
YL=L1*sin(90*pi/180+tA)+Lm*sin(90*pi/180+tA-180*pi/180)+d*sin(tA-180*pi/180); %correct

XR=(L9+L10)*cos(90*pi/180-tQ)+LR*cos(90*pi/180-tQ-180*pi/180+tD/2); %correct
YR=(L9+L10)*sin(90*pi/180-tQ)+LR*sin(90*pi/180-tQ-180*pi/180+tD/2); %correct

m=(YR-YL)/(XR-XL);
b=YL-m*XL;
M=-1/m;

syms Xj
Xx=solve(b+m*Xj==M*Xj,Xj);
Yx=m*Xx+b;
A=atan2(Yx,Xx);
    A*180/pi;
W=sqrt(Xx^2+Yx^2)-d;
Xf=W*cos(A);
Yf=W*sin(A);

%The below variables hold instructions for how the robot should move to 
%align the laser correctly.

dist_to_go=sqrt((Yf-Y2)^2+(Xf-X2)^2); %too long to simplify to a single eqn.
angle_to_go=atan2((Yf-Y2),(Xf-X2)); %too long to simplify to a single eqn.
angle_to_adjust=A-angle_to_go; %too long to simplify to a single eqn.
