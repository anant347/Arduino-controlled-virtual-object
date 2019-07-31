clc
clear all
a=arduino()
imu = mpu6050(a)

%Initialization
t1=0;
x=[];
y1=[];
y2=[];
y3=[];
y1(1)=0;y2(1)=0;y3(1)=0;

%Sensor provides angular velocity that is converted to rotation 
b=readAngularVelocity(imu); 

%Angular Velocity reported when sensor is stationary
y1c=b(1); 
y2c=b(2);
y3c=b(3);

%Opens Simulink 3D 
% UAV CAD FROM - "https://grabcad.com/library/uav-uas-concept-1" by Elias Gonzalez
[n,w] = vrimport(which('FilePath\UAV.stl')); 
view(w)

for i=2:100000
    tic
    b=readAngularVelocity(imu);
     % New_theta = Old_theta + Delta_Time x (angular_velocity - Stationary_angular_velocity)
    y1(i)=y1(i-1)+t1*(b(1)-y1c);
    y2(i)=y2(i-1)+t1*(b(2)-y2c);
    y3(i)=y3(i-1)+t1*(b(3)-y3c);
    x(i)=i;
    plot(x,y1,'r',x,y2,'g',x,y3,'b')
    
    rx=[1,0,0;0,cos(y1(i)),-sin(y1(i));0,sin(y1(i)),cos(y1(i))];
    ry=[cos(y2(i)),0,sin(y2(i));0,1,0;-sin(y2(i)),0,cos(y2(i))];
    rz=[cos(y3(i)),-sin(y3(i)),0;sin(y3(i)),cos(y3(i)),0;0,0,1];
    r=rx*ry*rz;
    
    axang=rotm2axang(r); %Rotation matrix to axis-angle representation 
    w.UAV_Transform.rotation = axang
    pause(0.025) %increase pause duration if object doesn't move
    t1=toc
end
