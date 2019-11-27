%%%Inverse Kinematics:
clear all; close all; clc

%The length of the links, these are constant, so they do not change during
%the simulation.
L_1 = 218;
L_2 = 273;

%Here we put the desired Angular Position (some angles are impossible for the robot
%due to the construction of the CrustCrawler):
a = 30; %q1 in kinematic simulation
b = -60; %q3 in kinematic simulation
c = 40; %q4 in kinematic simulation
%Here we set the orientation for the three angles (Roll, Pitch, Yaw):
th1=(deg2rad(a));
th2=(deg2rad(b));
th3=(deg2rad(c));

%Here are the links of the robot:
L0 = Link('alpha', 0, 'a', 0, 'd', 0, 'modified'); %Extra link in order to simulate
%the distance from the holder to the base 170cm
LB = Link('alpha',  0, 'a', 0, 'd', 243, 'offset', ((deg2rad(0))+th1), 'modified'); %rotational
L1 = Link('alpha',  (deg2rad(-90)), 'a', 0, 'd', 0, 'offset', ((deg2rad(0)+th2)), 'modified'); %rotational
L2 = Link('alpha',  0, 'a', 218, 'd', 0, 'offset', ((deg2rad(0)+th3)), 'modified'); %rotational
L3 = Link('alpha',  (deg2rad(90)), 'a', 273, 'd', 0, 'modified'); %rotational
%These offset are added to each link as we want the orientation of these, 
%atthe desired position.

%Now we find the Transformation matrices we need for the the desired
%transformation:
T1=transl(0, 0, 243)*trotz(th1);
T2=transl(0, 0, 0)*trotx(deg2rad(-90))*trotz(deg2rad(0)+th2);
T3=transl(218, 0, 0)*trotz(deg2rad(0)+th3);
T4=transl(273, 0, 0)*trotx(deg2rad(90));

%Multiplying the transformation matrices together gives us the desired
%transformation matrix:
TB_W=T1*T2*T3*T4;

%%Kinematic modeling, to see which position the angles gives:
CrustCrawler = SerialLink([L0 LB L1 L2 L3], 'name', 'Crusty');
q = [0 0 0 0 0];
Tf = CrustCrawler.fkine(q); %Position of  two last parallel joints at 
%the home position of the robot
CrustCrawler.teach(q); 
%In the simulation q2 and q5 is to be ignored..

%Before we calculate the thetas, we need the vector position of the end-effector:
Vec_pos=[TB_W(1, 4); TB_W(2, 4); TB_W(3, 4)]; %This is the vector position 
%from the base to the end-effector. 

%The vector we need is the vector position from joint 1 to the
%end-effector:
Vec1_4=Vec_pos-[T1(1,4); T1(2,4); T1(3,4)]; 
%Doing this eliminates the 243mm translation.

%The length of of 3D vector r:
r = sqrt((Vec1_4(3,1))^2+(Vec1_4(2, 1))^2+(Vec1_4(1,1))^2);

%%Theta1:
%t1 = 2*atan((-Vec1_4(1)+[-1 1]*sqrt(Vec1_4(1)^2+Vec1_4(2)^2))/Vec1_4(2));
t1 = atan2(Vec1_4(2), Vec1_4(1)); %Utilising atan2 to find the angle in radians.
Theta1 = (rad2deg(t1)); %Converting the radians to degrees to find theta1.

%%Theta2:
f_11 = atan2(Vec1_4(3,1), sqrt((Vec1_4(1,1))^2+(Vec1_4(2,1))^2)); 
f_22 = real(acos((L_1^2+r^2-L_2^2)/(2*r*L_1))); %Since we get an¢ imaginary angle number i.e. 45 + 34i,
%the entire equation is put inside a 'real()' to only get the real part of the complex number as this number is the angle.
phi1 = (rad2deg(f_11)); 
phi2 = (rad2deg(f_22));
Theta2_1 = -(phi1 + phi2); %Elbow up solution when theta3 < 0
Theta2_2 = (phi2 - phi1); %Elbow down solution when theta3 > 0 

%%Theta3:
f_33 = real(acos((L_1^2+L_2^2 - r^2)/(2*L_1*L_2)));
Theta3_1 = 180-(rad2deg(f_33));
Theta3_2 = 180+(rad2deg(f_33)); 




