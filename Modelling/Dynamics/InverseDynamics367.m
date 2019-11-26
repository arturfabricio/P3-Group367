%% Syms/Thetas %% -------------------------------%
%%///////////////////////////////////////////////%
%%-----------------------------------------------%
close all; clc

syms theta real;

syms theta1 real; %Angular position/ Thetas
syms theta2 real;
syms theta3 real;

syms dtheta1 real; %Velocity, first derivative of theta
syms dtheta2 real;
syms dtheta3 real;

syms ddtheta1 real; %Acceleration, second derivative of theta
syms ddtheta2 real;
syms ddtheta3 real;

syms G real; %Gravity

%% Fixed values -------------------------------%
%//////////////////////////////////////////////%
%----------------------------------------------%

% Rotation Matrices ---------------------------%

rotx(theta) = [1 0          0;
               0 cos(theta) -sin(theta);
               0 sin(theta) cos(theta)];

roty(theta) = [cos(theta)  0 sin(theta);
               0           1          0;
               -sin(theta) 0 cos(theta)];           
           
rotz(theta) = [cos(theta) -sin(theta) 0;
               sin(theta) cos(theta)  0;
               0          0           1];

% Inertia Tensors -------------------------------------------------------%

J_1 = [669569.82/1000000000  6077.18/1000000000       647.33/1000000000    ;  %%Intertia Tensor for Body 1
	   6077.18/1000000000	 195265.12/1000000000	  -106618.14/1000000000;
	   647.33/1000000000	 -106618.14/1000000000	  624990.97/1000000000];

J_2 = [692722.40/1000000000  -15239.66/1000000000   -9195.90/1000000000  ;    %%Intertia Tensor for Body 2
       -15239.66/1000000000  273781.56/1000000000   309064.46/1000000000 ; 
       -9195.90/1000000000   309064.46/1000000000   500958.90/1000000000];    

J_3 = [589773.91/1000000000  3283.28/1000000000    -21231.24/1000000000 ;      %%Intertia Tensor for Body 3
       3283.28/1000000000    674810.11/1000000000  -60899.68/1000000000 ;
       -21231.24/1000000000  -60899.68/1000000000  167509.76/1000000000];
   
g = [0; 0; G]; %Gravity vector
z = [0; 0; 1]; %z unit vector
Q_dotdot = [ddtheta1; ddtheta2; ddtheta3]; %Acceleration vecotr

% Masses of links ----------------------------------------%

M1 = 0.42516;
M2 = 0.20997;
M3 = 0.28144;

% Position Vector for the Links ----------------------%

L1 = [0; 0; 0.243];  %Link 1
L2 = [0.2144; 0; 0]; %Link 2
L3 = [0.2554; 0; 0]; %Link 3

% Position Vectors for the CoM -----------------------%
L1_COM = [0; 0; 0.243];
L2_COM = [0.16780; 0; 0];
L3_COM = [0.16354; 0; 0];

%% Position vectors relative to the local frame -------------------------------------------%
%%/////////////////////////////////////////////////////////////////////////////////////////%
%%/////////////////////////////////////////////////////////////////////////////////////////%
%%-----------------------------------------------------------------------------------------%

% Notation --------------------------------------------------------------%
% s_i = local frame end effector position of link i                      %
% sc_i = local frame center of mass position of link i                   %
% Ri = rotation matrix of link i to move it into the coordinate system   %
% of link 1                                                              %
% r_i = end effector position of link i in the global frame              %
% rc_i = center of mass position of link i in the global frame           %
%------------------------------------------------------------------------%

%Link 1
s_1 = L1
sc_1 = s_1/2

%Link 2
R2 = rotz(theta1)*rotx(deg2rad(90))*rotz(theta2)
s_2 = R2* L2           
sc_2 = R2*(L2_COM) 

%Link 3
R3 = rotz(theta1)*rotx(deg2rad(90))*rotz(theta2+theta3)
s_3 = R3*L3 
sc_3 = R3*(L3_COM) 

% The "vpa" function: vpa(x) uses variable-precision floating-point arithmetic (VPA) 
% to evaluate each element of the symbolic input x to at least d significant digits, 
% where d is the value of the digits function. The default value of digits
% is 32. It bascially rounds the numbers to a certain number of decimals.

%Global Position Vectors
r_1 = vpa(s_1,2)            %End-effector position vector of Link 1
r_2 = vpa(r_1+s_2,2)        %End-effector position vector of Link 2
r_3 = vpa(r_2+s_3,2)        %End-effector position vector of Link 3
rc_1 = vpa(sc_1,2)          %COM position vector of Link 1
rc_2 = vpa(r_1+sc_2,2)      %COM position vector of Link 2
rc_3 = vpa(r_2+sc_3,2)      %COM position vector of Link 3

%r=vpa(simplify([rc_1;rc_2;rc_3]),2) 

%% Velocities of the Bodies ----------------------------------------------%
%%////////////////////////////////////////////////////////////////////////%
%%////////////////////////////////////////////////////////////////////////%
%%------------------------------------------------------------------------%

%Angular Velocities (omegas)
omega_1 = [0; 0; dtheta1];
omega_2 = dtheta2*[R2(1,3); R2(2,3); R2(3,3)]+omega_1;
omega_3 = dtheta3*[R3(1,3); R3(2,3); R3(3,3)]+omega_2;

%Velocities of EE
v_1 = vpa(cross(omega_1,s_1),2);
v_2 = vpa(v_1 + cross(omega_2,s_2),2);
v_3 = vpa(v_2 + cross(omega_3,s_3),2);

%Velocities of COM
vc_1 = vpa(cross(omega_1,sc_1),2);
vc_2 = vpa(v_1 + cross(omega_2,sc_2),2);
vc_3 = vpa(v_2 + cross(omega_3,sc_3),2);

%% Dynamics --------------------------------------------%
%%------------------------------------------------------%
%%------------------------------------------------------%

%% Lagrangian ------------------------------------------%

% Notation -------------------------%
% L - Lagragian Formulation---------%
% T - Kinetic Energy----------------%
% V - Potential Energy--------------%
%-----------------------------------%

% MAIN FORMULA!! ----------------------------%
%                                            %
%                 L = T - V                  %
%                                            %
%--------------------------------------------%

% Link 1
T1 = vpa(1/2*M1*dot(transpose(vc_1),vc_1) + (0.5* dot((J_1*omega_1),transpose(omega_1))),2); %Kinetic energy
V1x = vpa(M1*dot(transpose(g),rc_1),2);                                                       %Potential energy 

% Link 2
J_2_O = vpa(R2*J_2*transpose(R2),2);                                                          %Rotated Inertia Tensor
T2 = vpa((1/2*M2*dot(transpose(vc_2),vc_2))+(0.5*dot((J_2_O*omega_2),transpose(omega_2))),2); %Kinetic energy
V2x = vpa(M2*dot(transpose(g),rc_2),2);                                                        %Potential energy

% Link 3
J_3_O = vpa(R3*J_3*transpose(R3),2);                                                          %Rotated Inertia Tensor
T3 = vpa((1/2*M3*dot(transpose(vc_3),vc_3))+(0.5*dot((J_3_O*omega_3),transpose(omega_3))),2); %Kinetic energy
V3x = vpa(M3*dot(g,transpose(rc_3)),2);                                                        %Potential energy;

% Lagrangian

Lx = vpa(T1 - V1x + T2 - V2x + T3 - V3x,2);

% Differentiation to obtain LaGrange
pD_T1 = vpa(diff(Lx, dtheta1),2);
d_pD_T1 = diff(pD_T1, theta1)*dtheta1+diff(pD_T1,theta2) *dtheta2 + diff(pD_T1,theta3)*dtheta3 + diff(pD_T1,dtheta1)*ddtheta1 + diff(pD_T1,dtheta2)*ddtheta2 + diff(pD_T1,dtheta3)*ddtheta3;
pD_V1 = diff(Lx, theta1);
pD_T2 = diff(Lx, dtheta2);
d_pD_T2 = diff(pD_T2, theta1)*dtheta1+diff(pD_T2,theta2) *dtheta2 + diff(pD_T2,theta3)*dtheta3 + diff(pD_T2,dtheta1)*ddtheta1 + diff(pD_T2,dtheta2)*ddtheta2 + diff(pD_T2,dtheta3)*ddtheta3;
pD_V2 = diff(Lx, theta2);
pD_T3 = diff(Lx, dtheta3);
d_pD_T3 = diff(pD_T3, theta1)*dtheta1+diff(pD_T3,theta2) *dtheta2 + diff(pD_T3,theta3)*dtheta3 + diff(pD_T3,dtheta1)*ddtheta1 + diff(pD_T3,dtheta2)*ddtheta2 + diff(pD_T3,dtheta3)*ddtheta3;
pD_V3 = diff(Lx, theta3);

tau_1 = simplify(d_pD_T1-pD_V1); %General formula for the torque of joint 1
tau_2 = simplify(d_pD_T2-pD_V2); %General formula for the torque of joint 2
tau_3 = simplify(d_pD_T3-pD_V3); %General formula for the torque of joint 3

% Symbolic torque equation derived using lagrangian approach
Tau_syms=vpa([tau_1;tau_2;tau_3],3);

%The entries of the mass matrix ----------------%
M(1,1) = vpa(simplify(diff(tau_1, ddtheta1)),2);%
M(1,2) = vpa(simplify(diff(tau_1, ddtheta2)),2);%
M(1,3) = vpa(simplify(diff(tau_1, ddtheta3)),2);%
M(2,1) = vpa(simplify(diff(tau_2, ddtheta1)),2);%
M(2,2) = vpa(simplify(diff(tau_2, ddtheta2)),2);%
M(2,3) = vpa(simplify(diff(tau_2, ddtheta3)),2);%
M(3,1) = vpa(simplify(diff(tau_3, ddtheta1)),2);%
M(3,2) = vpa(simplify(diff(tau_3, ddtheta2)),2);%
M(3,3) = vpa(simplify(diff(tau_3, ddtheta3)),2);%
%-----------------------------------------------%

%Defining the mass matrix
MassMatrix = vpa(M,2);

ang1 = 0;
ang2 = 0;
ang3 = 0;

vel1 = 0;
vel2 = 0;
vel3 = 0;

acc1 = 0;
acc2 = 0;
acc3 = 0;

%Finding the viscous friction and coulumb vector

tv1 = subs(tau_1,[dtheta1,dtheta2,dtheta3],[vel1,vel2,vel3]);
tv2 = subs(tau_2,[dtheta1,dtheta2,dtheta3],[vel1,vel2,vel3]);
tv3 = subs(tau_3,[dtheta1,dtheta2,dtheta3],[vel1,vel2,vel3]);

Y(1,1) = tau_1 - tv1;
Y(2,1) = tau_2 - tv2;
Y(3,1) = tau_3 - tv3;

%Viscous friction and coulumb vector
ViscousFrictionAndCoulumbVector = vpa(Y,2);

%Finding the gravity vector
GravVec(1,1) = vpa(simplify(diff(tau_1,G)),2)*G;
GravVec(2,1) = vpa(simplify(diff(tau_2,G)),2)*G;
GravVec(3,1) = vpa(simplify(diff(tau_3,G)),2)*G;

%Gravity Vector
GravityVector = vpa(GravVec,2);

%State space equation or the final general equation
Tau = M*Q_dotdot + Y + GravVec;

%Print state space equation
Torque = vpa(Tau,2);

%% Testing values ----------------------%

     %Angular positions
    ang1 = 0;
    ang2 = 0;
    ang3 = 0;

    %Angular velocities
    vel1 = 0;
    vel2 = 0;
    vel3 = 0;

    %Angular acceleration
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;

    %Gravitational constants
    gConst = [0; 0; 9.81];
    gConst = 9.81;

    Tau1 = subs(Tau(1,1),[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,gConst]);
    Tau2 = subs(Tau(2,1),[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,gConst]);
    Tau3 = subs(Tau(3,1),[theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,ddtheta1,ddtheta2,ddtheta3,G],[ang1,ang2,ang3,vel1,vel2,vel3,acc1,acc2,acc3,gConst]);
    ComputedTorque = [Tau1; Tau2; Tau3];

    vpa(ComputedTorque,3)

 %}
 
%% Simulation --------------------------------------------% 

syms t
g = 9.801;

T = 5; % second
N = 620; % resolution
i = 0; 
for t = linspace(0, T, N)
    
    i = i + 1; time(i) = t;
    
%   A = table2array(Var(:,1));     
%   B = table2array(Var2(:,4));
%   C = table2array(current2(:,1));
%     
%   time2 = 0:0.0083:600*0.0083;
%     
    %%%%%%%%%%%%%%% Joint 1: Angular Displacement, Velocity, Acceleration
    theta__1(i) = 0;                    %A1*sin(f1*t); 
    theta__dot1(i) = 0;                 %A1*f1*cos(f1*t); 
    theta__ddot1(i) = 0;                %-A1*f1^2*sin(f1*t); 
    
    %%%%%%%%%%%%%%% Joint 2: Angular Displacement, Velocity, Acceleration
    theta__2(i) = (3/50)*pi*t^2-(1/125)*pi*t^3;         % A2*sin(f2*t); 
    theta__dot2(i) = (3/25)*pi*t-(3/125)*pi*t^2;        %A2*f2*cos(f2*t); %plus 45degrees
    theta__ddot2(i) = (3/25)*pi-(6/125)*pi*t;           %-A2*f2^2*sin(f2*t); %plus 45degrees
    
    %%%%%%%%%%%%%%% Joint 3: Angular Displacement, Velocity, Acceleration
    theta__3(i) = 0;                %A3*sin(f3*t); 
    theta__dot3(i) = 0;             %A3*f3*cos(f3*t); %plus 45degrees
    theta__ddot3(i) = 0;            %-A3*f3^2*sin(f3*t); %plus 45degrees
            
    %% Mass Matrices -------------------------------------------------------%        
            
            H11 = 0.0092151149274593274927330935487638*cos(2.0*theta__2(i) + 0.0016537685168669369099548250625196) + 0.0098681239655333298024348197660315*cos(2.0*theta__2(i) + theta__3(i)) + 0.0038061225791984702449640284182902*cos(2.0*theta__2(i) + 2.0*theta__3(i) - 0.00086263128387421025174479894810127) + 0.0098681239655333298024348197660315*cos(theta__3(i)) + 0.014928710769026363351884743793393;
            H12 = 0.00030920123707807765276147966018886*cos(theta__2(i) + 0.029745211506575693050659791057165) - 0.000064494469346179929533052804823259*cos(theta__2(i) + theta__3(i) - 0.3354506542275594480843190898434);
            H13 = -0.000064494469346179929533052804823259*cos(theta__2(i) + theta__3(i) - 0.3354506542275594480843190898434);

            H21 = 0.00030920123707807765276147966018886*cos(theta__2(i) + 0.029745211506575693050659791057165) - 0.000064494469346179929533052804823259*cos(theta__2(i) + theta__3(i) - 0.3354506542275594480843190898434);
            H22 = 0.019736247931066659604869639532063*cos(theta__3(i)) - 4.375e-35*cos(2.0*theta__2(i)) - 5.75e-36*cos(4.0*theta__1(i)) - 6.25e-36*cos(4.0*theta__1(i))*cos(2.0*theta__2(i)) + 0.027044820278148158707496670332719;
            H23 = 0.0098681239655333298024348197660315*cos(theta__3(i)) + 1.25e-36*cos(4.0*theta__1(i))*cos(theta__3(i)) - 1.25e-36*cos(2.0*theta__2(i))*cos(theta__3(i)) + 1.25e-36*sin(2.0*theta__2(i))*sin(theta__3(i)) + 1.25e-36*cos(4.0*theta__1(i))*cos(2.0*theta__2(i))*cos(theta__3(i)) - 1.25e-36*cos(4.0*theta__1(i))*sin(2.0*theta__2(i))*sin(theta__3(i)) + 0.0076947158860647989473352045480322;
            
            H31 = -0.000064494469346179929533052804823259*cos(theta__2(i) + theta__3(i) - 0.3354506542275594480843190898434);
            H32 = 6.25e-37*cos(4.0*theta__1(i) + theta__3(i)) - 1.25e-36*cos(2.0*theta__2(i) + theta__3(i)) + 6.25e-37*cos(2.0*theta__2(i) - 4.0*theta__1(i) + theta__3(i)) + 6.25e-37*cos(4.0*theta__1(i) + 2.0*theta__2(i) + theta__3(i)) + 0.0098681239655333298024348197660315*cos(theta__3(i)) + 6.25e-37*cos(4.0*theta__1(i) - 1.0*theta__3(i)) + 0.0076947158860647989473352045480322;
            H33 = 0.0076947158860647989473352045480322 - 2.5e-36*cos(2.0*theta__2(i) + 2.0*theta__3(i)) - 2.5e-36*cos(2.0*theta__2(i) - 2.0*theta__3(i));

     %% Velocity Vector --------------------------------------------------%
     
            C1 =  0.000060899680001114120386773720383644*theta__dot2(i)^2*sin(theta__2(i) + theta__3(i)) - 0.00002123124000164011704328004270792*theta__dot2(i)^2*cos(theta__2(i) + theta__3(i)) - 0.00002123124000164011704328004270792*theta__dot3(i)^2*cos(theta__2(i) + theta__3(i)) - 0.00030906445999789866618812084197998*theta__dot2(i)^2*sin(theta__2(i)) + 0.000060899680001114120386773720383644*theta__dot3(i)^2*sin(theta__2(i) + theta__3(i)) - 0.0000091959000005914504072279669344425*theta__dot2(i)^2*cos(theta__2(i)) - 0.0098681239655333298024348197660315*theta__dot1(i)*theta__dot3(i)*sin(theta__3(i)) - 0.019736247931066659604869639532063*theta__dot1(i)*theta__dot2(i)*sin(2.0*theta__2(i) + theta__3(i)) - 0.0098681239655333298024348197660315*theta__dot1(i)*theta__dot3(i)*sin(2.0*theta__2(i) + theta__3(i)) + 0.0000065665599997544177313102409243584*theta__dot1(i)*theta__dot2(i)*cos(2*theta__2(i) + 2*theta__3(i)) + 0.0000065665599997544177313102409243584*theta__dot1(i)*theta__dot3(i)*cos(2*theta__2(i) + 2*theta__3(i)) - 0.000030479319999443532651639543473721*theta__dot1(i)*theta__dot2(i)*cos(2.0*theta__2(i)) - 0.0076122423261367232458712369863104*theta__dot1(i)*theta__dot2(i)*sin(2*theta__dot2(i) + 2*theta__dot3(i)) - 0.0076122423261367232458712369863104*theta__dot1(i)*theta__dot3(i)*sin(2*theta__2(i) + 2*theta__3(i)) - 0.018430204652042995637910417100567*theta__dot1(i)*theta__dot2(i)*sin(2.0*theta__2(i)) - 0.00004246248000328023408656008541584*theta__dot2(i)*theta__dot3(i)*cos(theta__2(i) + theta__3(i)) + 0.00012179936000222824077354744076729*theta__dot2(i)*theta__dot3(i)*sin(theta__2(i) + theta__3(i));
            C2 =  0.00001523965999972176632581977173686*theta__dot1(i)^2*cos(2.0*theta__2(i)) - 0.0098681239655333298024348197660315*theta__dot3(i)^2*sin(theta__3(i)) + 0.0092151023260214978189552085502833*theta__dot1(i)^2*sin(2.0*theta__2(i)) + 4.375e-35*theta__dot2(i)^2*sin(2.0*theta__2(i)) - 0.019736247931066659604869639532063*theta__dot2(i)*theta__dot3(i)*sin(theta__3(i)) - 0.0000032832799998772088656551204621792*theta__dot1(i)^2*cos(2.0*theta__2(i))*cos(2.0*theta__3(i)) + 0.0038061211630683616229356184931552*theta__dot1(i)^2*cos(2.0*theta__2(i))*sin(2.0*theta__3(i)) + 0.0038061211630683616229356184931552*theta__dot1(i)^2*cos(2.0*theta__3(i))*sin(2.0*theta__2(i)) + 6.25e-36*theta__dot2(i)^2*cos(4.0*theta__1(i))*sin(2.0*theta__2(i)) + 0.0000032832799998772088656551204621792*theta__dot1(i)^2*sin(2.0*theta__2(i))*sin(2.0*theta__dot3(i)) + 0.0098681239655333298024348197660315*theta__dot1(i)^2*cos(2.0*theta__2(i))*sin(theta__3(i)) + 0.0098681239655333298024348197660315*theta__dot1(i)^2*sin(2.0*theta__2(i))*cos(theta__3(i)) + 2.5e-36*theta__dot2(i)^2*cos(2.0*theta__2(i))*sin(theta__3(i)) - 1.25e-36*theta__dot3(i)^2*cos(4.0*theta__1(i))*sin(theta__3(i)) + 1.25e-36*theta__dot3(i)^2*cos(2.0*theta__2(i))*sin(theta__3(i)) + 1.25e-36*theta__dot3(i)^2*sin(2.0*theta__2(i))*cos(theta__3(i)) - 2.5e-36*theta__dot2(i)^2*cos(4.0*theta__1(i))*cos(2.0*theta__2(i))*sin(theta__3(i)) - 1.25e-36*theta__dot3(i)^2*cos(4.0*theta__1(i))*cos(2.0*theta__2(i))*sin(theta__3(i)) - 1.25e-36*theta__dot3(i)^2*cos(4.0*theta__1(i))*sin(2.0*theta__2(i))*cos(theta__3(i));
            C3 =  0.0049340619827666649012174098830158*theta__dot1(i)^2*sin(theta__3(i)) + 0.0098681239655333298024348197660315*theta__dot2(i)^2*sin(theta__3(i)) - 0.0000032832799998772088656551204621792*theta__dot1(i)^2*cos(2.0*theta__2(i) + 2.0*theta__3(i)) + 0.0038061211630683616229356184931552*theta__dot1(i)^2*sin(2.0*theta__2(i) + 2.0*theta__3(i)) + 5.0e-36*theta__dot2(i)^2*sin(2.0*theta__2(i) + 2.0*theta__3(i)) + 5.0e-36*theta__dot3(i)^2*sin(2.0*theta__2(i) + 2.0*theta__3(i)) + 0.0049340619827666649012174098830158*theta__dot1(i)^2*sin(2.0*theta__2(i) + theta__3(i)) + 6.25e-37*theta__dot2(i)^2*sin(4.0*theta__1(i) + theta__3(i)) + 1.7858241435743426370435208870233e-36*theta__dot2(i)^2*sin(2.0*theta__2(i) + theta__3(i)) - 6.25e-37*theta__dot2(i)^2*sin(2.0*theta__2(i) - 4.0*theta__1(i) + theta__3(i)) - 6.25e-37*theta__dot2(i)^2*sin(4.0*theta__1(i) + 2.0*theta__2(i) + theta__3(i)) - 6.25e-37*theta__dot2(i)^2*sin(4.0*theta__dot1(i) - theta__3(i)) - 5.3582414357434263704352088702327e-37*theta__dot2(i)^2*sin(2.0*theta__2(i) - theta__3(i)) - 1.0716482871486852740870417740465e-36*theta__dot2(i)*theta__dot3(i)*sin(theta__3(i)) + 1.0716482871486852740870417740465e-36*theta__dot2(i)*theta__dot3(i)*sin(2.0*theta__2(i) + theta__3(i));
            
     %% Gravity Vector ----------------------------------------------------%       

            G1 =  0;
            G2 =  g*(0.046*cos(theta__2(i) + theta__3(i)) + 0.096*cos(theta__2(i)));
            G3 =  0.046*g*cos(theta__2(i) + theta__3(i));

%% Dynamics Matrices
Mx = [H11, H12 , H13; H21 , H22 , H23; H31 , H32 ,H33];
Vx = [C1;C2;C3];
G = [G1;G2;G3];
Theta__ddot = [theta__ddot1(i);theta__ddot2(i);theta__ddot3(i)];

beta =[C1; C2; C3]+[G1;G2;G3];

%% Compute Torque
tauVector = Mx*Theta__ddot+beta;
tau1(i) = tauVector(1);
tau2(i) = tauVector(2);
tau3(i) = tauVector(3);

%% Compute Acceleration 
acceleration = Mx\(tauVector-beta);
acc1(i) = acceleration(1);
acc2(i) = acceleration(2);
acc3(i) = acceleration(3);

end

%%%% Computed Torque for Joint 2-------------------------------------------% 
figure(6)                                                                  % 
clf                                                                        %
figure(6)                                                                  %
hold on                                                                    %
title('Computed and Actual Torque - MX-106 (Joint 2)')                     %
%plot(time, tau1, '-.r','Linewidth',3)                                     %
plot(time, tau2, 'g','Linewidth',2)                                        %
%plot(time, tau3, 'b')                                                     %
%plot(VarName2)                                                            %
%plot(time1,smooth(A,100),'b','Linewidth',2)                               %
hold off                                                                   %
legend('Computed Torque', 'Actual Torque')                                 %
grid on;                                                                   %
xlabel('Time (s)'); ylabel('Torque (N/m)');                                %
%%%%-----------------------------------------------------------------------%


% % Plotting the Angular Displacement, Angular Velocity ---%
% figure(2)
% clf
% figure(2)
% subplot(2, 1, 1)
% hold on
% plot(time, theta__2, 'c','Linewidth',1)
% plot(time2, C, 'k','Linewidth',1)
% legend('Computed Position', 'Actual Position') 
% hold off
% grid on; 
% xlabel('Time (s)'); ylabel('Angular Displacement (rad)'); 

% subplot(2, 1, 2)
% hold on
% plot(time, theta__dot2, 'm','Linewidth',1)
% plot(time2, smooth(C), 'k','Linewidth',1)
% legend('Computed Velocity', 'Actual Velocity') 
% hold off
% grid on; 
% xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)'); 
