close all;
clc;
clear;
syms t1 t2 t3 
vars = [t1 t2 t3];

t1=deg2rad(-45);
t2=deg2rad(-170);
t3=deg2rad(100);

Q = [   0      ,0      ,243      ,t1      ;
     -pi/2     ,0      ,0        ,t2      ;
        0      ,218    ,0        ,t3      ;
     pi/2      ,273    ,0        ,0          ];
     
N=size(Q,1);
B=1;

for i=1:N   
A{i} = [cos(Q(i,4))             ,-sin(Q(i,4))             ,0            ,Q(i,2)              ;
        sin(Q(i,4))*cos(Q(i,1)) ,cos(Q(i,4))*cos(Q(i,1))  ,-sin(Q(i,1)) ,-sin(Q(i,1))*Q(i,3) ;
        sin(Q(i,4))*sin(Q(i,1)) ,cos(Q(i,4))*sin(Q(i,1))  ,cos(Q(i,1))  ,cos(Q(i,1))*Q(i,3)  ;
        0                       ,0                        ,0            ,1                  ];
B=(B*A{i});
end
T0_1=A{1};
T1_2=A{2};
T2_3=A{3};
T3_4=A{4};


Tfinal=T0_1*T1_2*T2_3*T3_4;
        