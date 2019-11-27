%% Kinematics Fanuc M6iB/6S
clear
clc
close all

%% Denavit-Hartenberg Parameters
L(1) = Link('d',243   ,'a',0     ,'alpha',0     ,'modified');
L(2) = Link('d',0     ,'a',0     ,'alpha',-pi/2 ,'modified');
L(3) = Link('d',0     ,'a',280   ,'alpha',0     ,'modified');
L(4) = Link('d',0     ,'a',148   ,'alpha',pi/2  ,'modified');
L(5) = Link('d',0     ,'a',125   ,'alpha',0     ,'modified');

Fanuc = SerialLink([L], 'name', 'Old Yellow');
t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
t5 = 0;
q = [t1 t2 t3 t4 t5]; %%set of angles desired
Fanuc.plotopt = {'workspace' [-1500,1500,-1500,1500,-1500,1500]};
Fanuc.teach;
Fanuc.plot (q);