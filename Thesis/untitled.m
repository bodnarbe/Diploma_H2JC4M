% Szimbolikus változók létrehozása
syms alp1 alp2 alp3 alp4 alp5 alp6 alp7 alp8 alp9
syms a1 a2 a3 a4 a5 a6 a7 a8 a9 
syms th1 th2 th3 th4 th5 th6 th7 th8 th9
syms d1 d2 d3 d4 d5 d6 d7 d8 d9
syms alpha beta gamma



% (alpha,a,theta,d)
    % [0,0,theta_1,d_1]
    % [0,a_2,0,d_2]
    % [0,a_3,0,d_3]
    % [0,0,theta_4,0]
    % [alpha_5,0,theta_5,0]
    % [0,0,0,d_6]
    % [alpha_7,0,theta_7,0]
    % [0,a_8,theta_8,0]
    % [alpha_9,0,theta_9,d_9]

% Davenport-hatványok (Davenportian matrix) létrehozása
DH_0_1 =[[cos(th1),-1*sin(th1),0,0],
         [cos(0)*sin(th1),cos(0)*cos(th1),-1*sin(0),-1*d1*sin(0)],
         [sin(0)*sin(th1),sin(0)*cos(th1),cos(0),d1*cos(0)],
         [0,0,0,1]];

DH_1_2 =[[cos(0),-1*sin(0),0,a2],
         [cos(0)*sin(0),cos(0)*cos(0),-1*sin(0),-1*d2*sin(0)],
         [sin(0)*sin(0),sin(0)*cos(0),cos(0),d2*cos(0)],
         [0,0,0,1]];

DH_2_3 =[[cos(0),-1*sin(0),0,a3],
         [cos(0)*sin(0),cos(0)*cos(0),-1*sin(0),-1*d3*sin(0)],
         [sin(0)*sin(0),sin(0)*cos(0),cos(0),d3*cos(0)],
         [0,0,0,1]];

DH_3_4 =[[cos(-1*pi / 2),-1*sin(-1*pi / 2),0,0],
         [cos(0)*sin(-1*pi / 2),cos(0)*cos(-1*pi / 2),-1*sin(0),-1*0*sin(0)],
         [sin(0)*sin(-1*pi / 2),sin(0)*cos(-1*pi / 2),cos(0),0*cos(0)],
         [0,0,0,1]];

DH_4_5 =[[cos(th5),-1*sin(th5),0,0],
         [cos(-1*pi / 2)*sin(th5),cos(-1*pi / 2)*cos(th5),-1*sin(-1*pi / 2),-1*0*sin(-1*pi / 2)],
         [sin(-1*pi / 2)*sin(th5),sin(-1*pi / 2)*cos(th5),cos(-1*pi / 2),0*cos(-1*pi / 2)],
         [0,0,0,1]];

DH_5_6 =[[cos(0),-1*sin(0),0,0],
         [cos(0)*sin(0),cos(0)*cos(0),-1*sin(0),-1*d6*sin(0)],
         [sin(0)*sin(0),sin(0)*cos(0),cos(0),d6*cos(0)],
         [0,0,0,1]];

DH_6_7 =[[cos(th7),-1*sin(th7),0,0],
         [cos(pi / 2)*sin(th7),cos(pi / 2)*cos(th7),-1*sin(pi / 2),-1*0*sin(pi / 2)],
         [sin(pi / 2)*sin(th7),sin(pi / 2)*cos(th7),cos(pi / 2),0*cos(pi / 2)],
         [0,0,0,1]];

DH_7_8 =[[cos(-1*pi / 2),-1*sin(-1*pi / 2),0,a8],
         [cos(0)*sin(-1*pi / 2),cos(0)*cos(-1*pi / 2),-1*sin(0),-1*0*sin(0)],
         [sin(0)*sin(-1*pi / 2),sin(0)*cos(-1*pi / 2),cos(0),0*cos(0)],
         [0,0,0,1]];

DH_8_9 =[[cos(th9),-1*sin(th9),0,0],
         [cos(-1*pi / 2)*sin(th9),cos(-1*pi / 2)*cos(th9),-1*sin(-1*pi / 2),-1*d9*sin(-1*pi / 2)],
         [sin(-1*pi / 2)*sin(th9),sin(-1*pi / 2)*cos(th9),cos(-1*pi / 2),d9*cos(-1*pi / 2)],
         [0,0,0,1]];

R_z_1 = [cos(alpha) -sin(alpha) 0,
        sin(alpha) cos(alpha) 0,
        0 0 1];
R_y_2 = [cos(beta) 0 sin(beta),
        0 1 0,
        -sin(beta) 0 cos(beta)];
R_x_3 = [  1 0 0,
        0 cos(gamma) -sin(gamma),
        0 sin(gamma) cos(gamma)];


% Davenport-hatványok szorzása
result = R_z_1*R_y_2*R_x_3;

% Eredmény kiíratása
disp(result);

% Szimbolikus kifejezések egyszerűsítése
simplified_result = simplify(result);

% Egyszerűsített eredmény kiíratása
disp(simplified_result);