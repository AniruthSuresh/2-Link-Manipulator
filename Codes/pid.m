function output = pid(~,s, q1f, q2f,  m1, m2, l1, l2, g, kp1,kp2,kd1,kd2,ki1,ki2)
 
    % Declare the State variables [ x1 ,x2 , q1 , q2 , q1. , q2. ] 
    x1 = s(1);
    x2 = s(2);
    q1 = s(3);
    q2 = s(4);
    q1dot = s(5);
    q2dot = s(6);

    x1dot = q1f - q1;
    x2dot = q2f - q2;


    f1 = kp1 * x1dot - kd1 * q1dot + ki1 *x1;
    f2 = kp2 * x2dot - kd2 * q2dot + ki2 *x2;
    

    %Definitions 
    M11 = (m1 + m2) * (l1*l1) + m2 * l2*(l2 + 2 * l1 * cos(q2));
    M22 = m2 * l2*l2;
    M12=m2*l2*(l2+l1*cos(q2));
    M21=M12;
    
    % Create the Mass matrix M
    M = [M11, M12; M21, M22];
    
    % Coriolis and Centrifugal Matrix 
    c11 = -m2 * l1 * l2 * sin(q2) * q2dot;
    c12 = -m2 * l1 * l2 * sin(q2) * (q1dot + q2dot);
    c21 = 0;
    c22 = m2 * l1 * l2 * sin(q2) * q2dot;
    
    % Create the Coriolis and Centrifugal matrix C
    C = [c11, c12; c21, c22];
    
    % Gravitational Matrix 
    G1 = m1*l1*g*cos(q1) + m2*g*(l2*cos(q1+q2) + l1*cos(q1));
    G2 = m2 * l2 * g * cos(q1+q2);
    
    % Create the Gravitatinal matrix G 
    G = [G1;G2];
    
    f = [f1;f2];
    Tau_matrix = M * f;
    
    q_dot_matrix = [q1dot ; q2dot];
    
    
    q_double_dot_matrix = (inv(M))* ( - C*q_dot_matrix-G) + f; % Quicker than inv (M) ( same operation as inverse ) 
     
    q_dd_1 = q_double_dot_matrix(1); % first row 
    q_dd_2 = q_double_dot_matrix(2);  % Second row 
     
    output = [x1dot ; x2dot ; q1dot ; q2dot ; q_dd_1 ; q_dd_2]; 


end

