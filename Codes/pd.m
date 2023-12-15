function [output] = pd(~, s, m1, m2, l1, l2, g, kp1, kd1, kp2, kd2,q1_fin, q2_fin)
    q1 = s(1);
    q2 = s(2);
    q1dot = s(3);
    q2dot = s(4);

    e1 = q1_fin - q1;
    e2 = q2_fin - q2;

    f1 = kp1 * e1 - kd1 * q1dot;
    f2 = kp2 * e2 - kd2 * q2dot;

     %Definitions 
    M11 = (m1 + m2) * (l1*l1) + m2 * l2*(l2 + 2 * l1 * cos(q2));
    M22 = m2 * l2*l2;
    M12 = m2*l2*(l2+l1*cos(q2));
    M21 = M12;
    
    % Create the Mass matrix M
    M = [M11, M12; M21, M22];
    
    % Coriolis and Centrifugal Matrix 
    c11 = -m2 * l1 * l2 * sin(q2) * q1dot;
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

    f = [f1; f2];

    tau_matrix = (M) * f;
  
    dq = [q1dot; q2dot];
    
    m_inverse = inv(M);
    q_double_dot_matrix = ((m_inverse) * (-C * dq - G)) + f;
    ddq1 = q_double_dot_matrix(1);
    ddq2 = q_double_dot_matrix(2);

    output = [q1dot; q2dot; ddq1; ddq2];
end