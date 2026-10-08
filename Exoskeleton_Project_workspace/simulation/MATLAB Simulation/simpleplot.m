function simpleplot(u)
    q1 = u(1);
    q2 = u(2);

    t = u(3);

    Q = [q1; q2];
    
    L1 = u(4);
    L2 = u(5);
    L3 = u(6); % to end effector middle of hand
    
    L4=0.02;

    T0_W = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, L1+L2; 0, 0, 0, 1] * [RotY(pi/2), [0;0;0]; 0, 0, 0, 1];
    T1_0 = [1, 0, 0, L1; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    T2_0 = [cos(q1), 0, -sin(q1), L1; sin(q1), 0, cos(q1), 0; 0, -1, 0, -L3; 0, 0, 0, 1];
    T3_0 = [cos(q1)*cos(q2), -cos(q1)*sin(q2), -sin(q1), L1 - L2*sin(q1); cos(q2)*sin(q1), -sin(q1)*sin(q2), cos(q1), L2*cos(q1); -sin(q2), -cos(q2), 0, -L3; 0, 0, 0, 1];
    
    % position of force sensor
    Tp1_3 = [eye(3), [0;L4;0]; [0,0,0,1]];
    Tp2_3 = [eye(3), [0;-L4;0]; [0,0,0,1]];
    Tp1_0 = T3_0 * Tp1_3;
    Tp2_0 = T3_0 * Tp2_3;
    Tp1_W = T0_W * Tp1_0;
    Tp2_W = T0_W * Tp2_0;

    T1_W = T0_W * T1_0;
    T2_W = T0_W * T2_0;
    T3_W = T0_W * T3_0;
    
    Tcm1_0 = [cos(q1), -sin(q1), 0, L1; sin(q1), cos(q1), 0, 0; 0, 0, 1, -L3/2; 0, 0, 0, 1];
    Tcm2_0 = [cos(q1)*cos(q2), -cos(q1)*sin(q2), -sin(q1), L1 - (L2*sin(q1))/2; cos(q2)*sin(q1), -sin(q1)*sin(q2), cos(q1), (L2*cos(q1))/2; -sin(q2), -cos(q2), 0, -L3; 0, 0, 0, 1];

    Tcm1_W = T0_W * Tcm1_0;
    Tcm2_W = T0_W * Tcm2_0;
    
    Transformation_structure(:,:,1)=T0_W;
    Transformation_structure(:,:,2)=T1_W;
    Transformation_structure(:,:,3)=T2_W;
    Transformation_structure(:,:,4)=T3_W;

    Transformation_structure_cm(:,:,1)=Tcm1_W;
    Transformation_structure_cm(:,:,2)=Tcm2_W;

    Transformation_structure_p(:,:,1)=Tp1_W;
    Transformation_structure_p(:,:,2)=Tp2_W;

    sampleTime=0.04;
    if(~mod(t,sampleTime))
        clf;
        view([120,20])
        xlim([-L2,L2])
        ylim([-1.5*L2,1.5*L2])
        zlim([-0.5*L2,2*L2])
        plotJoint(Transformation_structure, true, 'O_')
        %plotJoint(Transformation_structure_cm, true, 'cm')
        plotJoint(Transformation_structure_p, true, 'p')
    end


end