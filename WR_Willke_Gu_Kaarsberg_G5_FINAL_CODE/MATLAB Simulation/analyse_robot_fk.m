function H = analyse_robot_fk()
    
    syms q1 q2 L1 L2 L3 real
    
    %DH = [0 0 L1 0];         % 0-1
    %DH = [q1 -L3 0 -pi/2];   % 1-2
    DH=[q2 L2 0 0];        % 2-3
    %DH =  [q2 L2/2 0 0];     % 2-CM3
          %q1 -L3 0 -pi/2;   % 1-CM2
          %q2 L2/2 0 0];     % 2-CM3
    
    H_stack = forward_kinematics(DH);
      
    % ef transformation
    H = H_stack(:,:,end);
    cp_H = char(H)

end

function H_stack = forward_kinematics(DH)
    dof = size(DH, 1);
    H_stack = sym(zeros(4,4,dof + 1));
    H_stack(:,:,1) = sym(eye(4));
    
    for i = 2 : dof + 1
        H_stack(:,:,i) = H_stack(:,:,i-1)*relativeTrans(DH(i-1,:));
    end
    
end

function H = relativeTrans(dh)
    Hz = [RotZ(dh(1)) [0 0 dh(2)]'; 0 0 0 1];
    Hx = [RotX(dh(4)) [dh(3) 0 0]'; 0 0 0 1];
    H = Hz*Hx;
end

function R = RotZ(a)
    R=[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end

function R = RotX(a)
    R=[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end