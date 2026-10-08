function [M,C,G] = analyse_jacobian()
    syms q1 q2 qp1 qp2 L1 L2 L3 real
    Q = [q1; q2];
    Qp = [qp1; qp2];
    
    T1_0 = [1, 0, 0, L1; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    T2_1 = [cos(q1), 0, -sin(q1), 0; sin(q1), 0, cos(q1), 0; 0, -1, 0, -L3; 0, 0, 0, 1];
    T3_2 = [cos(q2), -sin(q2), 0, 0; sin(q2), cos(q2), 0, 0; 0, 0, 1, L2; 0, 0, 0, 1];
    
    Tcm1_1 = [cos(q1), 0, -sin(q1), 0; sin(q1), 0, cos(q1), 0; 0, -1, 0, -L3; 0, 0, 0, 1];
    Tcm2_2 = [cos(q2), -sin(q2), 0, 0; sin(q2), cos(q2), 0, 0; 0, 0, 1, L2/2; 0, 0, 0, 1];
    Tcm2_1 = T2_1 * Tcm2_2;
    
    Tcm_1 = {Tcm1_1, Tcm2_1};
    
    Jcm1_1 = sym(zeros(6,2));
    Jcm2_1 = sym(zeros(6,2));
    
    Jcm1_1(4:6,1) = [sym(0); sym(0); sym(1)];
    Jcm1_1(1:3,1) = cross(Jcm1_1(4:6,1),Tcm1_1(1:3,4));
    
    Jcm2_1(4:6,1) = [sym(0); sym(0); sym(1)];
    Jcm2_1(1:3,1) = cross(Jcm2_1(4:6,1),Tcm2_1(1:3,4));
    Jcm2_1(4:6,2) = T2_1(1:3,1:3) * Jcm2_1(4:6,1);
    Jcm2_1(1:3,2) = cross(Jcm2_1(4:6,2),Tcm2_2(1:3,4));
    
    syms m1 m2 gx gy gz real
    m = [m1; m2];
    gxyz = [gx; gy; gz];
    syms I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333 real
    
    I2 = [I211 I212 I213;
          I212 I222 I223;
          I213 I223 I233];
    
    I3 = [I311 I312 I313;
          I312 I322 I323;
          I313 I323 I333];
    
    Rcm1_1 = Tcm1_1(1:3,1:3);
    Rcm2_1 = Tcm2_1(1:3,1:3);
    
    %M = m1 * Jcm1_1(1:3,:)' * Jcm1_1(1:3,:) + Jcm1_1(4:6,:)' * Rcm1_1 * I2 * Rcm1_1' * Jcm1_1(4:6,:) + ...
    %    m2 * Jcm2_1(1:3,:)' * Jcm2_1(1:3,:) + Jcm2_1(4:6,:)' * Rcm2_1 * I3 * Rcm2_1' * Jcm2_1(4:6,:);
    %M = simplify(M);
    M = m1 * Jcm2_1(1:3,:)' * Jcm2_1(1:3,:) + Jcm2_1(4:6,:)' * Rcm2_1 * I3 * Rcm2_1' * Jcm2_1(4:6,:);

    C=sym(zeros(2,2));
    G=sym(zeros(2,1));
    for k = 1:2
        P = sym(0);
        for j = 1:2
            %TODO: check whether -/+ for g vector
            P=P-m(j)*gxyz'*Tcm_1{j}(1:3,4);
            for i = 1:2
                C(k,j) = C(k,j) + 0.5 * (diff(M(k,j),Q(i))+diff(M(k,i),Q(j))-diff(M(i,j),Q(k)))*Qp(i);
            end
        end
        G(k) = diff(P,Q(k));
    end
end
