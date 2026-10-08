function plotJoint(HT, DrawLink, name_prefix)
    grid on 
    hold on
    % % Visualize the cage and its respective coordinate frames
    tOff=0.01;
    xl=xlim;
    xLength=0.05;
    yl=ylim;
    yLength=0.03;
    zl=zlim;
    zLength=0.05;
    
    axisX_0=[xLength;0;0];
    axisY_0=[0;yLength;0];
    axisZ_0=[0;0;zLength];
    
    % % Visualize the Origin O_0
    
    O_0=[0;0;0];
    
    plot3(O_0(1),O_0(2),O_0(3), 'k .','MarkerSize',10)
    text(O_0(1)+tOff,O_0(2)+tOff,O_0(3)+tOff, 'O_W','FontSize',14);
    %Plot x-axis
    plot3([O_0(1);axisX_0(1)],[O_0(2);axisX_0(2)],[O_0(3);axisX_0(3)],'r -', 'Linewidth',2)
    %Plot y-axis
    plot3([O_0(1);axisY_0(1)],[O_0(2);axisY_0(2)],[O_0(3);axisY_0(3)],'g -', 'Linewidth',2)
    %Plot z-axis
    plot3([O_0(1);axisZ_0(1)],[O_0(2);axisZ_0(2)],[O_0(3);axisZ_0(3)],'b -', 'Linewidth',2)
    
    [~,~,n]=size(HT);
    
    for i=1:n
        name=[name_prefix num2str(i-1)];

        % % Visualize the Origin Oi_0
        Oi_0(:,i)=HT(:,:,i)*[O_0;1];
        axisX1_0=HT(:,:,i)*[axisX_0;1];
        axisY1_0=HT(:,:,i)*[axisY_0;1];
        axisZ1_0=HT(:,:,i)*[axisZ_0;1];
        
        % Origin CF1
        plot3(Oi_0(1,i),Oi_0(2,i),Oi_0(3,i), 'k .','MarkerSize',10)
        text(Oi_0(1,i)+tOff,Oi_0(2,i)+tOff,Oi_0(3,i)+tOff, name,'FontSize',12);
        %Plot x-axis
        plot3([Oi_0(1,i);axisX1_0(1)],[Oi_0(2,i);axisX1_0(2)],[Oi_0(3,i);axisX1_0(3)],'r -', 'Linewidth',2)
        %Plot y-axis
        plot3([Oi_0(1,i);axisY1_0(1)],[Oi_0(2,i);axisY1_0(2)],[Oi_0(3,i);axisY1_0(3)],'g -', 'Linewidth',2)
        %Plot z-axis
        plot3([Oi_0(1,i);axisZ1_0(1)],[Oi_0(2,i);axisZ1_0(2)],[Oi_0(3,i);axisZ1_0(3)],'b -', 'Linewidth',2)
    end
    
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    % xlim([-0.5,0.5]);
    % ylim([-0.5,0.5]);
    % zlim([-0.2,0.8])
    if DrawLink==true
        for i=1:n-1
            plot3([Oi_0(1,i);Oi_0(1,i+1)],[Oi_0(2,i);Oi_0(2,i+1)],[Oi_0(3,i);Oi_0(3,i+1)],'k', 'Linewidth',1)
        end
    end
end

