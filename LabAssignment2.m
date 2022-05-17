classdef LabAssignment2 < handle
    properties

        Dobot_A2 %Robot class
        workspace = [-5 5 -3 3 -0.5 2];
        pill_height = {};        
        pillMatrix = {};

        Greenselect1 = 0;
        Purpleselect1 = 0;
        Orangeselect1 = 0;
        
    end
    
    methods
function obj = LabAssignment2(tablepose) %constructor

    startpose =transl(0.4,-0.3,0);
    tablepose = transl(0,0,0); 
    obj.GenerateEnvironment(startpose,tablepose); %call generate environment fuction
    disp('Press Enter to Continue');

    pause();
end

function GenerateEnvironment(obj,centre,tablepose)    
    
    %ENVIRONMENT
    [f,v,data] = plyread('table.ply','tri');
    
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    
    x_table = tablepose(1,4);
    y_table = tablepose(2,4);
    Offset = centre(3,4) - 0.554; % table is 0.554m tall, and so this is to asjust the offset of the table so that the robot can sit on top of it
    
    trisurf(f,v(:,1) + x_table, v(:,2) + y_table, v(:,3) + Offset,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    hold on
    camlight;
    axis equal;
    view(3);

    
    [f,v,data] = plyread('fire_extinguisher.ply','tri');
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    % Then plot the trisurf
    trisurf(f,v(:,1) - 1, v(:,2) - 1, v(:,3) + 0 ,'FaceVertexCData',...
    vertexColours,'EdgeColor','interp','EdgeLighting','flat');

       surf([-2.5,-2.5;2.5,2.5],[-2,2;-2,2],[-0.5,-0.5;-0.5,-0.5],...
       'CData',imread('concrete.jpg'),'FaceColor','texturemap');
       
       surf([0.1,-0.1;0.1,-0.1],[0.1,0.1;0.2,0.2],[0,0;0,0],...
       'CData',imread('pick_up_sign.jpg'),'FaceColor','texturemap');
  %ENVIRONMENT
  
  %Generating pills and robot
       disp('Generating Dobot');
       dobot = Dobot_A2;
       qHome1 = dobot.model.getpos;
       
%        Boxrot = BoxClass(transl(0,0,0));  
       Boxinitpos = BoxClass(transl(0.25,0,0.03));  
       
       
       Green1 = GreenPills(transl(0.25,-0.2,0.03));  %0.03 is the height of the pill bottle, translating so that they sit on top of the table
       Green2 = GreenPills(transl(0.25,-0.3,0.03));
       Green3 = GreenPills(transl(0.25,-0.4,0.03));
       
       Orange1 = OrangePills(transl(0,-0.2,0.03));  
       Orange2 = OrangePills(transl(0,-0.3,0.03));
       Orange3 = OrangePills(transl(0,-0.4,0.03));
       
       Purple1 = PurplePills(transl(-0.25,-0.2,0.03));  
       Purple2 = PurplePills(transl(-0.25,-0.3,0.03));
       Purple3 = PurplePills(transl(-0.25,-0.4,0.03));
       
       
       
q1 = dobot.model.getpos;
q2 = dobot.model.ikcon(Green1.GreenPillsPose);
qMatrix = jtraj(q1,q2,50);


for i = 1:50
    dobot.model.animate(qMatrix(i,:));                                    % Moving the robot near the GreenBlock
    drawnow();
end


        q1 = dobot.model.getpos;
        q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
        qMatrix5 = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            dobot.model.animate(qMatrix5(i,:));
            newPose1 = dobot.model.fkine(qMatrix5(i,:));
            Green1.move(newPose1);            
            drawnow();
end   

end


    end
end