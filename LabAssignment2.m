classdef LabAssignment2 < handle
    properties

        UR5 %Robot class
        UR3 %Robot class
        workspace = [-5 5 -3 3 -0.5 2];
        brick_height = {};        
        brickMatrix = {};

        end
    
    methods
function obj = LabAssignment2(tablepose,brick_1,brick_2,brick_3,brick_4,brick_5,brick_6,brick_7,brick_8,brick_9) %constructor

startpose =transl(0.4,-0.3,0);
tablepose = transl(0,0,0); 


    brick_1 = [-0.5;0.5;0]; % position of each brick
    brick_2 = [0.4;-0.6;0];
    brick_3 = [-0.85;-1;0];
    brick_4 = [-0.45;0.85;0];
    brick_5 = [0.7;0.2;0];
    brick_6 = [0.9;0.9;0];
    brick_7 = [0.75;-0.38;0];
    brick_8 = [1;-0.5;0]; 
    brick_9 = [1;1;0];


    obj.LocationforBricks(brick_1,brick_2,brick_3,brick_4,brick_5,brick_6,brick_7,brick_8,brick_9); %call placement of bricks fuction
    obj.GenerateEnvironment(startpose,tablepose); %call generate environment fuction

    disp('Press Enter to Continue');

    pause();
end

function GenerateEnvironment(obj,centre,tablepose)    
    [f,v,data] = plyread('tabletocentre.ply','tri');
    
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    
    x_table = tablepose(1,4);
    y_table = tablepose(2,4);
    Offset = centre(3,4) - 0.495; % table is 0.495m tall, and so this is to asjust the offset of the table so that the robot can sit on top of it
    
    trisurf(f,v(:,1) + x_table, v(:,2) + y_table, v(:,3) + Offset,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    hold on
    camlight;
    axis equal;
    view(3);
    
    [f,v,data] = plyread('fire_extinguisher.ply','tri');
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    % Then plot the trisurf
    trisurf(f,v(:,1) + 3, v(:,2) + 2.5, v(:,3) + 0 ,'FaceVertexCData',...
        vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    
%     [f,v,data] = plyread('fence.ply','tri');
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%     trisurf(f,v(:,1) + x_table, v(:,2) + y_table+0.75, v(:,3) + Offset-1 ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%    
%     for i = 1:9
%         [f,v,data] = plyread('Brick.ply','tri');
%         % find corner size (vertex) of the bricks
%         determinesize = size(v,1);
%         %moving the origin of the brick to the top of the table
%         Originpoint = (sum(v)/determinesize); 
%         brickVertex = v - repmat(Originpoint,determinesize,1);
%         
%         % Scale the colours to be 0-to-1 (they are originally 0-to-255
%         vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%         % Then plot the trisurf
%         obj.brickMatrix{i} = trisurf(f,brickVertex(:,1) + obj.brick_height{i}(1,4), ...
%             brickVertex(:,2) + obj.brick_height{i}(2,4), brickVertex(:,3) + (0.06671/2), ...
%             'FaceVertexCData',vertexColours,'EdgeColor', 'interp','EdgeLighting','flat');
%     end
       surf([-5,-5;5,5],[-3.5,3;-3.5,3],[-0.5,-0.5;-0.5,-0.5],...
           'CData',imread('concrete.jpg'),'FaceColor','texturemap');

       
       
       
       surf([-5,-5;5,5],[-3.5,3;-3.5,3],[-0.5,-0.5;-0.5,-0.5],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
      

       disp('Generating UR5');
       self.UR5 = UR5(transl(0.85,-0.4, 0)); %generate UR3 robot
%        
            
end

function LocationforBricks(object,brick_1,brick_2,brick_3,brick_4,brick_5,brick_6,brick_7,brick_8,brick_9)
    
    object.brick_height{1} = brick_1; %create instance of each brick as an object
    object.brick_height{2} = brick_2;
    object.brick_height{3} = brick_3;
    object.brick_height{4} = brick_4;
    object.brick_height{5} = brick_5;
    object.brick_height{6} = brick_6;
    object.brick_height{7} = brick_7;
    object.brick_height{8} = brick_8;
    object.brick_height{9} = brick_9;
    
    position = eye(4); %create positions as a 4x4 matrix
    position = position(1:4,1:3); 

    for i= 1:9
        object.brick_height{i} = [[object.brick_height{i}]; 1];
        object.brick_height{i} = [[position] [object.brick_height{i}]];
    end
    clc;
    
end

    end
end