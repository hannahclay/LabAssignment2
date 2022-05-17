classdef GenerateEnvironment < handle
    properties

        Dobot_A2 %Robot class
        workspace = [-5 5 -3 3 -0.5 2];
        pill_height = {};        
        pillMatrix = {};

        end
    
    methods
function GenerateEnvironment(obj,centre,tablepose)    
    [f,v,data] = plyread('table.ply','tri');
    
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    
    x_table = tablepose(1,4);
    y_table = tablepose(2,4);
    Offset = centre(3,4) - 0.554; % table is 0.495m tall, and so this is to asjust the offset of the table so that the robot can sit on top of it
    
    trisurf(f,v(:,1) + x_table, v(:,2) + y_table, v(:,3) + Offset,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    hold on
    camlight;
    axis equal;
    view(3);
    
    
    for i = 1:9
        [f,v,data] = plyread('pill_bottle.ply','tri');
        % find corner size (vertex) of the pills
        determinesize = size(v,1);
        %moving the origin of the pill to the top of the table
        Originpoint = (sum(v)/determinesize); 
        pillVertex = v - repmat(Originpoint,determinesize,1);
        
        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        % Then plot the trisurf
        obj.pillMatrix{i} = trisurf(f,pillVertex(:,1) + obj.pill_height{i}(1,4), ...
            pillVertex(:,2) + obj.pill_height{i}(2,4), pillVertex(:,3) + (0.06671/2), ...
            'FaceVertexCData',vertexColours,'EdgeColor', 'interp','EdgeLighting','flat');
    end
    
    
    
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
  
       disp('Generating Dobot');
       dobot = Dobot_A2();                                                       
       dobot.PlotModel3d();
       disp('lifting');
       dobot.lift(true);
       pause(0.5); 
       disp('lowering');
       dobot.lift(false); 
            
end

    end
end