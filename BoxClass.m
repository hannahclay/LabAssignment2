classdef BoxClass < handle
    properties
        BoxPose;
        updatedPoints;
        BoxVertexCount;
        BoxVerts;
        midPoint;
        Box_h;

    end
   methods
       function self = BoxClass(BoxPose)
                self.BoxPose = BoxPose;
                self.location(self.BoxPose);
                
       end
       function location(self,BoxPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('box_brown.ply','tri');

        % Get vertex count
        self.BoxVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.BoxVertexCount;
        self.BoxVerts = v - repmat(self.midPoint,self.BoxVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.Box_h = trisurf(f,self.BoxVerts(:,1),self.BoxVerts(:,2), self.BoxVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        self.BoxPose = BoxPose;
        self.updatedPoints = [self.BoxPose * [self.BoxVerts,ones(self.BoxVertexCount,1)]']';  

        % Now update the Vertices
        self.Box_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)

                self.BoxPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.BoxPose * [self.BoxVerts,ones(self.BoxVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.Box_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
