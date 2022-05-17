classdef OrangePills < handle
    properties
        OrangePillsPose;
        updatedPoints;
        OrangePillsVertexCount;
        OrangePillsVerts;
        midPoint;
        OrangePills_h;

    end
   methods
       function self = OrangePills(OrangePillsPose)
                self.OrangePillsPose = OrangePillsPose;
                self.location(self.OrangePillsPose);
                
       end
       function location(self,OrangePillsPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('OrangePills.ply','tri');

        % Get vertex count
        self.OrangePillsVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.OrangePillsVertexCount;
        self.OrangePillsVerts = v - repmat(self.midPoint,self.OrangePillsVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.OrangePills_h = trisurf(f,self.OrangePillsVerts(:,1),self.OrangePillsVerts(:,2), self.OrangePillsVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        self.OrangePillsPose = OrangePillsPose;
        self.updatedPoints = [self.OrangePillsPose * [self.OrangePillsVerts,ones(self.OrangePillsVertexCount,1)]']';  

        % Now update the Vertices
        self.OrangePills_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)

                self.OrangePillsPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.OrangePillsPose * [self.OrangePillsVerts,ones(self.OrangePillsVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.OrangePills_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
