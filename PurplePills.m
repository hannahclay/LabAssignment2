classdef PurplePills < handle
    properties
        PurplePillsPose;
        updatedPoints;
        PurplePillsVertexCount;
        PurplePillsVerts;
        midPoint;
        PurplePills_h;

    end
   methods
       function self = PurplePills(PurplePillsPose)
                self.PurplePillsPose = PurplePillsPose;
                self.location(self.PurplePillsPose);
                
       end
       function location(self,PurplePillsPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('PurplePills.ply','tri');

        % Get vertex count
        self.PurplePillsVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.PurplePillsVertexCount;
        self.PurplePillsVerts = v - repmat(self.midPoint,self.PurplePillsVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.PurplePills_h = trisurf(f,self.PurplePillsVerts(:,1),self.PurplePillsVerts(:,2), self.PurplePillsVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        self.PurplePillsPose = PurplePillsPose;
        self.updatedPoints = [self.PurplePillsPose * [self.PurplePillsVerts,ones(self.PurplePillsVertexCount,1)]']';  

        % Now update the Vertices
        self.PurplePills_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)

                self.PurplePillsPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.PurplePillsPose * [self.PurplePillsVerts,ones(self.PurplePillsVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.PurplePills_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
