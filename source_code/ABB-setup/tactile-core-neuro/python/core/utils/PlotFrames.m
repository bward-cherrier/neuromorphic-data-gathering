classdef PlotFrames < handle
    properties  
        hfig
        hvid
        pos = 400*[1.5 1.1 1 1];
        frameRate = 5; %2.25
    end  

    methods
        function self = PlotFrames()               
            self.hfig = figure('Name','Frame', 'color','w', 'MenuBar','None', 'Position',self.pos); 
            clf; hold; axis('equal', 'off') 
            
            self.hvid = VideoWriter(fullfile(getenv('TEMPPATH'), [self.hfig.Name '.avi']));
            self.hvid.FrameRate = self.frameRate;
            open(self.hvid);
            
            for i = 1:10; writeVideo(self.hvid, getframe(self.hfig)); end
        end

        function update(self, frame, y)
            for i = 1:size(frame, 4)
                figure(self.hfig); imshow(squeeze(frame), 'InitialMagnification','fit', 'border','tight')     
                text(size(frame,2)/2,size(frame,2)/40, ['pose = (' sprintf(' %4.1f',y) ' )'], 'color','r', ...
                    'fontsize',12, 'fontweight','bold', 'horizontalalignment','center')
                hold off; drawnow
            end

            writeVideo(self.hvid, getframe(self.hfig));
        end
        
        function finish(self, filename)
            savefig(self.hfig, [filename '.fig']);
            close(self.hvid);
            movefile(fullfile(self.hvid.Path,self.hvid.Filename), [filename '.avi'])
        end
        
    end 
end

