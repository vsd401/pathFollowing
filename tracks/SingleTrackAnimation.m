%------------------------------------------------------------------
% Programed by: 
%   - Lucas Rath (lucasrm25@gmail.com)
%   - 
%   -

%   Generate Animation for the main_singletrack.m script
%------------------------------------------------------------------


classdef SingleTrackAnimation < handle
    
    properties
        % object that contains track coordinates
        racetrack @ RaceTrack
        
        % variables that contain history of vehicle states and inputs to be ploted
        mu_x_pred_opt
        var_x_pred_opt
        u_pred_opt
        x_ref

        % Track animation handles
        h_fig
        h_ltrack
        h_rtrack
        h_mu_x_pred_opt
        h_var_x_pred_opt
        h_x_ref
        h_x_trace
        h_car
        
        % Scope handles
        h_scopex
        h_scopeu
        
        % covariance ellipse properties
        ell_npoints = 30     % number of points to make an ellipse
        ell_level = 2        % plot ell_level*sigma ellipse curves
        
        k   % current time step
        N   % horizon length
    end
    
    methods
        function obj = SingleTrackAnimation(racetrack, mu_x_pred_opt)
            obj.racetrack       = racetrack;
            obj.mu_x_pred_opt   = mu_x_pred_opt;
%             obj.var_x_pred_opt  = var_x_pred_opt;
%             obj.u_pred_opt      = u_pred_opt;
%             obj.x_ref           = x_ref;
            
            % get horizon length from inputs  
            obj.N = size(obj.mu_x_pred_opt,2);  
        end
        
        function initTrackAnimation(obj)
        % -----------------------------------------------------------------
        %   Init Animation of the vehicle driving the track. Please call
        %   updateTrackAnimation(k) to move forward with the animation.
        % -----------------------------------------------------------------
            obj.h_fig = figure('Color','w','Position',[468 128 872 633]);
            title('Adaptive Gaussian-Process MPC')
            hold on;
            grid on;
            axis equal;
            
            % -------------------------------------------------------------
            %   plot track asphalt
            % -------------------------------------------------------------
            n = length(obj.racetrack.track_l);
            v = [obj.racetrack.track_l(:,1:n)'; obj.racetrack.track_r(:,1:n)'];   % <n,2>
            f = [1:n-1 ; 2:n; n+2:2*n; n+1:2*n-1]';
            patch('Faces',f,'Vertices',v,'FaceColor',[0.95 0.95 0.95],'LineStyle', 'none')
            
            % -------------------------------------------------------------
            %   plot track borders
            % -------------------------------------------------------------
            obj.h_ltrack = plot(obj.racetrack.track_l(1,:),obj.racetrack.track_l(2,:),'k');
            obj.h_rtrack = plot(obj.racetrack.track_r(1,:),obj.racetrack.track_r(2,:),'k');
            
            %%plot ref trajectory
            obj.h_x_ref = plot(obj.racetrack.track_c(1,:),obj.racetrack.track_c(2,:),'k--',LineWidth=0.2);
            % -------------------------------------------------------------
            %   setup state predictions
            % -------------------------------------------------------------
            k = 1;
            
            % trace vehicle path
            obj.h_x_trace = patch(obj.mu_x_pred_opt(1,1,1:k),obj.mu_x_pred_opt(2,1,1:k),obj.mu_x_pred_opt(3,1,1:k),...
                                  'EdgeColor','interp',...
                                  'Marker','none',...
                                  'LineWidth',4,...
                                  'EdgeAlpha',0.5); 
            
            % plot car
            obj.h_car = patch('Faces',1:4,'Vertices',NaN(4,2),...
                        'EdgeColor','black',...
                        'FaceColor','none',...
                        'LineWidth',1);
            
            % reference trajectory
            obj.h_x_ref = plot(NaN,NaN,...
                        '-','LineWidth',1.0, 'Marker','none',...
                        'MarkerFaceColor',[0.5 0.5 0.5],'MarkerEdgeColor','k', 'Color','k',...
                        'MarkerSize',1,...
                        'DisplayName','Projected optimal trajectory',...
                        'XDataSource', 'obj.x_ref(1,:,obj.k)',...
                        'YDataSource', 'obj.x_ref(2,:,obj.k)');

            % optimal prediction means
            obj.h_mu_x_pred_opt = patch(obj.mu_x_pred_opt(1,:,k),obj.mu_x_pred_opt(2,:,k),obj.mu_x_pred_opt(3,:,k),...
                        'EdgeColor','interp','Marker','o',...
                        'MarkerSize',1,...
                        'MarkerFaceColor','flat',...
                        'DisplayName','Optimal trajectory' );
%                     
%             % plot prediction covariance ellipses
%             ell = NaN(obj.ell_level, obj.ell_npoints);
%             for i=1:obj.N
%                 obj.h_var_x_pred_opt{i} = patch('Faces',1:obj.ell_npoints,'Vertices',ell','FaceColor',[1 0 0],'FaceAlpha',0.3,'LineStyle', 'none');
%             end      
                             
                              
            % display legend, colorbar, etc.
%             leg = legend([obj.h_mu_x_pred_opt,obj.h_x_ref],'Location','northeast');
            c = colorbar;
            c.Label.String = 'Vehicle predicted velocity [m/s]';
%             caxis([5 25])
            
            % lock up axis limits
            xlim('manual')
            ylim('manual')
            drawnow
        end
        
        function initScope(obj)
        % -----------------------------------------------------------------
        %   Init Scope that shows vehicle prediction signals
        % -----------------------------------------------------------------
            obj.h_scopex = figure('Position',[-1006 86 957 808]);
            names = {'I-x','I-y','psi','V-vx','V-vy','psidot','track_dist'};
            angles = [0 0 1 0 0 1 0];
            for i=1:numel(names)
                subplot(4,2,i);
                p = plot(NaN);
                if angles(i)
                    p.YDataSource = sprintf('rad2deg(obj.mu_x_pred_opt(%d,:,obj.k))',i);
                else
                    p.YDataSource = sprintf('obj.mu_x_pred_opt(%d,:,obj.k)',i);
                end
                xlabel('Prediction horizon')
                grid on;
                title(names{i});
            end
            
            obj.h_scopeu = figure('Position',[-1879 93 867 795]);
            names = {'delta','T','Track vel'};
            angles = [1 0 0];
            for i=1:numel(names)
                subplot(2,2,i);
                p = plot(NaN);
                if angles(i)
                    p.YDataSource = sprintf('rad2deg(obj.u_pred_opt(%d,:,obj.k))',i);
                else
                    p.YDataSource = sprintf('obj.u_pred_opt(%d,:,obj.k)',i);
                end
                xlabel('Prediction horizon')
                grid on;
                title(names{i});
            end
        end
        
        function status = updateTrackAnimation(obj,k)
        % -----------------------------------------------------------------
        %   Update track animation with the current time step k. Beware
        %   that the vectors obj.mu_x_pred_opt and obj.x_ref must have the
        %   correct values at position (:,:,k)
        % -----------------------------------------------------------------
            status = 0;
            if k < 1 || k > size(obj.mu_x_pred_opt,3) || any(isnan(obj.mu_x_pred_opt(:,:,k)),'all')
                return;
            end

            % update predicted Trajectory    
            vel = obj.mu_x_pred_opt(3,:,k);
            obj.h_mu_x_pred_opt.XData = [obj.mu_x_pred_opt(1,:,k) NaN];
            obj.h_mu_x_pred_opt.YData = [obj.mu_x_pred_opt(2,:,k) NaN];
            obj.h_mu_x_pred_opt.CData = [vel min(vel)];
            
            
% %             update predicted Trajectory
%             obj.h_x_ref.XData = obj.mu_x_pred_opt(1,1,1:k);
%             obj.h_x_ref.YData = obj.mu_x_pred_opt(2,1,1:k);
            
%             % update actural trace
            veltrace = squeeze(obj.mu_x_pred_opt(3,1,1:k))';
            obj.h_x_trace.XData = [squeeze(obj.mu_x_pred_opt(1,1,1:k))' NaN];
            obj.h_x_trace.YData = [squeeze(obj.mu_x_pred_opt(2,1,1:k))' NaN];
            obj.h_x_trace.CData = [veltrace NaN];
            
            % update car
            carpos = obj.mu_x_pred_opt(1:2,1,k); %[0;0]
            psi    = obj.mu_x_pred_opt(4,1,k); %deg2rad(30);
            car_w = 1;
            car_l = 2;
            V_carpoints = [[car_l/2;car_w/2],[car_l/2;-car_w/2],[-car_l/2;-car_w/2],[-car_l/2;car_w/2]];
            I_carpoints = [cos(psi) -sin(psi);
                           sin(psi)  cos(psi)] * V_carpoints + carpos;
            obj.h_car.Vertices = I_carpoints';
            
            % no error when updating graphics
            status = 1;
        end
        
        function updateScope(obj,k)
        % -----------------------------------------------------------------
        %   Update scope with signals from the current time step k. Beware
        %   that the vectors obj.mu_x_pred_opt and obj.u_pred_opt must have 
        %   the correct values at position (:,:,k)
        % -----------------------------------------------------------------
            obj.k = k;
            refreshdata(obj.h_scopex,'caller');
            refreshdata(obj.h_scopeu,'caller');
        end
        
        function recordvideo(obj, videoName, format, FrameRate)
        % -----------------------------------------------------------------
        %   Record video of the track animation
        % -----------------------------------------------------------------
            % video rec
            videoframes = struct('cdata',[],'colormap',[]);
            obj.initTrackAnimation();
            xlim manual
            ylim manual
            for k=1:size(obj.mu_x_pred_opt,3)
                status = obj.updateTrackAnimation(k);
                if status == 0
                    break;
                end
                videoframes(k) = getframe(obj.h_fig);
            end
            % -----------------------------------------------------------------
            %   Save video
            % -----------------------------------------------------------------
            writerObj = VideoWriter(videoName,format);
            writerObj.FrameRate = FrameRate;
            open(writerObj);
            % Write out all the frames.
            numberOfFrames = length(videoframes);
            for k=1:numberOfFrames 
               writeVideo(writerObj, videoframes(k));
            end
            close(writerObj);
            disp('Video saved successfully')
        end
    end
end