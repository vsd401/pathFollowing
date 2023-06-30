close all
% initialize variables to store simulation results
filename= 'Trackinfo.txt';

out.mu_x_pred_opt  = NaN(4, N+1, size(NMPC_statePred.signals.values,1));         % mean of optimal state prediction sequence
s = NaN(N+1,1);
n = NaN(N+1,1);
alpha = NaN(N+1,1);
v = NaN(N+1,1);
% sss =  reshape(NMPC_statePred.signals.values,251,51,8);
%提取数据，顺序为s,ey,phi,v,
% out.u_pred_opt     = NaN(mpc.m, mpc.N,   kmax);         % open-loop optimal input prediction
for i = 1: size(NMPC_statePred.signals.values,1)
    % load track
    for j = 1:N+1
        % transform data
        psi
        s(j) =     NMPC_statePred.signals.values(i,6+(j-1)*size(model.x0,2)); % s的预测值
        n(j) =     NMPC_statePred.signals.values(i,3+(j-1)*size(model.x0,2)); % n的预测值
        alpha(j) = NMPC_statePred.signals.values(i,1+(j-1)*size(model.x0,2)); % 这是phi_dot的预测值
        v(j) =     NMPC_statePred.signals.values(i,2+(j-1)*size(model.x0,2)); % v的预测值
%         distance=3;
        [x, y, ~,v] = transformProj2Orig(s, n, alpha, v,filename);
        out.mu_x_pred_opt(1,:,i)=x;
        out.mu_x_pred_opt(2,:,i)=y;
        out.mu_x_pred_opt(3,:,i)=v;
        out.mu_x_pred_opt(4,:,i)=alpha;

    end
end
out.mu_x_pred_opt(4,1,:)=psi.Data;
% start animation
trackAnim = SingleTrackAnimation(track, out.mu_x_pred_opt);
trackAnim.initTrackAnimation();
% trackAnim.initScope();
for k=2:N %size(NMPC_statePred.signals.values,1)
%     status = obj.updateTrackAnimation(k);
%     if ~ trackAnim.updateTrackAnimation(k)
%         break;
%     end
    trackAnim.updateTrackAnimation(k)
%     trackAnim.updateScope(k);
    pause(0.15);
    drawnow;
end


%% Record video

FrameRate = 7;
source_folder = pwd;
videoName = fullfile(source_folder,sprintf('trackAnimVideo-%s',date));
videoFormat = 'Motion JPEG AVI';
trackAnim.recordvideo(videoName, videoFormat, FrameRate);