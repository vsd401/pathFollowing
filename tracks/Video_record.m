close all
% initialize variables to store simulation results
filename= 'Trackinfo.txt';
NN =  N+1;
out.mu_x_pred_opt  = NaN(4, NN, size(NMPC_statePred.signals.values,1));         % mean of optimal state prediction sequence

%提取数据，顺序为s,ey,phi,v,
% out.u_pred_opt     = NaN(mpc.m, mpc.N,   kmax);         % open-loop optimal input prediction
data_reshape = reshape(NMPC_statePred.signals.values,size(NMPC_statePred.signals.values,1),nx,N+1);
for i = 1: size(NMPC_statePred.signals.values,1)     %transformProj2Orig(si,ni,alpha,v,filename)
        S  = reshape(data_reshape(i,6,1:NN),NN,1);
        NI = reshape(data_reshape(i,5,1:NN),NN,1);
        ALPHA  = reshape(data_reshape(i,1,1:NN),NN,1);  %actually it's alpha_dot prediction 
        V = sqrt((reshape(data_reshape(i,2,1:NN),NN,1)).^2+(reshape(data_reshape(i,3,1:NN),NN,1)).^2);
        [x, y, alpha,v] = transformProj2Orig(S,NI, ALPHA, V,filename);
        out.mu_x_pred_opt(1,:,i)=x;
        out.mu_x_pred_opt(2,:,i)=y;
        out.mu_x_pred_opt(3,:,i)=v;
        out.mu_x_pred_opt(4,:,i)=alpha;
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
    trackAnim.updateTrackAnimation(k);
%     trackAnim.updateScope(k);
    pause(0.05);
    drawnow;
end


%% Record video

FrameRate = 100;
source_folder = pwd;
videoName = fullfile(source_folder,sprintf('trackAnimVideo-%s',date));
videoFormat = 'Motion JPEG AVI';
trackAnim.recordvideo(videoName, videoFormat, FrameRate);