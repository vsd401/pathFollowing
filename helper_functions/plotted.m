%% Simulate
close all
dt = T / N;
Tf = 35.00;  % maximum simulation time[s]
Nsim = round(Tf / dt)+1;
% sref_N = 3;  % reference for final reference progress

% initialize data structs

sim( 'simulink_model_closedloop.slx')

%% Plots
t = linspace(0.0, Nsim * dt, Nsim);
t=t';
% Plot results
figure(1);
subplot(1,2,1);
plot(t, simU.Data(:,1), 'g');
xlabel('t');
ylabel('der_{delta}');

subplot(1,2,2);
plot(t, simU.Data(:,2), 'g');
xlabel('t');
ylabel('der_{Tc}'); 

figure(2)
subplot(2,2,1);
plot(t, simX.Data(:,5), 'r');
xlabel('t');
ylabel('e_y');

subplot(2,2,2);
plot(t, simX.Data(:,6), 'g');
xlabel('t');
ylabel('S');

subplot(2,2,3);
plot(t, simX.Data(:,7), 'g');
xlabel('t');
ylabel('delta'); 

subplot(2,2,4);
plot(t, simX.Data(:,8), 'g');
xlabel('t');
ylabel('Tc'); 

figure(3)
hold on
plot(t, simX.Data(:,2));
xlabel('t');
ylabel('v');
 
grid;
xlim([t(1), t(end)]);
% Plot track
figure(4);
plotTrackProj(simX.Data, track_file);

% Plot alat
alat = zeros(Nsim,1);alon= zeros(Nsim,1);
for i = 1:Nsim
    alat(i) = full(constraint.alat(simX.Data(i,:),simU.Data(i,:)));
    alon(i) = full(constraint.alon(simX.Data(i,:),simU.Data(i,:)));
end
figure(5);
subplot(2,1,1)
plot(t, alat);
line([t(1), t(end)], [constraint.alat_min, constraint.alat_min], 'LineStyle', '--', 'Color', 'k');
line([t(1), t(end)], [constraint.alat_max, constraint.alat_max], 'LineStyle', '--', 'Color', 'k');
xlabel('t');
ylabel('alat');
xlim([t(1), t(end)]);

subplot(2,1,2)
plot(t, alon);
line([t(1), t(end)], [constraint.alon_min, constraint.alon_min], 'LineStyle', '--', 'Color', 'k');
line([t(1), t(end)], [constraint.alon_max, constraint.alon_max], 'LineStyle', '--', 'Color', 'k');
xlabel('t');
ylabel('alon');
xlim([t(1), t(end)]);

    % check if one lap is done and break and remove entries beyond
    for i = 1:length(t)
        s0 = simX.Data(i,6);
        if s0 > Sref(end) + 0.1
            % find where vehicle first crosses start line
            N0 = find(diff(sign(simX.Data(:, 1))));
            N0 = N0(1);
            Nsim = i - N0 + 1;  % correct to final number of simulation steps for plotting
            simx = simX.Data(N0:i, :);
            simU = simU.Data(N0:i, :);
            break
        end
    end
    fprintf('the lap time is: %f\n',norm(t(i)));
    
    figure(22);
    plotTrackProj(simx, track_file);
    title('one lap')
