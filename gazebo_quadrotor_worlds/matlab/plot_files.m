clear all;
clc;

system('rostopic echo -p -b ski_pursuit.bag quadrotor/position >quad_pos.csv');
system('rostopic echo -p -b ski_pursuit.bag skier/position >skier_pos.csv');

quad_pos_all = csvread('quad_pos.csv', 1, 0);
quad_pos_all(:,1) = quad_pos_all(:,1)/1e6;
skier_pos_all = csvread('skier_pos.csv', 1, 0);
skier_pos_all(:,1) = skier_pos_all(:,1)/1e6;

quad_pos = quad_pos_all(1,:);
skier_pos = skier_pos_all(1,:);

figure;
hold on;
plotHandleQuadXY = plot(quad_pos(:,2), quad_pos(:,3),'b');
plotHandleSkierXY = plot(skier_pos(:,2), skier_pos(:,3),'r');

max_time = max(quad_pos_all(end,1), skier_pos_all(end,1));
all_x = [quad_pos_all(:,2) ; skier_pos_all(:,2)];
all_y = [quad_pos_all(:,3) ; skier_pos_all(:,3)];
max_x = max(all_x);
min_x = min(all_x);
max_y = max(all_y);
min_y = min(all_y);
xlim([min_x max_x+0.0001]);
ylim([min_y max_y+0.0001]);

figure;
hold on;
plotHandleQuadZ = plot(quad_pos(:,1), quad_pos(:,4), 'b');
plotHandleSkierZ = plot(skier_pos(:,1), skier_pos(:,4), 'r');
all_z = [quad_pos_all(:,4) ; skier_pos_all(:,4)];
max_z = max(all_z);
min_z = min(all_z);
xlim([0 max_time+0.0001]);
ylim([min_z max_z+0.0001]);

lastIdxQuad = 1;
lastIdxSkier = 1;
lastTimeQuad = quad_pos(1,1);
lastTimeSkier = skier_pos(1,1);

for i=1:1:max_time
    
    if(i >= lastTimeQuad && lastIdxQuad < size(quad_pos_all,1))   % add new data to plot
        lastIdxQuad = lastIdxQuad + 1;
        lastTimeQuad = quad_pos_all(lastIdxQuad,1);
        set(plotHandleQuadXY,'XData',[get(plotHandleQuadXY,'XData'), quad_pos_all(lastIdxQuad,2)], ...
            'YData',[get(plotHandleQuadXY,'YData'), quad_pos_all(lastIdxQuad,3)]);
        
        set(plotHandleQuadZ,'XData',[get(plotHandleQuadZ,'XData'), quad_pos_all(lastIdxQuad,1)], ...
            'YData',[get(plotHandleQuadZ,'YData'), quad_pos_all(lastIdxQuad,4)]);
        
    end
    
    if(i >= lastTimeSkier && lastIdxSkier < size(skier_pos_all,1))   % add new data to plot
        lastIdxSkier = lastIdxSkier + 1;
        lastTimeSkier = skier_pos_all(lastIdxSkier,1);
        set(plotHandleSkierXY,'XData',[get(plotHandleSkierXY,'XData'), skier_pos_all(lastIdxSkier,2)], ...
            'YData',[get(plotHandleSkierXY,'YData'), skier_pos_all(lastIdxSkier,3)]);
        
        set(plotHandleSkierZ,'XData',[get(plotHandleSkierZ,'XData'), skier_pos_all(lastIdxSkier,1)], ...
            'YData',[get(plotHandleSkierZ,'YData'), skier_pos_all(lastIdxSkier,4)]);
    end
    
    pause(0.001);
end