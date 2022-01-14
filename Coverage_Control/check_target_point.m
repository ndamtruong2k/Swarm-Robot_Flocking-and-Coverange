function checked = check_target_point(clientID,vrep,handle,targe_point,tolerance)
state = zeros(8,1);
point = zeros(8,3);
for i = 4:11
    [~,state(i-3),point(i-3,:),~,~] = vrep.simxReadProximitySensor(clientID,handle(i),vrep.simx_opmode_buffer);
end
detect = find(state==true);
trans_point = zeros(4,size(detect,2));
if ~isempty(detect)
    for i = 1:size(detect,1)
        [~,sensor_rot] = vrep.simxGetObjectOrientation(clientID,handle(detect(i)+3),handle(1),vrep.simx_opmode_buffer);
        [~,sensor_pos] = vrep.simxGetObjectPosition(clientID,handle(detect(i)+3),handle(1),vrep.simx_opmode_buffer);
        trans_point(:,i) = [Rotation(sensor_rot) sensor_pos'; 0 0 0 1]*[point(detect(i),:) 1]';
        trans_point(1) = 0;            
    end
    trans_point = trans_point(1:3,:);
    checked = 0;
    for i = 1:size(trans_point,2)
        targe_point_ang = atan2(targe_point(2),targe_point(3));
        sensing_point_ang = atan2(trans_point(2,i),trans_point(3,i));
        err = abs(sensing_point_ang-targe_point_ang);
        if err <= tolerance
            checked = 1;
        end
    end
else
    checked = 0;
end