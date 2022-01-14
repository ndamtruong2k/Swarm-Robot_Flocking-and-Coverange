function  distance = epuck_control(ID,vrep,handle,Goal)


state = zeros(8,1);
point = zeros(8,3);
if ID>-1
    
%         chuyen diem dich(Goal) tu world frame sang robot frame
        [~, ori] = vrep.simxGetObjectOrientation(ID,handle(1),-1,vrep.simx_opmode_buffer);
        
        [~, target_pos] = vrep.simxGetObjectPosition(ID,handle(1),-1,vrep.simx_opmode_buffer);
        Goal_in_robot_frame = inv([Rotation(ori) target_pos';0 0 0 1])*[Goal 1]';
        Goal_in_robot_frame = Goal_in_robot_frame(1:3,1);
        Goal_in_robot_frame(1) = 0;
        obj_handle = zeros(8,1);
        
%         doc cam bien
        for i = 4:11
            [~,state(i-3),point(i-3,:),obj_handle(i-3),~] = vrep.simxReadProximitySensor(ID,handle(i),vrep.simx_opmode_buffer);
        end
      
        detect = find(state==true);
        trans_point = zeros(4,size(detect,2));
%         chuyen tu sensor frame sang robot frame
        if ~isempty(detect)
            for i = 1:size(detect,1)
                [~,sensor_rot] = vrep.simxGetObjectOrientation(ID,handle(detect(i)+3),handle(1),vrep.simx_opmode_buffer);
                [~,sensor_pos] = vrep.simxGetObjectPosition(ID,handle(detect(i)+3),handle(1),vrep.simx_opmode_buffer);
                
                trans_point(:,i) = [Rotation(sensor_rot) sensor_pos'; 0 0 0 1]*[point(detect(i),:) 1]';
                trans_point(1) = 0;
                
            end
            trans_point = trans_point(1:3,:);
            
%             nhan data tu cac neibor
            
            s = (30)*seperation(trans_point',0.1);
            g = (1/(3*norm(Goal_in_robot_frame)))*Goal_in_robot_frame';            
           
            
%             tune he s=o o day(here)
            Vel = 3*s + g;
            Vel(isnan(Vel)) =0;
            Vel(1) = 0;
            theta = acos(dot(Vel,[0 0 1])/norm(Vel));
            if Vel(2) > 0
                theta = -theta; 
            end

            theta(isnan(theta)) =0;
            [vl,vr] = move_epuck(ID,vrep,handle,theta, Vel);
            distance = norm(Goal_in_robot_frame);
            
        else
%             neu sensor khong detect duoc gi -> di chuyen toi dich(Goal)
            Vel = (1/(2*norm(Goal_in_robot_frame)))*Goal_in_robot_frame';
            Vel(isnan(Vel)) =0;
            Vel(1) = 0;
            theta = acos(dot(Vel,[0 0 1])/norm(Vel));
            if Vel(2) > 0
                theta = -theta; 
            end
            theta(isnan(theta)) =0;
            [vl, vr] = move_epuck(ID,vrep,handle,theta, Vel);
            distance = norm(Goal_in_robot_frame);
        end
        
end



