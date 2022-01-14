function  epuck_control(ID,handle,robots_handle,Goal, mode,vrep)


state = zeros(8,1);
point = zeros(8,3);
if ID>-1
    if mode == 0
%         mode ==0 : khoi tao lan dau tien chay(thu tuc trong remote API)
        [~] = vrep.simxGetObjectOrientation(ID,handle(1),-1,vrep.simx_opmode_streaming);
        [~] = vrep.simxGetObjectPosition(ID,handle(1),-1,vrep.simx_opmode_streaming);
        
        for i = 4:11
            [~] = vrep.simxReadProximitySensor(ID,handle(i),vrep.simx_opmode_streaming);
            [~] = vrep.simxGetObjectOrientation(ID,handle(i),handle(1),vrep.simx_opmode_streaming);
            [~] = vrep.simxGetObjectPosition(ID,handle(i),handle(1),vrep.simx_opmode_streaming);
        end
        vrep.simxGetObjectVelocity (ID,handle(1),vrep.simx_opmode_streaming);
        for i = 1:length(robots_handle)
            [~] = vrep.simxGetStringSignal(ID,num2str(robots_handle(i)),vrep.simx_opmode_streaming);
        end
        
    else
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
            data = [];
            for i = 1:size(detect,1)
                x = obj_handle(detect(i));
                detected_handle = 0;
                while x~= -1
                    [~,x] = vrep.simxGetObjectParent (ID,x,vrep.simx_opmode_blocking);
                    if x~= -1
                        detected_handle = x;
                    end
                end
                if ~isempty(find(robots_handle==detected_handle))
                    [~,packed_data] = vrep.simxGetStringSignal(ID,num2str(detected_handle),vrep.simx_opmode_buffer);
                    unpack_data = vrep.simxUnpackFloats(packed_data);
                    data = [data;unpack_data];
                end
            end
            a = (1/5)*alignment(data,ori);
            s = (30)*seperation(trans_point',0.15);
            g = (1/(3*norm(Goal_in_robot_frame)))*Goal_in_robot_frame';
            c = (1/3)*cohension(trans_point');
            
           
            
%             tune he s=o o day(here)
            Vel = c + s + a + g;
            Vel(isnan(Vel)) =0;
            Vel(1) = 0;
            theta = acos(dot(Vel,[0 0 1])/norm(Vel));
            if Vel(2) > 0
                theta = -theta; 
            end

            theta(isnan(theta)) =0;
            [vl,vr] = move_epuck(ID,vrep,handle,theta, Vel);
%             truyen du lieu cua robot di(van toc dai va huong cua robot so voi world frame)
            if (ori(2) > 0)
                if ori(1)>0
                    ori(2) = pi - ori(2);
                end
            else
                if ori(1)>0
                    ori(2) = -pi - ori(2);
                end
            end
            data_sent = [norm([vl vr]) ori(2)];
            data_sent = vrep.simxPackFloats(data_sent);
            vrep.simxSetStringSignal( ID,num2str(handle(1)),data_sent,vrep.simx_opmode_oneshot);
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
            if ori(2) > 0
                if ori(1)>0
                    ori(2) = pi - ori(2);
                end
            else
                if ori(1)>0
                    ori(2) = -pi - ori(2);
                end
            end
            
            data_sent = [norm([vr vl]) ori(2)];
            data_sent = vrep.simxPackFloats(data_sent);
            vrep.simxSetStringSignal( ID,num2str(handle(1)),data_sent,vrep.simx_opmode_oneshot);
        end
    end    
end



