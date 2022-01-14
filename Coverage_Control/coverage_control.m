function [state Goal Lc_target assigned] = coverage_control(clientID,vrep,handle,state,Goal,assigned)
while true
    switch state
        % Unassigned
        case 0 
            % Nếu mà chưa phải ở trong tập assigned được chấp nhận chuyển
            % lên trạng thái tiếp theo
            if isempty(find(assigned == handle(1)))
                state = state+1;
            end
            Lc_target = [];
        
        % Assigned
        case 1
            % Thuật toán di chuyển của Robot
            % Khoảng cách của robot cách điểm đích
            dis = epuck_control(clientID,vrep,handle,Goal);
            
            % Khi cách đích một khoảng <= 0.05(m) cho vận tốc bằng 0 và
            % chuyển lên trạng thái mới
            if dis <= 0.05
                vrep.simxSetJointTargetVelocity(clientID,handle(2),0,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,handle(3),0,vrep.simx_opmode_blocking);
                Goal(1,:) = []; 
                state = state + 1;
            end
            Lc_target = [];
        
        % Occupied
        case 2
            
            % Tạo các điểm vị trí ảo khi đã bị chiếm đóng
            [~,pose] = vrep.simxGetObjectPosition(clientID,handle(1),-1,vrep.simx_opmode_blocking);
            assigned = [assigned;handle(1) pose];
            Lc_target = generate_virtual_target(clientID,vrep,handle,0.3);
            break;
    end
end