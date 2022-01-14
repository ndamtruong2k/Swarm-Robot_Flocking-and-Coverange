%% Distributed Coverage Control for Networked Multi-Robot Systems
clear all;
clc;
% Connect Simulation
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% Number of Robot
ep_num = 17;

%% Initialize 
% Khởi tạo robot và bánh trái, bánh phải của robot
object = string(zeros(11,1));
object(1) = "ePuck#";
object(2) = "ePuck_leftJoint#";
object(3) = "ePuck_rightJoint#";
for i = 4:11
    object(i) = append("ePuck_proxSensor",num2str(i-3)+"#");
end
object = repmat(object,1,ep_num);

% object handle table(size: handles x 1):
% 1,robot's handle
% 2,left joint's handle
% 3, right joint's handle
% 4->11, sensor1->8's handle
handle = zeros(11,ep_num);

% get object handle
for i = 1:size(handle,2)
    for j = 1:size(handle,1)
        [~,handle(j,i)] = vrep.simxGetObjectHandle(clientID,append(convertStringsToChars(object(j,i)),num2str(i-1)),vrep.simx_opmode_blocking); 
    end
end

% initialize streaming operation mode
for i = 1:ep_num
    [~] = vrep.simxGetObjectOrientation(clientID,handle(1,i),-1,vrep.simx_opmode_streaming);
    [~] = vrep.simxGetObjectPosition(clientID,handle(1,i),-1,vrep.simx_opmode_streaming);
        
    for j = 4:11
        [~] = vrep.simxReadProximitySensor(clientID,handle(j,i),vrep.simx_opmode_streaming);
        [~] = vrep.simxGetObjectOrientation(clientID,handle(j,i),handle(1,i),vrep.simx_opmode_streaming);
        [~] = vrep.simxGetObjectPosition(clientID,handle(j,i),handle(1,i),vrep.simx_opmode_streaming);
    end
    [~] = vrep.simxGetObjectVelocity (clientID,handle(1,i),vrep.simx_opmode_streaming);
    
end

%% Setup
% Tạo thông số ban đầu
iterations = 2000000;
Goal = []
state = zeros(ep_num,1);
assigned = [];
targets = [];
landmark = [];

%Số lượng robot chưa assigned
unassigned = ep_num;

% Cho con robot đầu tiên là landmark
state(1) = 2;

%% Loop
for i = 1:iterations
    for j = 1:ep_num
        % Thuật toán coverage control được ứng dụng 
        [state(j), Goal, Lc_target,assigned] = coverage_control(clientID,vrep,handle(:,j),state(j),Goal,assigned);

        % Nếu mà robot đang ở trạng thái Occupied và chưa phải là
        % landmark thì cập nhật robot là landmark
        if (state(j)==2)&&(isempty(find(landmark == handle(1,j))))
            landmark = [landmark handle(1,j)];
            targets = [targets; Lc_target];
        end

        % Giá trị đích mà robot cần đi tới
        Goal = [Goal;targets(1,:)]; 

        % Kiểm tra xem giá trị đi tới có là landmark hay không nếu là landmark
        % thì xóa giá trị đích đó và cập nhật giá trị đích mới
        % Do cảm biến không đo được toàn bộ nên bọn em sử dụng hàm này
        while Checked(Goal,assigned(:,2:4)) == 1
            targets(1,:)=[];
            Goal(1,:)=[];
            Goal = [Goal;targets(1,:)];
        end

        %Nếu điểm đích được chấp nhận thì xóa trong hộp mục tiêu
        targets(1,:) = [];
        
        %In ra các mục tiêu còn lại cần lấp đầy
        disp(targets);
        disp('End.');
        
        %Loại bỏ robot khi không còn là Unassigned
        if(state(j)==2)
            unassigned = unassigned - 1;
        end
    end
    
    %Khi không còn robot nào unassigned  
    if(unassigned <= 0)
        break;
    end
end
vrep.simxFinish(-1);
vrep.delete();
