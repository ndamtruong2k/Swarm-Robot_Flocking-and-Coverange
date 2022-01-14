function target = generate_virtual_target(clientID,vrep,handle,rh)
    % Gọi vị trí góc hiện tại của robot sử dụng op_mode_blocking để nhận
    % giá trị hiện tại chính xác của robot tuy nhiên sẽ bị delay khi sử
    % dụng matlab
    [~, ori] = vrep.simxGetObjectOrientation(clientID,handle(1),-1,vrep.simx_opmode_blocking)
    
    % Thiết lập giá trị góc Phi0 ban đầu
    Phi0 = ori(2);
    if (ori(2) > 0)
        if ori(1)>0
            Phi0 = pi - ori(2);
        end
    else
        if ori(1)>0
            Phi0 = -pi - ori(2);
        end
    end
    Phi0 = -Phi0;
    Phih = (2*pi)/6;
    point = [];
    target_check =[];
    for i = 1:6
        Phij = Phi0 + (i-1)*Phih;
        targe_point = [0 rh*sin(Phij) rh*cos(Phij)];
        
        % Sử dụng cảm biến để check xem chỗ tạo điểm có robot bị chiếm đóng
        % chưa nếu có loại bỏ điểm đó nhưng do cảm biến phan bố không đều
        % có những điểm mù mà robot chiếm đóng nhưng robot không cảm nhận
        % được
        if check_target_point(clientID,vrep,handle,targe_point,pi/6)==0
            point = [point; targe_point];
        end
    end
    
    % Đổi từ hệ tọa độ robot sang hệ tọa độ chung
    target = zeros(4,size(point,1));
    for i = 1:size(point,1)
        [~, P] = vrep.simxGetObjectPosition(clientID,handle(1),-1,vrep.simx_opmode_blocking);
        target(:,i) = [Rotation(ori) P';0 0 0 1]*[point(i,:) 1]';
    end
    target = target(1:3,:);
    target = target';
end