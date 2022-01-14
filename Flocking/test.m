vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

Goal= [1 1 0];
ep_num = 10;

object = string(zeros(11,1));
object(1) = "ePuck#";
object(2) = "ePuck_leftJoint#";
object(3) = "ePuck_rightJoint#";
for i = 4:11
    object(i) = append("ePuck_proxSensor",num2str(i-3)+"#");
end
object = repmat(object,1,ep_num);
handle = zeros(11,ep_num);

cuboid_handle = vrep.simxGetObjectHandle(clientID,'Cuboid',vrep.simx_opmode_blocking);
for i = 1:size(handle,2)
    for j = 1:size(handle,1)

        [~,handle(j,i)] = vrep.simxGetObjectHandle(clientID,append(convertStringsToChars(object(j,i)),num2str(i-1)),vrep.simx_opmode_blocking); 
    end
end


iterations = 2000000;
for i = 1:iterations
    for j = 1:ep_num
        start = tic;
        if i ==1
            epuck_control(clientID,handle(:,j),handle(1,:),Goal,0,vrep);
        else
            
            epuck_control(clientID,handle(:,j),handle(1,:),Goal,1,vrep)
        end
        toc(start)
    end
end
vrep.simxFinish(-1);
vrep.delete();