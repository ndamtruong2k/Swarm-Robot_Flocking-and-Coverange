function test_alignment(ID,vrep,handle,Vel, data)
R = 0.0203; %mm
L = 0.056;
if norm(Vel)==0
    norm(Vel);
    v_des = 0;
%     v_des = 0.01;
    angle = mean(data(:,2));
    om_des = 0.8*angle;
    
    vr = (v_des + L*om_des)/R;
    vl = (v_des - L*om_des)/R;
    vrep.simxSetJointTargetVelocity(ID,handle(2),vl,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(ID,handle(3),vr,vrep.simx_opmode_blocking);
end