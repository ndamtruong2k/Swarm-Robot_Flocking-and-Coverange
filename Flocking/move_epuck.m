function [vl vr] = move_epuck(ID,vrep,handle,angle, Vel)


R = 0.0203; %mm
L = 0.056;

[~,lin,ang] = vrep.simxGetObjectVelocity (ID,handle(1),vrep.simx_opmode_buffer)
if norm(Vel)~=0
    v_des = norm(Vel)/15;
%     v_des = 0.05;
    om_des = 0.3*angle;
    
    vr = (v_des + L*om_des)/R;
    vl = (v_des - L*om_des)/R;
    vrep.simxSetJointTargetVelocity(ID,handle(2),vl,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(ID,handle(3),vr,vrep.simx_opmode_blocking);
    vr = vr*R;
    vl = vl*R;
else
    vr = 0;
    vl = 0;
    vrep.simxSetJointTargetVelocity(ID,handle(2),0,vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(ID,handle(3),0,vrep.simx_opmode_blocking);
end

