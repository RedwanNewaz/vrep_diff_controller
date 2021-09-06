local matrix_mul = function( m1, m2 )
    -- this function is borrowed from https://github.com/davidm/lua-matrix/blob/master/lua/matrix.lua
	-- multiply rows with columns
	local mtx = {}
	for i = 1,#m1 do
		mtx[i] = {}
		for j = 1,#m2[1] do
			local num = m1[i][1] * m2[1][j]
			for n = 2,#m1[1] do
				num = num + m1[i][n] * m2[n][j]
			end
			mtx[i][j] = num
		end
	end
	return mtx
end

-----------------------------------------------------------DiffController----------------------------------------------------------

DiffController= {_TYPE='module', _NAME='DiffController', _VERSION='0.9.05.2021'}


function DiffController:new(o, robotName, goalObjectName, wheelMotorNames, length, radius)
    --[[
        robotName - name of the robot is required for state estimation 
        goalObjectName - the goal object name is required for error calculation 
        wheelMotorNames - this is a list which contains left and right wheel joints 
        this list must have a specfic order. For instance, for two wheel robot wheelMotorNames 
        should contain left wheel first then right wheel information. Similarly, for four wheel robot 
        the order should be as follows: 'joint_front_left_wheel', 'joint_front_right_wheel', 'joint_back_right_wheel', 'joint_back_left_wheel'
        radius - discuss with the specific robot manual to learn more about wheel radius
        length - the distance between two wheels.  Discuss with the specific robot manual to learn more about it. 
    ]]
    o = o or {}
    setmetatable(o, self)
    self.__index = self
    self.length = length 
    self.radius = radius 
    self.goal = sim.getObjectHandle(goalObjectName, -1)
    self.robot = sim.getObjectHandle(robotName, -1)
    self.motorHandles = {}
    for i = 1, #wheelMotorNames do 
        self.motorHandles[i] = sim.getObjectHandle(wheelMotorNames[i],-1)
    end
    self.__initialized = false
    return o
 end

 function DiffController:setGains(Kp_rho, Kp_alpha, Kp_beta)
    self.Kp_rho = Kp_rho
    self.Kp_alpha = Kp_alpha
    self.Kp_beta = Kp_beta
    self.__initialized = true

 end
 

function DiffController:unitodiff(v, w)
    local vR, vL, length, radius 
     
    vR = (2*v + w * self.length) / ( 2 * self.radius)
    vL = (2*v - w * self.length) / ( 2 * self.radius)
    return vR, vL
end 


function DiffController:euler_from_quaternion(q)
        --[[
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        ]]
        local x, y, z, w, t0, t1, t2, t3, t4, roll_x, pitch_y, yaw_z
        
        x, y, z, w = q[1], q[2], q[3], q[4]
 
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        
        if t2 > 1.0 then 
            t2 = 1.0
        end
        if t2 < -1.0 then
            t2 = -1.0
        end       
        
        pitch_y = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return {roll_x, pitch_y, yaw_z}
end



function DiffController:get_state(objHandle)
    local pos = sim.getObjectPosition(objHandle,-1)
    local quat = sim.getObjectQuaternion(objHandle,-1)
    local euler = self:euler_from_quaternion(quat)
    return {pos[1], pos[2], euler[3] + math.pi/2}
end 




function DiffController:computeUnicycleControl(x, goal)

    local x_diff, y_diff, theta, theta_goal, rho, alpha, beta, v, w
    

    x_diff = goal[1] - x[1]
    y_diff = goal[2] - x[2]

    theta_goal = goal[3]
    theta = math.mod((x[3] - math.pi/2), (2 * math.pi)) 

    rho = math.sqrt(x_diff * x_diff + y_diff * y_diff)
    alpha = (math.atan2(y_diff, x_diff)
             - theta + math.pi) % (2 * math.pi) - math.pi
    beta = (theta_goal - theta - alpha + math.pi) % (2 * math.pi) - math.pi

    v = self.Kp_rho * rho
    w = self.Kp_alpha * alpha + self.Kp_beta * beta

    if alpha > math.pi / 2 or alpha < -math.pi / 2 then
        v = -v
    end
    return v, w

end



function DiffController:update()

    if not self.__initialized then 
        print('controller is not initialized with gains')
        return
    end

    local x, g,  v, w, velRight, velLeft
    x = self:get_state(self.robot)
    g = self:get_state(self.goal)
    v, w = self:computeUnicycleControl(x, g)
    velRight, velLeft = self:unitodiff(v, w)

    if #self.motorHandles == 2 then           
        sim.setJointTargetVelocity(self.motorHandles[1], velLeft)
        sim.setJointTargetVelocity(self.motorHandles[2], velRight)
    elseif #self.motorHandles == 4 then -- 4 wheel drives 
        local wheel_spin = {{velLeft}, {velRight}}
        local B = {{1, 0}, {0, -1}, {0, -1}, {1, 0}}
        wheel_spin = matrix_mul(B, wheel_spin)
        sim.setJointTargetVelocity(self.motorHandles[1],    wheel_spin[1][1])
        sim.setJointTargetVelocity(self.motorHandles[2],    wheel_spin[2][1])
        sim.setJointTargetVelocity(self.motorHandles[3],    wheel_spin[3][1])
        sim.setJointTargetVelocity(self.motorHandles[4],    wheel_spin[4][1])
    end 

 end