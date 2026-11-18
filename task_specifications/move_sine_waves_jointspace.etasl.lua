--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
-- 
--  GNU Lesser General Public License Usage
--  Alternatively, this file may be used under the terms of the GNU Lesser
--  General Public License version 3 as published by the Free Software
--  Foundation and appearing in the file LICENSE.LGPLv3 included in the
--  packaging of this file. Please review the following information to
--  ensure the GNU Lesser General Public License version 3 requirements
--  will be met: https://www.gnu.org/licenses/lgpl.html.
-- 
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Lesser General Public License for more details.

require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")
reqs = require("task_requirements")

task_description = [[Moves every robot joint along its own sine wave around its initial position (joint space, no Cartesian constraints): the target of joint i is q_i(t) = q_i(0) + amplitudes[i]*sin(2*pi*frequencies[i]*t), with t the time since the start of the task, so the motion starts at the current joint positions.
Each joint is driven to its target by a position constraint with feedback gain K_joint (with feedforward of the target velocity). No input channels are needed.
A monitor finishes the task with the event/action "exit" when the task has run for execution_time seconds.
Outputs: time and the position x_tcp, y_tcp, z_tcp [m] of the "tcp_frame" of the robot model w.r.t. the robot base (world) frame.]]

param = reqs.parameters(task_description,{
    reqs.params.array({name="amplitudes", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, description="Amplitude of the sine wave of each joint, in [rad] for revolute joints ([m] for prismatic joints), i.e. the maximum deviation from the initial joint position. One value per joint, in the same order as the joints of the robot model (robot.robot_joints); the number of values must equal the number of joints. 0 keeps that joint at its initial position; a negative value inverts the direction of the wave.", required=true, minimum = -6.28318, maximum=6.28318, minItems = 1}),
    reqs.params.array({name="frequencies", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, description="Frequency of the sine wave of each joint in [Hz] (cycles per second). One value per joint, in the same order as the joints of the robot model (robot.robot_joints); the number of values must equal the number of joints. 0 keeps that joint at its initial position; a negative value inverts the direction of the wave.", required=true, minimum = -6.28318, maximum=6.28318, minItems = 1}),
    reqs.params.scalar({name="execution_time", description="Duration of the task in [s]: when this time has elapsed since the start of the task, the task finishes with the event \"exit\". 0 makes the task finish right after it starts.", default = 5, required=false, minimum = 0}),
    reqs.params.scalar({name="K_joint", description="Feedback gain [1/s] of the joint position constraints that make each joint follow its sine wave. Higher values correct tracking errors faster; 0 means only the target velocity is applied, without correcting position errors.", default = 2, required=false, minimum = 0}),
})

robot = reqs.robot_model({"tcp_frame"})


-- ========================================= PARAMETERS ===================================

amplitudes = param.get("amplitudes")

if #robot.robot_joints ~= #amplitudes then
    error("The number of robot.robot_joints and the specified number of amplitudes must coincide")
end


frequencies = param.get("frequencies")

if #robot.robot_joints ~= #frequencies then
    error("The number of robot.robot_joints and the specified number of frequencies must coincide")
end

execution_time = param.get("execution_time")



-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
for i=1,#robot.robot_joints do
    local current_jnt   = ctx:getScalarExpr(robot.robot_joints[i])

    local tgt = amplitudes[i]*sin(2*math.pi*frequencies[i]*time) + initial_value(time, current_jnt)

    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt - tgt ,
        priority = 2,
        K=param.get("K_joint")
    };

    
end


-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-constant(execution_time)
}

-- Monitor{
--     context=ctx,
--     name='finish_after_on_ended',
--     upper=0.0,
--     actionname='debug',
--     expr=time-constant(1)
-- }


tcp_frame = robot.getFrame("tcp_frame")

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(tcp_frame)))
ctx:setOutputExpression("y_tcp",coord_y(origin(tcp_frame)))
ctx:setOutputExpression("z_tcp",coord_z(origin(tcp_frame)))
