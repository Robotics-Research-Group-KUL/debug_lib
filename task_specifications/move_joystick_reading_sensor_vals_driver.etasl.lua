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


-- ========================================= PARAMETERS ===================================
task_description = [[Debug variant of the joystick teleoperation task: moves a robot frame (task_frame) with the linear and/or angular velocity of a 6D joystick (a.k.a. spacemouse) while also reading sensor values provided by the robot driver.
The joystick twist is read from the input channel named by joystick_input_channel (default "joystick_input") (Twist, typically fed by a TwistInputHandler, e.g. from /spacenav/twist), multiplied by linear_scale/angular_scale and imposed as a desired velocity of task_frame, expressed along the axes of the robot base (world) frame of the robot model; the constraints only prescribe velocity (feedforward, no position feedback).
If activate_linear (activate_angular) is false, the position (orientation) of task_frame is instead kept at its initial value.
It also creates the driver input channels <joint_vel_prefix>0..3, <joint_torque_prefix>0..3, <joint_current_prefix>0..3 (Scalar; by default joint_vel_0..3, joint_torque_0..3, joint_current_0..3), and (names given by the *_channel parameters, by default) task_frame_pos (Vector), task_frame_quat (Rotation), task_frame_twist (Twist), task_frame_wrench (Wrench), base_pos (Vector), base_quat (Rotation) and base_twist (Twist); they are only read (for debugging) and do not influence the motion.
There is no monitor: the task runs until it is stopped externally. Outputs: time and the position x_tf, y_tf, z_tf [m] of task_frame w.r.t. the robot base frame.]]

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="linear_scale", description="Dimensionless factor multiplying the linear velocity [m/s] coming from the joystick before it is imposed on task_frame. 1 uses the joystick values unchanged, values between 0 and 1 slow the motion down, 0 blocks the linear motion and a negative value inverts its direction. Only used if activate_linear is true.", default = 1, required=false}),
    reqs.params.scalar({name="angular_scale", description="Dimensionless factor multiplying the angular velocity [rad/s] coming from the joystick before it is imposed on task_frame. 1 uses the joystick values unchanged, values between 0 and 1 slow the rotation down, 0 blocks the rotation and a negative value inverts its direction. Only used if activate_angular is true.", default = 1, required=false}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (defined in the robot model, e.g. the tool center point) that is moved by the joystick. Its velocities are expressed along the axes of the robot base (world) frame.", default = "tcp_frame", required=false}),
    reqs.params.bool({name="activate_linear", description="If true, the linear velocity of the joystick controls the linear velocity of task_frame. If false, the position of task_frame is kept at its initial value.", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the angular velocity of the joystick controls the angular velocity of task_frame. If false, the orientation of task_frame is kept at its initial value.", default = false, required=true}),
    reqs.params.scalar({name="weight_linear", description="Weight (dimensionless) of the translational constraints of task_frame: the three joystick velocity constraints if activate_linear is true, or the constraint that keeps the initial position otherwise. It sets their relative importance w.r.t. other constraints of the same priority; 0 disables them.", default = 1, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_angular", description="Weight (dimensionless) of the rotational constraints of task_frame: the three joystick angular velocity constraints if activate_angular is true, or the constraint that keeps the initial orientation otherwise. It sets their relative importance w.r.t. other constraints of the same priority; 0 disables them.", default = 1, required=false, minimum = 0}),
    reqs.params.scalar({name="K_keep_translation", description="Feedback gain [1/s] of the constraint that keeps the position of task_frame at its initial value (only used if activate_linear is false). Higher values correct deviations faster; 0 means no correction of position errors.", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="K_keep_rotation", description="Feedback gain [1/s] of the constraint that keeps the orientation of task_frame at its initial value (only used if activate_angular is false). Higher values correct deviations faster; 0 means no correction of orientation errors.", default = 4, required=false, minimum = 0}),

    reqs.params.string({name="joystick_input_channel", description="Name of the input channel (type twist) from which the joystick velocities are read: the varname of the input handler that provides it (e.g. a TwistInputHandler subscribed to /spacenav/twist). Change it to use another joystick or input handler", default = "joystick_input", required=false}),
    reqs.params.string({name="joint_vel_prefix", description="Prefix of the names of the four input channels (type scalar) with the joint velocities read from the robot driver: the channels are <prefix>0, <prefix>1, <prefix>2 and <prefix>3. Only read (debugging), they do not influence the motion", default = "joint_vel_", required=false}),
    reqs.params.string({name="joint_torque_prefix", description="Prefix of the names of the four input channels (type scalar) with the joint torques read from the robot driver: the channels are <prefix>0, <prefix>1, <prefix>2 and <prefix>3. Only read (debugging), they do not influence the motion", default = "joint_torque_", required=false}),
    reqs.params.string({name="joint_current_prefix", description="Prefix of the names of the four input channels (type scalar) with the joint currents read from the robot driver: the channels are <prefix>0, <prefix>1, <prefix>2 and <prefix>3. Only read (debugging), they do not influence the motion", default = "joint_current_", required=false}),
    reqs.params.string({name="task_frame_pos_channel", description="Name of the input channel (type vector) with the task frame pos read from the robot driver. Only read (debugging), it does not influence the motion", default = "task_frame_pos", required=false}),
    reqs.params.string({name="task_frame_quat_channel", description="Name of the input channel (type rotation) with the task frame quat read from the robot driver. Only read (debugging), it does not influence the motion", default = "task_frame_quat", required=false}),
    reqs.params.string({name="task_frame_twist_channel", description="Name of the input channel (type twist) with the task frame twist read from the robot driver. Only read (debugging), it does not influence the motion", default = "task_frame_twist", required=false}),
    reqs.params.string({name="task_frame_wrench_channel", description="Name of the input channel (type wrench) with the task frame wrench read from the robot driver. Only read (debugging), it does not influence the motion", default = "task_frame_wrench", required=false}),
    reqs.params.string({name="base_pos_channel", description="Name of the input channel (type vector) with the base pos read from the robot driver. Only read (debugging), it does not influence the motion", default = "base_pos", required=false}),
    reqs.params.string({name="base_quat_channel", description="Name of the input channel (type rotation) with the base quat read from the robot driver. Only read (debugging), it does not influence the motion", default = "base_quat", required=false}),
    reqs.params.string({name="base_twist_channel", description="Name of the input channel (type twist) with the base twist read from the robot driver. Only read (debugging), it does not influence the motion", default = "base_twist", required=false}),
})
linear_scale    = constant(param.get("linear_scale"))
angular_scale    = constant(param.get("angular_scale"))

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

-- ========================================= Variables coming from topic input handlers ===================================
joystick_input   = ctx:createInputChannelTwist(param.get("joystick_input_channel"))


joint_vel_0   = ctx:createInputChannelScalar(param.get("joint_vel_prefix") .. "0")
joint_vel_1   = ctx:createInputChannelScalar(param.get("joint_vel_prefix") .. "1")
joint_vel_2   = ctx:createInputChannelScalar(param.get("joint_vel_prefix") .. "2")
joint_vel_3   = ctx:createInputChannelScalar(param.get("joint_vel_prefix") .. "3")

joint_torque_0   = ctx:createInputChannelScalar(param.get("joint_torque_prefix") .. "0")
joint_torque_1   = ctx:createInputChannelScalar(param.get("joint_torque_prefix") .. "1")
joint_torque_2   = ctx:createInputChannelScalar(param.get("joint_torque_prefix") .. "2")
joint_torque_3   = ctx:createInputChannelScalar(param.get("joint_torque_prefix") .. "3")

joint_current_0   = ctx:createInputChannelScalar(param.get("joint_current_prefix") .. "0")
joint_current_1   = ctx:createInputChannelScalar(param.get("joint_current_prefix") .. "1")
joint_current_2   = ctx:createInputChannelScalar(param.get("joint_current_prefix") .. "2")
joint_current_3   = ctx:createInputChannelScalar(param.get("joint_current_prefix") .. "3")


task_frame_pos   = ctx:createInputChannelVector(param.get("task_frame_pos_channel"))
task_frame_quat   = ctx:createInputChannelRotation(param.get("task_frame_quat_channel"))
task_frame_twist   = ctx:createInputChannelTwist(param.get("task_frame_twist_channel"))
task_frame_wrench   = ctx:createInputChannelWrench(param.get("task_frame_wrench_channel"))
base_pos   = ctx:createInputChannelVector(param.get("base_pos_channel"))
base_quat   = ctx:createInputChannelRotation(param.get("base_quat_channel"))
base_twist   = ctx:createInputChannelTwist(param.get("base_twist_channel"))
-- joystick_input = twist(vector(0,0,-0.05),vector(0,0,0))


-- =============================== INSTANTANEOUS FRAME ==============================


tf_inst = task_frame
-- tf_inst = inv(make_constant(task_frame))*task_frame





if param.get("activate_linear") then

    desired_vel_x = coord_x(transvel(joystick_input))*linear_scale
    desired_vel_y = coord_y(transvel(joystick_input))*linear_scale
    desired_vel_z = coord_z(transvel(joystick_input))*linear_scale
    -- Translation velocities
    Constraint{
        context = ctx,
        name    = "x_velocity",
        expr    = coord_x(origin(tf_inst)),
        target  = desired_vel_x*time,
        K       = 0,
        weight  = param.get("weight_linear"),
        priority= 2
    };
    
    Constraint{
        context = ctx,
        name    = "y_velocity",
        expr    = coord_y(origin(tf_inst)),
        target  = desired_vel_y*time,
        K       = 0,
        weight  = param.get("weight_linear"),
        priority= 2
    };
    
    Constraint{
        context = ctx,
        name    = "z_velocity",
        expr    = coord_z(origin(tf_inst)),
        target  = desired_vel_z*time,
        K       = 0,
        weight  = param.get("weight_linear"),
        priority= 2
    };
else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(tf_inst),
        target  = initial_value(time, origin(tf_inst)),
        K       = param.get("K_keep_translation"),
        weight  = param.get("weight_linear"),
        priority= 2
    };

end

if param.get("activate_angular") then

    desired_omega_x = coord_x(rotvel(joystick_input))*angular_scale
    desired_omega_y = coord_y(rotvel(joystick_input))*angular_scale
    desired_omega_z = coord_z(rotvel(joystick_input))*angular_scale

    -- Orientation velocities
    Constraint{
        context = ctx,
        name    = "x_angular",
        expr    = coord_x(getRotVec(rotation(tf_inst))) - desired_omega_x*time,
        K       = 0,
        weight  = param.get("weight_angular"),
        priority= 2
    };

    Constraint{
        context = ctx,
        name    = "y_angular",
        expr    = coord_y(getRotVec(rotation(tf_inst))) - desired_omega_y*time,
        K       = 0,
        weight  = param.get("weight_angular"),
        priority= 2
    };

    Constraint{
        context = ctx,
        name    = "z_angular",
        expr    = coord_z(getRotVec(rotation(tf_inst))) - desired_omega_z*time,
        K       = 0,
        weight  = param.get("weight_angular"),
        priority= 2
    };

else
    Constraint{
        context = ctx,
        name    = "keep_rotation_constant",
        expr    = rotation(tf_inst)*initial_value(time, rotation(tf_inst)),
        K       = param.get("K_keep_rotation"),
        weight  = param.get("weight_angular"),
        priority= 2
    };


end

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))




-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)
