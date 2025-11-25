# 🤖 Project: Fly your drone

Course: Robotics Lab

Student: Leonardo Riccardi / P38000358

# 🎯 Project Objective
The overarching goal of this project is to extend and validate PX4-Autopilot functionalities through three key development efforts: hardware configuration, control logic modification, and trajectory planning. This includes integrating a custom UAV model and modifying the force land node to implement a critical safety exception concerning pilot interruptions. The core technical requirement involves developing a smooth trajectory planner (non-zero intermediate velocity) in Offboard mode, with its precision and continuity rigorously proven by analyzing logged velocity and acceleration data.

##⚙️ Prerequisites and Setup
Before beginning, you must have QGroundControl (QGround) and PlotJuggler installed.

#Build 
Clone this package into the src folder of your ROS 2 workspace.
```
https://github.com/leonardoricc2002/rl25_homework_3.git
```
# Clone this package in the src folder of your ROS 2 workspace.
Build and source the setup files.

```
colcon build
```
```
source install/setup.bash
```
## 🚀 HOW TO LAUNCH
Terminal 1: PX4 SITL.Launch the drone in Gazebo. First, navigate to the dedicated PX4-Autopilot folder.
#Ensure QGroundControl is kept open.

```
cd src/PX4-Autopilot/
make px4_sitl gz_my_quadrotor
```
## 📊 ACTUATOR PLOT IN REAL-TIME
Terminal 2: Initialize DDS Bridge. Source the setup files and execute the command. DDS_run.sh initiates the DDS communication bridge, which is essential for enabling the exchange of flight-critical data between the PX4 flight simulator and the complementary control nodes developed within the ROS 2 ecosystem.

```
source install/setup.bash
```
```
cd src/aerial_robotics/
. DDS_run.sh
```
Terminal 3: PlotJuggler for Real-Time Actuator Output.Source the setup files in a new terminal and open PlotJuggler.
```
source install/setup.bash
```
```
cd src/aerial_robotics/ros2_ws/
ros2 run plotjuggler plotjuggler
```
# Visualization Setup
For real-time visualization:

1.Press Start.

2.Click on the Topic Name "fmu/out/actuator_outputs" and press OK.

3.Press '+' next to Custom Series.

4.Under fmu->out->actuator_outputs->output, drag output[0] into Input timeseries, provide a name, and click "Create New Timeseries".

5.Repeat this for output[1], output[2], and output[3].

6.Drag all four Actuator Outputs onto the graph.

7.In QGroundControl, execute a takeoff and move the drone with the joysticks to observe the real-time graph of the drone's actuators.

Tip: Increase the buffer size to view the complete trajectory trace.

## 🚨 MONITORING DRONE ALTITUDE (Force Land Implementation)
Keep your drone running in Gazebo with make px4_sitl gz_my_quadrotor and keep QGroundControl open.

Terminal 2: Initialize DDS Bridge.Source the setup files and execute DDS_run.sh.
```
source install/setup.bash
```
```
cd src/aerial_robotics/
. DDS_run.sh
```
Terminal 3: Start ROS 2 Bag Recording .Launch the ROS 2 bag command to record the drone's flight path into a folder named proof_flight.

```
source install/setup.bash
cd src/aerial_robotics/ros2_ws
```
```
ros2 bag record -o proof_flight /fmu/out/vehicle_local_position /fmu/out/manual_control_setpoint
```
Terminal 4: Launch Force Land Node .Source the setup files in a new terminal and launch the node that monitors the drone's altitude.

```
source install/setup.bash
```
```
ros2 launch force_land force_land.launch.py
```
#Testing the Safety Exception

1.In QGroundControl, execute a Takeoff and wait until it reaches 10 meters.

2.Use the left joystick to raise the drone's altitude to approximately 70 ft (or the set height limit). You will observe that the force land logic engages and begins to bring the drone down.

3.Use the right joystick (to provide manual roll/pitch input), and you will see a message indicating the pilot has regained control (the safety exception is triggered).

4.If you do not use the right joystick to regain control, the drone will continue its landing sequence.

5.After the sequence, press Ctrl+C in the third terminal to stop the recording.

Terminal 5: Analyze Logged Data with PlotJuggler

Source the setup files in the workspace and launch PlotJuggler.

```
source install/setup.bash
```
```
cd src/aerial_robotics/ros2_ws
ros2 run plotjuggler plotjuggler
```
# Data Visualization

1.Open PlotJuggler and press the button next to Data to load the data from the proof_flight folder.

2.Select metadata.yaml, choose /fmu/out/vehicle_local_position and /fmu/out/manual_control_setpoint, and click OK.

3.Press '+' near Custom Series.

4.Under /fmu/out/manual_control_setpoint, load throttle, roll, and pitch by dragging them into Input timeseries, choose a name, and click "Create New Timeseries".

5.Repeat the procedure, but load z from /fmu/out/vehicle_local_position (set the return value to be positive, e.g., return -value; since z is negative in NED coordinates).

6.Drag all these series onto the graph and analyze the system's behavior.

## 💖 DRONE EXECUTES A HEART-SHAPED TRAJECTORY
Keep your drone running in Gazebo with make px4_sitl gz_my_quadrotor and keep QGroundControl open.

Terminal 2.Source the setup files and execute DDS_run.sh.
```
source install/setup.bash
```
```
cd src/aerial_robotics/
. DDS_run.sh
```

Terminal 3. First, make the drone takeoff in QGroundControl. Then, launch the ROS 2 bag command to record the drone's flight path into a folder called heart_trajectory.
```
source install/setup.bash
cd src/aerial_robotics/ros2_ws 
```
```
ros2 bag record -o heart_trajectory /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/trajectory_setpoint
```

Terminal 4.Launch Trajectory Node . In your workspace, execute this command. The drone will execute a heart-shaped trajectory.
```
source install/setup.bash
```
```
ros2 run offboard_rl go_to_point
```
Terminal 5. Analyze Trajectory Data with PlotJuggler. Source the setup files in the workspace. After the drone completes the trajectory, launch PlotJuggler.
```
source install/setup.bash
```
```
cd src/aerial_robotics/ros2_ws
ros2 run plotjuggler plotjuggler
```
#Data Visualization and Analysis (Precision and Continuity)

1.Open PlotJuggler and press the button next to Data to load the data from heart_trajectory.

2.Select metadata.yaml, choose /fmu/out/vehicle_local_position and /fmu/out/vehicle_attitude, and click OK.

#Plot 1: 2D Trajectory (Shape Verification)

1.In the Timeseries List, navigate to /fmu/out/vehicle_local_position.

2.Click x, then hold CTRL on the keyboard and click y.

3.Right-click on the selected series and drag them onto the graph to display the 2D trajectory covered by the drone.

#Plot 2: Velocity Magnitude (Continuity Proof)

1.Split another plot.

2.Click '+' next to Custom Series.

3.Drag vx into Input timeseries, and vy and vz into Additional source timeseries.

4.Give it a name, and for the function, use the following to calculate the total speed:
return sqrt(value*value + v1*v1 + v2*v2)^(1/2) 

5.Click "Create New Timeseries".

#Plot 3: Acceleration Magnitude (Smoothness Proof)

Perform the same procedure as Plot 2, but using ax, ay, and az to calculate the total acceleration with the same formula.

#Plot 4: Yaw Angle (Orientation)

1.For the Yaw angle, go to /fmu/out/vehicle_attitude/q[].

2.Drag q[0] into Input timeseries, and q[1], q[2], and q[3] into Additional source timeseries.

3.Give it a name.

4.In the Function Library, double-click quat_to_yaw to apply the correct quaternion-to-yaw conversion function.

5.Click "Create New Timeseries".

6.Drag the Velocity Magnitude, Acceleration Magnitude, and Yaw plots onto separate graphs and analyze the four generated plots to verify the trajectory's precision and smoothness.
