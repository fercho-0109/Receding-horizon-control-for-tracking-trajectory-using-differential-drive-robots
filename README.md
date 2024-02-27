# Receding-Horizon-Control-for-tracking-trajectory-using-differential-drive-robots

Code developed for "A. Marino, C. Tiriolo, - Receding Horizon Tracking Trajectory Strategy for Feedback Linearized Differential-Drive".  
Research "Concordia University".  

# Sumary.
This repository contains an implementation of a Receding Horizon Control to solve the tracking trajectory problem in mobile robots. The robot used is a differential drive robot which is linearized by using Dynamic Feedback linearization. The key aspect of the control is the management of input constraints, which change across the linearization procedure. The simulation is performed using Matlab, and the validation of the controller is achieved by implementing the control in a Digital Twin of the Qbot2 robot provided by Quanser Company. this control implementation is based on [1] and is part of a master thesis of the owner of this repository.
# Problem Formulation.
Considering the differential drive model, the input constraints set, and a bounded trajectory 𝑟(𝑡),  
Design a feedback control lay [𝜔_𝑅,𝜔_𝐿 ]=𝑔(𝑝(𝑡),𝜃(𝑡),𝑟(𝑡)) such that the tracking error is bounded and [𝜔_𝑅,𝜔_𝐿 ]∈ 𝑈_𝑑,∀𝑡≥0


Given the following LTI system for the differential drive 
𝑥(𝑡+1)=𝐴𝑥(𝑡)+𝐵𝑢(𝑡), 
subject to state and input constraints 
𝑥(𝑡)∈𝒳, ∀𝑡≥0,  u(𝑡)∈𝒰, ∀𝑡≥0, 
Design a feedback control law:  
u(𝑡)=𝑔(𝑥(𝑡)), ∀𝑡≥0  
such that the robot can address the tracking trajectory and obstacle avoidance problems in static but unknowing environments.  
# Prerequisites
- The code was created and tested on the Matlab/Simulink 2023a environment and it requires Ellipsoidal Toolbox ET (https://www.mathworks.com/matlabcentral/fileexchange/21936-ellipsoidal-toolbox-et)
after installation Add the repo folder to the Matlab path.
- Install Quanser interactive labs "Qlabs" (https://es.mathworks.com/matlabcentral/fileexchange/123860-quanser-interactive-labs-for-matlab). This requires a license to get access to the digital twins 
# File description
The repository contains three main folders  
1. **Obstacle avoidance**: This folder contains a replication of the paper [[2](https://ieeexplore.ieee.org/document/10156498)] using Quanser enviroment. This is implemented using different linearization techniques "I/O static feedback linearization" and "Dynamic feedback linearization". Moreover, each one is simulated using Matlab and the Qbot 2e Digital Twin provided by Quanser. 
2. **Tracking Problem**: This folder contains a replication of the paper [[1](https://ieeexplore.ieee.org/document/9956741)] using Quanser enviroment. This is implemented using different linearization techniques "I/O static feedback linearization" and "Dynamic feedback linearization". Moreover, each one is simulated using Matlab and the Qbot 2e Digital Twin provided by Quanser.
3. **Tracking with Obstacle avoidance**: This folder contains the algorithm required to run the Tracking Trajectory with Obstacle avoidance control problem using the Quanser environment and I/O linearization technique.
### Bibliography
[1] Cristian Tiriolo, Giuseppe Franzè, and Walter Lucia. A receding horizon trajectory tracking strategy for input-constrained differential-drive robots via feedback linearization. IEEE Transactions on Control Systems Technology, 31(3):1460–1467, 2023.  
[2] Cristian Tiriolo, Giuseppe Franzè, and Walter Lucia. An obstacle-avoidance receding horizon control scheme for constrained differential-drive robot via dynamic feedback linearization. In 2023 American Control Conference (ACC), pages 1116–1121, 2023.

# Simulations 
### For Matlab Simulation  
Download the respective folder for each case called Matlab_simulation, Then for:
- Tracking problem: run "**Tracking_using_static_lin.m**" for I/O linearization or "**Tracking_using_dynmaic_lin.m**" for Dynamic linearization
- Obstacle Avoidance: run "**Obst_Avo_using_stat_lin.m**" for I/O linearization or "**Obst_Avoid_using_Dyn_lin.m**" for Dynamic linearization
- Tracking with OA: run "**Trackin_and_OA_sta_lin_5**"
### For Quanser Simulation using the Qbot2e Digital Twin.
Download the respective folder for each case called Quanser_simulation. Then, open the Quanser interactive labs and select Qbot 2e.
- **Tracking problem**
  - Setup the position of the robot, go to Options - Change reset location - choose x=-0.25, y=-1.75, rotation=180 deg
  - First, run "**Main_tracking_static_lin.m**" for I/O linearization or "**Main_tracking_using_dyn_lin_Quanser.m**" for Dynamic linearization. To configure the parameters.
  - Second, open and run the Simulink file "**Tracking_with_static_lin.slx**" for I/O linearization or "**Tracking_with_dynamic_lin.slx**" for Dynamic linearization. Then, the robot in the simulator should start to move and follow the trajectory that is the red line in the environment. If the connection with the simulator fails, close the simulator and open it again.
- **Obstacle Avoidance**
  - Setup the position of the robot, go to Options - Change reset location - choose x=0, y=-0.25, rotation=180 deg
  - First, run "**Main_Obst_Avoi_sta_lin.m**" for I/O linearization or "**Main_Obst_Avoi_Dyn_lin.m**" for Dynamic linearization. To configure the parameters and compute the path.
  - Second, open and run the Simulink file "**Obs_Avoi_with_sta_lin.slx**" for I/O linearization or "**Obst_Avoi_with_Dynamic_lin.slx**" for Dynamic linearization. Then, the robot in the simulator should start to move and perform waypoint tracking according to the path planner. If the connection with the simulator fails, close the simulator and open it again.
- **Tracking with OA**
  - Setup the position of the robot, go to Options - Change reset location - choose x=0.5, y=0.5, rotation=135 deg
  - First, run "**Main_track_map_OA_Qbot2.m**" for I/O linearization. To configure the parameters.
  - Second, open and run the Simulink file "**QBot2_Map_Track_OA_sta.slx**" for I/O linearization. Then, the robot in the simulator should start to move and follow the trajectory that is a parabola shape. Open the Video Display block to see the online mapping. If the connection with the simulator fails, close the simulator and open it again.

# Example to run an experiment  
**"Tracking Problem using Dynamic linearization"**
### Matlab simulation 
1. Download the folder Tracking Problem 
2. Open the folders Tracking Problem/TT-Dynamic_linearization/Matlab-simulation
3. Run the Matlab file  "**Tracking_using_dynmaic_lin.m**"
4. The simulation should start showing the following result  
![image](https://github.com/fercho-0109/RHC-Tracking-Trajectory-with-Obstacle-Avoidance/assets/40362695/9da97de6-8f37-4604-bd6f-a36ef1451159)
### Quanser simulation
1. Download the folder Tracking Problem 
2. Open the folders Tracking Problem/TT-Dynamic_linearization/Quanser-simulation
3. Open the Quanser interactive labs and select Qbot 2e.
4. Setup the position of the robot in the virtual environment Qlab, go to Options - Change reset location - choose x=-0.25, y=-1.75, rotation=180 deg
5. Run the Matlab file "**Main_tracking_using_dyn_lin_Quanser.m**"
6. Open and Run the Simulink file "**Tracking_with_dynamic_lin.slx**"
7. The Qbot 2e should start to move following the reference trajectory "red line"  
![image](https://github.com/PreCyseGroup/RHC-Tracking-Trajectory-with-Obstacle-Avoidance/assets/40362695/855b62e5-6ebd-4bf2-a85c-3464a9948a70)







  


