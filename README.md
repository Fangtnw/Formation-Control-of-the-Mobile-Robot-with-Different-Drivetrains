# Formation Control of the Mobile Robot with Different Drivetrains
This project is about Cooperative transport of the mobile robot with different drivetrains. Implemented for cooperative of 2 mobile robot by looking it as 1 long car-like robot using leader-follower concept.

1. Differential drive 
2. Ackermann drive 
3. Omnidirectional (Mecanum wheel)
   
![image](https://github.com/user-attachments/assets/ceaae877-36a2-4753-92e8-3c2df2b4400c)

Core function implemented
- Hybrid A* Planner
- Pure pursuit controller
- Localization using Nav2_AMCL (leader robot)
- Relative localization using Aruco Tag (Follower robot)
![DDbots_diagram](https://github.com/user-attachments/assets/ac3cfcb8-bd8a-4da3-a344-ec4522ccc5b6)

# Aruco tag for follower robot's relative loacalization
![image](https://github.com/user-attachments/assets/4353ca18-aa8b-49eb-9965-55667da5bc1e)


# Formation controller equation 
formation controller based on leader-follower structure
![image](https://github.com/user-attachments/assets/edb5299c-9a13-4a95-b6e6-25183e795149)


# Simulation Experiment
![image](https://github.com/user-attachments/assets/17afdc1d-209d-4780-b47b-15af26b0fce7)

# Real-world Experiment
![image](https://github.com/user-attachments/assets/24d01a00-a2ac-40f9-9eb5-27bcff926794)

trajectory output

![image](https://github.com/user-attachments/assets/cd844232-7341-480b-8295-4812d4b1a161)


Reference

https://github.com/YDLIDAR/ydlidar_ros2_driver.git

https://github.com/zhm-real/CurvesGenerator

https://github.com/RajPShinde/Hybrid-A-Star
