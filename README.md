# MATLAB-Fleet-Manager
"Autonomous Multi-Robot Path Planning and Deadlock Resolution Simulation"
# 🤖 Fleet Manager: Autonomous Multi-Robot Simulator

**Developer:** Emirhan Solak  
**Department:** Computer Engineering, Kocaeli University  
**Project Type:** MATLAB App Designer (Programmatic UI)

## 🎯 Overview
This project is a comprehensive simulation dashboard developed to manage multiple autonomous robots in a dynamic environment. It features real-time path planning, obstacle avoidance, and deadlock resolution mechanisms.

## 🚀 Features
- **Dynamic Map Generation:** Creates random environments with L-shaped, U-shaped, and block obstacles.
- **Path Planning Algorithms:**
  - 🌟 **A* (A-Star):** Optimal pathfinding.
  - ⚡ **Dijkstra:** Guaranteed shortest path.
  - 🎲 **PRM (Probabilistic Road Map):** For complex and narrow environments.
- **Fail-Safe Mechanism:** Automatically switches to A* if PRM fails to find a path.
- **Deadlock Management:** Robots detect each other within 2.5m and use a priority-based "Wait" logic to avoid collisions.
- **Real-Time Analytics:** Live graphs for Linear Velocity ($v$) and Wheel Velocities ($v_L, v_R$).

## 🛠️ How to Run
1. Ensure you have **MATLAB** installed.
2. Download the `FleetManager.m` file.
3. Open MATLAB and navigate to the folder.
4. Run the following command in the Command Window:
   ```matlab
   FleetManager
