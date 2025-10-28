# Design Optimization and Trajectory Planning of a Strawberry Harvesting Manipulator

**Inas Saoud¹, Hatim Idriss Jaafari¹, Asaad Chahboun¹, Naoufal Raissouni², Nizar Ben Achhab¹, Abdelilah Azyat¹**

¹Mathematics and Intelligent Systems Research Team, National School of Applied Sciences-Tangier, Abdelmalek Essaadi University, Tangier, Morocco

²National School of Applied Sciences-Tetuan, Abdelmalek Essaadi University, Tetuan, Morocco

_Corresponding Author: Inas Saoud (inas.saoud@etu.uae.ac.ma)_

---

## Abstract

This paper presents a systematic approach to optimizing the structural parameters of a 4-degree-of-freedom (DoF) strawberry harvesting manipulator to minimize its workspace. Unlike previous research that primarily concentrated on the spatial needs related to fruit distribution areas, this work addresses the spatial dynamics of different stages of the fruit-picking process. This is achieved by combining the workspace model method, mathematical modeling, and the GlobalSearch algorithm in the optimization process. A comprehensive verification was conducted using the Denavit-Hartenberg method to simulate the workspace of the optimal manipulator structure. This ensured that the manipulator effectively covered the entire harvesting space.

The research design involves exploring an optimal trajectory planning method by adopting a modified sine jerk profile that minimizes overall trajectory duration while maintaining good smoothness. The effectiveness of this method is demonstrated through a simulation of the trajectory of the four joints to drive the end effector from the initial position to the position of the strawberry. This approach yields execution times up to 27% shorter than in previous studies. The proposed method is useful for optimizing the physical and trajectory design of the harvesting manipulator that operates in confined and restricted environments to enhance efficiency, adaptability, and safety in harvesting operations.

**Keywords:** 4-degree-of-freedom, Denavit-Hartenberg method, GlobalSearch algorithm, Harvesting manipulator, Mathematical modeling, Modified sine jerk profile, Workspace model method

---

## 1. Introduction

Research has shown great interest in the development and implementation of harvesting robots. Most studies have focused on the identification and positioning of mature crops and automatic navigation systems, while the design of the robot body receives less attention. The manipulator is one of the main components of the harvesting robot; it must detach the fruit from the plant without damaging either the fruit or the plant.

It's common to use industrial robotic arms that perform multiple tasks. For example, Xiong et al. used a robust Mitsubishi (RV-2AJ, Japan) industrial manipulator to harvest strawberries. However, its high price makes it inappropriate for use in agricultural fields. Additionally, the small environment of tabletop cultivation restricted the robot adaptation, with a harvest success rate of only 50%. Industrial manipulators are generally very large, expensive, and complex to control.

The development of a well-designed strawberry harvesting robot that adapts to the narrow operating space of tabletop cultivation and meets the requirements of selective harvesting significantly enhances picking success. The performance of the manipulator is affected by the size of its parameters. Therefore, the optimization of the manipulator's structural parameters is a prerequisite for its design in terms of performance indexes, which include accuracy, workspace, and energy consumption.

### 1.1 Related Work

Several researchers have approached manipulator optimization from different angles:

- **Sun et al.** optimized the structure parameters of a remote center-of-motion (RCM) mechanism by analyzing the kinematic performance and reachable space, then selecting optimal parameters using the particle swarm optimization (PSO) algorithm.

- **Xiong et al.** aimed to minimize the invalid workspace and achieve a compact structure by optimizing the structural parameters of a dual-apple-picking manipulator using the non-dominated sorting genetic algorithm II (NSGA-II) and the CRITIC-TOPSIS method.

- **Zhang et al.** selected the workspace model method to optimize the structure parameters of tomato-picking manipulators, using MATLAB's optimization toolbox.

- **Zhu et al.** proposed a combined index considering workspace area and energy consumption to optimize hexapod walking robot leg parameters using genetic algorithms.

### 1.2 Research Gap and Contribution

Positioning accuracy and workspace minimization are important indexes for harvesting fragile fruit in a confined environment. However, combining various performance criteria in the optimization model necessitates a multi-stage process and extensive computational resources. This complexity may limit practical application, particularly as the complexity of the manipulator system rises.

Previous studies adopted two methods to optimize the robot's workspace: the parameter analysis method and the workspace model method. The parameter analysis method examines individual parameters independently, neglecting their interdependencies and potentially ignoring constraints crucial for optimization. In contrast, the workspace model method globally models the robot's workspace, taking into account the interactions between parameters.

**Our contributions include:**

1. Utilizing the workspace model method to define spatial requirements for the harvesting task
2. Constructing a mathematical model incorporating structural parameters and workspace constraints
3. Deploying the GlobalSearch algorithm to search for optimal structural parameters
4. Implementing a trigonometric S-curve trajectory with modified sine jerk model for improved performance

### 1.3 Trajectory Planning Considerations

Various trajectory-planning approaches have been employed in harvesting manipulators, focusing on smoothing trajectories to minimize jerky movements and improve positional accuracy:

- **Cycloid trajectory method** ensures continuity of the acceleration profile
- **Fifth-order polynomial interpolation** generates smooth joint space trajectories
- **Quintic B-spline trajectory planning** obtains smooth paths but may impose heavy computation loads

This work implements a trigonometric S-curve trajectory with the modified sine jerk model, which has not yet been used by picking manipulators. This method enables a compromise between smoothness, speed, and computational complexity.

---

## 2. Methodology

### 2.1 Configuration of the Picking Manipulator

The strawberries are cultivated in parallel rows on a tabletop. The fruits grow on both sides of the table. The platform moves along the centerline of the aisle between the plant rows and stops at the picking position to drive the manipulator to harvest the ripe crops one by one. The fruits are well exposed, allowing a visual sensor to detect and locate them and a manipulator to access and detach them.

A small, lightweight, and compact four-DoF arm robot can complete the harvesting task between parallel rows because it occupies less space with a maximum workspace envelope.

#### Manipulator Structure Parameters

- **l₁, l₂, l₃, l₄**: Lengths of the first link (vertical to base), upper arm, lower arm, and end-effector
- **Joint limits:**
  - Waist: [θ₁ₘᵢₙ, θ₁ₘₐₓ]
  - Shoulder: [θ₂ₘᵢₙ, θ₂ₘₐₓ]
  - Elbow: [θ₃ₘᵢₙ, 0]
  - Wrist: [θ₄ₘᵢₙ, θ₄ₘₐₓ]

The angles θ₁ and θ₂ consider the positive direction of the horizontal x-axis as a reference, and θ₃ takes the extension of the upper arm as a reference. During the picking process, the end-effector is always maintained horizontally. Counterclockwise rotation is set as the positive direction of the rotation angles.

### 2.2 Kinematics Model of the Articulated Manipulator

#### 2.2.1 Forward Kinematics

The forward kinematics modeling of the articulated manipulator was derived using the Denavit-Hartenberg method. This method is commonly used to calculate the forward kinematics equation of multijoint manipulators.

**Three main steps in the Denavit-Hartenberg method:**

1. **Assign coordinate frames** to each joint in the kinematic diagram according to Denavit-Hartenberg convention rules

2. **Define parameters** for each joint:
   - **θₙ**: Angle between Xₙ₋₁ and Xₙ measured around Zₙ₋₁
   - **αₙ**: Angle between Zₙ₋₁ and Zₙ measured around Xₙ
   - **rₙ**: Distance along Xₙ direction from origin of frame n-1 to origin of frame n
   - **dₙ**: Distance along Zₙ₋₁ direction from origin of frame n-1 to origin of frame n

**Denavit-Hartenberg Parameters:**

| Joint n | θₙ (°) | αₙ (°) | rₙ (m) | dₙ (m) |
| ------- | ------ | ------ | ------ | ------ |
| 1       | θ₁     | 90°    | 0      | l₁     |
| 2       | θ₂     | 0      | l₂     | 0      |
| 3       | θ₃     | 0      | l₃     | 0      |
| 4       | θ₄     | 0      | l₄     | 0      |

3. **Insert parameters** into the homogeneous transformation matrix from frame n-1 to frame n:

The homogeneous transformation matrix from frame 0 to frame 4 is expressed as:

**H₄⁰ = H₁⁰ × H₂¹ × H₃² × H₄³**

From the displacement vector T₄⁰ of the matrix H₄⁰, we can determine the position of the end-effector relative to the base:

```
T₄⁰ = [x₄, y₄, z₄]ᵀ
```

Where:

- x₄ = (l₂·cos(θ₂) + l₃·cos(θ₂ + θ₃) + l₄·cos(θ₂ + θ₃ + θ₄))·cos(θ₁)
- y₄ = (l₂·cos(θ₂) + l₃·cos(θ₂ + θ₃) + l₄·cos(θ₂ + θ₃ + θ₄))·sin(θ₁)
- z₄ = l₁ + l₂·sin(θ₂) + l₃·sin(θ₂ + θ₃) + l₄·sin(θ₂ + θ₃ + θ₄)

#### 2.2.2 Inverse Kinematics

We use the geometric method to solve the manipulator's inverse kinematics. The joint angular positions that correspond to the target end-effector position (x₄, y₄, z₄) were determined by considering that the end-effector keeps a horizontal orientation:

```
θ₁ = tan⁻¹(y₄/x₄)

θ₂ = tan⁻¹((z₄ - l₁)/√(x₄² + y₄²) - l₄) + cos⁻¹((l₂² + (√(x₄² + y₄²) - l₄)² + (z₄ - l₁)² - l₃²)/(2l₂√((√(x₄² + y₄²) - l₄)² + (z₄ - l₁)²)))

θ₃ = -cos⁻¹((-l₂² - l₃² + (√(x₄² + y₄²) - l₄)² + (z₄ - l₁)²)/(2l₂l₃))

θ₄ = -θ₂ - θ₃
```

### 2.3 Workspace Optimization

#### 2.3.1 Identify the Manipulator's Harvesting Zone

Identifying the spatial requirements for each step of the fruit-picking cycle is crucial for determining the reachability requirements of the manipulator. Each picking cycle of the arm robot consists of a sequence of steps:

1. Displacement of the end effector from the initial position towards the ripe fruit
2. Detaching the strawberry from the plant
3. Stepping back horizontally by 100 mm from the plant to prevent collision
4. Transporting the harvested strawberries to the container
5. Releasing the fruit into the container
6. Returning to the initial position to complete the harvesting cycle

**Strawberry Distribution Space:**

- Height range (z-axis): 850–1100 mm
- Width (x-axis): 200 mm
- Length (y-axis): approximately 350 mm

Based on the growth characteristics of the fruits, we consider that the distribution space has the shape of a cuboid with:

- Length (L): 350 mm
- Width (W): 200 mm
- Height (H): 250 mm

The dimensions are adjusted to accommodate the spatial requirements of the fruit-picking cycle. The end-effector's initial picking position is at least 100 mm away from the planting row and facing the opposite direction. This involves expanding the cuboid to new dimensions L×W'×H, with W' = 300 mm.

#### 2.3.2 Analyzing the Enclosure of Harvesting Space by Manipulator Workspace

The problem of enclosing the cuboid with dimensions L×W'×H within the manipulator's workspace can be simplified by addressing:

1. The enclosure of a rectangle with dimensions W''×H in the vertical plane (xoz)
2. The rotation of the waist joint around the z-axis with a corresponding angular range Δθ₁

This allows the manipulator to cover the entire cuboid effectively. The relationships are given as:

```
Δθ₁ ≥ 2 × tan⁻¹(L/2 / r₁)
W'' = r₂ - r₁
```

Where r₁ and r₂ represent the arc radius of the actual manipulator workspace limit on the xy-plane. With r₁ = 260 mm, we obtain:

- Δθ₁ ≥ 68°
- W'' = 327 mm

#### 2.3.3 Mathematical Model of the Optimization Problem

The objective is to search for the minimum area of the actual workspace P₁P₂P₃P₄ that contains the rectangle ABCD by optimizing the structural parameters of the manipulator.

**Area Formula:**

```
S_P₁P₂P₃P₄ = ((α_P₁ - α_P₄) × r²_OP₁ - (α_P₂ - α_P₃) × r²_OP₃) / 2
           = (θ₂ₘₐₓ - θ₂ₘᵢₙ) × l₂ × l₃ × (1 - cos(θ₃ₘᵢₙ))
```

**Optimization Variables:**

The area S_P₁P₂P₃P₄ is a function of l₂, l₃, θ₂ₘᵢₙ, θ₂ₘₐₓ, and θ₃ₘᵢₙ. The length of the first and last links primarily influence the vertical and horizontal positions of the workspace, respectively, but have minimal impact on workspace area compared to other structural parameters.

The designed kinematic parameters that need to be optimized are:

**X = [x₁, x₂, x₃, x₄, x₅] = [l₂, l₃, θ₂ₘᵢₙ, θ₂ₘₐₓ, θ₃ₘᵢₙ]**

**Objective Function:**

```
min f(x) = min(S_P₁P₂P₃P₄)
```

**Constraints:**

According to specific functional and operational requirements:

- l₄ = 150 mm
- l₁ = 300 mm (set so the manipulator can face the picking area)
- Shoulder joint center taken as coordinate origin
- Rectangle ABCD coordinates: A(a,b), B(c,b), C(a,d), D(c,d)
  - a = 110 mm, b = 300 mm, c = 437 mm, d = 50 mm

Five constraints ensure the rectangle ABCD is enveloped by the workspace:

1. **Constraint 1:** In arc P₁P₄ at z=b: √(b² + c²) ≤ r_OP₁
2. **Constraint 2:** In arc P₄P₃ at z=d: x_M + l₃·cos(β) ≥ c
3. **Constraint 3:** In arc P₁P₂ at z=z_N: x_N + l₃ ≤ a
4. **Constraint 4:** In arc P₂P₃ at z=d: r_OP₂ ≤ √(a² + d²)
5. **Constraint 5:** l₂ = l₃ (for compact and flexible structure)

#### 2.3.4 GlobalSearch Algorithm

The GlobalSearch algorithm was adopted because it is specifically designed to handle smooth, nonlinear, and non-convex optimization problems with multiple constraints. The GlobalSearch solver from MATLAB's Global Optimization Toolbox was used. It locates the global minimum solution by using a precise and fastest local solver, fmincon, with the sequential quadratic programming algorithm (SQP).

**Algorithm Process:**

1. Run fmincon from the specified starting point X₀
2. **Stage 1:** Generate 200 random start points and run the best Stage 1 point
3. **Stage 2:** Generate trial points, analyze each start point, and run fmincon if the point satisfies the basin and score criteria
4. **Final Step:** Generate a solution vector of the local minima value

**Simulation Parameters:**

| Parameter                         | Value                                        |
| --------------------------------- | -------------------------------------------- |
| Algorithm                         | SQP                                          |
| Number of trial points            | 2000                                         |
| Number of trial points in stage 1 | 200                                          |
| Maximum time GlobalSearch runs    | Infinity                                     |
| X₀                                | [0.4 m, 0.4 m, 0.35 rad, 1.2 rad, -1.92 rad] |

### 2.4 Trajectory Planning

The motion of the manipulator during each step of fruit picking is conducted in joint space. A trigonometric S-curve trajectory planning with the modified sine jerk model is adopted to generate the joint path from the start pose to the desired pose. This trajectory model compromises between motion smoothness, speed, and computational complexity.

This approach maintains smooth motion, leading to improved tracking accuracy, which is important for handling fragile strawberries in confined environments. The motion profile consists of fifteen segments that present the equational variation of jerk.

**Modified Sine Jerk Profile:**

```
         ⎧ J_peak·sin(π/(2T₁)·τᵢ),           t₀ ≤ t < t₁, t₁₂ ≤ t < t₁₃
         ⎪ J_peak,                            t₁ ≤ t < t₂, t₁₃ ≤ t < t₁₄
         ⎪ J_peak·sin(π/2·(1 + τᵢ/T₁)),      t₂ ≤ t < t₃, t₁₄ ≤ t ≤ t₁₅
j(t) =   ⎨ 0,                                t₃ ≤ t < t₄, t₇ ≤ t < t₈, t₁₁ ≤ t < t₁₂
         ⎪ -J_peak·sin(π/(2T₁)·τᵢ),          t₄ ≤ t < t₅, t₈ ≤ t < t₉
         ⎪ -J_peak,                           t₅ ≤ t < t₆, t₉ ≤ t < t₁₀
         ⎩ -J_peak·sin(π/2·(1 + τᵢ/T₁)),     t₆ ≤ t < t₇, t₁₀ ≤ t < t₁₁
```

Where:

- J_peak: Peak value of jerk
- tᵢ (i=0,1,...,15): Time boundary for each segment
- Tᵢ = tᵢ - tᵢ₋₁: Time interval of each segment
- τᵢ = t - tᵢ₋₁: Time relative to the commencement of interval [tᵢ₋₁, tᵢ]

The acceleration, velocity, and displacement functions of the trajectory are obtained by integrating the jerk function:

```
a(t) = a(tᵢ) + ∫[tᵢ to t] j(t) dt
v(t) = v(tᵢ) + ∫[tᵢ to t] a(t) dt
d(t) = d(tᵢ) + ∫[tᵢ to t] v(t) dt
```

---

## 3. Results and Discussion

### 3.1 Workspace Optimization

The proposed optimization process was conducted on a 4-DoF articulated manipulator. The GlobalSearch solver successfully determined the optimal structural parameters of the manipulator, minimizing the area of the workspace.

**Optimization Results:**

| Parameter                         | Value                                          |
| --------------------------------- | ---------------------------------------------- |
| X = [l₂, l₃, θ₂ₘᵢₙ, θ₂ₘₐₓ, θ₃ₘᵢₙ] | [0.265 m, 0.265 m, 0.6 rad, 2.2 rad, -2.7 rad] |
| Workspace area S_P₁P₂P₃P₄         | 0.2125 m²                                      |

To accommodate the step of transferring fruit to the container placed on the mobile platform, the base of the manipulator must be rotated 90°. This guarantees that the manipulator successfully completes every step of the fruit-picking cycle. Additionally, the manipulator will harvest the fruits grown on both sides of the robot by rotating the waist joint 180°. These adjustments expand the angle range of θ₁ from [-34°, 34°] to [-34°, 214°].

**Final Structural Parameters:**

| Link i | Length (mm) | Joint Angle Range |
| ------ | ----------- | ----------------- |
| 1      | 300         | [-34°, 214°]      |
| 2      | 265         | [34.4°, 126°]     |
| 3      | 265         | [-155°, 0°]       |
| 4      | 150         | [-126°, 121°]     |

We simulated the workspace of the robot using the forward kinematics equation to ensure coverage of the picking zone. From the simulation results, the optimal design of the manipulator can effectively cover the designated harvesting area. The robot will generate the same workspace in the left and right plant rows.

**Key Findings:**

This study focuses on the spatial requirements of the fruit-picking cycle, aiming to understand the spatial dynamics beyond the fruit distribution area. Previous research primarily focused on fruit distribution areas but overlooked other critical stages of the fruit-picking process. By addressing the spatial needs of several steps of harvesting simultaneously, the workspace model method facilitates the optimization of the manipulator structure parameters. This optimization not only minimizes unnecessary movements but also reduces the risk of collisions and damage to both the manipulator and surrounding crops.

### 3.2 Trajectory Planning

We simulated a path in joint space to demonstrate the effectiveness of this trajectory planning method. Specifically, we guided the manipulator's gripper from:

- **Initial position P₀:** (x₄=0.227 m, y₄=0 m, z₄=0.39 m)
- **Target fruit position P₁:** (x₄=0.56 m, y₄=0.175 m, z₄=0.589 m)

**Joint Angular Positions:**

| Joint | Position P₀ (rad) | Position P₁ (rad) |
| ----- | ----------------- | ----------------- |
| 1     | 0                 | 0.3               |
| 2     | 2.2               | 0.73              |
| 3     | -2.7              | -0.3              |
| 4     | 0.3               | -0.43             |

The joint motion profiles were obtained by simulation with MATLAB software. The generated trajectory ensures higher levels of continuity up to jerk during the whole motion. Thus, the arm robot tracks the generated trajectory without any abrupt change. This is beneficial to suppress the vibrations of the robot's movement and enhance position accuracy.

**Fruit Approaching Time Results:**

| Fruit Position (m) |      |      | Execution Time (s) |
| ------------------ | ---- | ---- | ------------------ |
| X₄                 | Y₄   | Z₄   |                    |
| 0.4                | 0.17 | 0.5  | 0.95691            |
| 0.36               | 0.1  | 0.56 | 0.97765            |
| 0.56               | 0    | 0.35 | 1.3743             |

By adopting the modified sine jerk model in the optimal design of the manipulator, the average time to approach the fruit from the initial position is **1.1 seconds**. The time required to approach the target fruit in other studies is about 1.5 seconds. **The execution time is reduced by approximately 27%**. This approach can reduce the execution time of each step of the picking cycle, which reduces harvesting time and improves productivity.

---

## 4. Conclusion

This study addressed the issue of optimizing both the structural parameters of the robotic manipulator and its trajectory design. We propose an approach for optimizing the manipulator's structural parameters to minimize its workspace, which combines the workspace model method, mathematical modeling, and the GlobalSearch algorithm.

**Key Contributions:**

1. By addressing the spatial requirements of different stages of the fruit-picking cycle in the optimization process, we ensure that the manipulator's workspace can effectively cover the picking area while occupying the smallest possible physical space.

2. This guarantees better performance, adaptability, and safety in harvesting operations.

3. We explored an optimal trajectory planning method that compromises between motion smoothness, speed, and computational complexity by adopting a modified sine jerk model.

4. The effectiveness of this method was demonstrated through the simulation of joint trajectories to drive the end effector from the initial position to the position of the strawberry, yielding execution times up to 27% shorter than previous studies.

**Applicability:**

Even though this optimization approach was initially developed for a 4-DoF articulated robot, its fundamental techniques are sufficiently general to be used with a wider variety of robotic systems with different joint configurations.

### Future Work

Our present research has primarily focused on kinematic aspects in the design and trajectory planning of manipulator systems. Incorporating dynamics considerations can significantly improve the robot system's capabilities and performance. In future work, it is important to address dynamics-related gaps in our study findings. This involves:

- Developing dynamic models
- Designing control algorithms that account for dynamic effects
- Combining dynamic issues to produce more effective solutions that are better adapted to the requirements of the actual agricultural environment

---

## Acknowledgements

This research was supported in part by the National Center for Scientific and Technical Research (CNRST) in Morocco through its Research Scholarships Grants program, under Grant Agreement number 29UAE2021.

---

## References

1. Fountas, S., Mylonas, N., Malounas, I., et al. (2020). Agricultural Robotics for Field Operations. _Sensors_, 20(9), 2672.

2. Jun, J., Kim, J., Seol, J., et al. (2021). Towards an Efficient Tomato Harvesting Robot: 3D Perception, Manipulation, and End-Effector. _IEEE Access_, 9, 17631-17640.

3. Yu, Y., Zhang, K., Liu, H., et al. (2020). Real-Time Visual Localization of the Picking Points for a Ridge-Planting Strawberry Harvesting Robot. _IEEE Access_, 8, 116556-116568.

4. Mao, W., Liu, H., Hao, W., et al. (2022). Development of a Combined Orchard Harvesting Robot Navigation System. _Remote Sensing_, 14(3), 675.

5. Wang, S., et al. (2022). Design, development and evaluation of latex harvesting robot based on flexible Toggle. _Robotics and Autonomous Systems_, 147, 103906.

6. Ma, Y., Zhang, W., Qureshi, W. S., et al. (2021). Autonomous navigation for a wolfberry picking robot using visual cues and fuzzy control. _Information Processing in Agriculture_, 8(1), 15-26.

7. Hu, G., et al. (2022). Simplified 4-DOF manipulator for rapid robotic apple harvesting. _Computers and Electronics in Agriculture_, 199, 107177.

8. Xiong, Y., Peng, C., Grimstad, L., et al. (2019). Development and field evaluation of a strawberry harvesting robot with a cable-driven gripper. _Computers and Electronics in Agriculture_, 157, 392-402.

9. Xiong, Y., Ge, Y., Grimstad, L., & From, P. J. (2020). An autonomous strawberry-harvesting robot: Design, development, integration, and field evaluation. _Journal of Field Robotics_, 37(2), 202-224.

10. Buerkle, A., et al. (2023). Towards industrial robots as a service (IRaaS): Flexibility, usability, safety and business models. _Robotics and Computer-Integrated Manufacturing_, 81, 102484.

11. Perzylo, A., et al. (2019). SMErobotics: Smart Robots for Flexible Manufacturing. _IEEE Robotics & Automation Magazine_, 26(1), 78-90.

12. Jing, Y., Jin, L., Shi, X., et al. (2020). Dimensional Optimization for Minimally Invasive Surgery Robot Based on Double Space and Kinematic Accuracy Reliability Index. _Journal of Engineering and Science in Medical Diagnostics and Therapy_, 3(2).

13. Xiong, Z., et al. (2022). Dual-Manipulator Optimal Design for Apple Robotic Harvesting. _Agronomy_, 12(12), 3128.

14. Sun, J., Yan, Z., & Du, Z. (2017). Optimal design of a novel remote center-of-motion mechanism for minimally invasive surgical robot. _IOP Conference Series: Earth and Environmental Science_, 69(1).

15. Zhang, S., Yuan, T., Wang, D., et al. (2016). Structure Optimization and Path Planning of Tomato Picking Manipulator. _Proceedings of the 9th International Symposium on Computational Intelligence and Design_, 356-360.

16. Zhu, Y., Jin, B., Li, W., & Li, S. (2014). Optimal design of hexapod walking robot leg structure based on energy consumption and workspace. _Transactions-Canadian Society for Mechanical Engineering_, 38(4), 305-317.

17. Zhao, Q., Li, L., Wu, Z., et al. (2022). Optimal Design and Experiment of Manipulator for Camellia Pollen Picking. _Applied Sciences_, 12(16), 8011.

18. Wu, J., Zhang, Y., Zhang, S., et al. (2021). Simulation Design of a Tomato Picking Manipulator. _Tehnicki Vjesnik_, 28(4), 1253-1261.

19. Cao, X., et al. (2021). A Multi-Objective Particle Swarm Optimization for Trajectory Planning of Fruit Picking Manipulator. _Agronomy_, 11(11), 2286.

20. Hu, X., Yu, H., Lv, S., & Wu, J. (2021). Design and experiment of a new citrus harvesting robot. _Proceedings of the International Conference on Control Science and Electric Power Systems_, 179-183.

21. Ting, H. Z., Zaman, M. H. M., Ibrahim, M. F., & Moubark, A. M. (2021). Kinematic Analysis for Trajectory Planning of Open-Source 4-DoF Robot Arm. _International Journal of Advanced Computer Science and Applications_, 12(6).

22. Molaei, F., & Ghatrehsamani, S. (2022). Kinematic-Based Multi-Objective Design Optimization of a Grapevine Pruning Robotic Manipulator. _AgriEngineering_, 4(3), 606-625.

23. Denavit, J., & Hartenberg, R. S. (1955). A Kinematic Notation for Lower-Pair Mechanisms Based on Matrices. _Journal of Applied Mechanics_, 22(2), 215-221.

24. Močnik, G., Kačič, Z., Šafarič, R., & Mlakar, I. (2022). Capturing Conversational Gestures for Embodied Conversational Agents Using an Optimized Kaneda–Lucas–Tomasi Tracker and Denavit–Hartenberg-Based Kinematic Model. _Sensors_, 22(21), 8318.

25. Ding, F., & Liu, C. (2018). Applying coordinate fixed Denavit–Hartenberg method to solve the workspace of drilling robot arm. _International Journal of Advanced Robotic Systems_, 15(4).

26. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2004). Inverse Kinematics. In _Robot Dynamics and Control_ (2nd ed., pp. 79-89). Wiley.

27. Mohamed, Z., & Capi, G. (2012). Development of a New Mobile Humanoid Robot for Assisting Elderly People. _Procedia Engineering_, 41, 345-351.

28. Wang, Y., Yang, Q., Bao, G., et al. (2011). Optimization Design and Experiment of Fruit and Vegetable Picking Manipulator. _Transactions of the Chinese Society for Agricultural Machinery_, 42, 191-195.

29. The MathWorks, Inc. (2020). Using GlobalSearch and MultiStart. In _MATLAB Global Optimization Toolbox User's Guide_ (R2020a ed.).

30. Ugray, Z., Lasdon, L., Plummer, J., et al. (2007). Scatter Search and Local NLP Solvers: A Multistart Framework for Global Optimization. _INFORMS Journal on Computing_, 19(3), 328-340.

31. Fang, Y., Qi, J., Hu, J., et al. (2020). An approach for jerk-continuous trajectory generation of robotic manipulators with kinematical constraints. _Mechanism and Machine Theory_, 153, 103957.

32. Feng, Q., et al. (2018). Design and test of robotic harvesting system for cherry tomato. _International Journal of Agricultural and Biological Engineering_, 11(1), 96-100.

---

## Author Biographies

### Inas Saoud

Inas Saoud received the Engineer degree in mechatronics engineering from the National School of Applied Sciences (ENSA), Abdelmalek Essaadi University, Tetuan, Morocco, in 2020. She is currently pursuing a Ph.D. in robotics at the National School of Applied Sciences in Tangier, Morocco. Her research focuses on agricultural robots, modeling and design, robotic device development, control, and optimization.

**Email:** inas.saoud@etu.uae.ac.ma

### Hatim Idriss Jaafari

Hatim Idriss Jaafari obtained a master's degree in cybersecurity and cybercriminality from Abdelmalek Essaadi University, ENSA Tangier, Morocco, in 2021. He is currently a Ph.D. student in the domain of secure communications in robotic operating systems and a member of the Mathematics and Intelligent Systems Research Team. His research focuses on securing communication channels, safeguarding data through encryption methods, and exploring the potential of blockchain technology to enhance security in various domains.

**Email:** hatimidriss.jaafari@etu.uae.ac.ma

### Asaad Chahboun

Asaad Chahboun is a highly qualified professional with an extensive educational background in telecommunications. He holds an M.S. and a Ph.D. degree in telecommunications from Abdelmalek Essaadi University and an engineering degree from the Central School of Arts and Businesses in Brussels, Belgium. Currently, he is a member of the Mathematics and Intelligent Systems research team in the Department of Information Systems and Communication at the National School of Applied Sciences in Tangier, Morocco. As a researcher, his focus includes wireless sensor networks, routing in ad hoc networks, the internet of things, network security, remote sensing, the development of remote sensing methods for land cover dynamic monitoring, and grid computing.

**Email:** achahboun@uae.ac.ma

### Naoufal Raissouni

Naoufal Raissouni received M.S. and Ph.D. degrees in physics (remote sensing, earth observation, and geoscience) at the Global Change Unit at the University of Valencia, Spain, in collaboration with Louis Pasteur University, Strasbourg, France, and the Centre National de la Recherche Scientifique, France. He is a professor and researcher in physics and remote sensing at the National Engineering School for Applied Sciences of Tetuan at the University of Abdelmalek Essaadi (UAE), Morocco. He heads the Remote Sensing and Geographic Information System Laboratory in the UAE, and he is president of the Remote Sensing and GIS Association in Morocco. His research interests include the retrieval of emissivity, atmospheric water vapor, and sea/land surface temperature from satellite images; wireless sensor networks; infrared radiometry; and in situ measurements.

**Email:** nraissouni@uae.ma

### Nizar Ben Achhab

Nizar Ben Achhab received a Ph.D. degree in telecommunications and informatics in 2011 and an M.S. degree in bioinformatics in 2004 from the University of Abdelmalek Essaadi (UAE), Tetuan, Morocco. Currently, he is a member of the Mathematics and Intelligent Systems Research Team. His current work includes remote sensing multitemporal and spatiotemporal studies of thermal infrared satellite imagery of the earth's surface, in-situ LST measurements, and the development of remote sensing methods for land cover dynamic monitoring. It also includes grid computing, multithreading, GPGPU, and hardware and software methods for time computing optimizations.

**Email:** nbenachhab@uae.ac.ma

### Abdelilah Azyat

Abdelilah Azyat received a Ph.D. degree in telecommunications and informatics in 2011 and an M.S. degree in bioinformatics in 2004 from the University of Abdelmalek Essaadi (UAE), Tetuan, Morocco. Currently, he is a member of the Mathematics and Intelligent Systems Research Team. His research interests include remote sensing, spatiotemporal studies of thermal infrared satellite imagery of the earth's surface, GIS, and mobile GIS.

**Email:** aazyat@uae.ac.ma

---

## Publication Information

**Journal:** Bulletin of Electrical Engineering and Informatics  
**Volume:** 13, No. 6  
**Date:** December 2024  
**Pages:** 3948-3959  
**ISSN:** 2302-9285  
**DOI:** 10.11591/eei.v13i6.7957  
**Journal homepage:** http://beei.org

**Article History:**

- Received: November 29, 2023
- Revised: April 18, 2024
- Accepted: May 17, 2024

**License:** This is an open access article under the CC BY-SA license.

---

## Appendix: Mathematical Notation

### Symbols and Parameters

| Symbol         | Description                               | Unit     |
| -------------- | ----------------------------------------- | -------- |
| l₁, l₂, l₃, l₄ | Link lengths                              | m        |
| θ₁, θ₂, θ₃, θ₄ | Joint angles                              | rad or ° |
| x₄, y₄, z₄     | End-effector position coordinates         | m        |
| r₁, r₂         | Arc radii of workspace limits             | mm       |
| L, W, H        | Cuboid dimensions (length, width, height) | mm       |
| Δθ₁            | Waist joint angular range                 | rad or ° |
| S_P₁P₂P₃P₄     | Workspace area                            | m²       |
| J_peak         | Peak value of jerk                        | m/s³     |
| a(t)           | Acceleration function                     | m/s²     |
| v(t)           | Velocity function                         | m/s      |
| d(t)           | Displacement function                     | m        |
| H_n^(n-1)      | Homogeneous transformation matrix         | -        |
| T_n^(n-1)      | Displacement vector                       | m        |
| R_n^(n-1)      | Rotation matrix                           | -        |

### Abbreviations

- **DoF:** Degree of Freedom
- **PSO:** Particle Swarm Optimization
- **NSGA-II:** Non-dominated Sorting Genetic Algorithm II
- **RCM:** Remote Center-of-Motion
- **SQP:** Sequential Quadratic Programming
- **DH:** Denavit-Hartenberg

---

## Notes for Researchers

This markdown document contains the complete rewritten version of the research paper on strawberry harvesting manipulator design optimization and trajectory planning. The document includes:

1. **Structured sections** following academic paper standards
2. **Complete mathematical formulations** with proper notation
3. **Comprehensive tables** for parameters and results
4. **Full references** in standard academic format
5. **Author biographies** and contact information
6. **Publication metadata** for citation purposes

The document can be:

- Exported to PDF using markdown-to-PDF converters
- Converted to LaTeX for journal submission
- Used directly in markdown-compatible platforms
- Modified for presentation slides or posters

For questions or clarifications about the methodology or results, please contact the corresponding author at inas.saoud@etu.uae.ac.ma.
