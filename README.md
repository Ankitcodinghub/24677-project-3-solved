# 24677-project-3-solved
**TO GET THIS SOLUTION VISIT:** [24677 Project 3 Solved](https://www.ankitcodinghub.com/product/24677-project-3-solved-2/)


---

📩 **If you need this solution or have special requests:** **Email:** ankitcoding@gmail.com  
📱 **WhatsApp:** +1 419 877 7882  
📄 **Get a quote instantly using this form:** [Ask Homework Questions](https://www.ankitcodinghub.com/services/ask-homework-questions/)

*We deliver fast, professional, and affordable academic help.*

---

<h2>Description</h2>



<div class="kk-star-ratings kksr-auto kksr-align-center kksr-valign-top" data-payload="{&quot;align&quot;:&quot;center&quot;,&quot;id&quot;:&quot;96314&quot;,&quot;slug&quot;:&quot;default&quot;,&quot;valign&quot;:&quot;top&quot;,&quot;ignore&quot;:&quot;&quot;,&quot;reference&quot;:&quot;auto&quot;,&quot;class&quot;:&quot;&quot;,&quot;count&quot;:&quot;0&quot;,&quot;legendonly&quot;:&quot;&quot;,&quot;readonly&quot;:&quot;&quot;,&quot;score&quot;:&quot;0&quot;,&quot;starsonly&quot;:&quot;&quot;,&quot;best&quot;:&quot;5&quot;,&quot;gap&quot;:&quot;4&quot;,&quot;greet&quot;:&quot;Rate this product&quot;,&quot;legend&quot;:&quot;0\/5 - (0 votes)&quot;,&quot;size&quot;:&quot;24&quot;,&quot;title&quot;:&quot;24677 Project 3 Solved&quot;,&quot;width&quot;:&quot;0&quot;,&quot;_legend&quot;:&quot;{score}\/{best} - ({count} {votes})&quot;,&quot;font_factor&quot;:&quot;1.25&quot;}">

<div class="kksr-stars">

<div class="kksr-stars-inactive">
            <div class="kksr-star" data-star="1" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="2" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="3" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="4" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" data-star="5" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
    </div>

<div class="kksr-stars-active" style="width: 0px;">
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
            <div class="kksr-star" style="padding-right: 4px">


<div class="kksr-icon" style="width: 24px; height: 24px;"></div>
        </div>
    </div>
</div>


<div class="kksr-legend" style="font-size: 19.2px;">
            <span class="kksr-muted">Rate this product</span>
    </div>
    </div>
<div class="page" title="Page 2">
<div class="layoutArea">
<div class="column">
1 Introduction

In this project, you will complete the following goals: 1. Design an lateral optimal controller

2. Implement A* path planning algorithm

[Remember to submit the write-up, plots, and codes on Gradescope.]

2 Model

The error-based linearized state-space for the lateral dynamics is as follows.

e1 is the distance to the center of gravity of the vehicle from the reference trajectory. e2 is the orientation error of the vehicle with respect to the reference trajectory.

e10 1 0 0e100  o  d e ̇ 0 −4Cα 4Cα −2Cα(lf−lr) e ̇ 2Cα 0 􏰂δ􏰃 −2Cα(lf−lr)−x ̇

</div>
</div>
<div class="layoutArea">
<div class="column">
1mx ̇ m mx ̇1mmx ̇  ̇

</div>
</div>
<div class="layoutArea">
<div class="column">
=

dte2 0 0

e ̇ 2Cα(lf −lr) 2 0 − Izx ̇

</div>
<div class="column">
0

2Cα(lf −lr) Iz

</div>
<div class="column">
1

2Cα(lf2 +lr2) − Izx ̇

</div>
<div class="column">
+ + ψdes e20 0F  0 

e ̇ 2Cαlf 0 2Cα(lf2 +lr2) 2 Iz − Izx ̇

</div>
</div>
<div class="layoutArea">
<div class="column">
In lateral vehicle dynamics, ψ ̇des is a time-varying disturbance in the state space equation. Its value is proportional to the longitudinal speed when the radius of the road is constant. When deriving the error-based state space model for controller design, ψ ̇des can be safely assumed to be zero.

e10 1 0 0e100

d e ̇ 0 −4Cα 4Cα −2Cα(lf−lr) e ̇ 2Cα 0 􏰂δ􏰃

</div>
</div>
<div class="layoutArea">
<div class="column">
1mx ̇ m mx ̇1m

</div>
</div>
<div class="layoutArea">
<div class="column">
= dte20 0

</div>
<div class="column">
0

</div>
<div class="column">
+  1e200F

Iz

</div>
</div>
<div class="layoutArea">
<div class="column">
e ̇ 2 C α ( l f − l r ) 2 0 − Izx ̇

</div>
<div class="column">
2 C α ( l f − l r ) Iz

</div>
<div class="column">
2 C α ( l f2 + l r2 ) e ̇ 2 C α l f 0

</div>
</div>
<div class="layoutArea">
<div class="column">
− Izx ̇ 2

d􏰂x􏰃 􏰂0 1􏰃􏰂x􏰃 􏰂0 0􏰃􏰂δ􏰃 􏰂 0 􏰃

</div>
</div>
<div class="layoutArea">
<div class="column">
For the longitudinal control:

dt x ̇ = 0 0 x ̇ + 0 m1 F + ψ ̇y ̇−fg

</div>
</div>
<div class="layoutArea">
<div class="column">
Assuming ψ ̇ = 0:

</div>
</div>
<div class="layoutArea">
<div class="column">
d􏰂x􏰃 􏰂0 1􏰃􏰂x􏰃 􏰂0 0􏰃􏰂δ􏰃 dtx ̇=00x ̇+0m1 F

</div>
</div>
<div class="layoutArea">
<div class="column">
2

</div>
</div>
</div>
<div class="page" title="Page 3">
<div class="layoutArea">
<div class="column">
3 P3: Problems

Exercise 1. [50pts] All the code related to Exercise 1 is under P3 student/P3-LQR/. For

the lateral control of the vehicle, design a discrete-time infinite horizon LQR controller.

You can reuse your longitudinal PID controller from part 1 of this project, or even im- prove upon it. However, it may require retuning based on observed performance.

Design the two controllers in your controller.py. You can make use of Webots’ built- in code editor, or use your own.

Check the performance of your controller by running the Webots simulation. You can press the play button in the top menu to start the simulation in real-time, the fast-forward button to run the simulation as quickly as possible, and the triple fast-forward to run the simulation without rendering (any of these options is acceptable, and the faster options may be better for quick tests). If you complete the track, the scripts will generate a performance plot via matplotlib. This plot contains a visualization of the car’s trajectory, and also shows the variation of states with respect to time.

Submit your controller.py and the final completion plot as described on the title page and report your score. Your controller is required to achieve the following performance criteria to receive full points:

<ol>
<li>Time to complete the loop = 250 s</li>
<li>Maximum deviation from the reference trajectory = 7.0 m</li>
<li>Average deviation from the reference trajectory = 3.5 m</li>
</ol>
Some hints that may be useful:

<ul>
<li>Make sure to discretize your continuous state-space system with the provided timestep (delT) prior to solving the ARE.</li>
<li>Using LQR requires manually creating two matrices Q and R. Q works to penalize state variables, while R penalizes control input. Try to think about what form your Q and R matrices should take for good performance.
<ul>
<li>– &nbsp;For Q, large values will heavily restrict changes to the respective states, while small values will allow the states to easily change.</li>
<li>– &nbsp;Similarly, in R, large values will heavily restrict control input, while small values will allow the control input to vary widely.</li>
<li>– &nbsp;One idea for tuning is to set the relevant indices of Q and R to 1
(max value of the corresponding state/input)2

in order to normalize the value. Make sure to experiment outside of this guideline

to determine the best performance. 3
</li>
</ul>
</li>
</ul>
</div>
</div>
</div>
<div class="page" title="Page 4">
<div class="layoutArea">
<div class="column">
– There is a relationship between Q and R, though it is subtle. For example, if you increase weights in Q, you are more heavily penalizing changes in the states, which will require more control input. This would imply that you should decrease weights in R to see an effect. Due to this, it may be helpful to keep either Q or R fixed while varying the other during tuning.

[10% Bonus]: Complete the loop within 130 s. The maximum deviation and the average deviation should be within in the allowable performance criteria mentioned above.

Exercise 2. [50pts] A* Planning

All the code related to Exercise 1 is under P3 student/P3-AStar/. In this exercise, we will implement A* path planning algorithm. Now there is another vehicle driving on the track. If the ego vehicle follows the original trajectory (which aligns with the track), it is likely to crash into the other vehicle. In order to perform the overtaking, we need to re-plan the trajectory. In this problem, we will implement A* path planning algorithm.

To simplify the problem, we initialize the other vehicle on the straight section of the track. You will overtake it on the straight section as well. We assume the other vehicle will drive in a straight line so we do not need to worry about trajectory prediction too much. As you get close to the other car, our controller will use A* algorithm to re-plan the path. The predicted trajectory of another vehicle is treated as obstacle in the cost map. In addition, we do not want to drive too far away from the track therefore we treat the area outside a certain distance from the center of the track as obstacle. In your implementation, you only need to find a path given a static and discretized map.

You only need to complete the plan function in P3 student/P3-AStar/AStar-scripts/ Astar script.py. To test the algorithm you implemented, simply run Astar script.py file, which will generate 3 figures of path planning results using the function you implemented. The maps are shown in Figure.1. Yellow region represents the obstacle. Red Crosses repre- sent the start and red dots represent the goal. If you think it plans reasonable paths, you can copy the code from your Astar script.py to P3 student/P3-AStar/controllers/main/Astar.py and then open Webots world file P3 student/P3-AStar/worlds/Project3.wbt. and try it with the Webots simulator and the controller you implemented in Exercise 1.

(a) (b) (c)

Figure 1: A* Testing cases

In your submission, 1. Save and attach the figures generated by runing Astar.py in your submission and 2. report the time of completion. As long as the generated figures show the

</div>
</div>
<div class="layoutArea">
<div class="column">
4

</div>
</div>
</div>
<div class="page" title="Page 5">
<div class="layoutArea">
<div class="column">
successful path planning and the car completes the racing without crashing, you will receive full marks for this problem.

Here is the pseudocode for A* path planning algorithm (Source: https://en.wikipedia. org/wiki/A*_search_algorithm).

<pre>   function calculate_path(node):
       path_ind.append(node.pose)
       while current.parent is not empty:
</pre>
<pre>           current := current.parent
</pre>
<pre>           path_ind.append(current.pose)
       return path_ind
</pre>
</div>
</div>
<div class="layoutArea">
<div class="column">
5

</div>
</div>
</div>
<div class="page" title="Page 6">
<div class="layoutArea">
<div class="column">
<pre>// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from
</pre>
􏰈→ node n.

function A_Star(start, goal, h)

<pre>    // Initialize start node and goal node class
</pre>
<pre>    // Calculate h_value and f_value of start node
    // For node n, g(node) is the cost of the cheapest path from start to n
</pre>
􏰈→ currently known

// f(node) = g(node) + h(node), which represents ourcurrent best guess

􏰈→ asto

// how short a path from start to finish can be if it goes through n

<pre>    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather
</pre>
􏰈→ than a hash-set. openSet := {start}

// For node n, closedSet[n] is the node immediately preceding it on the 􏰈→ cheapest path from start

<pre>    // to n currently known.
</pre>
<pre>    closedSet := an empty map
</pre>
while openSet is not empty

// Current is the node in open_list that has the lowest f value

// This operation can occur in O(1) time if openSet is a min-heap or

􏰈→ a priority queue

current := the node in openSet having the lowest fScore[] value openSet.Remove(current)

<pre>        // Append current node into the closed_list
</pre>
<pre>        if current = goal
            return reconstruct_path(closedSet, current)
</pre>
for each neighbor of current (can be computed by calling the 􏰈→ get_successor method):

// d(current,neighbor) is the weight of the edge from current to 􏰈→ neighbor

<pre>            // g(successor) = g(current) + d(current, successor)
            // h(successor) can be computed by calling the heuristic method
            // f(successor) = g(successor) + h(successor)
            // Don't forget to push the successor into open_list
</pre>
<pre>    // Open set is empty but goal was never reached
</pre>
<pre>    return failure
</pre>
</div>
</div>
<div class="layoutArea">
<div class="column">
6

</div>
</div>
</div>
<div class="page" title="Page 7">
<div class="layoutArea">
<div class="column">
4 Appendix (Already covered in P1)

</div>
</div>
<div class="layoutArea">
<div class="column">
where ay = dt2

</div>
<div class="column">
inertial

</div>
</div>
<div class="layoutArea">
<div class="column">
Figure 2: Bicycle model[2]

</div>
</div>
<div class="layoutArea">
<div class="column">
Figure 3: Tire slip-angle[2]

</div>
</div>
<div class="layoutArea">
<div class="column">
We will make use of a bicycle model for the vehicle, which is a popular model in the study of vehicle dynamics. Shown in Figure 2, the car is modeled as a two-wheel vehicle with two degrees of freedom, described separately in longitudinal and lateral dynamics. The model parameters are defined in Table 2.

4.1 Lateral dynamics

Ignoring road bank angle and applying Newton’s second law of motion along the y-axis: may =Fyf cosδf +Fyr

􏰀d2y􏰁

is the inertial acceleration of the vehicle at the center of geometry in the direction of the y axis, Fyf and Fyr are the lateral tire forces of the front and rear

</div>
</div>
<div class="layoutArea">
<div class="column">
7

</div>
</div>
</div>
<div class="page" title="Page 8">
<div class="layoutArea">
<div class="column">
wheels, respectively, and δf is the front wheel angle, which will be denoted as δ later. Two terms contribute to ay: the acceleration y ̈, which is due to motion along the y-axis, and the centripetal acceleration. Hence:

a y = y ̈ + ψ ̇ x ̇

Combining the two equations, the equation for the lateral translational motion of the vehicle

is obtained as:

y ̈=−ψ ̇x ̇+m1(Fyf cosδ+Fyr)

Moment balance about the axis yields the equation for the yaw dynamics as

ψ ̈Iz =lfFyf −lrFyr

The next step is to model the lateral tire forces Fyf and Fyr. Experimental results show that the lateral tire force of a tire is proportional to the “slip-angle” for small slip-angles when vehicle’s speed is large enough – i.e. when x ̇ ≥ 0.5 m/s. The slip angle of a tire is defined as the angle between the orientation of the tire and the orientation of the velocity vector of the vehicle. The slip angle of the front and rear wheel is

αf =δ−θVf αr =−θVr

where θV p is the angle between the velocity vector and the longitudinal axis of the vehicle, for p ∈ {f, r}. A linear approximation of the tire forces are given by

􏰄 y ̇ + l f ψ ̇ 􏰅 Fyf=2Cα δ− x ̇

􏰄 y ̇−lrψ ̇􏰅 Fyr=2Cα − x ̇

where Cα is called the cornering stiffness of the tires. If x ̇ &lt; 0.5 m/s, we just set Fyf and Fyr both to zeros.

4.2 Longitudinal dynamics

Similarly, a force balance along the vehicle longitudinal axis yields:

x ̈ = ψ ̇ y ̇ + a x max = F − Ff

Ff =fmg

where F is the total tire force along the x-axis, and Ff is the force due to rolling resistance

at the tires, and f is the friction coefficient.

</div>
</div>
<div class="layoutArea">
<div class="column">
8

</div>
</div>
</div>
<div class="page" title="Page 9">
<div class="layoutArea">
<div class="column">
4.3 Global coordinates

In the global frame we have:

4.4 System equation

</div>
<div class="column">
X ̇ =x ̇cosψ−y ̇sinψ Y ̇ =x ̇sinψ+y ̇cosψ

</div>
</div>
<div class="layoutArea">
<div class="column">
Gathering all of the equations, if x ̇ ≥ 0.5 m/s, we have:

</div>
</div>
<div class="layoutArea">
<div class="column">
2 C 􏰄 y ̇ + l ψ ̇ 􏰅 y ̈=−ψ ̇x ̇+α(cosδδ− f − r)

</div>
</div>
<div class="layoutArea">
<div class="column">
y ̇ − l ψ ̇ m x ̇ x ̇

</div>
</div>
<div class="layoutArea">
<div class="column">
x ̈ = ψ ̇y ̇ + m1 (F − fmg)

2lC􏰄 y ̇+lψ ̇􏰅2lC􏰄y ̇−lψ ̇􏰅

</div>
</div>
<div class="layoutArea">
<div class="column">
ψ ̈= f α δ− f − r α − r Iz x ̇ Iz x ̇

</div>
</div>
<div class="layoutArea">
<div class="column">
X ̇ =x ̇cosψ−y ̇sinψ Y ̇ =x ̇sinψ+y ̇cosψ

otherwise, since the lateral tire forces are zeros, we only consider the longitudinal model.

</div>
</div>
<div class="layoutArea">
<div class="column">
4.5 Measurements

The observable states are:

4.6 Physical constraints

The system satisfies the constraints that:

</div>
</div>
<div class="layoutArea">
<div class="column">
 x ̇ 

 y ̇ 

  ψ ̇   y = X  Y 

ψ

</div>
</div>
<div class="layoutArea">
<div class="column">
|δ|􏰆π6 rad

F 􏰇0andF 􏰆15736N x ̇ 􏰇 10−5 m/s

</div>
</div>
<div class="layoutArea">
<div class="column">
9

</div>
</div>
</div>
<div class="page" title="Page 10">
<div class="layoutArea">
<div class="column">
Name

δ or δf F

m

Cα Iz

Fpq

f delT

</div>
<div class="column">
Table 1: Model parameters. Unit

</div>
</div>
<div class="layoutArea">
<div class="column">
Description

Front wheel angle Total input force Vehicle mass

Cornering stiffness of each tire Yaw intertia

Tire force, p ∈ {x, y},q ∈ {f, r} Rolling resistance coefficient Simulation timestep

</div>
<div class="column">
rad N kg

N

kg mˆ2 N

N/A sec

</div>
<div class="column">
Value

Input Input 1888.6

20000

25854

Depends on input force 0.019

0.032

</div>
</div>
<table>
<tbody>
<tr>
<td>
<div class="layoutArea">
<div class="column">
(x ̇, y ̇)

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Vehicle’s velocity along the direction of vehicle frame

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m/s

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
State

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
(X,Y)

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Vehicle’s coordinates in the world frame

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
State

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
ψ , ψ ̇

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Body yaw angle, angular speed

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
rad, rad/s

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
State

</div>
</div>
</td>
</tr>
</tbody>
</table>
<table>
<tbody>
<tr>
<td>
<div class="layoutArea">
<div class="column">
lr

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Length from rear tire to the center of mass

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
1.39

</div>
</div>
</td>
</tr>
<tr>
<td>
<div class="layoutArea">
<div class="column">
lf

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
Length from front tire to the center of mass

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
m

</div>
</div>
</td>
<td>
<div class="layoutArea">
<div class="column">
1.55

</div>
</div>
</td>
</tr>
</tbody>
</table>
<div class="layoutArea">
<div class="column">
4.7 Simulation

</div>
</div>
<div class="layoutArea">
<div class="column">
Figure 4: Simulation code flow

</div>
</div>
<div class="layoutArea">
<div class="column">
Several files are provided to you within the controllers/main folder. The main.py script initializes and instantiates necessary objects, and also contains the controller loop. This loop runs once each simulation timestep. main.py calls your controller.py’s update method

</div>
</div>
<div class="layoutArea">
<div class="column">
10

</div>
</div>
</div>
<div class="page" title="Page 11">
<div class="layoutArea">
<div class="column">
on each loop to get new control commands (the desired steering angle, δ, and longitudinal force, F). The longitudinal force is converted to a throttle input, and then both control commands are set by Webots internal functions. The additional script util.py contains functions to help you design and execute the controller. The full codeflow is pictured in Figure 4.

Please design your controller in the your controller.py file provided for the project part you’re working on. Specifically, you should be writing code in the update method. Please do not attempt to change code in other functions or files, as we will only grade the relevant your controller.py for the programming portion. However, you are free to add to the CustomController class’s init method (which is executed once when the CustomController object is instantiated).

4.8 BaseController Background

The CustomController class within each your controller.py file derives from the Base- Controller class in the base controller.py file. The vehicle itself is equipped with a Webots-generated GPS, gyroscope, and compass that have no noise or error. These sensors are started in the BaseController class, and are used to derive the various states of the vehicle. An explanation on the derivation of each can be found in the table below.

</div>
</div>
<div class="layoutArea">
<div class="column">
Name (X, Y ) (x ̇,y ̇) ψ

ψ ̇

4.9 Trajectory Data

</div>
<div class="column">
Table 2: State Derivation.

Explanation

From GPS readings

From the derivative of GPS readings From the compass readings

From the gyroscope readings

</div>
</div>
<div class="layoutArea">
<div class="column">
The trajectory is given in buggyTrace.csv. It contains the coordinates of the trajectory as (x, y). The satellite map of the track is shown in Figure 5.

</div>
</div>
<div class="layoutArea">
<div class="column">
11

</div>
</div>
</div>
<div class="page" title="Page 12">
<div class="layoutArea">
<div class="column">
5 Reference

</div>
</div>
<div class="layoutArea">
<div class="column">
Figure 5: Buggy track[3]

</div>
</div>
<div class="layoutArea">
<div class="column">
<ol>
<li>Rajamani Rajesh. Vehicle Dynamics and Control. Springer Science &amp; Business Media, 2011.</li>
<li>Kong Jason, et al. “Kinematic and dynamic vehicle models for autonomous driving control design.” Intelligent Vehicles Symposium, 2015.</li>
<li>cmubuggy.org, https://cmubuggy.org/reference/File:Course_hill1.png</li>
<li>“PID Controller – Manual Tuning.” Wikipedia, Wikimedia Foundation, August 30th,
2020. https://en.wikipedia.org/wiki/PID_controller#Manual_tuning
</li>
</ol>
</div>
</div>
<div class="layoutArea">
<div class="column">
12

</div>
</div>
</div>
