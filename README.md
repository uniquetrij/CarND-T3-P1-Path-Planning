# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Model

The primary intuition for this model is as follows:

1. If the car is on a free lane with no other vehicles immediately ahead of it (say within 30 meters in front), the car should keep moving on the lane, possibly with its maximum speed limit. If the speed is lower than the max limit, the car should try to speed up so as to gradually reach the max speed.

2. If the car is being blocked by another vehicle ahead of it on the same lane, it should check whether any of the adjacent lanes are free so as to overtake the vehicle. If multiple such lanes exists, then the car should chose the lane that has maximum clearance ahead of it, to prevent too frequent lane switching unnecessarily.

3. Before changing a lane, the car should check whether another car is approaching from behind in the other lane. It should change lane only if the vehicle approaching is far behind or has much lesser velocity than the car so as to avoid collision. 

4. Also while changing lane, the car should keep enough distance from the vehicle ahead of it in the same lane to avoid collision.

Keeping the above points in mind, my model works as follows.
 
I calculate the front and rear clearance of each lane from the current position of the car using the `lanesCheck` method (lines 190-236). This method also preserves the speeds of the nearest front and rear vehicles. 

At first, only the current lane clearances are computed (line 389). Based on this, the next step is to choose whether to continue on the same lane or change to an adjacent lane. The lane selection us done using the `selectLane` method. If the front clearance is not enough (less than 30 meters), the method compute the clearances of the other lanes (line 268). The adjacent lane with max front clearance and adequate rear clearance is chosen (lines 270-292). If no suitable lanes were found, the car will continue on the same lane. Also the variable `stayInLane` keeps a track of how long the car has been in the same lane before it can change the lane. This prevents too frequent lane switching.

Finally it is required to regulate the velocity of the car. This is done by the `checkVelocity` method. If the car is being obstructed by another vehicle and adjacent lanes are not available to shift to, the speed needs to be reduced to that of the vehicle its following (lines 245-244). Otherwise, its velocity should tend to reach to the max limit (lines 246-149). In either case, to prevent jerking, it is necessary to check that the velocity change is not very large (lines 251-259). 

Once the lane and velocity is determined, it is time to create a trajectory for the vehicle's path. A total of 8 waypoints  (x,y and yaw heading of the car) are generated, first two of them being the previous and current states respectively. The other 6 points are future trajectory estimation points at intervals of 60 meters from one another. These points are then converted to car coordinates from map coordinates that are finally used to compute a spline that passes through each of them. Now using this spline, I generated other waypoints with the lane and velocity computed previously at an interval of 0.02 seconds. Next I converted the points back to map coordinates and returned them to the simulator to drive the car through that path (lines 395-483). These section was taken directly as described in the project walkthrough.

## Conclusion

The result of the execution of the model on the simulator was very good. The car maintained maximum velocity whenever possible, and changed lanes whenever required and a lane was available. Otherwise it maintained the velocity of the car preceding it unless one of the adjacent lanes cleared up. 

The parameters tuned are specific to the hardware on which the model was tested. It might need retuning if the hardware changes.







