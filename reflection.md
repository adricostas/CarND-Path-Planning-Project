

[image1]: /home/acl/Pictures/sim_image.png
[image2]: /home/acl/Pictures/FSM.png
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

In the following I will adress the [Rubic](https://review.udacity.com/#!/rubrics/1020/view) points of this Path Planning Project

## Introduction
The target of this project was to drive a car safely around a circular track which has a length of 4.32 miles. To be successful the car was not allowed to:
1. Collide with another car
2. Leave the right three lines
3. The car has to stay within its current line, except for reasonable overtaking maneuvers
4. Do not drive over 50mph
5. Do not make movements that have a too high acceleration or jerk

My final submission code was able to drive the car safely around the track multiple times. The following video shows the first round with multiple line chances and an emergency brake after a car in front of me changed into my line.

![alt text][image1]


## Path planning

### Speed Limit
To drive the car within the speed limit of 50mph I set the speed limit for the car slightly below 50 MPH to 49.5 MPH.  

```cpp
  // maximal velocity!
  double max_vel = 49.5;
``` 

### Max Acceleration and Jerk
Like the speed limit, the max acceleration and jerk are handled by the spline approximation technique which is exceptionally good explained in the walkthrough video of the course. After following the provided code hints from the video and using the provided spline technique from this [source](http://kluge.in-chemnitz.de/opensource/spline/) the  the acceleration and jerk was in range at all times .

### Staying in lane and lane changes for overtaking maneuvers
My approach to decide when to stay in lane and when to overtake was to use a Finite State Machine with the following states: Keep Lane, Lane Change Left and Lane Change Right. The transitions among them are made after evaluating the cost of each action.

![alt text][image2]

In order to carry out this, I followed the steps shown below:

* Make predictions about the every detected vehicle. 

```cpp
	for(int i=0; i < sensor_fusion.size(); i++){
          
          check_d = sensor_fusion[i][6];
          check_lane = getLaneNumber(check_d);

          vx = sensor_fusion[i][3];
          vy = sensor_fusion[i][4];
          check_speed = sqrt(vx*vx + vy*vy);
          check_car_s = sensor_fusion[i][5];

          //Predict where that car will be in the future
          check_car_s += ((double) prev_size * 0.02 * check_speed);

          predictions[sensor_fusion[i][0]] = pair<int,double>(check_lane, check_car_s);
      

        }
```


* Define cost functions. For the keep lane action I have defined a cost that takes into account the distance between the target lane (middle lane) and the current lane and the distance to the front car in the current lane. For the lane changes is necessary to check if overtaking is safe. In order to do that I take into account the front and back gaps between the ego_car and the vehicles that are driving in the destination lane. 

```cpp
	float cost_for_action(unordered_map<int,pair<int,double> >  predictions, string action, int current_lane, int target_lane, int available_lanes, int target_speed){
	  map<int, float> front_gaps;
	  map<int, float> back_gaps;
	  for (int l=0; l<available_lanes; l++) {
	    front_gaps[l] = numeric_limits<float>::max();
	    back_gaps[l] = numeric_limits<float>::max();
	  }

	  int s = predictions[-1].second;
	  for (auto const& entry: predictions) {
	    auto vehicle_id = entry.first;
	    int vl = entry.second.first, vs = entry.second.second;
	    int gap = vs - s;
	    if (vehicle_id != -1 && gap > 0 && front_gaps[vl] > gap) {
	      front_gaps[vl] = gap;
	    }

	    if (vehicle_id != -1 && gap <= 0 && back_gaps[vl] > -gap) {
	      back_gaps[vl] = -gap;
	    }
	  }

	  int diff_lane = current_lane - target_lane;
	  if (action == "KL") {
	    float diff_lane_cost = abs(diff_lane);
	    float front_gap_cost = target_speed / front_gaps[current_lane];    
	    return diff_lane_cost + 1.5*front_gap_cost ;
	  }
	  
	  if (action == "LCR") {
	    if (current_lane == available_lanes - 1 ) return numeric_limits<float>::max();
	    float diff_lane_cost = abs(diff_lane+1);
	    float front_gap_cost = target_speed / front_gaps[current_lane + 1];
	    float  back_gap_cost = target_speed / back_gaps[current_lane + 1];
	    return diff_lane_cost + 1.5*front_gap_cost + 2*back_gap_cost;
	  }
	  if (action == "LCL") {
	    if (current_lane == 0) return numeric_limits<float>::max();
	    float diff_lane_cost = abs(diff_lane-1);
	    float front_gap_cost = target_speed / front_gaps[current_lane - 1];
	    float  back_gap_cost = target_speed / back_gaps[current_lane - 1];
	    return diff_lane_cost + 1.5*front_gap_cost + 2*back_gap_cost;
	  }
	  return numeric_limits<float>::max();
	}

```
* Define transitions between states and actions for each state. When the Finite States Machine is in "Keep Lane" state, I check if there is a car too close in front in the same lane in order to reduce the velocity of the ego_car and avoid the collision. In the change lane states I modify the variable that indicates the lane in which that car should stay.

```cpp
	void update_state(unordered_map<int, pair<int,double> > predictions, string &state,  int target_lane, float target_speed){
	  int current_lane = predictions[-1].first;
	  vector<string> possible_actions;
	  if (state == "KL") {
	    possible_actions = { "KL", "LCL", "LCR" };
	  }
	  if (state == "LCL") {
	    possible_actions = { "KL", "LCL" };
	  }
	  if (state == "LCR") {
	    possible_actions = { "KL", "LCR" };
	  }

	  vector<float> costs;
	  for (auto const& a: possible_actions) {
	    float cost = cost_for_action(predictions, a, current_lane, target_lane, 3, target_speed);
	    costs.push_back(cost);   
	 
	}

	  int min_pos = distance(costs.begin(), min_element(costs.begin(), costs.end()));  
	  state = possible_actions[min_pos];

	}

	if(state.compare("KL") == 0)
		{
		  double ego_s = predictions[-1].first;
		  double vs;
		  double vl;
		  int vehicle_id;
		  for (auto const& entry: predictions) {
		    vehicle_id = entry.first;
		    vl = entry.second.first;
		    vs = entry.second.second;
		    int gap = vs - car_s;
		    if (vehicle_id != -1 && gap > 0 && gap < 30 && lane == vl) {
		       too_close = true;              
		    }
		  }
		  }    
	    
	else if(state.compare("LCL") == 0)
	{
		  lane = lane - 1;
	}
	else if(state.compare("LCR") == 0)
	{
	    lane = lane + 1;
	}
	if(too_close){
	    ref_vel -= 0.224;
	}
	else if(ref_vel < max_vel){
	    ref_vel += 0.224
	}


```


### Further improvements
1) Consider PLCR and PLCL state in my Finite States Machine.
2) If the car is behind another car and cannot overtake it is not set to the speed of the car in front. Instead the car decelerate if it is to close and accelerate if it is too far behind. It would be more efficient to set the speed to speed of the car in front. 
3) In order to compute the cost of changing lane the velocity of the vehicles could be use. For instance, we shouldn't change lane if the car behind us in the destination lane is driving faster than us and is relatively close.

   





   





 
 the highway loop.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

