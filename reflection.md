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
.