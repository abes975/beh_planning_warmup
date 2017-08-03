#include <iostream>
#include "vehicle.hpp"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a)
{
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;
}

/* Copy constructor */


Vehicle::~Vehicle() {}

vector<string> valid_next_states(string& state)
{
    vector<string> valid_states;
    if(state == "CS") {
      exit(1);
    } else if (state == "KL") {
        valid_states.push_back("KL");
        //valid_states.push_back("PLCL");
        //valid_states.push_back("PLCR");
        valid_states.push_back("LCL");
        valid_states.push_back("LCR");
    } else if (state == "PLCL") {
        //valid_states.push_back("PLCL");
        valid_states.push_back("LCL");
    } else if (state == "PLCR") {
        //valid_states.push_back("PLCR");
        valid_states.push_back("LCR");
    } else if (state == "LCR") {
        //valid_states.push_back("LRC");
        //valid_states.push_back("PLCL");
        //valid_states.push_back("PLCR");
        valid_states.push_back("KL");
        valid_states.push_back("LCL");
        valid_states.push_back("LCR");
    } else if (state == "LCL") {
        //valid_states.push_back("LCL");
        //valid_states.push_back("PLCL");
        //valid_states.push_back("PLCR");
        valid_states.push_back("KL");
        valid_states.push_back("LCL");
        valid_states.push_back("LCR");
    }
    return valid_states;
}


void filter_prediction_by_lane(const map<int, vector< vector<int> >>& pred, int lane, map<int, vector< vector<int> >>& filtered)
{
  map<int, vector< vector<int> >>::const_iterator cit;
  for(cit = pred.begin(); cit!= pred.end(); ++cit) {
    if (cit->first != -1) {
      const vector< vector<int> >& data = cit->second;
      for(int i = 0; i < data.size(); i++)
          if(data[i][0] == lane) {
            filtered[cit->first] = data;
            break;
          }
    }
  }
}

struct cost_data
{
  	int proposed_lane;
		double avg_speed;
		int max_accel;
		double rms_acceleration;
		int closest_approach;
		int end_distance_to_goal;
		int end_lanes_from_goal;
		bool collides;
    int collision_time;
};


double change_lane_cost(const Vehicle& v, struct cost_data& data)
{
	int proposed_lanes = data.end_lanes_from_goal;
  // VEry bad name...
	int cur_lanes = data.proposed_lane;
	double cost = 0.0;
	if (proposed_lanes > cur_lanes)
		cost = abs(proposed_lanes- cur_lanes) * 10000;
	if (proposed_lanes < cur_lanes)
	 	cost = -10000 * abs(proposed_lanes- cur_lanes);
	if (cost != 0)
	//	cout << "!!cost for lane change is " << cost << endl;
  //cout << "Change lane cost = " << cost << endl;
	return cost;
}

double out_of_the_road_cost(const Vehicle& v, struct cost_data& data)
{
  double cost = 0;
  if (data.proposed_lane < 0 || data.proposed_lane > 3)
    cost = 1000000;
  cout << "Out of the road cost " << cost << endl;
  return cost;
}


double distance_from_goal_lane(const Vehicle& v, struct cost_data& data)
{
	int distance = abs(data.end_distance_to_goal);
	distance = max(distance, 1);
  //cout << "dentro costo distanza distance from goal = " << distance << " avg_speed = " << data.avg_speed << endl;

	double time_to_goal = double(distance) / data.avg_speed;
	int lanes = data.end_lanes_from_goal;
	double multiplier = double(5 * lanes) / time_to_goal;
  //cout << "dentro costo distanza Ma anche qui come cazzo " << lanes << " time to goal " << time_to_goal << " multiplier = " << multiplier << endl;
	double cost = multiplier * 100000;
  cout << "distance goal lane cost = " << cost << endl;
	return cost;
}

double inefficiency_cost(const Vehicle& v, struct cost_data& data)
{
	double speed = data.avg_speed;
	double target_speed = v.target_speed;
  //cout << " trajectory_avg_speed = " << speed << " target_speed = " << target_speed << endl;
	int diff = target_speed - speed;
	double pct = double (diff) / target_speed;
	double multiplier = pow(pct,2);
	double cost = multiplier * 1000;
  cout << "inefficenciency cost = " << cost << endl;
  return cost;
}

double collision_cost(const Vehicle& v, struct cost_data& data)
{
  double cost = 0;
  if (data.collides) {
    //cout << "MERDISSIMA WE WILL HAVE A COLLISION AT TIME = " << data.collision_time << endl;
		int time_till_collision = data.collision_time;
		//exponent = (float(time_til_collision) ) ** 2
		double mult = exp(-pow(time_till_collision,2));
		cost = mult * 1000000;
  }
  cout << "Collision cost = " << cost << endl;
  return cost;
}

double buffer_cost(const Vehicle& v, struct cost_data& data)
{
  double cost;
	int closest = data.closest_approach;
  // This means collision....
	if (closest == 0) {
    cost = 10 * 100000;
    cout << "buffer cost = " << cost << endl;
    return cost;
  }

	double timesteps_away = closest / data.avg_speed;
	if(timesteps_away > 1.5) {
    cost = 0.0;
    cout << "buffer cost = " << cost << endl;
		return cost;
  }

	double multiplier = 1.0 - pow((timesteps_away / 1.5), 2);
	cost =  multiplier * 100000;
  cout << "buffer cost = " << cost << endl;
  return cost;
}

struct AccelCmp {
  bool operator()(const int a, const int b) {
    return abs(a) > abs(b);
  }
};

double calculate_total_cost(const Vehicle& v, vector<vector<int> >& future_traj,
            const map<int, vector<vector<int> >>& predictions)
{
    double cost = 0.0;
    AccelCmp acc_cmp;
    map<int,vector < vector<int> >> filtered;
    int L = 1;
    // future[i][0] = lane for time i
    // future[i][1] = s for time i
    vector<int> before_move = future_traj[0];
    vector<int> start = future_traj[1];
    vector<int> end = future_traj.back();
    int end_distance_to_goal = v.goal_s - end[1];
    //cout << "dentro calculate_total_cost final trajectory lane should be " << end[0]  << "and goal lane should be = " << v.goal_lane << endl;
	  int end_lanes_from_goal = abs(v.goal_lane - end[0]);
    double dt = double(future_traj.size()-1);
    int proposed_lane = start[0];
    //cout << " Delta time = " << dt << " end[1] = " << end[1] << " vehicle.s " << before_move[1] << endl;
    double avg_speed = double(end[1] - before_move[1]) / dt;

    vector<int> accels;
    int closest_approach = std::numeric_limits<int>::max();
    int dist;
    bool collides = false;
    int collision_time = future_traj.size();
    // In filtered we have only trajectories of other vehicle that are in
    // the same lane as our vehicle...so we will use them
    // check time after time if we collide.
    filter_prediction_by_lane(predictions, proposed_lane, filtered);
    // calculate some data...
    vector<vector<int>>::const_iterator cit;
    vector<int>::const_iterator v_cit;
    //int prev_s = future_traj[0][1];
    //int prev_a = future_traj[0][3];
    for (cit = future_traj.begin()+1; cit != future_traj.end(); cit++) {
        //int tmp_l = cit->at(0);
        int tmp_s = cit->at(1);
        int tmp_v = cit->at(2);
        int tmp_a = cit->at(3);
        accels.push_back(tmp_a);
        // Check collision between two contigous future states...
        map<int, vector<vector<int> >>::const_iterator m_cit;
        for(m_cit = filtered.begin(); m_cit != filtered.end(); ++m_cit) {
          //cout << "checking collision with car id " << m_cit->first << endl;
          const vector<vector<int>>& others = m_cit->second;
          for(int t = 1; t < future_traj.size(); t++) {
            int other_prev_s = others[t-1][1];
            //cout << "considering time prevision = " << t << endl;
            //cout << "others[" << t << "][" << 1 << "] = " << others[t][1] << endl;
            // We have at index 1 the s position of other vehicles in the same lane
            if((others[t-1][1] < tmp_s && others[t][1] >= tmp_s) ||
              (others[t-1][1] > tmp_s && others[t][1] <= tmp_s) ||
              (others[t-1][1] == tmp_s && (others[t][1] - others[t-1][1] <= tmp_v))) {
                //cout << "Merdissima we have a collision at time " << t << endl;
                collides = true;
                if(t < collision_time)
                  collision_time = t;
            }
            dist = abs(others[t][1] - tmp_s);
            if (dist < closest_approach)
              closest_approach = dist;
//          }
          }
        }
    }
    int max_accel = *max_element(accels.begin(), accels.end(), acc_cmp);
    vector<int> rms_accels;
    for(int i = 0; i < (int) accels.size(); i++)
      rms_accels.push_back(pow(accels[i],2));
	  int num_accels = rms_accels.size();
	  double rms_acceleration = double(std::accumulate(rms_accels.begin(), rms_accels.end(), 0)) / num_accels;

    struct cost_data c;

    c.proposed_lane = proposed_lane;
    c.avg_speed = avg_speed != 0 ? avg_speed : std::numeric_limits<double>::epsilon();
    //c.avg_speed = avg_speed;
    c.max_accel = max_accel;
    c.rms_acceleration = rms_acceleration;
    c.closest_approach = closest_approach;
    c.end_distance_to_goal = end_distance_to_goal;
    c.end_lanes_from_goal = end_lanes_from_goal;
    c.collides = collides;
    c.collision_time = collision_time;

    cost += out_of_the_road_cost(v, c);
    cost += change_lane_cost(v, c);
    cost += distance_from_goal_lane(v, c);
    cost += inefficiency_cost(v, c);
    cost += collision_cost(v, c);
    cost += buffer_cost(v, c);
    return cost;
}

void Vehicle::get_next_state1(map<int, vector< vector<int> > >& predictions, int horizon=1)
{
    double cost;
    double min_cost = std::numeric_limits<double>::max();
    vector<string> states = valid_next_states(state);
    for(int i = 0; i < states.size(); i++) {
        cost = 0;
        // create copy of our vehicle
        Vehicle experiment = *this;
        experiment.state = states[i];
        //cout << "Evaluating state: " << states[i] <<  endl;
        experiment.realize_state(predictions);
        vector<vector<int> > future = experiment.generate_predictions(horizon);
        cost = calculate_total_cost(experiment, future, predictions);
        //cout << "\tTotal cost for state " << experiment.state << " = " << cost << " curren min cost " << min_cost << " for state " << this->state << endl;
        if(cost < min_cost) {
          min_cost = cost;
          //cout << "\tNEW min cost is " << min_cost << " for state " << experiment.state << endl;
          this->state = experiment.state;
        }
    }
    cout << "NEXT STATE WILL BE " << this->state << endl << endl;
}


// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions)
{
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    /*
    map<int,vector < vector<int> > >::const_iterator cit;
    for(cit = predictions.begin(); cit != predictions.end(); ++cit) {
        if(cit->first != -1) {
          cout << "Key: " << cit->first << ": " << endl;
          const vector < vector<int> > &v = cit->second;
          for(int i = 0; i <  *(int) v.size(); i++) {
              cout << " Timestamp: " << i << endl;
              for(int j = 0; j < (int) v[i].size(); j++) {
                // idx 0 lane, idx 1 : s, idx 2 = v, idx 3 = acc
                cout << "v["<< i << "]["<<j <<"] = " << v[i][j] << endl;
              }
          }
        }
        cout << endl;
    }
    */
    //cout << "Chiamata la update ...Entering state is " << state << endl << endl
    get_next_state1(predictions, 5);
    //cout << "Fine predizioni...Exiting state is  "<< state << endl << endl;
}


void Vehicle::configure(vector<int> road_data)
{
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    max_acceleration = road_data[2];
    goal_lane = road_data[3];
    goal_s = road_data[4];
}

string Vehicle::display()
{
	ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt = 1)
{
  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t)
{
  	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time)
{
	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps)
{
	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1;

	for (int t = 0; t < timesteps+1; t++) {
	 if( collides_with(other, t) ){
			collider_temp.collision = true;
			collider_temp.time = t;
    	return collider_temp;
  	}
	}
return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions)
{
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0) {
    	realize_constant_speed();
    } else if(state.compare("KL") == 0) {
    	realize_keep_lane(predictions);
    } else if(state.compare("LCL") == 0) {
    	realize_lane_change(predictions, "L");
    } else if(state.compare("LCR") == 0) {
    	realize_lane_change(predictions, "R");
    } else if(state.compare("PLCL") == 0) {
    	realize_prep_lane_change(predictions, "L");
    } else if(state.compare("PLCR") == 0) {
    	realize_prep_lane_change(predictions, "R");
    }
}

void Vehicle::realize_constant_speed()
{
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int ss)
{
  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while(it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;
    if((v[0][0] == lane) && (v[0][1] > ss)) {
  	 in_front.push_back(v);
    }
    it++;
  }

  if(in_front.size() > 0) {
  	int min_s = 1000;
  	vector<vector<int>> leading = {};
  	for(int i = 0; i < in_front.size(); i++) {
  		if((in_front[i][0][1] - ss) < min_s) {
            min_s = (in_front[i][0][1]-ss);
            leading = in_front[i];
  		}
  	}
  	int next_pos = leading[1][1];
  	int my_next = ss + this->v;
  	int separation_next = next_pos - my_next;
  	int available_room = separation_next - preferred_buffer;
  	max_acc = min(max_acc, available_room);
  }
  return max_acc;
}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions)
{
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction)
{
	int delta = -1;
  if (direction.compare("L") == 0) {
  	delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction)
{

  int delta = -1;
  if (direction.compare("L") == 0) {
  	delta = 1;
  }
  int lane = this->lane + delta;


  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while(it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }
    it++;
  }

  if(at_behind.size() > 0) {
  	int max_s = -1000;
  	vector<vector<int> > nearest_behind = {};
  	for(int i = 0; i < at_behind.size(); i++) {
      if((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
  			nearest_behind = at_behind[i];
  		}
  	}
  	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
  	int delta_v = this->v - target_vel;
  	int delta_s = this->s - nearest_behind[0][1];
  	if(delta_v != 0) {
		  int time = -2 * delta_s/delta_v;
  		int a;
  		if (time == 0) {
  			a = this->a;
  		} else {
        a = delta_v/time;
  		}
  		if(a > this->max_acceleration) {
  			a = this->max_acceleration;
  		}
  		if(a < -this->max_acceleration) {
        a = -this->max_acceleration;
  		}
  		this->a = a;
  	}	else {
		  int my_min_acc = max(-this->max_acceleration,-delta_s);
  		this->a = my_min_acc;
  	}
  }
  else
    this->can_change_line = true;
    this->delta = delta;
}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10)
{
  vector<vector<int> > predictions;
  if(horizon < 1)
    horizon = 1;
  // Prediction at position 0 is current state!!!
  for( int i = 0; i <= horizon; i++) {
    vector<int> check1 = state_at(i);
    //vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(check1);
  }
  return predictions;
}
