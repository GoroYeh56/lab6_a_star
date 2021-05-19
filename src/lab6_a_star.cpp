#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" // KEY!!
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"

#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <set>
#include<vector>
using namespace std;

// #define DEBUG_LAB2
#define DEBUG_PROGRAM
// #define MANHATTAN_DISTANCE
// #define DEBUG_MAP
// #define DEBUG_OBSTACLE

/*
    Publish:f
        /cmd_vel
            geometry_msgs/Twist.msg
            - Vector3 linear (x, y, z)
            - Vector3 angular

    Subscribe:
        /map
            - Header
            - MapMetaData   info
            - int8[] data

        /move_base_simple/goal
            geometry_msgs/PoseStamped.msg
            - header
            - pose
        /robot_pose
            geometry_msgs/Twist.msg

        geometry_msgs/Pose.msg:
        - Point
            float64 x
            float64 y
            float64 z
        - Quaternion
            float64 x
            float64 y
            float64 z
            float64 w
*/

ros::Publisher cmd_vel_pub;
ros::Subscriber robot_pose_sub;
ros::Subscriber goal_pose_sub;
ros::Subscriber map_sub;

const float pi = 3.14159265358979323846;

/* 
0: A_star_planning
1: Tracking
2: Finished
3: MOVING
4: TURNING
5: IDLE
*/
enum State
{
    A_star_planning,
    Tracking_state,
    Finished,
    MOVING,
    TURNING,
    IDLE
};
State program_state = Finished; //Initial turtle state/pose.
State robot_state = MOVING;              //Initial turtle state/pose.

float rho, alpha, beta;
float goal_x = 0;
float goal_y = 0;
float goal_z = 0;
float goal_theta = 0;
float robot_x = 0;
float robot_y = 0;
float robot_theta = 0;

float error_x;
float error_y;
float error_theta;

void Robot_Pose_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{

    // robot current pose feedback (from sensors measurement)
    robot_x = msg->linear.x;
    robot_y = msg->linear.y;
    robot_theta = msg->angular.z;

    // Constrain robot_theta to [-pi, pi]
    if (robot_theta > pi)
    {
        while (robot_theta > pi)
            robot_theta -= 2 * pi;
    }
    if (robot_theta <= -pi)
    {
        while (robot_theta <= 2 * pi)
            robot_theta += 2 * pi;
    }

    float delta_x = goal_x - robot_x;
    float delta_y = goal_y - robot_y;
    float delta_theta = goal_theta - robot_theta;

    // Convert to polar coordinate.
    rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

    // % xc : robot_x.
    // % xi : initial_x.

    // % 1st - quardrum
    float x_c = goal_x;
    float y_c = goal_y;
    float angle_c = goal_theta;

    float x_i = robot_x;
    float y_i = robot_y;
    float angle = robot_theta;

    float x_err = error_x;
    float y_err = error_y;

    if (x_c - x_i >= 0 && y_c - y_i >= 0)
    {
        alpha = (atan2(y_err, x_err) - angle);
        beta = angle_c - atan2(y_err, x_err);
    }
    // % 2nd - quardrum
    if (x_c - x_i <= 0 && y_c - y_i >= 0)
    {
        alpha = (atan2(y_err, x_err) - angle);
        beta = angle_c + (2 * pi - atan2(y_err, x_err));
    }
    // % 3rd quardrum
    if (x_c - x_i <= 0 && y_c - y_i <= 0)
    {
        alpha = pi + atan2(y_err, x_err) - angle + pi;
        beta = -(pi + atan2(y_err, x_err) - angle_c) + pi;
    }

    // % 4th quardrum
    if (x_c - x_i >= 0 && y_c - y_i <= 0)
    {
        alpha = pi + atan2(y_err, x_err) - angle + pi;
        beta = -(pi + atan2(y_err, x_err) - angle_c) + pi;
    }

    // alpha = atan2(delta_y, delta_x) - (robot_theta);

    /* ------ TODO: 4 Quadrant ------ */

    // alpha constraint: -pi ~ pi.
    if (alpha <= -pi)
    {
        while (alpha <= -pi)
        {
            alpha += 2 * pi;
        }
    }
    if (alpha > pi)
    {
        while (alpha > pi)
            alpha -= 2 * pi;
    }

    if (beta <= -pi)
    {
        while (beta <= -pi)
        {
            beta += 2 * pi;
        }
    }
    if (beta > pi)
    {
        while (beta > pi)
            beta -= 2 * pi;
    }

    // beta = delta_theta - alpha;
}

float Final_goal_theta = 0;
void Goal_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Once receive a command, change state!
    program_state = A_star_planning;
    robot_state = MOVING;

    float qx, qy, qz, qw;
    goal_x = (msg->pose.position.x)*(100/20); // (100/20) : CovertFactor.
    goal_y = (msg->pose.position.y)*(100/20); // one grid = 10cm

    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;

    goal_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qy * qy));

	if(goal_x<0&&goal_theta>0)
	{
		goal_theta = -1*goal_theta;
	}
    Final_goal_theta = goal_theta;
    // goal_theta = thetaConstraint(goal_theta);

}

// #define WIDTH 100
// #define HEIGHT 100

#define WIDTH 640
#define HEIGHT 608
#define NUM_NEIGHBORS 8



// #define SINGLE_GOAL_TEST
// #define TEST_SMALL_GOAL

/* ---- Goal locations ----- */

/*
#ifndef SINGLE_GOAL_TEST
    #define NUM_OF_GOALS 4
    #ifdef TEST_SMALL_GOAL
    const int goals[4][2] =
        {
            {3, 4},
            {3, -2},
            {-4, -3},
            {-4, 4}};
    #else
    const int goals[4][2] =
        {
            {30, 40},
            {30, -20},
            {-40, -30},
            {-40, 40}
            };
    #endif
#else
    #define NUM_OF_GOALS 1
    const int goals[1][2] =
    {
        {-40, -30}
    };

#endif
*/



// const int EXPAND_NUMBER = 2;
#define EXPAND_NUMBER 3// Expand to 4 grids. 40cm
#define EXPAND_DIAGONAL
string Pathname;
string Envname;

bool read_map = false;
int env_map[640*608];

// For visualization
char path_map[640*608];

// Note: this ONLY run ONE TIME !
void Map_Callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(!read_map){
            ROS_INFO("I get msg from map!!! Hehehaha Tony Chiu!");
        for (int i = 0; i < HEIGHT * WIDTH; i++)
        {
            env_map[i] = msg->data[i];
            if(env_map[i] == 100){
                path_map[i] = '#';                    
            }
            else if(env_map[i] == 0)
                path_map[i] = ' ';
            else
                path_map[i] = '-'; // -1
        }


        // First Step: Expand (Note: use 50, otherwise, the world block since we use a for-loop!)
        for (int i = 0; i < HEIGHT * WIDTH; i++)
        {
            // printf("Data[%d][%d]: %d\n", i % WIDTH, i / WIDTH, env_map[i]);    
            // Expand obstacle:
                if(env_map[i] == 100){
                    int x = i%WIDTH;
                    int y = i/WIDTH;

                    // cout<<"expand index: "<<i<<", x: "<<x<<", y: "<<y<<" \n";
                    if(x==0 || x==99 || y==0 || y==99)
                        continue;   // Don't expand the boundary since we might threaten the goal location!
                    
                    for(int k=1; k<=EXPAND_NUMBER; k++){

                        if(x-k >=0 && env_map[x-k +y*WIDTH]!=100 ){
                            env_map[(x-k)+y*WIDTH] = 50;
                            path_map[(x-k)+y*WIDTH] = '#';
                        }
                        if(x+k <100 && env_map[x+k +y*WIDTH  ]!=100  ){
                            env_map[(x+k)+y*WIDTH] = 50;
                            path_map[(x+k)+y*WIDTH] = '#';
                        }
                        if(y-k >=0 && env_map[(x)+(y-k)*WIDTH]!=100  ){
                            env_map[(x)+(y-k)*WIDTH] = 50;
                            path_map[x+ (y-k)*WIDTH] = '#';                        
                        }
                        if(y+k <100 && env_map[x+ (y+k)*WIDTH]!=100  ){
                            env_map[(x)+(y+k)*WIDTH] = 50;
                            path_map[x+ (y+k)*WIDTH] = '#';
                        }

                        #ifdef EXPAND_DIAGONAL
                        // Diagonal.
                        // Left Down
                        if( x-k >=0 && y-k >=0 ){
                            env_map[(x-k)+(y-k)*WIDTH] = 50;                        
                            path_map[(x-k)+(y-k)*WIDTH] = '#';
                            // Fill to right side
                            // for(int m=0; m<EXPAND_NUMBER; m++){
                            //     if( (x-k+m) <100){
                            //         env_map[(x-k+m)+(y-k)*WIDTH] = 50;                           
                            //         path_map[(x-k+m)+(y-k)*WIDTH] = '#';                                    
                            //     }
                            // }

                        }
                        // Right Down
                        if( x+k <100 && y-k >=0){
                            env_map[(x+k)+(y-k)*WIDTH] = 50;
                            path_map[(x+k)+(y-k)*WIDTH] = '#';

                            // Fill to Left side
                            // for(int m=0; m<EXPAND_NUMBER; m++){
                            //     if( (x+k-m) >=0){
                            //         env_map[(x+k-m)+(y-k)*WIDTH] = 50;                           
                            //         path_map[(x+k-m)+(y-k)*WIDTH] = '#';                                    
                            //     }
                            // }

                            
                        }
                        // Left Up.
                        if( x-k >=0 && y+k < 100){
                            env_map[(x-k)+(y+k)*WIDTH] = 50;                           
                            path_map[(x-k)+(y+k)*WIDTH] = '#';
                            // for(int m=0; m<EXPAND_NUMBER; m++){
                            //     if( (x-k+m) <100){
                            //         env_map[(x-k+m)+(y+k)*WIDTH] = 50;                           
                            //         path_map[(x-k+m)+(y+k)*WIDTH] = '#';                                    
                            //     }
                            // }

                        }
                        // Right Up
                        if( x+k <100 && y+k <100){
                            env_map[(x+k)+(y+k)*WIDTH] = 50;
                            path_map[(x+k)+(y+k)*WIDTH] = '#';

                            // Fill to Left side
                            // for(int m=0; m<EXPAND_NUMBER; m++){
                            //     if( (x+k-m) >=0){
                            //         env_map[(x+k-m)+(y+k)*WIDTH] = 50;                           
                            //         path_map[(x+k-m)+(y+k)*WIDTH] = '#';                                    
                            //     }
                            // }

                        }

                        // Need to fill rectangle between new diagnonal & new obstacle!
                        /*
                            Ex.: 
                                    #  #
                                     # #
                                      ##
                                    ####    Original:   #
                        
                        */
                        #endif
                    }
                }
            }
        // Second Step: Transform 50 to 100/
        for (int i=0; i<HEIGHT*WIDTH; i++){
            if(env_map[i] == 50)
                env_map[i] = 100;
        }            

            ROS_INFO("Done expanding obstacle.");


        Envname = "Expand_Obs_map" + to_string(EXPAND_NUMBER) + ".txt";
        Pathname = "Path"+ to_string(EXPAND_NUMBER)+ ".txt";

        ROS_INFO("Env Name: %s",Envname.c_str());
        // cout<<"Envname: "<<Envname<<endl;

        ofstream myfile(Envname.c_str(), ios::out | ios::binary);
        // myfile.open("/home/catkin_ws/src/lab3_A_star/Path01.txt");
        // myfile.open();
        if (myfile.is_open())
        {

            for(int j=HEIGHT; j>=0; j--){
                for(int i=0; i<WIDTH; i++){
                 
                    myfile<<path_map[i+j*WIDTH];
                }
                myfile<<endl;
            }

            // myfile << "\n Path after reaching goal "<<goals[counter][0]<<", "<<goals[counter][1]<<"\n";
            ROS_INFO("Done writing to output file!");
            // myfile << "This is another line.\n";
            // for(int count = 0; count < size; count ++){
            //     myfile << x[count] << " " ;
            // }
            myfile.close();
        }
        else cout << "Unable to open file";

            read_map = true;
        }
#ifdef DEBUG_MAP
    for (int i = 0; i < HEIGHT * WIDTH; i++)
    {
        printf("Data[%d][%d]: %d\n", i / WIDTH, i % WIDTH, env_map[i]);
    }
#endif
}

/* -------- Kinematics ---------- */
const float wheel_r = 0.06;         //cm
const float wheel_separaton = 0.24; //cm

const float Max_Wheel_Speed = 2 * pi;

const float max_v = 0.4; // m/s
const float max_w = 0.4;   // rad/s

// const float max_v = 2*wheel_r*Max_Wheel_Speed;
// const float max_w = 2*max_v/wheel_separaton;

/* -------- Gain ---------- */
// const float Krho = 1;   // 2
// const float Ka = 5;     // 8
// const float Kb = -1;    // -1.5
const float Kp = 0.3; // 0.5


const float Krho = 0.5;   // 2
const float Ka = 2.5;     // 8
const float Kb = -1.12 ;    // -1.5


int counter = 0; // indicate "which goal" we are planning
                // Initial goal: goals[0]

/* 
    Now: More State.
    A Big for-loop: (4 goal locations.)
        1. A_star_Planning state.
        2. Tracking state
            A for-loop to send several /goal:
            (1) Moving state.
            (2) Turning state.
        3. IDLE (end)

*/

/* Note:
    Map ( -5 ~ +5 ) into (0 ~ 100)
    Each grid in data[0~9999] is 10m/100 = 10 cm !
    Augmented the unit: make robot_x => *100/10 (unit: 10cm)
*/


/*
    Lab6 Map:
    Map[0][0] => (-10, -21.2)m
    Each grid: data[640 * 608] => 5cm
    Augmented the unit: make robot_x => *100/20 (unit: 5cm)
*/



int Convert_Unit(float num){
    // return (int)num*100/10;
    return (int)num*100/20;
}

// Note: Now the map start from 
int ID_to_X(int id){
    // return id%WIDTH - 50;
    return id%WIDTH - 10*(100/20);
}

int ID_to_Y(int id){
    // return id/WIDTH -50;
    return id/WIDTH- 21.2*(100/20);
}

class location{
public:
    location();     // default constructor.
    location(int x, int y){ // Note: x,y should be 5cm unit!
        this->x = x;
        this->y = y;
        this->id = (x+ (10*(100)/20 )) + (y+ (21.2*100/20) )*WIDTH; 
        // this->id = (x+50) + (y+50)*WIDTH;   // Since origin: (-50,-50) in 10cm unit
        this->f = this->g = 0; // initialize
    }
    int id; // from its x,y
    int x;  // unit: 10cm
    int y;  // unit: 10cm

    float f;
    float g;
    int parent; // an id.
};


vector<location> Path; // Path.push_back(location)

float dist(location a, location b){
    return abs(sqrt(pow((b.x - a.x) ,2) + pow((b.y - a.y) ,2)));
}

float h(location a, location b){
    #ifndef MANHATTAN_DISTANCE
        return abs(sqrt(pow((b.x - a.x) ,2) + pow((b.y - a.y) ,2))); // straight line distance
    #else
        return abs(b.x - a.x) + abs(b.y - a.y);
    #endif
}

void Append_Path(int node_id){
    // Append node_id in front of Path.
    int x = ID_to_X(node_id);
    int y = ID_to_Y(node_id);
    Path.insert(Path.begin(), location(x,y));
    return;
}


// int Parent[10000];
// float f_cost[10000];
// float g_cost[10000];
int Parent[640*608];
float f_cost[640*608];
float g_cost[640*608];


float v, w;
geometry_msgs::Twist twist;

bool should_break = false;


///// LEE /////
#define PI 3.14159265358979323846
float x_r=0;
float y_r=0;
float theta_r=0;
// float x,y,theta;
float rpho;
float delta_x,delta_y,delta_theta;


float thetaConstraint(float thi)
{
	while(abs(thi)>PI)
	{
		thi = (thi > 0) ?  thi - 2*PI : thi + 2*PI;
	}
	thi = (thi == -1*PI) ? -1*thi : thi;
	return thi;
}


///////////////

void Tracking()
{
    // stuck in here!
    while(!should_break){

            error_x = goal_x - robot_x;
            error_y = goal_y - robot_y;
            error_theta = goal_theta - robot_theta;

        #ifdef DEBUG_LAB2
            // ROS_INFO("Robot-State: %d, Goal theta: %.2f, Robot Theta: %.2f", robot_state, goal_theta, robot_theta);
            ROS_INFO("Error: %.3f %.3f %.3f", error_x, error_y, error_theta);
        #endif

            switch (robot_state)
            {
            case MOVING:
                ////////// Main Control Loop! /////////////
                if (abs(error_x) < 0.2 && abs(error_y) < 0.2 && abs(error_theta) < 0.2)
                {
                // if (abs(error_x) < 0.01 && abs(error_y) < 0.01)
                // if (abs(error_x) < 0.3 && abs(error_y) < 0.3)
                // {
                    robot_state = IDLE;
                }
                else if (abs(error_x) < 0.2 && abs(error_y) < 0.2)
                {
                    robot_state = TURNING;
                }
                else
                {
                    /* ----- For Consistency ----- */
                    delta_x = error_x;
                    delta_y = error_y;
                    delta_theta = -error_theta;
                    theta_r = goal_theta;

                    // cout<<"Moving now!!\n";
                    rpho = sqrt(pow(delta_x,2) + pow(delta_y,2));
                    alpha = atan2(delta_y,delta_x) - theta_r - delta_theta;
                    alpha = thetaConstraint(alpha);
                    
                    beta = -1*(alpha + delta_theta);
                    beta = thetaConstraint(beta);
                    
                    v = Krho*rpho;
                    w = Ka*alpha + Kb*beta;

                    /* --- Control Law --- */
                    // v = Krho * rho;
                    // w = Ka * alpha + Kb * beta;
                    // ROS_INFO("Robot is MOVING !!!");
                }
                break;
            case TURNING:

                if (abs(error_theta) < 0.2 || abs(robot_theta + 2 * pi) < 0.2 || abs(error_theta - 2 * pi) < 0.2)
                {
                    robot_state = IDLE;
                }
                else
                {
                    /* --- Control Law --- */
                    v = 0;
                    w = Kp * error_theta;
                }

                break;
            case IDLE:
                // Stop and listen to new goal.
                // reset error signals.
                v = 0;
                w = 0;
                // ROS_INFO("Robot IDLE and break!");
                should_break = true; // break while-loop and return to the main function!
        #ifdef DEBUG_LAB2
                ROS_INFO("Robot reached the goal and is now IDLE.");
        #endif

                break;
            }

            /* Command Inputs Constraints */
            v = (v>max_v)? max_v : v;
            v = (v<-max_v)?-max_v: v;
            w = (w>max_w)? max_w : w;
            w = (w<-max_w)?-max_w: w;

            // Publish cmd_vel
            twist.linear.x = v;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = w;
            cmd_vel_pub.publish(twist);

            ros::spinOnce();
            // if(should_break)
            //     break;

    }

}



///////////////// LEE Controller /////////////////


#define kp 0.5 
#define ka 2.5 
#define kB -1.12 
// float x_r=0;
// float y_r=0;
// float theta_r=0;
float x,y,theta;
// float rpho;
bool command = false;
//-------
// float delta_x,delta_y,delta_theta;
bool theta_test = false;

float degreeConverter(float xx,float yy, float zz, float ww)
{
	return atan2(2*(ww*zz+xx*yy),1-2*(zz*zz+yy*yy));
}



void Tracking_LEE(){
    while(!should_break){

		delta_x = x_r - x;
    	delta_y = y_r - y;
    	delta_theta = theta - theta_r;
    	cout<<"X_r: "<<x_r<<" Y_r "<<y_r<<" Theta_r: "<<theta_r<<"\n";
    	cout<<"delta X: "<<delta_x<<" delta Y: "<<delta_y<<" delta Theta: "<<delta_theta<<"\n";
    	if((abs(delta_theta)<0.3)||(abs(delta_theta+2*PI)<0.3)||(abs(delta_theta-2*PI)<0.3))
    	{
    		theta_test = true;
		}else
		{
			theta_test = false;
		}
		// if( abs(delta_x)<0.1 && abs(delta_y)<0.1 && (theta_test))
		// {
		// 	command = false;//Robot into idle state until new command given
		// }
		switch(command)
		{
			case false:
				//ROS_INFO("IDLE now!!");
				cout<<"IDLE now!!\n";
				v = 0;
				w = 0;
			break;
			case true:
				//ROS_INFO("Moving now!!");
				cout<<"Moving now!!\n";
				rpho = sqrt(pow(delta_x,2) + pow(delta_y,2));
				alpha = atan2(delta_y,delta_x) - theta_r - delta_theta;
				alpha = thetaConstraint(alpha);
				
				beta = -1*(alpha + delta_theta);
				beta = thetaConstraint(beta);
				
				v = kp*rpho;
				w = ka*alpha + kB*beta;
			break;		
		}
		cout<<" V: "<<v<<" W: "<<w<<"\n";
		//ROS_INFO("V: "+v+" W: "+w); 
		twist.linear.x = v;
		twist.linear.y = 0;
		twist.linear.z = 0;
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = w;
		cmd_vel_pub.publish(twist);
		ros::spinOnce();
	}

}
// #define LEE_CONTROL

void A_star_algorithm()
{

    #ifdef LEE_CONTROL
    robot_x = x;
    robot_y = y;
    goal_x = x_r;
    goal_y = y_r;
    robot_theta = theta;
    goal_theta = theta_r;
    #endif

    // A star planning from robot_pose to goals[counter].
    // robot_pose: use robot_x, robot_y.
    // goals[counter][0], goals[counter][1]
    
    // Input: None (robot_pose, goal_pose)
    // Output: The Path! (A sequence of points!)

    // set<location> Open_list;
    // set<location> Closed_list;

    // int *Parent = new int(HEIGHT*WIDTH);
    // float *f_cost = new float(WIDTH*HEIGHT);
    // float *g_cost = new float(WIDTH*HEIGHT);

    memset(Parent, 0, sizeof(Parent));
    memset(f_cost, 0, sizeof(f_cost));
    memset(g_cost, 0, sizeof(g_cost));

    // for(int i=0; i< HEIGHT*WIDTH; i++){
    //     g_cost[i] = 0;
    //     f_cost[i] = 0;
    // }

    // reset path.
    Path.clear();


    set<int> Open_list;
    set<int> Closed_list;
    set<int>::iterator itr;

    // location goal(goals[counter][0], goals[counter][1]);
    location goal(goal_x, goal_y); // subscribe from /move_base/goal

    ROS_INFO("Round %d, Goal: (%d,%d). id:data[%d]", counter, goal.x, goal.y, goal.id);

    location init(Convert_Unit(robot_x), Convert_Unit(robot_y));
    Open_list.insert(init.id);
    Parent[init.id] = -1; // indicate start node.


    ROS_INFO("Robot cur at: %d, %d. data[%d]", init.x, init.y, init.id);

    int cur_x, cur_y;   // for picking x.
    
    while(!Open_list.empty()){

        // x: pick the node with min f_cost.
        float min_f = 100;
        for (itr = Open_list.begin(); itr != Open_list.end(); itr++)
        {
            int id = *itr;
            if( f_cost[id] < min_f){
                min_f = f_cost[id];
                cur_x = ID_to_X(id);
                cur_y = ID_to_Y(id);
            }
        }
        location chosen_loc(cur_x, cur_y);        
        
        // std::cout<<"Pick x: "<<chosen_loc.x<<", y: "<<chosen_loc.y<<" , min f: " <<min_f<< " Parent: "<< Parent[chosen_loc.id]<<std::endl;
        if(chosen_loc.x ==  goal.x && chosen_loc.y ==  goal.y){
            int id = chosen_loc.id;
            Append_Path(id);
            while(Parent[id] != -1){
                Append_Path(Parent[id]);
                // cout<< "id: "<<id<<", ("<<ID_to_X(Parent[id])<<", "<<ID_to_Y(Parent[id])<<")"<<endl;
                id = Parent[id];            
            
            }
            break; // break from while loop
        }

        // auto it = Open_list.find(chosen_loc.id);
        Open_list.erase(chosen_loc.id);
        // ROS_INFO("Move (%d,%d) {data[%d]} from Open-list to Closed-list!", chosen_loc.x, chosen_loc.y, chosen_loc.id);
        Closed_list.insert(chosen_loc.id);
        // cout<<"Openlist: "<<Open_list.size()<<" elements, Closed-list: "<<Closed_list.size()<<endl;

        int neighbor_x, neighbor_y;
        
        // Check each chosen_loc's neighbor!
        for(int index=0; index< NUM_NEIGHBORS; index++){
            switch (index)
            {
            case 0://right
                neighbor_x = cur_x + 1;
                neighbor_y = cur_y;
            break;
            case 1:// left
                neighbor_x = cur_x - 1;
                neighbor_y = cur_y;
            break;
            case 2:// up
                neighbor_x = cur_x ;
                neighbor_y = cur_y+1;
            break;
            case 3:// down
                neighbor_x = cur_x;
                neighbor_y = cur_y - 1;
            break;
            case 4:// right+down
                neighbor_x = cur_x + 1;
                neighbor_y = cur_y - 1;
            break;
            case 5:// right+up
                neighbor_x = cur_x + 1;
                neighbor_y = cur_y + 1;
            break;
            case 6:// left down
                neighbor_x = cur_x - 1;
                neighbor_y = cur_y - 1;
            break;
            case 7://  left up
                neighbor_x = cur_x - 1;
                neighbor_y = cur_y + 1;
            break;

            }
            
            // Out of boundaries
            // if(neighbor_x < -50 || neighbor_y < -50 || neighbor_x >= 50 || neighbor_y >= 50){
            //     continue;
            // }
            if(neighbor_x < -10*(100/20) || neighbor_y < -21.2*(100/20) || neighbor_x >= 640*0.05*(100/20)-10*(100/20) || neighbor_y >= 608*0.05*(100/20)-21.2*(100/20) ){
                continue;
            }


            location neighbor(neighbor_x, neighbor_y);
            // ROS_INFO("Visit (%d,%d)'s neighbor: (%d,%d).",cur_x, cur_y, neighbor_x, neighbor_y);
            // cout<<"env_map: "<<env_map[neighbor.id]<<endl;

            if(neighbor.id >= WIDTH*HEIGHT || neighbor.id <0){
                ROS_INFO("Segmentation Fault! Index out of range.");
            }
            // if(env_map[neighbor.id] == 100 || env_map[neighbor.id]==-1){
            if(env_map[neighbor.id] == 100 ){
                // NOT walkable. 
                #ifdef DEBUG_OBSTACLE
                ROS_INFO("Cannot walk this way since it's occupied! Index: %d",index);
                #endif
                continue;
            }

            if( Closed_list.find(neighbor.id) != Closed_list.end() ){
                // ROS_INFO("Neigbhor (%d,%d) in closed list. No longer consider!", neighbor.x, neighbor.y);
                continue;
            }
           
             
            float gt = g_cost[neighbor.id] + dist(chosen_loc , neighbor);
            // cout<<"gt : "<<gt<<" = g["<<neighbor.id<<"] "<<g_cost[neighbor.id]<<" + dist: "<<dist(chosen_loc , neighbor)<<endl;
            bool change_parent = false; // default
                   
            // check whether to change y's parent -> x.
            if( Open_list.find(neighbor.id) == Open_list.end()){
                Open_list.insert(neighbor.id);
                change_parent = true;
            }
            else if( gt < g_cost[neighbor.id]){
                change_parent = true;
            }
            /*  =========================== */
            
            if(change_parent){
                Parent[neighbor.id] = chosen_loc.id;
                g_cost[neighbor.id] = gt;
                f_cost[neighbor.id] = g_cost[neighbor.id] + h(neighbor, goal);
            }
            
        } // end for-each neighbor    
    }    // end  while (!Openlist.empty())

    if(Path.empty()){
        ROS_INFO("Fails to find a path!");
    }
    else{
        // #ifdef DEBUG_PATH
        for(auto &element: Path){
            path_map[element.id] = '$';
            // cout<<"Loc: ("<<element.x<<", "<< element.y <<")\n";
        }

        // for(int j=0; j<HEIGHT; j++){
        //     for(int i=0; i<WIDTH; i++){
        //         cout<<path_map[i+j*WIDTH]<<" ";
        //     }
        //     cout<<endl;
        // }
        ROS_INFO("Start writing to output file!");

        
        ofstream myfile(Pathname.c_str(), ios::out | ios::binary);
        // myfile.open("/home/catkin_ws/src/lab3_A_star/Path01.txt");
        // myfile.open();
        if (myfile.is_open())
        {

            for(int j=HEIGHT; j>=0; j--){
                for(int i=0; i<WIDTH; i++){
                    myfile<<path_map[i+j*WIDTH];
                }
                myfile<<endl;
            }

            myfile << "\n Path after reaching goal "<<goal_x<<", "<<goal_y<<"\n";
            ROS_INFO("Done writing to output file!");
            // myfile << "This is another line.\n";
            // for(int count = 0; count < size; count ++){
            //     myfile << x[count] << " " ;
            // }
            myfile.close();
        }
        else cout << "Unable to open file";

        // #endif
    }


    // free(Parent);
    // free(f_cost);
    // free(g_cost);
    // ROS_INFO("Free memory here!");
    
}


//Robot command goes here.
void cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& cmd)
{
    program_state = A_star_planning;
	command = true;
	x_r = (cmd->pose.position.x)*(100/20);
	y_r = (cmd->pose.position.y)*(100/20);
	
	float xx_r = cmd->pose.orientation.x;
	float yy_r = cmd->pose.orientation.y;
	float zz_r = cmd->pose.orientation.z;
	float ww_r = cmd->pose.orientation.w;
	theta_r = degreeConverter(xx_r,yy_r,zz_r,ww_r);
	theta_r = thetaConstraint(theta_r);
	if(x_r<0&&theta_r>0)
	{
		theta_r = -1*theta_r;
	}
	//----------------------------------
	
}

//Here we take the pose of the robot
void poseCallback(const geometry_msgs::Twist::ConstPtr& pose)
{
	x = pose->linear.x;
    y = pose->linear.y;
    theta = pose->angular.z;
	theta = thetaConstraint(theta);
}



//////////////////////////////////////////

float deg2rad( float degree){
    return degree*pi/180;
}

void Initial_Control(float goal_x, float goal_y, float goal_theta){

    // stuck in here!
    while(!should_break){

            error_x = goal_x - robot_x;
            error_y = goal_y - robot_y;
            error_theta = goal_theta - robot_theta;

        #ifdef DEBUG_LAB2
            // ROS_INFO("Robot-State: %d, Goal theta: %.2f, Robot Theta: %.2f", robot_state, goal_theta, robot_theta);
            ROS_INFO("Error: %.3f %.3f %.3f", error_x, error_y, error_theta);
        #endif

            switch (robot_state)
            {
            case MOVING:
                ////////// Main Control Loop! /////////////
                if (abs(error_x) < 0.01 && abs(error_y) < 0.01 && abs(error_theta) < 0.1)
                {
                    robot_state = IDLE;
                }
                else if (abs(error_x) < 0.01 && abs(error_y) < 0.01)
                {
                    robot_state = TURNING;
                }
                else
                {
                    /* --- Control Law --- */
                    v = Krho * rho;
                    w = Ka * alpha + Kb * beta;
                    // ROS_INFO("Robot is MOVING !!!");
                }
                break;
            case TURNING:

                if (abs(error_theta) < 0.1 || abs(robot_theta + 2 * pi) < 0.1 || abs(error_theta - 2 * pi) < 0.1)
                {
                    robot_state = IDLE;
                }
                else
                {
                    /* --- Control Law --- */
                    v = 0;
                    w = Kp * error_theta;
                }

                break;
            case IDLE:
                // Stop and listen to new goal.
                // reset error signals.
                v = 0;
                w = 0;
                // ROS_INFO("Robot IDLE and break!");
                should_break = true; // break while-loop and return to the main function!
        #ifdef DEBUG_LAB2
                ROS_INFO("Robot reached the goal and is now IDLE.");
        #endif

                break;
            }

            /* Command Inputs Constraints */
            v = (v>max_v)? max_v : v;
            v = (v<-max_v)?-max_v: v;
            w = (w>max_w)? max_w : w;
            w = (w<-max_w)?-max_w: w;

            // Publish cmd_vel
            twist.linear.x = v;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = w;
            cmd_vel_pub.publish(twist);

            ros::spinOnce();
            // if(should_break)
            //     break;

    }


}


int Good_angles[5] ={
    90,
    -90,
    90,
    90,
    90
};

int main(int argc, char **argv)
{
    // Initialize the node here
    ros::init(argc, argv, "a_star_tracking");
    ros::NodeHandle node;

    cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    #ifndef LEE_CONTROL
    robot_pose_sub = node.subscribe("/robot_pose", 10, Robot_Pose_Callback);
    goal_pose_sub = node.subscribe("/move_base_simple/goal", 10, Goal_Pose_Callback);
    #else
    robot_pose_sub = node.subscribe("/robot_pose", 10, poseCallback);
    goal_pose_sub = node.subscribe("/move_base_simple/goal", 10, cmdCallback);
    #endif
    map_sub = node.subscribe("map", 1, Map_Callback);

    // Set the publish rate here
    ros::Rate rate(10);


    // should_break = false;
    // Initial_Control(0, 0, deg2rad(178) );
    // ROS_INFO("Robot Done Initial Turning!");
    error_x = error_y = error_theta = 0; //reset config.

    // Main Control Loop.
    // After receive a new goal: keep localization task.
    while (ros::ok())
    {           
        // After reading map
        if(read_map){
        // ROS_INFO("while-loop: %d", counter++);

        // #ifdef DEBUG_PRORAM    
            switch (program_state)
            {
            case A_star_planning:
            
                A_star_algorithm();
                // Save path to .txt
                counter++;
                program_state = Tracking_state;

                /*  Initial angle control*/
                // should_break = false;
                // robot_state = MOVING;
                // Initial_Control(robot_x, robot_y, deg2rad(Good_angles[counter-1]) );
                // ROS_INFO("Done Turning to %d",Good_angles[counter-1] );

                should_break = false; // For tracking state.
                robot_state = MOVING; // Initial Robot State.
                
            break;
            case Tracking_state:
                // Now, pass Path (element.x, element.y) to A series of Control Problem!
                // for(int i=6; i<Path.size(); i+=6){
                for(int i=4; i<Path.size(); i+=4){
                // for(int i=0; i<Path.size(); i++){
                    i = (i+4 >= Path.size()-1)? Path.size()-1:i;
                    // goal_x = (float)Path[i].x/10  ; // Convert to meter unit
                    // goal_y = (float)Path[i].y/10  ; // Since we use (10cm) in Path.x,.y. Convert to "m" !
                    goal_x = (float)Path[i].x*20/100  ; // Convert to meter unit
                    goal_y = (float)Path[i].y*20/100  ; // Since we use (10cm) in Path.x,.y. Convert to "m" !
                    // goal_theta = 0; // TODO: Don't know orientation! XD
                    if(i==Path.size()-1){
                        goal_theta = Final_goal_theta;
                    }
                    else{
                        goal_theta = atan2(error_y,error_x);
                    }
                    
                    ROS_INFO("Loc: (%f,%f)", goal_x, goal_y);
                    
                    #ifndef LEE_CONTROL
                    Tracking();
                    #else
                    Tracking_LEE();
                    command = true;
                    #endif
                    should_break = false; // reset flag
                    robot_state = MOVING; // reset Robot_state
                }
                ROS_INFO("Reach Goal: (%f,%f)", goal_x, goal_y);
                // After reached this point.      
                // if(counter< NUM_OF_GOALS){
                //     program_state = A_star_planning;
                // }
                // else{
                program_state = Finished;
                //     ROS_INFO("Finish lab3! ^_^ !");
                // }
            break;
            case Finished:

                // free(env_map);
            #ifdef DEBUG_PROGRAM
                        // ROS_INFO("Program in IDLE state.");
            #endif
                // return 0;
            break;
            }
        }
        
        // #endif
        ros::spinOnce(); // Allow processing of incoming messages
        rate.sleep();
    }
}
