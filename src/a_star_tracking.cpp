#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" // KEY!!
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"

#include <iostream>
#include <cmath>
// #include <set>
#include<vector>
using namespace std;

// #define DEBUG_LAB2

/*
    Publish:
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


enum State
{
    A_star_planning,
    Tracking_state,
    Finished,
    MOVING,
    TURNING,
    IDLE
};
State program_state = A_star_planning; //Initial turtle state/pose.
State robot_state = IDLE;              //Initial turtle state/pose.

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

void Goal_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Once receive a command, change state!
    robot_state = MOVING;
    float qx, qy, qz, qw;
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;

    goal_theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qy * qy));
}

#define WIDTH 100
#define HEIGHT 100
#define NUM_NEIGHBORS 8

int *env_map;
// Note: this ONLY run ONE TIME !
void Map_Callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_INFO("I get msg from map!!! Hehehaha Tony Chiu!");
    // env_map = msg->data;
    

    for (int i = 0; i < WIDTH; i++)
    {
        for (int j = 0; j < HEIGHT; j++)
        {
            env_map[i + j*WIDTH] = msg->data[i + j*WIDTH];

#ifdef DEBUG_MAP
            printf("%d ", msg->data[i * WIDTH + j]);
#endif
        }
#ifdef DEBUG_MAP
        printf("\n");
#endif
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

const float max_v = 0.5; // m/s
const float max_w = 1;   // rad/s

// const float max_v = 2*wheel_r*Max_Wheel_Speed;
// const float max_w = 2*max_v/wheel_separaton;

/* -------- Gain ---------- */
const float Krho = 2;
const float Ka = 8;
const float Kb = -1.5;
const float Kp = 0.5;

/* ---- Goal locations ----- */
#define NUM_OF_GOALS 4
// const int goals[4][2] =
//     {
//         {3, 4},
//         {3, -2},
//         {-4, -3},
//         {-4, 4}};

const int goals[4][2] =
    {
        {30, 20},
        {30, -20},
        {-40, -30},
        {-40, 40}};

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


int Convert_Unit(float num){
    return (int)num*100/10;
}

int ID_to_X(int id){
    return id%WIDTH - 50;
}

int ID_to_Y(int id){
    return id/WIDTH -50;
}

class location{
public:
    location();     // default constructor.
    location(int x, int y){
        this->x = x;
        this->y = y;
        this->id = (x+50) + (y+50)*WIDTH;   // Since origin: (-50,-50) in 10cm unit
        this->f = this->g = 0; // initialize
    }
    int id; // from its x,y
    int x;  // unit: 10cm
    int y;  // unit: 10cm

    float f;
    float g;
    int parent; // an id.
};


list<location> Path; // Path.push_back(location)

float dist(location a, location b){
    return abs(sqrt(pow((b.x - a.x) ,2) + pow((b.y - a.y) ,2)));
}

float h(location a, location b){
    return abs(sqrt(pow((b.x - a.x) ,2) + pow((b.y - a.y) ,2))); // straight line distance
}

void Append_Path(int node_id){
    // Append node_id in front of Path.
    int x = ID_to_X(node_id);
    int y = ID_to_Y(node_id);
    Path.push_front(location(x,y));
    return;
}




void A_star_algorithm()
{
    // A star planning from robot_pose to goals[counter].
    // robot_pose: use robot_x, robot_y.
    // goals[counter][0], goals[counter][1]
    
    // Input: None (robot_pose, goal_pose)
    // Output: The Path! (A sequence of points!)

    // set<location> Open_list;
    // set<location> Closed_list;

    int *Parent = new int(HEIGHT*WIDTH);
    float *f_cost = new float(WIDTH*HEIGHT);
    float *g_cost = new float(WIDTH*HEIGHT);

    // memset(f_cost, 0, sizeof(f_cost));
    // memset(g_cost, 0, sizeof(g_cost));

    // for(int i=0; i< HEIGHT*WIDTH; i++){
    //     g_cost[i] = 0;
    //     f_cost[i] = 0;
    // }

    // reset path.
    Path.clear();


    set<int> Open_list;
    set<int> Closed_list;
    set<int>::iterator itr;

    location goal(goals[counter][0], goals[counter][1]);

    ROS_INFO("Round %d, Goal: (%d,%d). id:data[%d]", counter, goal.x, goal.y, goal.id);

    location init(Convert_Unit(robot_x), Convert_Unit(robot_y));
    Open_list.insert(init.id);
    Parent[init.id] = -1; // indicate start node.


    ROS_INFO("Robot cur at: %d, %d. data[%d]", init.x, init.y, init.id);

    int cur_x, cur_y;   // for picking x.
    
    while(!Open_list.empty()){
        // cout<<"Open list: \n";
        // for(auto &item: Open_list)
        //     cout<<item.x<<", "<<item.y<<"\n";
        // x: pick the node with min f_cost.
        float min_f = 100;
        for (itr = Open_list.begin(); itr != Open_list.end(); itr++)
        {
            int id = *itr;
            // if( f_cost[id] < min_f){
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

        auto it = Open_list.find(chosen_loc.id);
        Open_list.erase(it);
        // ROS_INFO("Move (%d,%d) {data[%d]} from Open-list to Closed-list!", chosen_loc.x, chosen_loc.y, chosen_loc.id);
        Closed_list.insert(chosen_loc.id);

        // cout<<"Openlist: "<<Open_list.size()<<" elements, Closed-list: "<<Closed_list.size()<<endl;

        // for(auto &ele:Open_list)
        //     cout<<ele<<" ";
        // cout<<endl;


        // for(auto &ele:Closed_list)
        //     cout<<ele<<" ";
        // cout<<endl;

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
            if(neighbor_x < -50 || neighbor_y < -50 || neighbor_x >= 50 || neighbor_y >= 50){
                continue;
            }

            location neighbor(neighbor_x, neighbor_y);
            // ROS_INFO("Visit (%d,%d)'s neighbor: (%d,%d).",cur_x, cur_y, neighbor_x, neighbor_y);
            // cout<<"env_map: "<<env_map[neighbor.id]<<endl;

            if(neighbor.id >= 10000 || neighbor.id <0){
                ROS_INFO("Sementation Fault! Index out of range.");
            }

            
            if(env_map[neighbor.id] == 100 || env_map[neighbor.id]==-1){
                // NOT walkable. 
                ROS_INFO("Cannot walk this way since it's occupied! Index: %d",index);
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
            if(  Open_list.find(neighbor.id) == Open_list.end()){
                Open_list.insert(neighbor.id);
                change_parent = true;
            }
            else if( gt < g_cost[neighbor.id]){
                change_parent = true;
            }
            /*  =========================== */
            
            if(change_parent){
                // neighbor.parent = chosen_loc.id;
                // neighbor.g = gt;
                // neighbor.f = neighbor.g + h(neighbor, goal);
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
        for(auto &element: Path){
            cout<<"Loc: ("<<element.x<<", "<< element.y <<")\n";
        }
    }


    free(Parent);
    free(f_cost);
    free(g_cost);
    
}

float v, w;
geometry_msgs::Twist twist;

void Tracking()
{

    error_x = goal_x - robot_x;
    error_y = goal_y - robot_y;
    error_theta = goal_theta - robot_theta;

#ifdef DEBUG_LAB2
    ROS_INFO("State: %d, Goal theta: %.2f, Robot Theta: %.2f", state, goal_theta, robot_theta);
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

}

int main(int argc, char **argv)
{
    // Initialize the node here
    ros::init(argc, argv, "a_star_tracking");
    ros::NodeHandle node;

    ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber robot_pose_sub = node.subscribe("/robot_pose", 10, Robot_Pose_Callback);
    ros::Subscriber goal_pose_sub = node.subscribe("/move_base_simple/goal", 10, Goal_Pose_Callback);
    ros::Subscriber map_sub = node.subscribe("map", 100, Map_Callback);

    // Set the publish rate here
    ros::Rate rate(100);
    
    env_map = new int(HEIGHT*WIDTH);

    // Main Control Loop.

    // After receive a new goal: keep localization task.
    while (ros::ok())
    {
        switch (program_state)
        {
        case A_star_planning:
          
            A_star_algorithm();
            program_state = Finished;
            ROS_INFO("Finish lab3! ^_^ !");
            // program_state = Tracking_state;
        break;
        case Tracking_state:
            Tracking();
            if(counter< NUM_OF_GOALS){
                counter++;
                program_state = A_star_planning;
            }
            else{
                program_state = Finished;
                ROS_INFO("Finish lab3! ^_^ !");
            }
        break;
        case Finished:
            // return 0;
        #ifdef DEBUG_PROGRAM
                    ROS_INFO("Program ended.");
        #endif
        break;
        }

        ros::spinOnce(); // Allow processing of incoming messages
        rate.sleep();
    }
}
