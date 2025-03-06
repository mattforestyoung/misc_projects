
// C++ library includes
#include <deque>
#include <cmath>
#include <termios.h>    // For keyboard input
#include <unistd.h>     // for keyboard input
#include <iostream>
#include <algorithm>    // std::find

// ROS1 Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <turtlesim/Pose.h>

// Custom includes
#include "colors.h"

namespace TurtlesimPathPlanner
{

//! @brief Status enum for each waypoint, to indicate the current progress 
//!        towards it 
enum WaypointStatus {
    PENDING,
    IN_PROGRESS,
    PAUSED,
    COMPLETE
};


//! @brief Struct for all waypoint data
struct Waypoint
{
    uint32_t num;
    WaypointStatus status; 
    turtlesim::Pose pose;
};


//! @brief Class for receiving, plotting, and publishing paths to the turtlebot
class PlannerClass
{
    public:

        //! @brief Constructor
        //! @param [in] handle The node handle to use for this class
        PlannerClass(ros::NodeHandle& handle);

        //! @brief Basic run function to check the waypoint queue, handle 
        //!        keyboard input and publish commands to the turtle
        void Run();

    private:

        //! @brief Callback for updating the internal turtlebot pose. This will 
        //! be called at approx 60 Hz
        //! @param [in] pose Current pose of the turtle
        void UpdatePose(const turtlesim::Pose& pose);


        //! @brief Callback for receiving user-specified waypoints
        //! @param [in] pose A desired pose of the turtle, to be used as a 
        //!        waypoint
        void ReceiveWaypoint(const turtlesim::Pose& pose);


        //! @brief Calculate the euclidean distance to the input waypoint
        //! @param [in] target_waypoint The waypoint to calculate the distance
        //!        to, from the current pose
        //! @return The euclidean distance between the current pose and target 
        //!         point
        float DistanceToWaypoint(Waypoint& target_point);


        //! @brief Calculate the azimuth to the input waypoint 
        //! @param [in] target_waypoint The waypoint to calculate the azimuth
        //!        to, from the current pose
        //! @return The azimuth between the current pose and target point
        float AzimuthToWaypoint(Waypoint& target_point);


        //! @brief A proportional controller to command the turtle to move to 
        //!        the next waypoint
        //! @param [in] distance Euclidean distance to the waypoint
        //! @param [in] azimuth Azimuth to the waypoint
        //! @return A twist message that will command the turtle in the 
        //!         direction of the next waypoint, as defined by the input
        //!         distance and azimuth
        geometry_msgs::Twist ProportionalController(const float& distance, const float& azimuth);


        //! @brief A controller that commands the turtle to move to 
        //!        the next waypoint in straight lines by rotating on the spot
        //!        to face the waypoint before move towards it
        //! @param [in] distance Euclidean distance to the waypoint
        //! @param [in] azimuth Azimuth to the waypoint
        //! @return A twist message that will command the turtle in the 
        //!         direction of the next waypoint, as defined by the input
        //!         distance and azimuth
        geometry_msgs::Twist StraightLineController(const float& distance, const float& azimuth);


        //! @brief Handle keyboard input in a non-blocking manner. Return true 
        //!        if there is a valid key press, and false otherwise
        //! @param [in, out] move_cmd The twist message to be populated by this
        //!        function, if the user inputs a keyboard command
        //! @return True if one of the move (arrow) keys was pressed, false
        //!         otherwise
        bool KeyboardInput(geometry_msgs::Twist& move_cmd);


        // ROS classes
        ros::NodeHandle internal_node_handle_;  //!< Internal copy of node handle
        ros::Subscriber pose_subscriber_;       //!< Subscribes to turtle poses from simulator
        ros::Subscriber waypoint_subscriber_;   //!< Subscribes to user-created waypoints
        ros::Publisher move_cmd_publisher_;     //!< Publishes move commands to the turtle

        // Internal storage variables
        turtlesim::Pose current_pose_;          //!< Most recent pose of the turtle
        std::deque<Waypoint> waypoints_queue_;  //!< Queue of waypoints to follow
        float waypoint_tolerance_;              //!< Turtle must be within this distance to "reach" its waypoint
        double waypoints_delay_sec_;            //!< Time to delay resuming waypoint following after a user input
        double time_of_last_key_press_;         //!< ROS time in sec when last valid key was pressed
        uint32_t waypoints_received_ = 0;       //!< Total number of waypoints received

        // Speeds for the turtle, have separate speeds for user input and path 
        // planning, because the default settings for the planner may be too
        // much for the user
        float linear_velocity_user_;
        float linear_velocity_planner_;    
        float angular_velocity_user_;   
        float angular_velocity_planner_;

        //! List of valid move keys, for comparing with user input
        std::list<int> valid_move_keys_ = {UP_ARROW, DOWN_ARROW, LEFT_ARROW, RIGHT_ARROW};
};

} // namespace turtlesim