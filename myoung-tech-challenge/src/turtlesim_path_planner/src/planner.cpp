#include "turtlesim_path_planner/planner.hpp"


namespace TurtlesimPathPlanner
{
    PlannerClass::PlannerClass(ros::NodeHandle& handle)
    {
        ROS_INFO("Setting up Turtlesim Path Planner Node...");

        // Set constants
        waypoint_tolerance_         = 1.0;
        linear_velocity_user_       = 1.5;
        linear_velocity_planner_    = 0.5;
        angular_velocity_user_      = 3.0;
        angular_velocity_planner_   = 6.0;
        waypoints_delay_sec_        = 5.0;
        
        // Parse the node handle
        internal_node_handle_ = handle;

        // Set up listeners/publishers
        pose_subscriber_ = internal_node_handle_.subscribe(
            "turtle1/pose", 10, &PlannerClass::UpdatePose, this
        );
        waypoint_subscriber_ = internal_node_handle_.subscribe(
            "turtle1/waypoint", 10, &PlannerClass::ReceiveWaypoint, this
        );
        move_cmd_publisher_ = internal_node_handle_.advertise<geometry_msgs::Twist>(
            "turtle1/cmd_vel", 10
        );

        // Set up for the keyboard handler
        set_raw_mode(true); 
        ROS_INFO_STREAM(
            "\n---------------------------------------------------" <<
            "\nUse the following arrow and letter keys to manually" <<
            "\ncontrol the turtle\n"<<
            "\nUp    : move forward" <<
            "\nDown  : move backward" <<
            "\nLeft  : rotate counter-clockwise" <<
            "\nRight : rotate clockwise\n" <<
            "\nq/z   : increase/decrease max speeds by 10%" <<
            "\nw/x   : increase/decrease only linear speed by 10%" <<
            "\ne/c   : increase/decrease only angular speed by 10%\n" <<
            "\nPress Ctrl+C to exit node" <<
            "\n---------------------------------------------------"
        );
    };


    void PlannerClass::UpdatePose(const turtlesim::Pose& pose)
    {
        current_pose_ = pose;
    }


    void PlannerClass::ReceiveWaypoint(const turtlesim::Pose& pose)
    {
        waypoints_received_++;
        Waypoint new_waypoint = {waypoints_received_, WaypointStatus::PENDING, pose};
        waypoints_queue_.push_back(new_waypoint);

        // TODO: reject a waypoint near or identical to the current one

        ROS_INFO_STREAM(
            "Received waypoint " << new_waypoint.num << " to [" 
            << new_waypoint.pose.x << ", " << new_waypoint.pose.y << "]"
        );
    }


    float PlannerClass::DistanceToWaypoint(Waypoint& target_point)
    {
        return sqrt(
            pow(target_point.pose.x - current_pose_.x, 2) +
            pow(target_point.pose.y - current_pose_.y, 2)
        );
    }


    float PlannerClass::AzimuthToWaypoint(Waypoint& target_point)
    {
        // Note: atan2 returns a result that matches the orientation convention
        // of the turtlesim [Pi, -Pi], so no conversion is required.
        return atan2(
            (target_point.pose.y - current_pose_.y),
            (target_point.pose.x - current_pose_.x)
        );
    }


    geometry_msgs::Twist PlannerClass::ProportionalController(
        const float& distance, const float& azimuth)
    {
        geometry_msgs::Twist move_cmd;

        move_cmd.linear.x = linear_velocity_planner_ * distance;
        move_cmd.linear.y = 0.0;
        move_cmd.linear.z = 0.0;
        move_cmd.angular.x = 0.0;
        move_cmd.angular.y = 0.0;
        move_cmd.angular.z = angular_velocity_planner_ * (azimuth - current_pose_.theta);

        return move_cmd;
    }


    geometry_msgs::Twist PlannerClass::StraightLineController(
        const float& distance, const float& azimuth)
    {
        geometry_msgs::Twist move_cmd;

        // If not facing the waypoint, rotate on the spot to face it
        if (abs(azimuth - current_pose_.theta) > waypoint_tolerance_ / 10.0)
        {
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = angular_velocity_planner_ * (azimuth - current_pose_.theta);
        }
        else // If facing the waypoint, move straight towards it
        {
            move_cmd.linear.x = linear_velocity_planner_ * distance;
            move_cmd.angular.z = 0.0;
        }

        return move_cmd;
    }


    bool PlannerClass::KeyboardInput(geometry_msgs::Twist& move_cmd)
    {
        // Poll the keyboard for input, if -1, nothing was pressed, return early
        int key = quick_read();
        if (key == -1)
        { 
            return false;
        }

        // Check for one of the move keys
        switch (key)
        {
            case ARROW_UP:      // move forward
                move_cmd.linear.x = linear_velocity_user_;
                break;
            case ARROW_DOWN:    // move backward
                move_cmd.linear.x = -linear_velocity_user_;
                break;
            case ARROW_LEFT:    // rotate CCW
                move_cmd.angular.z = angular_velocity_user_;
                break;
            case ARROW_RIGHT:   // rotate CW
                move_cmd.angular.z = -angular_velocity_user_;
                break;
            case 113:           // 'q' increase ALL speeds by 10%
                linear_velocity_user_  = linear_velocity_user_ * 1.1;
                angular_velocity_user_ = angular_velocity_user_* 1.1;
                break;
            case 122:           // 'z' decrease ALL speeds by 10%
                linear_velocity_user_  = linear_velocity_user_ * 0.9;
                angular_velocity_user_ = angular_velocity_user_* 0.9;        
                break;
            case 119:           // 'w' increase LINEAR speed by 10%
                linear_velocity_user_  = linear_velocity_user_ * 1.1;
                break;
            case 120:           // 'x' decrease LINEAR speed by 10%
                linear_velocity_user_  = linear_velocity_user_ * 0.9;
                break;
            case 101:           // 'e' increase ANGULAR speed by 10%
                angular_velocity_user_ = angular_velocity_user_* 1.1;
                break;
            case 99:            // 'c' decrease ANGULAR speed by 10%
                angular_velocity_user_ = angular_velocity_user_* 0.9; 
                break;    
        }

        // We only want to interrupt waypoint following if the user presses one
        // of the arrow move keys, in which case return true. Otherwise return
        // false
        if (std::find(valid_move_keys_.begin(), valid_move_keys_.end(), key) != valid_move_keys_.end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    void PlannerClass::Run()
    {
        geometry_msgs::Twist move_cmd;  // Zeroes by default

        // If there is any valid keyboard command, prioritize this over any 
        // outstanding waypoint by pausing it 
        if (KeyboardInput(move_cmd))
        {
            // Publish the manual move command
            move_cmd_publisher_.publish(move_cmd);

            time_of_last_key_press_ = ros::Time::now().toSec();

            // Pause the current waypoint (if not already paused)
            if (!waypoints_queue_.empty() && 
                waypoints_queue_.front().status != WaypointStatus::PAUSED
            )
            {
                waypoints_queue_.front().status = WaypointStatus::PAUSED;
                ROS_INFO_STREAM(
                    "User taking manual control, pausing waypoint " << 
                    waypoints_queue_.front().num
                );
            }
        }

        // Check to see if the time since the last valid move key was pressed 
        // has elapsed, in which case we can start or resume waypoint following
        double time_elapsed = ros::Time::now().toSec() - time_of_last_key_press_;
        if (!waypoints_queue_.empty() && 
            time_elapsed > waypoints_delay_sec_
        )
        {
            switch (waypoints_queue_.front().status) 
            {
                case WaypointStatus::PENDING:
                    ROS_INFO_STREAM("Starting waypoint " << waypoints_queue_.front().num);
                    waypoints_queue_.front().status = WaypointStatus::IN_PROGRESS;
                    break;
                case WaypointStatus::PAUSED:
                    ROS_INFO_STREAM(
                        waypoints_delay_sec_ << " seconds since last user " <<
                        "input, resuming waypoint " << waypoints_queue_.front().num
                    );
                    waypoints_queue_.front().status = WaypointStatus::IN_PROGRESS;
                    break;
            }
        }

        // If we have a valid waypoint and it's not paused, move to it
        if (!waypoints_queue_.empty() && 
            waypoints_queue_.front().status != WaypointStatus::PAUSED
        )
        {
            // Calculate the distance to the current waypoint
            float distance = DistanceToWaypoint(waypoints_queue_.front());
            float azimuth = AzimuthToWaypoint(waypoints_queue_.front());

            // Calculate if we have reached the waypoint, if not move towards it
            if (distance < waypoint_tolerance_)
            {
                // Stop the turtle
                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = 0.0;   
                move_cmd_publisher_.publish(move_cmd);

                // Update the waypoint
                waypoints_queue_.front().status = WaypointStatus::COMPLETE;
                ROS_INFO_STREAM("Reached waypoint " << waypoints_queue_.front().num);
                waypoints_queue_.pop_front();
            }
            else 
            {
                waypoints_queue_.front().status = WaypointStatus::IN_PROGRESS;
                move_cmd = ProportionalController(distance, azimuth);
            }

            // Publish the autonomous move command
            move_cmd_publisher_.publish(move_cmd);
        }
    }
}


int main(int argc, char** argv)
{
    // ROS node setup
    ros::init(argc, argv, "turtlesim_path_planner");
    ros::NodeHandle node;

    // Set up planning class instance
    TurtlesimPathPlanner::PlannerClass planner(node);

    while (ros::ok())
    {
        planner.Run();
        ros::spinOnce();
    }
}
