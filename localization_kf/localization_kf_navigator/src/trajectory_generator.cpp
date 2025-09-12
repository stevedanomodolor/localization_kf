#include "localization_kf_navigator/trajectory_generator.hpp"

namespace localization_kf_navigator
{

    TrajectoryGenerator::TrajectoryGenerator(std::string map_frame_id, float max_range)
        : _map_frame_id(map_frame_id), _max_range(max_range)
    {
    }

    TrajectoryGenerator::Path TrajectoryGenerator::generateTrajectory(const int &type, const Pose &ref_pose)
    {
        TrajectoryType trajectory_type = static_cast<TrajectoryType>(type);

        // Generate trajectory based on the type
        if (trajectory_type == TrajectoryType::CIRCLE)
        {
            // Generate circular trajectory
            return generateCircleTrajectory(ref_pose);
        }
        else if (trajectory_type == TrajectoryType::SQUARE)
        {
            // Generate square trajectory
            return generateSquareTrajectory(ref_pose);
        }
        else if (trajectory_type == TrajectoryType::FIGURE8)
        {
            // Generate figure8 trajectory
            return generateFigure8Trajectory(ref_pose);
        }
        else
        {
            // Handle unknown trajectory type
            throw std::invalid_argument("Unknown trajectory type");
        }
    }

    TrajectoryGenerator::Path TrajectoryGenerator::generateSquareTrajectory(const Pose &ref_pose)
    {
        Path path;
        path.header.frame_id = _map_frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // Starting position
        PoseStamped corner1;
        corner1.header.frame_id = _map_frame_id;
        corner1.pose = ref_pose;
        path.poses.push_back(corner1);

        // First corner
        PoseStamped corner2 = corner1;
        corner2.pose.position.x += _max_range;
        corner2.pose.orientation = get_yaw_between_points_2d(corner1.pose.position, corner2.pose.position);
        path.poses.push_back(corner2);

        // Second corner
        PoseStamped corner3 = corner2;
        corner3.pose.position.y += _max_range;
        corner3.pose.orientation = get_yaw_between_points_2d(corner2.pose.position, corner3.pose.position);
        path.poses.push_back(corner3);

        // Third corner
        PoseStamped corner4 = corner3;
        corner4.pose.position.x -= _max_range;
        corner4.pose.orientation = get_yaw_between_points_2d(corner3.pose.position, corner4.pose.position);
        path.poses.push_back(corner4);

        // Fourth corner
        PoseStamped corner5 = corner1;
        corner5.pose.orientation = get_yaw_between_points_2d(corner4.pose.position, corner5.pose.position);
        path.poses.push_back(corner5);

        // Return to starting position
        path.poses.push_back(corner1);

        return path;
    }

    TrajectoryGenerator::Path TrajectoryGenerator::generateFigure8Trajectory(const Pose &ref_pose)
    {
        Path path;
        path.header.frame_id = _map_frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // Starting position
        PoseStamped start;
        start.header.frame_id = _map_frame_id;
        start.pose = ref_pose;
        path.poses.push_back(start);

        // Generate figure 8 trajectory
        for (double t = 0.1; t <= 2 * M_PI; t += 0.1)
        {
            PoseStamped pose;
            pose.pose = ref_pose;
            pose.pose.position.x += _max_range * sin(t);
            pose.pose.position.y += _max_range * sin(2 * t);
            pose.pose.position.z = ref_pose.position.z;
            pose.pose.orientation = get_yaw_between_points_2d(path.poses.back().pose.position, pose.pose.position);
            path.poses.push_back(pose);
        }

        // Return to initial pose
        path.poses.push_back(start);

        return path;
    }

    TrajectoryGenerator::Path TrajectoryGenerator::generateCircleTrajectory(const Pose &ref_pose)
    {
        Path path;
        path.header.frame_id = _map_frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // start position
        PoseStamped start;
        start.header.frame_id = _map_frame_id;
        start.pose = ref_pose;
        path.poses.push_back(start);

        // Generate circular trajectory
        for (double t = 0; t <= 2 * M_PI; t += 0.1)
        {
            PoseStamped pose;
            pose.pose = ref_pose;
            pose.pose.position.x += _max_range * cos(t);
            pose.pose.position.y += _max_range * sin(t);
            pose.pose.position.z = ref_pose.position.z;
            pose.pose.orientation = get_yaw_between_points_2d(path.poses.back().pose.position, pose.pose.position);
            path.poses.push_back(pose);
        }

        // Return to initial pose
        path.poses.push_back(start);

        return path;
    }

} // namespace localization_kf_navigator