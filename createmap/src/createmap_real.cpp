#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/bind.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

// Map parameter
const double GRID_RESOLUTION = 0.02;
const int MAP_SIZE_X = 400;
const int MAP_SIZE_Y = 400;
const int SCAN_THRESHOLD = 8;
const int DECAY_FACTOR = 0;
const double SUSPICIOUS_ANGLE_THRESHOLD = 85.0 * M_PI / 180.0;  // Increased to 85 degrees (less strict)
const int OBSTACLE_CONFIDENCE_THRESHOLD = 2;  // Increased to require more obstacles nearby

// Record the number of times the grid is scanned by the laser
std::vector<std::vector<int>> scan_count(MAP_SIZE_X, std::vector<int>(MAP_SIZE_Y, 0));

// Record whether it is confirmed as an obstacle
std::vector<std::vector<bool>> confirmed_obstacles(MAP_SIZE_X, std::vector<bool>(MAP_SIZE_Y, false));

// Record the number of times the grid is visited, if not, the value is -1
std::vector<std::vector<bool>> visited(MAP_SIZE_X, std::vector<bool>(MAP_SIZE_Y, false));

ros::Publisher map_publisher;
std::shared_ptr<tf::TransformListener> tf_listener;
std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;

bool map_origin_initialized = false;
tf::Vector3 map_origin_offset;

// center the map frame and odom frame
void broadcastMapFrame(const ros::Time& stamp)
{
    tf::Transform transform;
    if (map_origin_initialized)
    {
        transform.setOrigin(-map_origin_offset);
    } 
    else
    {
        transform.setOrigin(tf::Vector3(0, 0, 0));
    }
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster->sendTransform(tf::StampedTransform(transform, stamp, "map", "odom"));
}

// Using bresenhamLine to get the whole grid points on the line
std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> line;
    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {
        line.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        { 
            err += dy; 
            x0 += sx; 
        }
        if (e2 <= dx)
        { 
            err += dx; 
            y0 += sy; 
        }
    }
    return line;
}

// World (x, y) -> Gridmap (x, y)
std::pair<int, int> worldToGrid(double x, double y)
{
    if (map_origin_initialized)
    {
        x += map_origin_offset.x();
        y += map_origin_offset.y();
    }
    int gx = static_cast<int>(x / GRID_RESOLUTION) + MAP_SIZE_X / 2;
    int gy = static_cast<int>(y / GRID_RESOLUTION) + MAP_SIZE_Y / 2;
    return {gx, gy};
}

// Modify isNearObstacles to be more selective
bool isNearObstacles(int x, int y, int radius) {
    // Only check along the line of sight, not in all directions
    int obstacle_count = 0;
    
    // Check along horizontal and vertical lines (more efficient and less aggressive)
    for (int i = -radius; i <= radius; ++i) {
        // Check horizontal line
        if (x+i >= 0 && x+i < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
            if (confirmed_obstacles[x+i][y]) {
                obstacle_count++;
            }
        }
        
        // Check vertical line
        if (x >= 0 && x < MAP_SIZE_X && y+i >= 0 && y+i < MAP_SIZE_Y) {
            if (confirmed_obstacles[x][y+i]) {
                obstacle_count++;
            }
        }
    }
    
    return obstacle_count >= OBSTACLE_CONFIDENCE_THRESHOLD;
}

bool isSuspiciousRay(const sensor_msgs::LaserScan::ConstPtr& scan, size_t index) {
    // If we're at the edges of the scan, don't consider it suspicious
    if (index <= 2 || index >= scan->ranges.size() - 3) {
        return false;
    }
    
    double range = scan->ranges[index];
    
    double prev_range2 = scan->ranges[index-2];
    double prev_range = scan->ranges[index-1];
    double next_range = scan->ranges[index+1];
    double next_range2 = scan->ranges[index+2];
    
    // If all neighbors are invalid, can't make a good judgment
    if (std::isnan(prev_range) || std::isnan(next_range) || 
        std::isnan(prev_range2) || std::isnan(next_range2)) {
        return false;
    }
    
    // only consider a ray suspicious if it's significantly different from multiple neighbors
    double avg_neighbor_range = (prev_range + next_range + prev_range2 + next_range2) / 4.0;
    double range_diff = std::abs(range - avg_neighbor_range);
    
    // only filter rays that are significantly longer than neighbors (passing through gaps)
    return (range > avg_neighbor_range * 2.0 && range_diff > 0.8);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
    ros::Time scan_time = scan->header.stamp;
    try 
    {
        // align time
        tf::StampedTransform laser_transform;
        tf_listener->lookupTransform("map", scan->header.frame_id, scan_time, laser_transform);

        tf::StampedTransform base_transform;
        tf_listener->lookupTransform("map", "base_footprint", scan_time, base_transform);

        // initialize the map
        if (!map_origin_initialized)
        {
            map_origin_offset = base_transform.getOrigin();
            map_origin_initialized = true;
            ROS_INFO("Map origin initialized at robot starting position: (%.2f, %.2f)",
                     map_origin_offset.x(), map_origin_offset.y());
        }

        /*
        1. Set range threshold
        2. Get the start point and end point
        3. Check the status of each grid (bresenhamLine)
        */
        const double min_range_threshold = 0.1;  // 10cm
        double angle = scan->angle_min;

        // Traverse all laser measurements
        for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) 
        {
            double range = scan->ranges[i];
            
            // Skip invalid measurements
            if (std::isnan(range) || range < min_range_threshold || range > scan->range_max) 
            {
                continue;
            }
            
            // skip suspicious rays
            if (isSuspiciousRay(scan, i)) {
                continue;
            }

            //map coordinates
            // Calculate laser endpoint in laser frame
            double x_laser = range * std::cos(angle);
            double y_laser = range * std::sin(angle);

            // Transform to map frame
            tf::Vector3 point_laser(x_laser, y_laser, 0.0);
            tf::Vector3 point_map = laser_transform * point_laser;

            // Get laser origin in map frame
            tf::Vector3 laser_origin = laser_transform.getOrigin();

            // Convert to grid coordinates
            auto [start_x, start_y] = worldToGrid(laser_origin.x(), laser_origin.y());
            auto [end_x, end_y] = worldToGrid(point_map.x(), point_map.y());

            // Skip if out of bounds
            if (start_x < 0 || start_x >= MAP_SIZE_X || start_y < 0 || start_y >= MAP_SIZE_Y 
                || end_x < 0 || end_x >= MAP_SIZE_X || end_y < 0 || end_y >= MAP_SIZE_Y)
            {
                continue;
            }

            // Get all points along the laser beam
            auto line_points = bresenhamLine(start_x, start_y, end_x, end_y);

            // Check if ray passes through any confirmed obstacle
            bool ray_passes_through_obstacle = false;
            for (size_t j = 0; j < line_points.size() - 1; ++j) {
                auto [x, y] = line_points[j];
                
                // Calculate distance from start point (robot position)
                int dx = x - start_x;
                int dy = y - start_y;
                double dist_from_start = std::sqrt(dx*dx + dy*dy);
                
                // Only apply near-obstacle check if not too close to robot
                if (confirmed_obstacles[x][y] || 
                    (dist_from_start > 20 && isNearObstacles(x, y, 1))) {
                    ray_passes_through_obstacle = true;
                    break;
                }
            }

            // Skip this ray if it passes through a confirmed obstacle
            if (ray_passes_through_obstacle) {
                continue;
            }

            // Process valid rays as before
            for (size_t j = 0; j < line_points.size(); ++j) {
                auto [x, y] = line_points[j];
                
                // Mark as visited
                visited[x][y] = true;

                if (j == line_points.size() - 1) {
                    // Endpoint (potential obstacle)
                    scan_count[x][y]++;
                    if (scan_count[x][y] >= SCAN_THRESHOLD) {
                        confirmed_obstacles[x][y] = true;
                    }
                } else {
                    // Points along the beam (free space)
                    scan_count[x][y] = std::max(0, scan_count[x][y] - DECAY_FACTOR);
                }
            }
        }
    } 
    catch (const tf::TransformException& ex) 
    {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }
}

void thickenWalls(std::vector<std::vector<bool>>& obstacles, int thickness = 1) {
    // For thinner walls, use smaller thickness
    thickness = std::max(1, thickness);
    
    std::vector<std::vector<bool>> temp = obstacles;
    
    for (int x = 0; x < MAP_SIZE_X; ++x) {
        for (int y = 0; y < MAP_SIZE_Y; ++y) {
            if (obstacles[x][y]) {
                // Use a smaller thickening radius
                for (int dx = -thickness; dx <= thickness; ++dx) {
                    for (int dy = -thickness; dy <= thickness; ++dy) {
                        // Apply thickening only if it's close to the obstacle
                        if (dx*dx + dy*dy <= thickness*thickness) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y) {
                                temp[nx][ny] = true;
                            }
                        }
                    }
                }
            }
        }
    }
    
    obstacles = temp;
}

void publishMap(const ros::TimerEvent& event) 
{
    ros::Time current_time = ros::Time::now();
    broadcastMapFrame(current_time);

    // Create temporary copy of confirmed_obstacles for wall thickening
    auto thickened_obstacles = confirmed_obstacles;
    
    // Apply wall thickening to close small gaps (adjust thickness as needed)
    int wall_thickness = 1;
    ros::NodeHandle nh;
    nh.param<int>("wall_thickness", wall_thickness, 1);
    thickenWalls(thickened_obstacles, wall_thickness);

    // Create and fill the occupancy grid message
    nav_msgs::OccupancyGrid map;
    map.header.stamp = current_time;
    map.header.frame_id = "map";

    map.info.resolution = GRID_RESOLUTION;
    map.info.width = MAP_SIZE_X;
    map.info.height = MAP_SIZE_Y;
    map.info.origin.position.x = -MAP_SIZE_X * GRID_RESOLUTION / 2.0;
    map.info.origin.position.y = -MAP_SIZE_Y * GRID_RESOLUTION / 2.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    // Fill the map data using thickened obstacles
    map.data.resize(MAP_SIZE_X * MAP_SIZE_Y);
    for (int y = 0; y < MAP_SIZE_Y; ++y) {
        for (int x = 0; x < MAP_SIZE_X; ++x) {
            int index = x + y * MAP_SIZE_X;
            if (!visited[x][y]) {
                map.data[index] = -1;  // Unknown
            } else if (thickened_obstacles[x][y]) {
                map.data[index] = 100;  // Occupied
            } else {
                map.data[index] = 0;    // Free
            }
        }
    }

    map_publisher.publish(map);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "create_gridmap");
    ros::NodeHandle nh;

    tf_listener = std::make_shared<tf::TransformListener>();
    tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 50, true);

    // Using message_filter is to align the timestamps of /scan and /tf
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 10);
    tf::MessageFilter<sensor_msgs::LaserScan> tf_filter(scan_sub, *tf_listener, "map", 10, nh);
    tf_filter.setTolerance(ros::Duration(0.1));
    tf_filter.registerCallback(boost::bind(&scanCallback, _1));

    // set a timer to publish the map
    ros::Timer map_timer = nh.createTimer(ros::Duration(0.05), publishMap);
    publishMap(ros::TimerEvent());
    ros::spin();
    return 0;
}

