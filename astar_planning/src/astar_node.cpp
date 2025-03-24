#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


nav_msgs::OccupancyGrid g_current_map;
bool g_has_map = false;
int g_width, g_height;
double g_resolution, g_origin_x, g_origin_y;
std::vector<int> g_map_data;
double g_robot_radius = 0.22;
ros::Publisher g_path_pub;
ros::Publisher g_raw_path_pub;
ros::Publisher g_inflated_map_pub;

// Bezier interpolation function
/*
TODO:
1. Calculate the Bezier basis
2. Interpolate the position based on the Bezier formula
3. Return the interpolated point
*/
geometry_msgs::PoseStamped cubicBezier(const geometry_msgs::PoseStamped& p0,
                                       const geometry_msgs::PoseStamped& p1,
                                       const geometry_msgs::PoseStamped& p2,
                                       const geometry_msgs::PoseStamped& p3,
                                       double t) 
{
    // step1: Calculate the Bezier basis
    double b0 = pow(1-t, 3);
    double b1 = 3 * t * pow(1-t, 2);
    double b2 = 3 * pow(t, 2) * (1-t);
    double b3 = pow(t, 3);

    geometry_msgs::PoseStamped pt;
    pt.header = p0.header;

    // step2: Interpolate the position based on the Bezier formula
    pt.pose.position.x = b0 * p0.pose.position.x + 
                         b1 * p1.pose.position.x + 
                         b2 * p2.pose.position.x + 
                         b3 * p3.pose.position.x;
    
    pt.pose.position.y = b0 * p0.pose.position.y + 
                         b1 * p1.pose.position.y + 
                         b2 * p2.pose.position.y + 
                         b3 * p3.pose.position.y;
    
    pt.pose.position.z = b0 * p0.pose.position.z + 
                         b1 * p1.pose.position.z + 
                         b2 * p2.pose.position.z + 
                         b3 * p3.pose.position.z;

    pt.pose.orientation.w = 1.0;

    // step3: Return the interpolated point
    return pt;
}

// Function to uniformly sample path points
nav_msgs::Path uniformSamplePath(const nav_msgs::Path& path_in, int num_samples = 20)
{
    nav_msgs::Path path_out;
    path_out.header = path_in.header;

    if (path_in.poses.size() <= num_samples)
    {
        return path_in;
    }

    // Compute the total path length
    double total_length = 0.0;
    std::vector<double> cumulative_lengths;
    cumulative_lengths.push_back(0.0);

    for (size_t i = 1; i < path_in.poses.size(); ++i)
    {
        double dx = path_in.poses[i].pose.position.x - path_in.poses[i-1].pose.position.x;
        double dy = path_in.poses[i].pose.position.y - path_in.poses[i-1].pose.position.y;
        double segment_length = std::sqrt(dx * dx + dy * dy);
        total_length += segment_length;
        cumulative_lengths.push_back(total_length);
    }

    // Compute the uniform sampling interval
    double segment_length = total_length / (num_samples - 1);

    // Add the starting point
    path_out.poses.push_back(path_in.poses.front());

    // Add uniformly distributed intermediate points
    for (int i = 1; i < num_samples - 1; ++i)
    {
        double target_length = i * segment_length;
        auto it = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), target_length);
        int idx = std::distance(cumulative_lengths.begin(), it);
        if (idx >= path_in.poses.size()) idx = path_in.poses.size() - 1;

        if (idx > 0)
        {
            int prev_idx = idx - 1;
            double prev_length = cumulative_lengths[prev_idx];
            double curr_length = cumulative_lengths[idx];
            double ratio = (target_length - prev_length) / (curr_length - prev_length);

            geometry_msgs::PoseStamped interpolated_pose;
            interpolated_pose.header = path_in.header;
            interpolated_pose.pose.position.x = path_in.poses[prev_idx].pose.position.x +
                ratio * (path_in.poses[idx].pose.position.x - path_in.poses[prev_idx].pose.position.x);
            interpolated_pose.pose.position.y = path_in.poses[prev_idx].pose.position.y +
                ratio * (path_in.poses[idx].pose.position.y - path_in.poses[prev_idx].pose.position.y);
            interpolated_pose.pose.orientation.w = 1.0;

            path_out.poses.push_back(interpolated_pose);
        }
        else
        {
            path_out.poses.push_back(path_in.poses[idx]);
        }
    }

    // Add the endpoint
    path_out.poses.push_back(path_in.poses.back());
    return path_out;
}

// calculate angle
double calculateAngle(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    return std::atan2(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
}

// Smoothing
/*
TODO:
1. Perform uniform sampling to ensure evenly distributed points
2. Apply a sliding window approach to smooth the path using cubic Bezier curves
3. Add the last point to ensure the path maintains its original endpoint
*/

nav_msgs::Path BezierSmoothing(const nav_msgs::Path& path_in)
{
    nav_msgs::Path path_out;
    path_out.header = path_in.header;

    if (path_in.poses.size() < 4)
    {
        return path_in;
    }

    // step 1, perform uniform sampling to ensure evenly distributed points
    int num_samples = 20;
    nav_msgs::Path sampled_path = uniformSamplePath(path_in, num_samples);

    // step2, apply a sliding window approach to smooth the path using cubic Bezier curves
    for (size_t i = 0; i < sampled_path.poses.size() - 3; i += 3)
    {
        // Get control points for the cubic Bezier curve
        geometry_msgs::PoseStamped p0 = sampled_path.poses[i];
        geometry_msgs::PoseStamped p1 = sampled_path.poses[i+1];
        geometry_msgs::PoseStamped p2 = sampled_path.poses[i+2];
        geometry_msgs::PoseStamped p3 = sampled_path.poses[i+3];
        
        // Calculate the angle difference between p0 and p3 to determine the interpolation step size
        double angle_diff = std::abs(calculateAngle(p0, p3));
        
        // Compute the number of interpolation points based on the angle difference
        int num_points = std::max(5, static_cast<int>(angle_diff * 10));
        
        // Generate interpolated points along the cubic Bezier curve
        for (int j = 0; j < num_points; ++j)
        {
            double t = static_cast<double>(j) / (num_points - 1);
            geometry_msgs::PoseStamped pt = cubicBezier(p0, p1, p2, p3, t);
            path_out.poses.push_back(pt);
        }
    }
    
    // step 3, add the last point to ensure the path maintains its original endpoint
    if (!sampled_path.poses.empty() && (path_out.poses.empty() || 
        path_out.poses.back().pose.position.x != sampled_path.poses.back().pose.position.x ||
        path_out.poses.back().pose.position.y != sampled_path.poses.back().pose.position.y))
    {
        path_out.poses.push_back(sampled_path.poses.back());
    }
    
    return path_out;
}

// coordinate transformation
bool worldToMap(double wx, double wy, int& mx, int& my)
{
    if (wx < g_origin_x || wy < g_origin_y) return false;
    mx = static_cast<int>((wx - g_origin_x) / g_resolution);
    my = static_cast<int>((wy - g_origin_y) / g_resolution);
    return (mx >= 0 && mx < g_width && my >= 0 && my < g_height);
}

void mapToWorld(int mx, int my, double& wx, double& wy)
{
    wx = g_origin_x + (mx + 0.5) * g_resolution;
    wy = g_origin_y + (my + 0.5) * g_resolution;
}

// inflate obstacles
/*
TODO:
1. Iterate through the occupancy grid
2. Expand obstacles using a square region
*/
void inflateObstacles()
{
    // Determine the inflation radius in grid cells
    const int inflate_cells = static_cast<int>(std::ceil(g_robot_radius / g_resolution));
    
    // Initialize inflated_map with -1 (unknown) for all cells
    std::vector<int> inflated_map(g_width * g_height, -1);
    
    // Copy the original map data
    g_map_data = std::vector<int>(g_current_map.data.begin(), g_current_map.data.end());

    // Iterate through occupancy grid
    for (int y = 0; y < g_height; ++y) {
        for (int x = 0; x < g_width; ++x) {
            int idx = y * g_width + x;
            if (g_map_data[idx] < 0) {
                inflated_map[idx] = -1;
            } else if (g_map_data[idx] < 50) {
                inflated_map[idx] = 0;
            } else {
                inflated_map[idx] = 100;
            }
        }å
    }
    
    // Expand obstacles using a square region
    for (int y = 0; y < g_height; ++y) {
        for (int x = 0; x < g_width; ++x) {
            int idx = y * g_width + x;
            
            // only inflate known obstacles
            if (g_map_data[idx] > 50) {
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < g_width && ny >= 0 && ny < g_height) {
                            int nidx = ny * g_width + nx;
                            
                            // only overwrite known free space
                            if (inflated_map[nidx] >= 0) {
                                inflated_map[nidx] = 100;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Update the global map with the inflated obstacles
    g_map_data = inflated_map;
    
    // Publish the inflated map for visualization
    nav_msgs::OccupancyGrid inflated_grid;
    inflated_grid.header = g_current_map.header;
    inflated_grid.info = g_current_map.info;
    
    // Convert from int to int8_t for ROS message
    inflated_grid.data.resize(g_map_data.size());
    for (size_t i = 0; i < g_map_data.size(); ++i) {
        inflated_grid.data[i] = static_cast<int8_t>(g_map_data[i]);
    }
    
    g_inflated_map_pub.publish(inflated_grid);
}


// Build path
void buildPath(const std::vector<int>& came_from, int goal_idx, std::vector<std::pair<int, int>>& path)
{
    path.clear();
    while (goal_idx != -1)
    {
        int x = goal_idx % g_width;
        int y = goal_idx / g_width;
        path.emplace_back(x, y);
        goal_idx = came_from[goal_idx];
    }
    std::reverse(path.begin(), path.end());
}

// A* Search algorithm to find the shortest path from start to goal
/*
TODO:
step1: Heuristic function (Euclidean distance)
step2: Initialize cost arrays and came_from array
step3: Set initial cost for the start node
step4: A* search loop
*/

bool aStarSearch(int start_x, int start_y, int goal_x, int goal_y, std::vector<std::pair<int, int>>& path)
{
    // 8 directions for movement (right, top-right, up, top-left, left, bottom-left, down, bottom-right)
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    // step1: Heuristic function (Euclidean distance)
    auto heuristic = [goal_x, goal_y](int x, int y) -> double {
        return std::sqrt(std::pow(x - goal_x, 2) + std::pow(y - goal_y, 2));
    };

    // Convert (x, y) coordinates to 1D index for accessing the grid arrays
    auto toIndex = [](int x, int y) { return y * g_width + x; };

    // step2: Initialize cost arrays and came_from array
    std::vector<double> g_cost(g_width * g_height, std::numeric_limits<double>::infinity());
    std::vector<double> f_cost(g_width * g_height, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(g_width * g_height, -1);
    std::vector<bool> closed(g_width * g_height, false);
    
    // Priority queue to select the node with the lowest f_cost (using std::greater for min-heap)
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>> open;

    // Start and goal indices in the 1D array
    int start_idx = toIndex(start_x, start_y);
    int goal_idx = toIndex(goal_x, goal_y);

    // step3: Set initial cost for the start node
    g_cost[start_idx] = 0;
    f_cost[start_idx] = heuristic(start_x, start_y);
    
    // Push the start node into the open list (priority queue)
    open.emplace(f_cost[start_idx], start_idx);

    // step4: A* search loop
    while (!open.empty())
    {
        // Get the node with the lowest f_cost from the open list
        int current_idx = open.top().second;
        open.pop();
        
        // if we've reached the goal, build the path and return
        if (current_idx == goal_idx)
        {
            buildPath(came_from, goal_idx, path);
            return true;
        }
        
        // mark the current node as closed
        if (closed[current_idx]) continue;
        closed[current_idx] = true;

        // Get the coordinates (x, y) of the current node from the 1D index
        int current_x = current_idx % g_width;
        int current_y = current_idx / g_width;

        // Explore all 8 possible neighbors
        for (int i = 0; i < 8; ++i)
        {
            int nx = current_x + dx[i];
            int ny = current_y + dy[i];
            
            // check if the neighbor is within the map bounds
            if (nx < 0 || nx >= g_width || ny < 0 || ny >= g_height)
                continue;
                
            int neighbor_idx = toIndex(nx, ny);
            
            // skip if the neighbor is in the closed list or is an obstacle
            if (closed[neighbor_idx] || g_map_data[neighbor_idx] > 50)
                continue;
                
            // calculate the cost to reach the neighbor through the current node
            double movement_cost = (i % 2 == 0) ? 1.0 : 1.414; // Diagonal movement costs more
            double tentative_g_cost = g_cost[current_idx] + movement_cost;
            
            // if this path to the neighbor is better than any previous one, update it
            if (tentative_g_cost < g_cost[neighbor_idx])
            {
                came_from[neighbor_idx] = current_idx;
                g_cost[neighbor_idx] = tentative_g_cost;
                f_cost[neighbor_idx] = tentative_g_cost + heuristic(nx, ny);
                
                // Add the neighbor to the open list
                open.emplace(f_cost[neighbor_idx], neighbor_idx);
            }
        }
    }

    // If the open list is empty and the goal has not been reached, no path exists
    return false;
}

// map callback
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    g_current_map = *msg;
    g_width = msg->info.width;
    g_height = msg->info.height;
    g_resolution = msg->info.resolution;
    g_origin_x = msg->info.origin.position.x;
    g_origin_y = msg->info.origin.position.y;
    g_has_map = true;
    inflateObstacles();
    ROS_INFO("Map received and obstacles inflated");
}

// goal callback
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    if (!g_has_map)
    {
        ROS_WARN("No map available");
        return;
    }

    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }

    double start_x = transform.transform.translation.x;
    double start_y = transform.transform.translation.y;
    double goal_x = goal_msg->pose.position.x;
    double goal_y = goal_msg->pose.position.y;

    int sx, sy, gx, gy;
    if (!worldToMap(start_x, start_y, sx, sy) || 
        !worldToMap(goal_x, goal_y, gx, gy))
    {
        ROS_WARN("Invalid start/goal coordinates");
        return;
    }

    std::vector<std::pair<int, int>> path_indices;
    if (!aStarSearch(sx, sy, gx, gy, path_indices))
    {
        ROS_WARN("A* search failed");
        return;
    }

    // generate path
    nav_msgs::Path raw_path;
    raw_path.header.frame_id = "map";
    raw_path.header.stamp = ros::Time::now();
    for (auto& p : path_indices)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        mapToWorld(p.first, p.second, pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1.0;
        raw_path.poses.push_back(pose);
    }

    g_raw_path_pub.publish(raw_path);

    // smooth path
    nav_msgs::Path smooth_path = BezierSmoothing(raw_path);
    g_path_pub.publish(smooth_path);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("robot_radius", g_robot_radius, 0.20);

    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal", 1, goalCallback);
    g_path_pub = nh.advertise<nav_msgs::Path>("global_path", 1);
    g_raw_path_pub = nh.advertise<nav_msgs::Path>("raw_path", 1);
    g_inflated_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1);

    ros::spin();
    return 0;
}
