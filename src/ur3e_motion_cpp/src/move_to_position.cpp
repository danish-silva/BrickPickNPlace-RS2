#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <mutex>
#include <atomic>
#include <thread>
#include <map>

static const std::string DEFAULT_PLANNING_GROUP = "ur_onrobot_manipulator";
static const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

// ---------------------------------------------------------------------------
// Structure to hold a target pose cleanly
// ---------------------------------------------------------------------------
struct TargetPose {
    double x, y, z;
    double roll, pitch, yaw;
    std::string name;
};

struct TargetJoints {
    std::vector<double> positions;
    std::string name;
};

// ---------------------------------------------------------------------------
// Global state for subscriber
// ---------------------------------------------------------------------------
std::vector<TargetPose> g_received_poses;
std::vector<TargetJoints> g_received_joint_targets;
std::mutex g_poses_mutex;
std::mutex g_joints_mutex;
std::atomic<bool> g_new_poses_available{false};
std::atomic<bool> g_new_joints_available{false};

// ---------------------------------------------------------------------------
// Apply joint constraints to prevent >180 degree rotations
// ---------------------------------------------------------------------------
void applyJointConstraints(moveit::planning_interface::MoveGroupInterface & move_group)
{
    moveit_msgs::msg::Constraints constraints;

    std::vector<std::pair<std::string, std::pair<double, double>>> joint_limits = {
        {"shoulder_pan_joint",  {-2*M_PI,  2*M_PI}},
        {"shoulder_lift_joint", {-M_PI,    M_PI  }},
        {"elbow_joint",         {-M_PI,    M_PI  }},
        {"wrist_1_joint",       {-M_PI,    M_PI  }},
        {"wrist_2_joint",       {-M_PI,    M_PI  }},
        {"wrist_3_joint",       {-M_PI,    M_PI  }},
    };

    for (auto & [name, limits] : joint_limits) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name      = name;
        jc.position        = (limits.first + limits.second) / 2.0;
        jc.tolerance_below = std::abs(jc.position - limits.first);
        jc.tolerance_above = std::abs(limits.second - jc.position);
        jc.weight          = 1.0;
        constraints.joint_constraints.push_back(jc);
    }

    move_group.setPathConstraints(constraints);
}

// ---------------------------------------------------------------------------
// Move to a single pose
// ---------------------------------------------------------------------------
bool moveToPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const TargetPose & target,
    const rclcpp::Logger & logger)
{
    RCLCPP_INFO(logger, "Moving to '%s' — x:%.2f y:%.2f z:%.2f r:%.2f p:%.2f y:%.2f",
        target.name.c_str(),
        target.x, target.y, target.z,
        target.roll, target.pitch, target.yaw);

    move_group.setStartStateToCurrentState();

    geometry_msgs::msg::Pose pose;
    pose.position.x = target.x;
    pose.position.y = target.y;
    pose.position.z = target.z;

    tf2::Quaternion q;
    q.setRPY(target.roll, target.pitch, target.yaw);
    pose.orientation = tf2::toMsg(q);

    move_group.setPoseTarget(pose);
    move_group.setNumPlanningAttempts(20);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(logger, "Planning FAILED for '%s'", target.name.c_str());
        return false;
    }

    bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (executed) {
        RCLCPP_INFO(logger, "Reached '%s' ✓", target.name.c_str());
    } else {
        RCLCPP_ERROR(logger, "Execution FAILED for '%s'", target.name.c_str());
    }

    return executed;
}

// ---------------------------------------------------------------------------
// Move through a list of poses, stopping if any movement fails
// ---------------------------------------------------------------------------
bool moveSequence(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::vector<TargetPose> & targets,
    const rclcpp::Logger & logger)
{
    RCLCPP_INFO(logger, "Starting sequence of %zu movements", targets.size());

    for (size_t i = 0; i < targets.size(); ++i) {
        RCLCPP_INFO(logger, "Step %zu / %zu", i + 1, targets.size());

        if (!moveToPose(move_group, targets[i], logger)) {
            RCLCPP_ERROR(logger, "Sequence aborted at step %zu / %zu — '%s' failed",
                i + 1, targets.size(), targets[i].name.c_str());
            return false;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(logger, "Sequence complete ✓ — all %zu movements succeeded", targets.size());
    return true;
}

bool moveToJoints(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const TargetJoints & target,
    const rclcpp::Logger & logger)
{
    std::map<std::string, double> joint_goal;
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
        joint_goal[JOINT_NAMES[i]] = target.positions[i];
    }

    RCLCPP_INFO(logger, "Moving to joint target '%s'", target.name.c_str());
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
        RCLCPP_INFO(logger, "  %s: %.3f rad", JOINT_NAMES[i].c_str(), target.positions[i]);
    }

    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(joint_goal);
    move_group.setNumPlanningAttempts(20);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(logger, "Planning FAILED for joint target '%s'", target.name.c_str());
        return false;
    }

    bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (executed) {
        RCLCPP_INFO(logger, "Reached joint target '%s'", target.name.c_str());
    } else {
        RCLCPP_ERROR(logger, "Execution FAILED for joint target '%s'", target.name.c_str());
    }

    return executed;
}

bool moveJointSequence(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const std::vector<TargetJoints> & targets,
    const rclcpp::Logger & logger)
{
    RCLCPP_INFO(logger, "Starting joint sequence of %zu movements", targets.size());

    for (size_t i = 0; i < targets.size(); ++i) {
        RCLCPP_INFO(logger, "Joint step %zu / %zu", i + 1, targets.size());

        if (!moveToJoints(move_group, targets[i], logger)) {
            RCLCPP_ERROR(logger, "Joint sequence aborted at step %zu / %zu",
                i + 1, targets.size());
            return false;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(logger, "Joint sequence complete");
    return true;
}

// ---------------------------------------------------------------------------
// Setup collision scene
// ---------------------------------------------------------------------------
void setupScene(moveit::planning_interface::PlanningSceneInterface & scene)
{
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    {
        moveit_msgs::msg::CollisionObject ground;
        ground.header.frame_id = "base_link";
        ground.id = "ground";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.0;
        primitive.dimensions[primitive.BOX_Y] = 2.0;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z    = -0.005;

        ground.primitives.push_back(primitive);
        ground.primitive_poses.push_back(pose);
        ground.operation = ground.ADD;
        objects.push_back(ground);
    }

    {
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "base_link";
        table.id = "table";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.2;
        primitive.dimensions[primitive.BOX_Y] = 0.8;
        primitive.dimensions[primitive.BOX_Z] = 0.05;

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x    =  0.3;
        pose.position.y    =  0.0;
        pose.position.z    = -0.025;

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(pose);
        table.operation = table.ADD;
        objects.push_back(table);
    }

    scene.applyCollisionObjects(objects);
    RCLCPP_INFO(rclcpp::get_logger("move_to_position"), "Scene objects added ✓");
}

// ---------------------------------------------------------------------------
// Print current end-effector pose
// ---------------------------------------------------------------------------
void printCurrentPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const rclcpp::Logger & logger)
{
    geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose();

    tf2::Quaternion q(
        current.pose.orientation.x,
        current.pose.orientation.y,
        current.pose.orientation.z,
        current.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(logger, "Current pose:");
    RCLCPP_INFO(logger, "  x: %.4f  y: %.4f  z: %.4f",
        current.pose.position.x,
        current.pose.position.y,
        current.pose.position.z);
    RCLCPP_INFO(logger, "  roll: %.1f°  pitch: %.1f°  yaw: %.1f°",
        roll  * 180.0 / M_PI,
        pitch * 180.0 / M_PI,
        yaw   * 180.0 / M_PI);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(
        "move_to_position",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();
    std::string planning_group = DEFAULT_PLANNING_GROUP;
    node->get_parameter("planning_group", planning_group);

    RCLCPP_INFO(logger, "Using MoveIt planning group '%s'", planning_group.c_str());

    // -----------------------------------------------------------------------
    // Subscribe to ordered_pose_array topic
    //
    // Message type: std_msgs/Float64MultiArray
    // Format: flat array of 6 values per pose [x, y, z, roll, pitch, yaw]
    //
    // Example for 2 poses:
    // data: [0.3, 0.25, 0.3, 3.14159, 0.0, 0.0,
    //        0.3, 0.25, 0.2, 3.14159, 0.0, 0.0]
    // -----------------------------------------------------------------------
    auto pose_subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "ordered_pose_array",
        10,
        [&logger](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

            // Each pose is exactly 6 values: x, y, z, roll, pitch, yaw
            const size_t POSE_SIZE = 6;

            if (msg->data.size() % POSE_SIZE != 0) {
                RCLCPP_ERROR(logger,
                    "Received array size %zu is not a multiple of 6 — ignoring",
                    msg->data.size());
                return;
            }

            size_t num_poses = msg->data.size() / POSE_SIZE;
            RCLCPP_INFO(logger, "Received %zu poses", num_poses);

            std::vector<TargetPose> new_poses;
            for (size_t i = 0; i < num_poses; ++i) {
                size_t offset = i * POSE_SIZE;
                TargetPose tp;
                tp.x     = msg->data[offset + 0];
                tp.y     = msg->data[offset + 1];
                tp.z     = msg->data[offset + 2];
                tp.roll  = msg->data[offset + 3];
                tp.pitch = msg->data[offset + 4];
                tp.yaw   = msg->data[offset + 5];
                tp.name  = "position_" + std::to_string(i + 1);

                RCLCPP_INFO(logger,
                    "  %s: x:%.3f y:%.3f z:%.3f r:%.3f p:%.3f y:%.3f",
                    tp.name.c_str(),
                    tp.x, tp.y, tp.z,
                    tp.roll, tp.pitch, tp.yaw);

                new_poses.push_back(tp);
            }

            std::lock_guard<std::mutex> lock(g_poses_mutex);
            g_received_poses = new_poses;
            g_new_poses_available = true;
        }
    );

    auto joint_subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "ordered_joint_array",
        10,
        [&logger](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            const size_t JOINT_TARGET_SIZE = 6;

            if (msg->data.size() % JOINT_TARGET_SIZE != 0) {
                RCLCPP_ERROR(logger,
                    "Received joint array size %zu is not a multiple of 6 — ignoring",
                    msg->data.size());
                return;
            }

            size_t num_targets = msg->data.size() / JOINT_TARGET_SIZE;
            RCLCPP_INFO(logger, "Received %zu joint target(s)", num_targets);

            std::vector<TargetJoints> new_targets;
            for (size_t i = 0; i < num_targets; ++i) {
                size_t offset = i * JOINT_TARGET_SIZE;
                TargetJoints target;
                target.name = "joint_target_" + std::to_string(i + 1);
                target.positions.assign(
                    msg->data.begin() + offset,
                    msg->data.begin() + offset + JOINT_TARGET_SIZE
                );
                new_targets.push_back(target);
            }

            std::lock_guard<std::mutex> lock(g_joints_mutex);
            g_received_joint_targets = new_targets;
            g_new_joints_available = true;
        }
    );

    // Spin node in background so subscriber and MoveGroupInterface can receive callbacks
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(15.0);

    moveit::planning_interface::PlanningSceneInterface scene;
    setupScene(scene);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    printCurrentPose(move_group, logger);

    // -----------------------------------------------------------------------
    // Wait for pose array then execute — loops so it re-runs on new data
    // -----------------------------------------------------------------------
    RCLCPP_INFO(logger, "Waiting for poses on 'ordered_pose_array' or joints on 'ordered_joint_array'...");

    while (rclcpp::ok()) {
        if (g_new_joints_available) {
            std::vector<TargetJoints> targets;
            {
                std::lock_guard<std::mutex> lock(g_joints_mutex);
                targets = g_received_joint_targets;
                g_new_joints_available = false;
            }

            RCLCPP_INFO(logger, "Executing joint sequence of %zu targets", targets.size());
            moveJointSequence(move_group, targets, logger);

            move_group.clearPathConstraints();
            RCLCPP_INFO(logger, "Joint sequence complete — waiting for next command...");
        } else if (g_new_poses_available) {
            std::vector<TargetPose> targets;
            {
                std::lock_guard<std::mutex> lock(g_poses_mutex);
                targets = g_received_poses;
                g_new_poses_available = false;
            }

            RCLCPP_INFO(logger, "Executing sequence of %zu poses", targets.size());
            moveSequence(move_group, targets, logger);

            move_group.clearPathConstraints();
            RCLCPP_INFO(logger, "Sequence complete — waiting for next pose array...");
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}

// ---------------------------------------------------------------------------------------------------------------

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/joint_constraint.hpp>
// #include <shape_msgs/msg/solid_primitive.hpp>

// #include <tf2/LinearMath/Matrix3x3.h>
// #include <cmath>

// static const std::string PLANNING_GROUP = "ur_manipulator";

// // ---------------------------------------------------------------------------
// // Structure to hold a target pose cleanly
// // ---------------------------------------------------------------------------
// struct TargetPose {
//     double x, y, z;
//     double roll, pitch, yaw;
//     std::string name;  // optional label for logging
// };

// // ---------------------------------------------------------------------------
// // Method 1 — Apply joint constraints to prevent >180 degree rotations
// // ---------------------------------------------------------------------------
// void applyJointConstraints(moveit::planning_interface::MoveGroupInterface & move_group)
// {
//     moveit_msgs::msg::Constraints constraints;

//     std::vector<std::pair<std::string, std::pair<double, double>>> joint_limits = {
//         {"shoulder_pan_joint",  {-2*M_PI,    2*M_PI}},   // loosened — full rotation allowed
//         {"shoulder_lift_joint", {-M_PI,      M_PI}},     // loosened from {-M_PI, 0}
//         {"elbow_joint",         {-M_PI,      M_PI}},
//         {"wrist_1_joint",       {-M_PI,      M_PI}},
//         {"wrist_2_joint",       {-M_PI,      M_PI}},
//         {"wrist_3_joint",       {-M_PI,      M_PI}},
//     };

//     for (auto & [name, limits] : joint_limits) {
//         moveit_msgs::msg::JointConstraint jc;
//         jc.joint_name = name;
//         jc.position = (limits.first + limits.second) / 2.0;
//         jc.tolerance_below = std::abs(jc.position - limits.first);
//         jc.tolerance_above = std::abs(limits.second - jc.position);
//         jc.weight = 1.0;
//         constraints.joint_constraints.push_back(jc);
//     }

//     move_group.setPathConstraints(constraints);
// }

// // ---------------------------------------------------------------------------
// // Move to a single pose — Method 1 + 2 combined
// // ---------------------------------------------------------------------------
// bool moveToPose(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     const TargetPose & target,
//     const rclcpp::Logger & logger)
// {
//     RCLCPP_INFO(logger, "Moving to '%s' — x:%.2f y:%.2f z:%.2f r:%.2f p:%.2f y:%.2f",
//         target.name.c_str(),
//         target.x, target.y, target.z,
//         target.roll, target.pitch, target.yaw);

//     move_group.setStartStateToCurrentState();

//     // Build target pose
//     geometry_msgs::msg::Pose pose;
//     pose.position.x = target.x;
//     pose.position.y = target.y;
//     pose.position.z = target.z;

//     tf2::Quaternion q;
//     q.setRPY(target.roll, target.pitch, target.yaw);
//     pose.orientation = tf2::toMsg(q);

//     move_group.setPoseTarget(pose);
//     move_group.setNumPlanningAttempts(20);

//     // Plan
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) {
//         RCLCPP_ERROR(logger, "Planning FAILED for '%s'", target.name.c_str());
//         return false;
//     }

//     // Execute
//     bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (executed) {
//         RCLCPP_INFO(logger, "Reached '%s' ✓", target.name.c_str());
//     } else {
//         RCLCPP_ERROR(logger, "Execution FAILED for '%s'", target.name.c_str());
//     }

//     return executed;
// }

// // ---------------------------------------------------------------------------
// // Move through a list of poses, stopping if any movement fails
// // ---------------------------------------------------------------------------
// bool moveSequence(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     const std::vector<TargetPose> & targets,
//     const rclcpp::Logger & logger)
// {
//     RCLCPP_INFO(logger, "Starting sequence of %zu movements", targets.size());

//     for (size_t i = 0; i < targets.size(); ++i) {
//         RCLCPP_INFO(logger, "Step %zu / %zu", i + 1, targets.size());

//         bool success = moveToPose(move_group, targets[i], logger);

//         if (!success) {
//             RCLCPP_ERROR(logger, "Sequence aborted at step %zu / %zu — '%s' failed",
//                 i + 1, targets.size(), targets[i].name.c_str());
//             return false;
//         }

//         // Small pause between movements
//         rclcpp::sleep_for(std::chrono::milliseconds(500));
//     }

//     RCLCPP_INFO(logger, "Sequence complete ✓ — all %zu movements succeeded", targets.size());
//     return true;
// }

// // ---------------------------------------------------------------------------
// // Setup collision scene
// // ---------------------------------------------------------------------------
// void setupScene(moveit::planning_interface::PlanningSceneInterface & scene)
// {
//     std::vector<moveit_msgs::msg::CollisionObject> objects;

//     {
//         moveit_msgs::msg::CollisionObject ground;
//         ground.header.frame_id = "base_link";
//         ground.id = "ground";

//         shape_msgs::msg::SolidPrimitive primitive;
//         primitive.type = primitive.BOX;
//         primitive.dimensions.resize(3);
//         primitive.dimensions[primitive.BOX_X] = 2.0;
//         primitive.dimensions[primitive.BOX_Y] = 2.0;
//         primitive.dimensions[primitive.BOX_Z] = 0.01;

//         geometry_msgs::msg::Pose pose;
//         pose.orientation.w = 1.0;
//         pose.position.x = 0.0;
//         pose.position.y = 0.0;
//         pose.position.z = -0.005;

//         ground.primitives.push_back(primitive);
//         ground.primitive_poses.push_back(pose);
//         ground.operation = ground.ADD;
//         objects.push_back(ground);
//     }

//     {
//         moveit_msgs::msg::CollisionObject table;
//         table.header.frame_id = "base_link";
//         table.id = "table";

//         shape_msgs::msg::SolidPrimitive primitive;
//         primitive.type = primitive.BOX;
//         primitive.dimensions.resize(3);
//         primitive.dimensions[primitive.BOX_X] = 1.2;
//         primitive.dimensions[primitive.BOX_Y] = 0.8;
//         primitive.dimensions[primitive.BOX_Z] = 0.05;

//         geometry_msgs::msg::Pose pose;
//         pose.orientation.w = 1.0;
//         pose.position.x =  0.3;
//         pose.position.y =  0.0;
//         pose.position.z = -0.025;

//         table.primitives.push_back(primitive);
//         table.primitive_poses.push_back(pose);
//         table.operation = table.ADD;
//         objects.push_back(table);
//     }

//     scene.applyCollisionObjects(objects);
//     RCLCPP_INFO(rclcpp::get_logger("move_to_position"), "Scene objects added ✓");
// }

// void printCurrentPose(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     const rclcpp::Logger & logger)
// {
//     // Get current pose
//     geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose();

//     double x = current.pose.position.x;
//     double y = current.pose.position.y;
//     double z = current.pose.position.z;

//     // Convert quaternion to RPY
//     tf2::Quaternion q(
//         current.pose.orientation.x,
//         current.pose.orientation.y,
//         current.pose.orientation.z,
//         current.pose.orientation.w
//     );
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     RCLCPP_INFO(logger, "Current pose:");
//     RCLCPP_INFO(logger, "  x: %.4f  y: %.4f  z: %.4f", x, y, z);
//     RCLCPP_INFO(logger, "  roll: %.4f  pitch: %.4f  yaw: %.4f", roll, pitch, yaw);
//     RCLCPP_INFO(logger, "  roll: %.1f°  pitch: %.1f°  yaw: %.1f°",
//         roll  * 180.0 / M_PI,
//         pitch * 180.0 / M_PI,
//         yaw   * 180.0 / M_PI);
// }

// // ---------------------------------------------------------------------------
// // Main
// // ---------------------------------------------------------------------------
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared(
//         "move_to_position",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     auto spinner = std::thread([&executor]() { executor.spin(); });

//     auto logger = node->get_logger();

//     moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
//     move_group.setMaxVelocityScalingFactor(0.3);
//     move_group.setMaxAccelerationScalingFactor(0.3);
//     move_group.setPlanningTime(15.0);

//     // Setup collision scene
//     moveit::planning_interface::PlanningSceneInterface scene;
//     setupScene(scene);
//     rclcpp::sleep_for(std::chrono::milliseconds(500));

//     printCurrentPose(move_group, logger);

//     std::vector<TargetPose> targets = {
//         {0.3,  0.25,  0.3,  M_PI,  0.0,  0.0,  "position_1"},
//         {0.3,  0.25,  0.2,  M_PI,  0.0,  0.0,  "position_2"},
//         {0.3,  0.25,  0.3,  M_PI,  0.0,  0.0,  "position_3"},
//         {-0.3,  0.25,  0.3,  M_PI,  0.0,  0.0,  "position_4"},
//     };

//     // Execute the sequence
//     moveSequence(move_group, targets, logger);

//     // Clear constraints when fully done
//     move_group.clearPathConstraints();

//     rclcpp::shutdown();
//     spinner.join();
//     return 0;
// }

// -------------------------------------------------------------------------------------------------------------------------------------


// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/joint_constraint.hpp>
// #include <shape_msgs/msg/solid_primitive.hpp>

// #include <tf2/LinearMath/Matrix3x3.h>
// #include <cmath>

// static const std::string PLANNING_GROUP = "ur_manipulator";

// // ---------------------------------------------------------------------------
// // Structure to hold a target pose cleanly
// // ---------------------------------------------------------------------------
// struct TargetPose {
//     double x, y, z;
//     double roll, pitch, yaw;
//     std::string name;  // optional label for logging
// };

// // ---------------------------------------------------------------------------
// // Method 1 — Apply joint constraints to prevent >180 degree rotations
// // ---------------------------------------------------------------------------
// void applyJointConstraints(moveit::planning_interface::MoveGroupInterface & move_group)
// {
//     moveit_msgs::msg::Constraints constraints;

//     std::vector<std::pair<std::string, std::pair<double, double>>> joint_limits = {
//         {"shoulder_pan_joint",  {-2*M_PI,    2*M_PI}},   // loosened — full rotation allowed
//         {"shoulder_lift_joint", {-M_PI,      M_PI}},     // loosened from {-M_PI, 0}
//         {"elbow_joint",         {-M_PI,      M_PI}},
//         {"wrist_1_joint",       {-M_PI,      M_PI}},
//         {"wrist_2_joint",       {-M_PI,      M_PI}},
//         {"wrist_3_joint",       {-M_PI,      M_PI}},
//     };

//     for (auto & [name, limits] : joint_limits) {
//         moveit_msgs::msg::JointConstraint jc;
//         jc.joint_name = name;
//         jc.position = (limits.first + limits.second) / 2.0;
//         jc.tolerance_below = std::abs(jc.position - limits.first);
//         jc.tolerance_above = std::abs(limits.second - jc.position);
//         jc.weight = 1.0;
//         constraints.joint_constraints.push_back(jc);
//     }

//     move_group.setPathConstraints(constraints);
// }

// // ---------------------------------------------------------------------------
// // Move to a single pose — Method 1 + 2 combined
// // ---------------------------------------------------------------------------
// bool moveToPose(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     const TargetPose & target,
//     const rclcpp::Logger & logger)
// {
//     RCLCPP_INFO(logger, "Moving to '%s' — x:%.2f y:%.2f z:%.2f r:%.2f p:%.2f y:%.2f",
//         target.name.c_str(),
//         target.x, target.y, target.z,
//         target.roll, target.pitch, target.yaw);

//     // Seed the IK search from a known good joint configuration
//     // moveit::core::RobotStatePtr seed_state = move_group.getCurrentState();
//     // const moveit::core::JointModelGroup* jmg =
//     //     seed_state->getJointModelGroup(PLANNING_GROUP);

//     // std::vector<double> seed = {
//     //     0.0,
//     //     -1.5707963267948966,
//     //     -1.5707963267948966,
//     //     -1.5707963267948966,
//     //     1.5707963267948966,
//     //     0.0
//     // };
//     // seed_state->setJointGroupPositions(jmg, seed);
//     // move_group.setStartState(*seed_state);

//     // Replace your custom seed with current robot state
//     // auto seed_state = move_group.getCurrentState();
//     // move_group.setStartState(*seed_state);

//     move_group.setStartStateToCurrentState();

//     // Build target pose
//     geometry_msgs::msg::Pose pose;
//     pose.position.x = target.x;
//     pose.position.y = target.y;
//     pose.position.z = target.z;

//     tf2::Quaternion q;
//     q.setRPY(target.roll, target.pitch, target.yaw);
//     pose.orientation = tf2::toMsg(q);

//     move_group.setPoseTarget(pose);
//     move_group.setNumPlanningAttempts(20);

//     // Plan
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) {
//         RCLCPP_ERROR(logger, "Planning FAILED for '%s'", target.name.c_str());
//         return false;
//     }

//     // Execute
//     bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (executed) {
//         RCLCPP_INFO(logger, "Reached '%s' ✓", target.name.c_str());
//     } else {
//         RCLCPP_ERROR(logger, "Execution FAILED for '%s'", target.name.c_str());
//     }

//     return executed;
// }

// // ---------------------------------------------------------------------------
// // Move through a list of poses, stopping if any movement fails
// // ---------------------------------------------------------------------------
// bool moveSequence(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     const std::vector<TargetPose> & targets,
//     const rclcpp::Logger & logger)
// {
//     RCLCPP_INFO(logger, "Starting sequence of %zu movements", targets.size());

//     for (size_t i = 0; i < targets.size(); ++i) {
//         RCLCPP_INFO(logger, "Step %zu / %zu", i + 1, targets.size());

//         bool success = moveToPose(move_group, targets[i], logger);

//         if (!success) {
//             RCLCPP_ERROR(logger, "Sequence aborted at step %zu / %zu — '%s' failed",
//                 i + 1, targets.size(), targets[i].name.c_str());
//             return false;
//         }

//         // Small pause between movements
//         rclcpp::sleep_for(std::chrono::milliseconds(500));
//     }

//     RCLCPP_INFO(logger, "Sequence complete ✓ — all %zu movements succeeded", targets.size());
//     return true;
// }

// // ---------------------------------------------------------------------------
// // Setup collision scene
// // ---------------------------------------------------------------------------
// void setupScene(moveit::planning_interface::PlanningSceneInterface & scene)
// {
//     std::vector<moveit_msgs::msg::CollisionObject> objects;

//     {
//         moveit_msgs::msg::CollisionObject ground;
//         ground.header.frame_id = "base_link";
//         ground.id = "ground";

//         shape_msgs::msg::SolidPrimitive primitive;
//         primitive.type = primitive.BOX;
//         primitive.dimensions.resize(3);
//         primitive.dimensions[primitive.BOX_X] = 2.0;
//         primitive.dimensions[primitive.BOX_Y] = 2.0;
//         primitive.dimensions[primitive.BOX_Z] = 0.01;

//         geometry_msgs::msg::Pose pose;
//         pose.orientation.w = 1.0;
//         pose.position.x = 0.0;
//         pose.position.y = 0.0;
//         pose.position.z = -0.005;

//         ground.primitives.push_back(primitive);
//         ground.primitive_poses.push_back(pose);
//         ground.operation = ground.ADD;
//         objects.push_back(ground);
//     }

//     {
//         moveit_msgs::msg::CollisionObject table;
//         table.header.frame_id = "base_link";
//         table.id = "table";

//         shape_msgs::msg::SolidPrimitive primitive;
//         primitive.type = primitive.BOX;
//         primitive.dimensions.resize(3);
//         primitive.dimensions[primitive.BOX_X] = 1.2;
//         primitive.dimensions[primitive.BOX_Y] = 0.8;
//         primitive.dimensions[primitive.BOX_Z] = 0.05;

//         geometry_msgs::msg::Pose pose;
//         pose.orientation.w = 1.0;
//         pose.position.x =  0.3;
//         pose.position.y =  0.0;
//         pose.position.z = -0.025;

//         table.primitives.push_back(primitive);
//         table.primitive_poses.push_back(pose);
//         table.operation = table.ADD;
//         objects.push_back(table);
//     }

//     scene.applyCollisionObjects(objects);
//     RCLCPP_INFO(rclcpp::get_logger("move_to_position"), "Scene objects added ✓");
// }

// void printCurrentPose(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     const rclcpp::Logger & logger)
// {
//     // Get current pose
//     geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose();

//     double x = current.pose.position.x;
//     double y = current.pose.position.y;
//     double z = current.pose.position.z;

//     // Convert quaternion to RPY
//     tf2::Quaternion q(
//         current.pose.orientation.x,
//         current.pose.orientation.y,
//         current.pose.orientation.z,
//         current.pose.orientation.w
//     );
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     RCLCPP_INFO(logger, "Current pose:");
//     RCLCPP_INFO(logger, "  x: %.4f  y: %.4f  z: %.4f", x, y, z);
//     RCLCPP_INFO(logger, "  roll: %.4f  pitch: %.4f  yaw: %.4f", roll, pitch, yaw);
//     RCLCPP_INFO(logger, "  roll: %.1f°  pitch: %.1f°  yaw: %.1f°",
//         roll  * 180.0 / M_PI,
//         pitch * 180.0 / M_PI,
//         yaw   * 180.0 / M_PI);
// }

// // ---------------------------------------------------------------------------
// // Main
// // ---------------------------------------------------------------------------
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared(
//         "move_to_position",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     auto spinner = std::thread([&executor]() { executor.spin(); });

//     auto logger = node->get_logger();

//     moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
//     move_group.setMaxVelocityScalingFactor(0.3);
//     move_group.setMaxAccelerationScalingFactor(0.3);
//     move_group.setPlanningTime(15.0);

//     // Setup collision scene
//     moveit::planning_interface::PlanningSceneInterface scene;
//     setupScene(scene);
//     rclcpp::sleep_for(std::chrono::milliseconds(500));

//     // Method 1 — apply joint constraints globally before any planning
//     //applyJointConstraints(move_group);

//     // -----------------------------------------------------------------------
//     // Define your list of target positions here
//     //                       x     y     z     roll  pitch      yaw   name
//     // -----------------------------------------------------------------------

//     printCurrentPose(move_group, logger);

//     std::vector<TargetPose> targets = {
//         {0.3,  0.25,  0.3,  M_PI,  0.0,  0.0,  "position_1"},
//         {0.3,  0.25,  0.2,  M_PI,  0.0,  0.0,  "position_2"},
//         {0.3,  0.25,  0.3,  M_PI,  0.0,  0.0,  "position_3"},
//         {-0.3,  0.25,  0.3,  M_PI,  0.0,  0.0,  "position_4"},
//     };

//     // Execute the sequence
//     moveSequence(move_group, targets, logger);

//     // Clear constraints when fully done
//     move_group.clearPathConstraints();

//     rclcpp::shutdown();
//     spinner.join();
//     return 0;
// }


