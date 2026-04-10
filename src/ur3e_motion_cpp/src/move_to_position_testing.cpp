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

// bool moveToPose(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     double x, double y, double z,
//     double roll, double pitch, double yaw)
// {
//     geometry_msgs::msg::Pose target_pose;
//     target_pose.position.x = x;
//     target_pose.position.y = y;
//     target_pose.position.z = z;

//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw);
//     target_pose.orientation = tf2::toMsg(q);

//     move_group.setPoseTarget(target_pose);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) return false;

//     return (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
// }

// bool moveToPoseCartesian(
//     moveit::planning_interface::MoveGroupInterface & move_group,
//     double x, double y, double z,
//     double roll, double pitch, double yaw,
//     double eef_step = 0.01,       // step size for Cartesian path (meters)
//     double jump_threshold = 0.0   // prevent sudden joint jumps
// )
// {
//     // Get current pose as starting waypoint
//     geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

//     // Create target pose
//     geometry_msgs::msg::Pose target_pose;
//     target_pose.position.x = x;
//     target_pose.position.y = y;
//     target_pose.position.z = z;

//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw);
//     q.normalize();  // normalize to avoid quaternion issues
//     target_pose.orientation = tf2::toMsg(q);

//     // Waypoints: start -> target
//     std::vector<geometry_msgs::msg::Pose> waypoints;
//     waypoints.push_back(target_pose);

//     moveit_msgs::msg::RobotTrajectory trajectory;
//     double fraction = move_group.computeCartesianPath(
//         waypoints,
//         eef_step,
//         jump_threshold,
//         trajectory,
//         /* avoid_collisions = */ true
//     );

//     if (fraction < 0.99)
//     {
//         RCLCPP_WARN(rclcpp::get_logger("move_to_position"),
//             "Cartesian path not fully planned (%.2f%% achieved)", fraction*100.0);
//         return false;
//     }

//     // Execute the trajectory
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     plan.trajectory_ = trajectory;

//     bool success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     return success;
// }

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
//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(0.1);
//     move_group.setPlanningTime(10.0);

//     // Setup collision scene
//     moveit::planning_interface::PlanningSceneInterface scene;
//     setupScene(scene);
//     rclcpp::sleep_for(std::chrono::milliseconds(500));

//     printCurrentPose(move_group, logger);

//     // std::vector<TargetPose> targets = {
//     //     {0.4,  0.1,  0.3,  M_PI,  0.0,  0.0,  "position_1"},
//     //     {0.3,  0.2,  0.2,  M_PI,  0.0,  0.0,  "position_2"},
//     //     {0.3, -0.2,  0.2,  M_PI,  0.0,  0.0,  "position_3"},
//     //     {0.4,  0.0,  0.3,  M_PI,  0.0,  0.0,  "home"},
//     // };

//     // Execute the sequence
//     //moveSequence(move_group, targets, logger);

//     moveToPoseCartesian(move_group, 0.4,  0.1,  0.3,  M_PI,  0.0,  0.0);
//     // Clear constraints when fully done
//     move_group.clearPathConstraints();

//     rclcpp::shutdown();
//     spinner.join();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Matrix3x3.h>

static const std::string PLANNING_GROUP = "ur_manipulator";

// ---------------------------------------------------------------------------
// Structure to hold a target pose cleanly
// ---------------------------------------------------------------------------
struct TargetPose {
    double x, y, z;
    double roll, pitch, yaw;
    std::string name;           // label for logging
    bool use_cartesian = true;  // true = Cartesian path, false = joint-space plan
};

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
// Setup collision scene
// ---------------------------------------------------------------------------
void setupScene(moveit::planning_interface::PlanningSceneInterface & scene)
{
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // Ground plane
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

    // Table surface
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
// Print the current end-effector pose to the console
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
// Joint-space move — used as fallback when Cartesian planning fails
// ---------------------------------------------------------------------------
bool moveToPoseJoint(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const TargetPose & target,
    const rclcpp::Logger & logger)
{
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
        RCLCPP_ERROR(logger, "  [joint] Planning FAILED for '%s'", target.name.c_str());
        return false;
    }

    bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!executed) {
        RCLCPP_ERROR(logger, "  [joint] Execution FAILED for '%s'", target.name.c_str());
    }
    return executed;
}

// ---------------------------------------------------------------------------
// Cartesian move — straight-line end-effector path
// Falls back to joint-space planning if the Cartesian path is < 99% complete
// ---------------------------------------------------------------------------
bool moveToPoseCartesian(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const TargetPose & target,
    const rclcpp::Logger & logger,
    double eef_step       = 0.01,   // metres per interpolation step
    double jump_threshold = 0.0)    // 0 = disable jump detection
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target.x;
    target_pose.position.y = target.y;
    target_pose.position.z = target.z;

    tf2::Quaternion q;
    q.setRPY(target.roll, target.pitch, target.yaw);
    q.normalize();
    target_pose.orientation = tf2::toMsg(q);

    std::vector<geometry_msgs::msg::Pose> waypoints = { target_pose };

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory,
        /* avoid_collisions = */ true
    );

    if (fraction < 0.99) {
        RCLCPP_WARN(logger,
            "  [cartesian] Only %.1f%% of path planned for '%s' — falling back to joint-space",
            fraction * 100.0, target.name.c_str());
        return moveToPoseJoint(move_group, target, logger);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    bool executed = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!executed) {
        RCLCPP_ERROR(logger, "  [cartesian] Execution FAILED for '%s'", target.name.c_str());
    }
    return executed;
}

// ---------------------------------------------------------------------------
// Move to a single pose — dispatches to Cartesian or joint-space
// ---------------------------------------------------------------------------
bool moveToPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const TargetPose & target,
    const rclcpp::Logger & logger)
{
    RCLCPP_INFO(logger, "Moving to '%s' [%s] — x:%.2f y:%.2f z:%.2f  r:%.2f p:%.2f y:%.2f",
        target.name.c_str(),
        target.use_cartesian ? "cartesian" : "joint",
        target.x, target.y, target.z,
        target.roll, target.pitch, target.yaw);

    if (target.use_cartesian) {
        return moveToPoseCartesian(move_group, target, logger);
    } else {
        return moveToPoseJoint(move_group, target, logger);
    }
}

// ---------------------------------------------------------------------------
// Execute a sequence of poses, aborting on first failure
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

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto logger = node->get_logger();

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setPlanningTime(10.0);

    // Setup collision scene
    moveit::planning_interface::PlanningSceneInterface scene;
    setupScene(scene);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Optionally apply joint constraints globally before any planning
    // applyJointConstraints(move_group);

    printCurrentPose(move_group, logger);

    // -----------------------------------------------------------------------
    // Define target sequence
    // Each pose: { x, y, z, roll, pitch, yaw, "name", use_cartesian }
    // Set use_cartesian = false to force joint-space planning for a step
    // -----------------------------------------------------------------------
    std::vector<TargetPose> targets = {
        {0.4,  0.1,  0.3,  M_PI,  0.0,  0.0,  "position_1",  true },
        {0.3,  0.2,  0.2,  M_PI,  0.0,  0.0,  "position_2",  true },
        {0.3, -0.2,  0.2,  M_PI,  0.0,  0.0,  "position_3",  true },
        {0.4,  0.0,  0.3,  M_PI,  0.0,  0.0,  "home",        true },
    };

    moveSequence(move_group, targets, logger);

    move_group.clearPathConstraints();

    rclcpp::shutdown();
    spinner.join();
    return 0;
}