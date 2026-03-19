#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const std::string PLANNING_GROUP = "ur_manipulator";

bool moveToPose(
    moveit::planning_interface::MoveGroupInterface & move_group,
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target_pose.orientation = tf2::toMsg(q);

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) return false;

    return (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
}

void setupScene(moveit::planning_interface::PlanningSceneInterface & scene)
{
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // -----------------------------------------------------------------------
    // Ground plane — sits at z=0, thick enough the planner respects it
    // -----------------------------------------------------------------------
    {
        moveit_msgs::msg::CollisionObject ground;
        ground.header.frame_id = "base_link";
        ground.id = "ground";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.0;   // 2m wide
        primitive.dimensions[primitive.BOX_Y] = 2.0;   // 2m deep
        primitive.dimensions[primitive.BOX_Z] = 0.01;  // 1cm thin slab

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = -0.005;  // half thickness below z=0

        ground.primitives.push_back(primitive);
        ground.primitive_poses.push_back(pose);
        ground.operation = ground.ADD;
        objects.push_back(ground);
    }

    // -----------------------------------------------------------------------
    // Table surface — add if your robot is mounted on a table
    // -----------------------------------------------------------------------
    {
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "base_link";
        table.id = "table";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.2;   // table length
        primitive.dimensions[primitive.BOX_Y] = 0.8;   // table width
        primitive.dimensions[primitive.BOX_Z] = 0.05;  // table top thickness

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x =  0.3;    // in front of robot
        pose.position.y =  0.0;
        pose.position.z = -0.025;  // sits on the ground

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(pose);
        table.operation = table.ADD;
        objects.push_back(table);
    }

    // Apply all objects at once
    scene.applyCollisionObjects(objects);
    RCLCPP_INFO(rclcpp::get_logger("move_to_position"), "Scene objects added ✓");
}

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
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(10.0);

    // -----------------------------------------------------------------------
    // Setup scene BEFORE planning
    // -----------------------------------------------------------------------
    moveit::planning_interface::PlanningSceneInterface scene;
    setupScene(scene);  // <-- add this

    // Small delay to let the scene update propagate to move_group
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ... rest of your code
    bool success = moveToPose(move_group, 0.4, 0.2, 0.4, 0.0, M_PI/2.0, 0.0);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}




// int main(int argc, char * argv[])
// {
//   // ---------------------------------------------------------------------------
//   // 1. Init ROS2
//   // ---------------------------------------------------------------------------
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared(
//     "move_to_position",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Spin node in background so MoveGroupInterface can receive callbacks
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   auto logger = node->get_logger();

//   // ---------------------------------------------------------------------------
//   // 2. Connect to MoveGroupInterface (talks to running move_group node)
//   // ---------------------------------------------------------------------------
//   RCLCPP_INFO(logger, "Connecting to move_group...");
//   moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

//   move_group.setMaxVelocityScalingFactor(0.3);
//   move_group.setMaxAccelerationScalingFactor(0.3);
//   move_group.setPlanningTime(10.0);
//   move_group.setNumPlanningAttempts(10);

//   RCLCPP_INFO(logger, "Connected! Planning frame: %s", move_group.getPlanningFrame().c_str());
//   RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // ---------------------------------------------------------------------------
//   // 3. Option A — Joint space goal
//   // ---------------------------------------------------------------------------
//   // Uncomment this block to use joint angles instead of Cartesian pose

//   // std::vector<double> joint_goal = {
//   //   0.0,                     // shoulder_pan
//   //   -M_PI / 2.0,             // shoulder_lift  (-90 deg)
//   //   M_PI / 2.0,              // elbow          ( 90 deg)
//   //   -M_PI / 2.0,             // wrist_1        (-90 deg)
//   //   -M_PI / 2.0,             // wrist_2        (-90 deg)
//   //   0.0                      // wrist_3
//   // };
//   // move_group.setJointValueTarget(joint_goal);

//   // ---------------------------------------------------------------------------
//   // 4. Option B — Cartesian (XYZ + RPY) goal  [ACTIVE]
//   // ---------------------------------------------------------------------------
//   // Set your target pose here — position in metres, angles in radians
//   // All values are relative to the robot base_link frame

// //   geometry_msgs::msg::Pose target_pose;

// //   // Position
// //   target_pose.position.x = 0.3;   // metres forward
// //   target_pose.position.y = 0.2;   // metres left
// //   target_pose.position.z = 0.4;   // metres up

// //   // Orientation — convert RPY to quaternion
// //   tf2::Quaternion q;
// //   q.setRPY(
// //     0.0,              // roll
// //     M_PI / 2.0,       // pitch — tool pointing down
// //     0.0               // yaw
// //   );
// //   target_pose.orientation = tf2::toMsg(q);

// //   move_group.setPoseTarget(target_pose);


// //   // ---------------------------------------------------------------------------
// //   // 5. Plan
// //   // ---------------------------------------------------------------------------
// //   RCLCPP_INFO(logger, "Planning...");
// //   moveit::planning_interface::MoveGroupInterface::Plan plan;
// //   bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

// //   if (!success) {
// //     RCLCPP_ERROR(logger, "Planning FAILED");
// //     rclcpp::shutdown();
// //     spinner.join();
// //     return 1;
// //   }

// //   RCLCPP_INFO(logger, "Plan found! Executing...");

// //   // ---------------------------------------------------------------------------
// //   // 6. Execute
// //   // ---------------------------------------------------------------------------
// //   auto result = move_group.execute(plan);

// //   if (result == moveit::core::MoveItErrorCode::SUCCESS) {
// //     RCLCPP_INFO(logger, "Motion complete ✓");
// //   } else {
// //     RCLCPP_ERROR(logger, "Execution FAILED — error code: %d", result.val);
// //   }

//   bool success = moveToPose(move_group, 0.3, 0.2, 0.4, 0.0, M_PI/2.0, 0.0);

//   // ---------------------------------------------------------------------------
//   // 7. Shutdown
//   // ---------------------------------------------------------------------------
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }