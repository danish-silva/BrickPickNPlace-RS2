//Including necessary headers for ROS2, MoveIt, and TF2
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>
#include <atomic>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_motion_mtc");
namespace mtc = moveit::task_constructor;

struct PoseData {
  double x, y, z;
  double roll, pitch, yaw;
};

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

  void setPoseData(const PoseData& brick_pose, const PoseData& target_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    brick_pose_ = brick_pose;
    target_pose_ = target_pose;
  }

  rclcpp::Node::SharedPtr node_;  // Public so main() can create subscriptions

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  PoseData brick_pose_;
  PoseData target_pose_;
  std::mutex pose_mutex_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("motion_node", options) }
{
}

//This is the object we are picking and placing
void MTCTaskNode::setupPlanningScene()
{
  PoseData brick;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    brick = brick_pose_;
  }

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.1, 0.05, 0.04 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = brick.x;
  pose.position.y = brick.y;
  pose.position.z = brick.z;
  pose.orientation.w = 1.0;
  object.pose = pose;
  object.operation = object.ADD;   // <-- this was missing

  // ground plane
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = "world";
  ground.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.0;
  primitive.dimensions[primitive.BOX_Y] = 1.0;
  primitive.dimensions[primitive.BOX_Z] = 0.1;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.06;

  ground.primitives.push_back(primitive);
  ground.primitive_poses.push_back(box_pose);
  ground.operation = ground.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  psi.applyCollisionObject(ground);
}

// This function creates and executes the MTC task. It first initializes the task, then plans a solution, and finally executes it. If any of these steps fail, it logs an error message.
void MTCTaskNode::doTask()
{
  RCLCPP_INFO(LOGGER, "Starting doTask");
  task_ = createTask();
  RCLCPP_INFO(LOGGER, "Task created");

  try
  {
    RCLCPP_INFO(LOGGER, "Initializing task");
    task_.init();
    RCLCPP_INFO(LOGGER, "Task initialized successfully");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting task planning for 5 solutions");
  if (!task_.plan(10)) // If we cant come up with 2 versions of the task, we consider it a failure. 2 is the minimum
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Task planning succeeded");

  task_.introspection().publishSolution(*task_.solutions().front());

  RCLCPP_INFO(LOGGER, "Executing task");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Task execution succeeded");

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_onrobot_manipulator";
  const auto& hand_group_name = "ur_onrobot_gripper";
  const auto& hand_frame = "gripper_tcp";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  //Adding the current state of the robot as the first stage of the task.
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.005);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(15.0); // If we cant find a path to the pick pose in 15 seconds, we consider it a failure
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });     
                                          
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.03, 0.20);

      // Approach from above — move downward in world frame
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;   // changed from hand_frame to world
      vec.vector.z = 1.0;             // negative Z = downward
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI);   // only 0° and 180° — gripper aligned with long axis
      stage->setMonitoredStage(current_state_ptr);

      // Top-down grasp: gripper Z axis points downward into the brick
      // Fingers align with the brick's long axis (X axis)
      // This transform rotates the gripper so it approaches from above
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();

      // Offset the grasp point upward so the gripper doesn't collide with the brick surface
      // Adjust this value based on your gripper finger length
      grasp_frame_transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(64);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.03, 0.20); //Finding solutions that lift the object at least 3cm, but no more than 20cm. This is to avoid collisions with the environment and to ensure a successful lift.
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(15.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      PoseData target;
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        target = target_pose_;
      }

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";      // use world frame, not object frame
      target_pose_msg.pose.position.x = target.x;     // place position x
      target_pose_msg.pose.position.y = target.y;     // place position y  
      target_pose_msg.pose.position.z = target.z;     // place position z
      
      tf2::Quaternion q;
      q.setRPY(target.roll, target.pitch, target.yaw);
      target_pose_msg.pose.orientation = tf2::toMsg(q);
      
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.03, 0.20);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

      task.add(std::move(place));
  }

  {
    // auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("camera_home"); //Change this to robot's home configuration
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  
  // -----------------------------------------------------------------------
  // Subscribe to ordered_pose_array topic
  //
  // Message type: std_msgs/Float64MultiArray
  // Format: flat array of 6 values per pose [x, y, z, roll, pitch, yaw]
  //
  // Expected: exactly 2 poses (brick position and target position)
  // data: [brick_x, brick_y, brick_z, brick_roll, brick_pitch, brick_yaw,
  //        target_x, target_y, target_z, target_roll, target_pitch, target_yaw]
  // -----------------------------------------------------------------------
  std::atomic<bool> poses_received{false};
  PoseData brick_pose{};
  PoseData target_pose{};
  
  auto pose_subscription = mtc_task_node->node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "ordered_pose_array",
      10,
      [&brick_pose, &target_pose, &poses_received](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          const size_t POSE_SIZE = 6;
          const size_t EXPECTED_POSES = 2;  // brick and target

          if (msg->data.size() != EXPECTED_POSES * POSE_SIZE) {
              RCLCPP_ERROR(LOGGER,
                  "Expected array size %zu but received %zu — ignoring",
                  EXPECTED_POSES * POSE_SIZE, msg->data.size());
              return;
          }

          // Parse brick position
          brick_pose.x = msg->data[0];
          brick_pose.y = msg->data[1];
          brick_pose.z = msg->data[2];
          brick_pose.roll = msg->data[3];
          brick_pose.pitch = msg->data[4];
          brick_pose.yaw = msg->data[5];

          // Parse target position
          target_pose.x = msg->data[6];
          target_pose.y = msg->data[7];
          target_pose.z = msg->data[8];
          target_pose.roll = msg->data[9];
          target_pose.pitch = msg->data[10];
          target_pose.yaw = msg->data[11];

          RCLCPP_INFO(LOGGER, "Received poses:");
          RCLCPP_INFO(LOGGER, "  Brick: x:%.3f y:%.3f z:%.3f r:%.3f p:%.3f y:%.3f",
              brick_pose.x, brick_pose.y, brick_pose.z,
              brick_pose.roll, brick_pose.pitch, brick_pose.yaw);
          RCLCPP_INFO(LOGGER, "  Target: x:%.3f y:%.3f z:%.3f r:%.3f p:%.3f y:%.3f",
              target_pose.x, target_pose.y, target_pose.z,
              target_pose.roll, target_pose.pitch, target_pose.yaw);

          poses_received = true;
      }
  );

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Wait for pose data to arrive
  RCLCPP_INFO(LOGGER, "Waiting for poses on 'ordered_pose_array'...");
  while (rclcpp::ok() && !poses_received) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  if (!poses_received) {
    RCLCPP_ERROR(LOGGER, "Timeout waiting for pose data");
    rclcpp::shutdown();
    spin_thread->join();
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Received pose data, setting up scene and executing task");
  mtc_task_node->setPoseData(brick_pose, target_pose);
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

// //Including necessary headers for ROS2, MoveIt, and TF2
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/task_constructor/task.h>
// #include <moveit/task_constructor/solvers.h>
// #include <moveit/task_constructor/stages.h>
// #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #endif
// #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
// #include <tf2_eigen/tf2_eigen.hpp>
// #else
// #include <tf2_eigen/tf2_eigen.h>
// #endif

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_motion_mtc");
// namespace mtc = moveit::task_constructor;

// class MTCTaskNode
// {
// public:
//   MTCTaskNode(const rclcpp::NodeOptions& options);

//   rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

//   void doTask();

//   void setupPlanningScene();

// private:
//   // Compose an MTC task from a series of stages.
//   mtc::Task createTask();
//   mtc::Task task_;
//   rclcpp::Node::SharedPtr node_;
// };

// rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
// {
//   return node_->get_node_base_interface();
// }

// MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
//   : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
// {
// }

// //This is the object we are picking and placing
// void MTCTaskNode::setupPlanningScene()
// {
//   moveit_msgs::msg::CollisionObject object;
//   object.id = "object";
//   object.header.frame_id = "world";
//   object.primitives.resize(1);
//   object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
//   object.primitives[0].dimensions = { 0.1, 0.05, 0.04 };

//   geometry_msgs::msg::Pose pose;
//   pose.position.x = 0.3;
//   pose.position.y = 0.3;
//   pose.position.z = 0.02;
//   pose.orientation.w = 1.0;
//   object.pose = pose;
//   object.operation = object.ADD;   // <-- this was missing

//   // ground plane
//   moveit_msgs::msg::CollisionObject ground;
//   ground.header.frame_id = "world";
//   ground.id = "box1";

//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 1.0;
//   primitive.dimensions[primitive.BOX_Y] = 1.0;
//   primitive.dimensions[primitive.BOX_Z] = 0.1;

//   geometry_msgs::msg::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.0;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = -0.06;

//   ground.primitives.push_back(primitive);
//   ground.primitive_poses.push_back(box_pose);
//   ground.operation = ground.ADD;

//   moveit::planning_interface::PlanningSceneInterface psi;
//   psi.applyCollisionObject(object);
//   psi.applyCollisionObject(ground);
// }

// // This function creates and executes the MTC task. It first initializes the task, then plans a solution, and finally executes it. If any of these steps fail, it logs an error message.
// void MTCTaskNode::doTask()
// {
//   task_ = createTask();

//   try
//   {
//     task_.init();
//   }
//   catch (mtc::InitStageException& e)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (!task_.plan(5)) // If we cant come up with 2 versions of the task, we consider it a failure. 2 is the minimum
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//     return;
//   }
//   task_.introspection().publishSolution(*task_.solutions().front());

//   auto result = task_.execute(*task_.solutions().front());
//   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//     return;
//   }

//   return;
// }

// mtc::Task MTCTaskNode::createTask()
// {
//   mtc::Task task;
//   task.stages()->setName("demo task");
//   task.loadRobotModel(node_);

//   const auto& arm_group_name = "ur_onrobot_manipulator";
//   const auto& hand_group_name = "ur_onrobot_gripper";
//   const auto& hand_frame = "gripper_tcp";

//   // Set task properties
//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", hand_group_name);
//   task.setProperty("ik_frame", hand_frame);

// // Disable warnings for this line, as it's a variable that's set but not used in this example
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//   mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
// #pragma GCC diagnostic pop

//   //Adding the current state of the robot as the first stage of the task.
//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(0.5);
//   cartesian_planner->setMaxAccelerationScalingFactor(0.5);
//   cartesian_planner->setStepSize(.01);

//   auto stage_open_hand =
//       std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//   stage_open_hand->setGroup(hand_group_name);
//   stage_open_hand->setGoal("open");
//   task.add(std::move(stage_open_hand));

//   auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
//       "move to pick",
//       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//   stage_move_to_pick->setTimeout(15.0); // If we cant find a path to the pick pose in 15 seconds, we consider it a failure
//   stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
//   task.add(std::move(stage_move_to_pick));

//   mtc::Stage* attach_object_stage =
//     nullptr;  // Forward attach_object_stage to place pose generator

//   {
//     auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
//     task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
//     grasp->properties().configureInitFrom(mtc::Stage::PARENT,
//                                           { "eef", "group", "ik_frame" });     
                                          
//     {
//       auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
//       stage->properties().set("marker_ns", "approach_object");
//       stage->properties().set("link", hand_frame);
//       stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//       stage->setMinMaxDistance(0.03, 0.20);

//       // Approach from above — move downward in world frame
//       geometry_msgs::msg::Vector3Stamped vec;
//       vec.header.frame_id = "world";   // changed from hand_frame to world
//       vec.vector.z = -1.0;             // negative Z = downward
//       stage->setDirection(vec);
//       grasp->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
//       stage->properties().configureInitFrom(mtc::Stage::PARENT);
//       stage->properties().set("marker_ns", "grasp_pose");
//       stage->setPreGraspPose("open");
//       stage->setObject("object");
//       stage->setAngleDelta(M_PI);   // only 0° and 180° — gripper aligned with long axis
//       stage->setMonitoredStage(current_state_ptr);

//       // Top-down grasp: gripper Z axis points downward into the brick
//       // Fingers align with the brick's long axis (X axis)
//       // This transform rotates the gripper so it approaches from above
//       Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
//       Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
//                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY()) *
//                             Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitZ());
//       grasp_frame_transform.linear() = q.matrix();

//       // Offset the grasp point upward so the gripper doesn't collide with the brick surface
//       // Adjust this value based on your gripper finger length
//       grasp_frame_transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

//       auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
//       wrapper->setMaxIKSolutions(8);
//       wrapper->setMinSolutionDistance(1.0);
//       wrapper->setIKFrame(grasp_frame_transform, hand_frame);
//       wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//       wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//       grasp->insert(std::move(wrapper));
//     }

//     {
//       auto stage =
//           std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
//       stage->allowCollisions("object",
//                             task.getRobotModel()
//                                 ->getJointModelGroup(hand_group_name)
//                                 ->getLinkModelNamesWithCollisionGeometry(),
//                             true);
//       grasp->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
//       stage->setGroup(hand_group_name);
//       stage->setGoal("closed");
//       grasp->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
//       stage->attachObject("object", hand_frame);
//       attach_object_stage = stage.get();
//       grasp->insert(std::move(stage));
//     }

//     {
//       auto stage =
//           std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
//       stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//       stage->setMinMaxDistance(0.03, 0.20); //Finding solutions that lift the object at least 3cm, but no more than 20cm. This is to avoid collisions with the environment and to ensure a successful lift.
//       stage->setIKFrame(hand_frame);
//       stage->properties().set("marker_ns", "lift_object");

//       // Set upward direction
//       geometry_msgs::msg::Vector3Stamped vec;
//       vec.header.frame_id = "world";
//       vec.vector.z = 1.0;
//       stage->setDirection(vec);
//       grasp->insert(std::move(stage));
//     }

//     task.add(std::move(grasp));
//   }

//   {
//     auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
//         "move to place",
//         mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//     stage_move_to_place->setTimeout(15.0);
//     stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//     task.add(std::move(stage_move_to_place));
//   }

//   {
//     auto place = std::make_unique<mtc::SerialContainer>("place object");
//     task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
//     place->properties().configureInitFrom(mtc::Stage::PARENT,
//                                           { "eef", "group", "ik_frame" });
    
//     {
//       // Sample place pose
//       auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
//       stage->properties().configureInitFrom(mtc::Stage::PARENT);
//       stage->properties().set("marker_ns", "place_pose");
//       stage->setObject("object");

//       geometry_msgs::msg::PoseStamped target_pose_msg;
//       target_pose_msg.header.frame_id = "world";      // use world frame, not object frame
//       target_pose_msg.pose.position.x = -0.25;          // place position x
//       target_pose_msg.pose.position.y = 0.2;          // place position y  
//       target_pose_msg.pose.position.z = 0.03;         // same height as pick — sitting on surface
//       target_pose_msg.pose.orientation.w = 1.0;
//       stage->setPose(target_pose_msg);
//       stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

//       // Compute IK
//       auto wrapper =
//           std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
//       wrapper->setMaxIKSolutions(8);
//       wrapper->setMinSolutionDistance(1.0);
//       wrapper->setIKFrame("object");
//       wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//       wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//       place->insert(std::move(wrapper));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//       stage->setGroup(hand_group_name);
//       stage->setGoal("open");
//       place->insert(std::move(stage));
//     }

//     {
//       auto stage =
//           std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
//       stage->allowCollisions("object",
//                             task.getRobotModel()
//                                 ->getJointModelGroup(hand_group_name)
//                                 ->getLinkModelNamesWithCollisionGeometry(),
//                             false);
//       place->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
//       stage->detachObject("object", hand_frame);
//       place->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
//       stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//       stage->setMinMaxDistance(0.03, 0.20);
//       stage->setIKFrame(hand_frame);
//       stage->properties().set("marker_ns", "retreat");

//       // Set retreat direction
//       geometry_msgs::msg::Vector3Stamped vec;
//       vec.header.frame_id = "world";
//       vec.vector.z = 1.0;
//       stage->setDirection(vec);
//       place->insert(std::move(stage));
//     }

//       task.add(std::move(place));
//   }

//   {
//     // auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
//     auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
//     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//     stage->setGoal("camera_home"); //Change this to robot's home configuration
//     task.add(std::move(stage));
//   }

//   return task;
// }

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;
//   options.automatically_declare_parameters_from_overrides(true);

//   auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
//   rclcpp::executors::MultiThreadedExecutor executor;

//   auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
//     executor.add_node(mtc_task_node->getNodeBaseInterface());
//     executor.spin();
//     executor.remove_node(mtc_task_node->getNodeBaseInterface());
//   });

//   mtc_task_node->setupPlanningScene();
//   mtc_task_node->doTask();

//   spin_thread->join();
//   rclcpp::shutdown();
//   return 0;
// }