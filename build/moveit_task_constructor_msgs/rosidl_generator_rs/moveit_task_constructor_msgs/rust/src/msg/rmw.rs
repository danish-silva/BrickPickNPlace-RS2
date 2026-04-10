#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__Property() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__Property__init(msg: *mut Property) -> bool;
    fn moveit_task_constructor_msgs__msg__Property__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Property>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__Property__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Property>);
    fn moveit_task_constructor_msgs__msg__Property__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Property>, out_seq: *mut rosidl_runtime_rs::Sequence<Property>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__Property
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Property {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub value: rosidl_runtime_rs::String,

}



impl Default for Property {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__Property__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__Property__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Property {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__Property__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__Property__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__Property__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Property {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Property where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/Property";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__Property() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__Solution() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__Solution__init(msg: *mut Solution) -> bool;
    fn moveit_task_constructor_msgs__msg__Solution__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Solution>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__Solution__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Solution>);
    fn moveit_task_constructor_msgs__msg__Solution__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Solution>, out_seq: *mut rosidl_runtime_rs::Sequence<Solution>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__Solution
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// id of generating task

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Solution {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,

    /// planning scene of start state
    pub start_scene: moveit_msgs::msg::rmw::PlanningScene,

    /// set of all sub solutions involved
    pub sub_solution: rosidl_runtime_rs::Sequence<super::super::msg::rmw::SubSolution>,

    /// (ordered) sequence of actual trajectories
    pub sub_trajectory: rosidl_runtime_rs::Sequence<super::super::msg::rmw::SubTrajectory>,

}



impl Default for Solution {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__Solution__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__Solution__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Solution {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__Solution__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__Solution__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__Solution__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Solution {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Solution where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/Solution";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__Solution() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__SolutionInfo() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__SolutionInfo__init(msg: *mut SolutionInfo) -> bool;
    fn moveit_task_constructor_msgs__msg__SolutionInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SolutionInfo>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__SolutionInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SolutionInfo>);
    fn moveit_task_constructor_msgs__msg__SolutionInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SolutionInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<SolutionInfo>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__SolutionInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// unique id within task

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolutionInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,

    /// associated cost
    pub cost: f32,

    /// associated comment, usually providing failure hint
    pub comment: rosidl_runtime_rs::String,

    /// id of stage that created this trajectory
    pub stage_id: u32,

    /// name of the planner that created this solution
    pub planner_id: rosidl_runtime_rs::String,

    /// markers, e.g. providing additional hints or illustrating failure
    pub markers: rosidl_runtime_rs::Sequence<visualization_msgs::msg::rmw::Marker>,

}



impl Default for SolutionInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__SolutionInfo__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__SolutionInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SolutionInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SolutionInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SolutionInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SolutionInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SolutionInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SolutionInfo where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/SolutionInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__SolutionInfo() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__StageDescription() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__StageDescription__init(msg: *mut StageDescription) -> bool;
    fn moveit_task_constructor_msgs__msg__StageDescription__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StageDescription>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__StageDescription__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StageDescription>);
    fn moveit_task_constructor_msgs__msg__StageDescription__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StageDescription>, out_seq: *mut rosidl_runtime_rs::Sequence<StageDescription>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__StageDescription
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// static description of a stage

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StageDescription {
    /// unique id within task
    pub id: u32,

    /// parent id, parent_id == id means root
    pub parent_id: u32,

    /// name of this stage
    pub name: rosidl_runtime_rs::String,

    /// flags: interface, ...
    pub flags: u32,

    /// properties
    pub properties: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Property>,

}



impl Default for StageDescription {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__StageDescription__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__StageDescription__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StageDescription {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__StageDescription__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__StageDescription__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__StageDescription__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StageDescription {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StageDescription where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/StageDescription";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__StageDescription() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__StageStatistics() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__StageStatistics__init(msg: *mut StageStatistics) -> bool;
    fn moveit_task_constructor_msgs__msg__StageStatistics__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StageStatistics>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__StageStatistics__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StageStatistics>);
    fn moveit_task_constructor_msgs__msg__StageStatistics__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StageStatistics>, out_seq: *mut rosidl_runtime_rs::Sequence<StageStatistics>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__StageStatistics
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// dynamically changing information for a stage

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StageStatistics {
    /// unique id within task
    pub id: u32,

    /// successful solution IDs of this stage, sorted by increasing cost
    pub solved: rosidl_runtime_rs::Sequence<u32>,

    /// (optional) failed solution IDs of this stage
    pub failed: rosidl_runtime_rs::Sequence<u32>,

    /// number of failed solutions (if failed is empty)
    pub num_failed: u32,

    /// total computation time in seconds
    pub total_compute_time: f64,

}



impl Default for StageStatistics {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__StageStatistics__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__StageStatistics__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StageStatistics {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__StageStatistics__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__StageStatistics__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__StageStatistics__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StageStatistics {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StageStatistics where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/StageStatistics";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__StageStatistics() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__SubSolution() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__SubSolution__init(msg: *mut SubSolution) -> bool;
    fn moveit_task_constructor_msgs__msg__SubSolution__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SubSolution>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__SubSolution__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SubSolution>);
    fn moveit_task_constructor_msgs__msg__SubSolution__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SubSolution>, out_seq: *mut rosidl_runtime_rs::Sequence<SubSolution>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__SubSolution
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// generic solution information

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SubSolution {

    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::super::msg::rmw::SolutionInfo,

    /// IDs of subsolutions
    pub sub_solution_id: rosidl_runtime_rs::Sequence<u32>,

}



impl Default for SubSolution {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__SubSolution__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__SubSolution__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SubSolution {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SubSolution__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SubSolution__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SubSolution__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SubSolution {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SubSolution where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/SubSolution";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__SubSolution() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__SubTrajectory() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__SubTrajectory__init(msg: *mut SubTrajectory) -> bool;
    fn moveit_task_constructor_msgs__msg__SubTrajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SubTrajectory>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__SubTrajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SubTrajectory>);
    fn moveit_task_constructor_msgs__msg__SubTrajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SubTrajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<SubTrajectory>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__SubTrajectory
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// generic solution information

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SubTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::super::msg::rmw::SolutionInfo,

    /// trajectory execution information, like controller configuration
    pub execution_info: super::super::msg::rmw::TrajectoryExecutionInfo,

    /// trajectory
    pub trajectory: moveit_msgs::msg::rmw::RobotTrajectory,

    /// planning scene of end state as diff w.r.t. start state
    pub scene_diff: moveit_msgs::msg::rmw::PlanningScene,

}



impl Default for SubTrajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__SubTrajectory__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__SubTrajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SubTrajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SubTrajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SubTrajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__SubTrajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SubTrajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SubTrajectory where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/SubTrajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__SubTrajectory() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__TaskDescription() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__TaskDescription__init(msg: *mut TaskDescription) -> bool;
    fn moveit_task_constructor_msgs__msg__TaskDescription__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TaskDescription>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__TaskDescription__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TaskDescription>);
    fn moveit_task_constructor_msgs__msg__TaskDescription__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TaskDescription>, out_seq: *mut rosidl_runtime_rs::Sequence<TaskDescription>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__TaskDescription
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// unique id of this task

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskDescription {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,

    /// list of all stages, including the task stage itself
    pub stages: rosidl_runtime_rs::Sequence<super::super::msg::rmw::StageDescription>,

}



impl Default for TaskDescription {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__TaskDescription__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__TaskDescription__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TaskDescription {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TaskDescription__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TaskDescription__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TaskDescription__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TaskDescription {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TaskDescription where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/TaskDescription";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__TaskDescription() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__TaskStatistics() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__TaskStatistics__init(msg: *mut TaskStatistics) -> bool;
    fn moveit_task_constructor_msgs__msg__TaskStatistics__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TaskStatistics>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__TaskStatistics__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TaskStatistics>);
    fn moveit_task_constructor_msgs__msg__TaskStatistics__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TaskStatistics>, out_seq: *mut rosidl_runtime_rs::Sequence<TaskStatistics>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__TaskStatistics
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// unique id of generating task

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskStatistics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,

    /// list of all stages, including the task stage itself
    pub stages: rosidl_runtime_rs::Sequence<super::super::msg::rmw::StageStatistics>,

}



impl Default for TaskStatistics {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__TaskStatistics__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__TaskStatistics__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TaskStatistics {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TaskStatistics__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TaskStatistics__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TaskStatistics__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TaskStatistics {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TaskStatistics where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/TaskStatistics";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__TaskStatistics() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__init(msg: *mut TrajectoryExecutionInfo) -> bool;
    fn moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrajectoryExecutionInfo>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrajectoryExecutionInfo>);
    fn moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrajectoryExecutionInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<TrajectoryExecutionInfo>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// List of controllers to use when executing the trajectory

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrajectoryExecutionInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub controller_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for TrajectoryExecutionInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrajectoryExecutionInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrajectoryExecutionInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrajectoryExecutionInfo where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/msg/TrajectoryExecutionInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo() }
  }
}


