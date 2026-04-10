
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__init(msg: *mut ExecuteTaskSolution_Goal) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Goal>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Goal>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Goal>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub solution: super::super::msg::rmw::Solution,

}



impl Default for ExecuteTaskSolution_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__init(msg: *mut ExecuteTaskSolution_Result) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Result>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Result>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Result>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_Result {
    /// result of execution
    pub error_code: moveit_msgs::msg::rmw::MoveItErrorCodes,

}



impl Default for ExecuteTaskSolution_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_Result where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__init(msg: *mut ExecuteTaskSolution_Feedback) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Feedback>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Feedback>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_Feedback>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_Feedback {
    /// finished subtrajectory id / number
    pub sub_id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sub_no: u32,

}



impl Default for ExecuteTaskSolution_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__init(msg: *mut ExecuteTaskSolution_FeedbackMessage) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_FeedbackMessage>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_FeedbackMessage>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_FeedbackMessage>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::ExecuteTaskSolution_Feedback,

}



impl Default for ExecuteTaskSolution_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage() }
  }
}




#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__init(msg: *mut ExecuteTaskSolution_SendGoal_Request) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Request>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Request>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Request>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::ExecuteTaskSolution_Goal,

}



impl Default for ExecuteTaskSolution_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__init(msg: *mut ExecuteTaskSolution_SendGoal_Response) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Response>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Response>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_SendGoal_Response>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for ExecuteTaskSolution_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__init(msg: *mut ExecuteTaskSolution_GetResult_Request) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Request>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Request>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Request>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for ExecuteTaskSolution_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__init(msg: *mut ExecuteTaskSolution_GetResult_Response) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Response>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Response>);
    fn moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteTaskSolution_GetResult_Response>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::ExecuteTaskSolution_Result,

}



impl Default for ExecuteTaskSolution_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteTaskSolution_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteTaskSolution_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/action/ExecuteTaskSolution_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response() }
  }
}






#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteTaskSolution_SendGoal;

impl rosidl_runtime_rs::Service for ExecuteTaskSolution_SendGoal {
    type Request = ExecuteTaskSolution_SendGoal_Request;
    type Response = ExecuteTaskSolution_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal() }
    }
}




#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteTaskSolution_GetResult;

impl rosidl_runtime_rs::Service for ExecuteTaskSolution_GetResult {
    type Request = ExecuteTaskSolution_GetResult_Request;
    type Response = ExecuteTaskSolution_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult() }
    }
}


