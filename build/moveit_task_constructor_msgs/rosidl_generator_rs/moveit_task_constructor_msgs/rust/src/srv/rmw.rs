#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__srv__GetSolution_Request() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__srv__GetSolution_Request__init(msg: *mut GetSolution_Request) -> bool;
    fn moveit_task_constructor_msgs__srv__GetSolution_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSolution_Request>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__srv__GetSolution_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSolution_Request>);
    fn moveit_task_constructor_msgs__srv__GetSolution_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSolution_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSolution_Request>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__srv__GetSolution_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSolution_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub solution_id: u32,

}



impl Default for GetSolution_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__srv__GetSolution_Request__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__srv__GetSolution_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSolution_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__srv__GetSolution_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__srv__GetSolution_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__srv__GetSolution_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSolution_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSolution_Request where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/srv/GetSolution_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__srv__GetSolution_Request() }
  }
}


#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__srv__GetSolution_Response() -> *const std::ffi::c_void;
}

#[link(name = "moveit_task_constructor_msgs__rosidl_generator_c")]
extern "C" {
    fn moveit_task_constructor_msgs__srv__GetSolution_Response__init(msg: *mut GetSolution_Response) -> bool;
    fn moveit_task_constructor_msgs__srv__GetSolution_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSolution_Response>, size: usize) -> bool;
    fn moveit_task_constructor_msgs__srv__GetSolution_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSolution_Response>);
    fn moveit_task_constructor_msgs__srv__GetSolution_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSolution_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSolution_Response>) -> bool;
}

// Corresponds to moveit_task_constructor_msgs__srv__GetSolution_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSolution_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub solution: super::super::msg::rmw::Solution,

}



impl Default for GetSolution_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !moveit_task_constructor_msgs__srv__GetSolution_Response__init(&mut msg as *mut _) {
        panic!("Call to moveit_task_constructor_msgs__srv__GetSolution_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSolution_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__srv__GetSolution_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__srv__GetSolution_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { moveit_task_constructor_msgs__srv__GetSolution_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSolution_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSolution_Response where Self: Sized {
  const TYPE_NAME: &'static str = "moveit_task_constructor_msgs/srv/GetSolution_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__moveit_task_constructor_msgs__srv__GetSolution_Response() }
  }
}






#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__moveit_task_constructor_msgs__srv__GetSolution() -> *const std::ffi::c_void;
}

// Corresponds to moveit_task_constructor_msgs__srv__GetSolution
#[allow(missing_docs, non_camel_case_types)]
pub struct GetSolution;

impl rosidl_runtime_rs::Service for GetSolution {
    type Request = GetSolution_Request;
    type Response = GetSolution_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__moveit_task_constructor_msgs__srv__GetSolution() }
    }
}


