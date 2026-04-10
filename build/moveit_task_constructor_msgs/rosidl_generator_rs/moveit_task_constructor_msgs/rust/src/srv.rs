#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to moveit_task_constructor_msgs__srv__GetSolution_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSolution_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub solution_id: u32,

}



impl Default for GetSolution_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSolution_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetSolution_Request {
  type RmwMsg = super::srv::rmw::GetSolution_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        solution_id: msg.solution_id,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      solution_id: msg.solution_id,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      solution_id: msg.solution_id,
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__srv__GetSolution_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSolution_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub solution: super::msg::Solution,

}



impl Default for GetSolution_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSolution_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetSolution_Response {
  type RmwMsg = super::srv::rmw::GetSolution_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        solution: super::msg::Solution::into_rmw_message(std::borrow::Cow::Owned(msg.solution)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        solution: super::msg::Solution::into_rmw_message(std::borrow::Cow::Borrowed(&msg.solution)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      solution: super::msg::Solution::from_rmw_message(msg.solution),
    }
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


