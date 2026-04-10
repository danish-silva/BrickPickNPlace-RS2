
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Goal

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub solution: super::msg::Solution,

}



impl Default for ExecuteTaskSolution_Goal {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_Goal::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_Goal {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_Goal;

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


// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Result

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_Result {
    /// result of execution
    pub error_code: moveit_msgs::msg::MoveItErrorCodes,

}



impl Default for ExecuteTaskSolution_Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_Result::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_Result {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        error_code: moveit_msgs::msg::MoveItErrorCodes::into_rmw_message(std::borrow::Cow::Owned(msg.error_code)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        error_code: moveit_msgs::msg::MoveItErrorCodes::into_rmw_message(std::borrow::Cow::Borrowed(&msg.error_code)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      error_code: moveit_msgs::msg::MoveItErrorCodes::from_rmw_message(msg.error_code),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_Feedback

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_Feedback::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_Feedback {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_Feedback;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        sub_id: msg.sub_id,
        sub_no: msg.sub_no,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      sub_id: msg.sub_id,
      sub_no: msg.sub_no,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      sub_id: msg.sub_id,
      sub_no: msg.sub_no,
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_FeedbackMessage

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::action::ExecuteTaskSolution_Feedback,

}



impl Default for ExecuteTaskSolution_FeedbackMessage {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_FeedbackMessage::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_FeedbackMessage {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_FeedbackMessage;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        feedback: super::action::ExecuteTaskSolution_Feedback::into_rmw_message(std::borrow::Cow::Owned(msg.feedback)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        feedback: super::action::ExecuteTaskSolution_Feedback::into_rmw_message(std::borrow::Cow::Borrowed(&msg.feedback)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      feedback: super::action::ExecuteTaskSolution_Feedback::from_rmw_message(msg.feedback),
    }
  }
}






// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::action::ExecuteTaskSolution_Goal,

}



impl Default for ExecuteTaskSolution_SendGoal_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_SendGoal_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_SendGoal_Request {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_SendGoal_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
        goal: super::action::ExecuteTaskSolution_Goal::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
        goal: super::action::ExecuteTaskSolution_Goal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
      goal: super::action::ExecuteTaskSolution_Goal::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_SendGoal_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for ExecuteTaskSolution_SendGoal_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_SendGoal_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_SendGoal_Response {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_SendGoal_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::UUID,

}



impl Default for ExecuteTaskSolution_GetResult_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_GetResult_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_GetResult_Request {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_GetResult_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Owned(msg.goal_id)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_id: unique_identifier_msgs::msg::UUID::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal_id)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_id: unique_identifier_msgs::msg::UUID::from_rmw_message(msg.goal_id),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution_GetResult_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteTaskSolution_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::action::ExecuteTaskSolution_Result,

}



impl Default for ExecuteTaskSolution_GetResult_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::action::rmw::ExecuteTaskSolution_GetResult_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteTaskSolution_GetResult_Response {
  type RmwMsg = super::action::rmw::ExecuteTaskSolution_GetResult_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
        result: super::action::ExecuteTaskSolution_Result::into_rmw_message(std::borrow::Cow::Owned(msg.result)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
        result: super::action::ExecuteTaskSolution_Result::into_rmw_message(std::borrow::Cow::Borrowed(&msg.result)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
      result: super::action::ExecuteTaskSolution_Result::from_rmw_message(msg.result),
    }
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






#[link(name = "moveit_task_constructor_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution() -> *const std::ffi::c_void;
}

// Corresponds to moveit_task_constructor_msgs__action__ExecuteTaskSolution
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteTaskSolution;

impl rosidl_runtime_rs::Action for ExecuteTaskSolution {
  // --- Associated types for client library users ---
  /// The goal message defined in the action definition.
  type Goal = ExecuteTaskSolution_Goal;

  /// The result message defined in the action definition.
  type Result = ExecuteTaskSolution_Result;

  /// The feedback message defined in the action definition.
  type Feedback = ExecuteTaskSolution_Feedback;

  // --- Associated types for client library implementation ---
  /// The feedback message with generic fields which wraps the feedback message.
  type FeedbackMessage = super::action::ExecuteTaskSolution_FeedbackMessage;

  /// The send_goal service using a wrapped version of the goal message as a request.
  type SendGoalService = super::action::ExecuteTaskSolution_SendGoal;

  /// The generic service to cancel a goal.
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

  /// The get_result service using a wrapped version of the result message as a response.
  type GetResultService = super::action::ExecuteTaskSolution_GetResult;

  // --- Methods for client library implementation ---
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__moveit_task_constructor_msgs__action__ExecuteTaskSolution() }
  }

  fn create_goal_request(
    goal_id: &[u8; 16],
    goal: super::action::rmw::ExecuteTaskSolution_Goal,
  ) -> super::action::rmw::ExecuteTaskSolution_SendGoal_Request {
   super::action::rmw::ExecuteTaskSolution_SendGoal_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
      goal,
    }
  }

  fn split_goal_request(
    request: super::action::rmw::ExecuteTaskSolution_SendGoal_Request,
  ) -> (
    [u8; 16],
   super::action::rmw::ExecuteTaskSolution_Goal,
  ) {
    (request.goal_id.uuid, request.goal)
  }

  fn create_goal_response(
    accepted: bool,
    stamp: (i32, u32),
  ) -> super::action::rmw::ExecuteTaskSolution_SendGoal_Response {
   super::action::rmw::ExecuteTaskSolution_SendGoal_Response {
      accepted,
      stamp: builtin_interfaces::msg::rmw::Time {
        sec: stamp.0,
        nanosec: stamp.1,
      },
    }
  }

  fn get_goal_response_accepted(
    response: &super::action::rmw::ExecuteTaskSolution_SendGoal_Response,
  ) -> bool {
    response.accepted
  }

  fn get_goal_response_stamp(
    response: &super::action::rmw::ExecuteTaskSolution_SendGoal_Response,
  ) -> (i32, u32) {
    (response.stamp.sec, response.stamp.nanosec)
  }

  fn create_feedback_message(
    goal_id: &[u8; 16],
    feedback: super::action::rmw::ExecuteTaskSolution_Feedback,
  ) -> super::action::rmw::ExecuteTaskSolution_FeedbackMessage {
    let mut message = super::action::rmw::ExecuteTaskSolution_FeedbackMessage::default();
    message.goal_id.uuid = *goal_id;
    message.feedback = feedback;
    message
  }

  fn split_feedback_message(
    feedback: super::action::rmw::ExecuteTaskSolution_FeedbackMessage,
  ) -> (
    [u8; 16],
   super::action::rmw::ExecuteTaskSolution_Feedback,
  ) {
    (feedback.goal_id.uuid, feedback.feedback)
  }

  fn create_result_request(
    goal_id: &[u8; 16],
  ) -> super::action::rmw::ExecuteTaskSolution_GetResult_Request {
   super::action::rmw::ExecuteTaskSolution_GetResult_Request {
      goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
    }
  }

  fn get_result_request_uuid(
    request: &super::action::rmw::ExecuteTaskSolution_GetResult_Request,
  ) -> &[u8; 16] {
    &request.goal_id.uuid
  }

  fn create_result_response(
    status: i8,
    result: super::action::rmw::ExecuteTaskSolution_Result,
  ) -> super::action::rmw::ExecuteTaskSolution_GetResult_Response {
   super::action::rmw::ExecuteTaskSolution_GetResult_Response {
      status,
      result,
    }
  }

  fn split_result_response(
    response: super::action::rmw::ExecuteTaskSolution_GetResult_Response
  ) -> (
    i8,
   super::action::rmw::ExecuteTaskSolution_Result,
  ) {
    (response.status, response.result)
  }
}


