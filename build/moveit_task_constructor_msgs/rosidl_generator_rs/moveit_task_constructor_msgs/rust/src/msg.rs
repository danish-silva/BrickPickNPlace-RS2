#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to moveit_task_constructor_msgs__msg__Property

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Property {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub description: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub value: std::string::String,

}



impl Default for Property {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Property::default())
  }
}

impl rosidl_runtime_rs::Message for Property {
  type RmwMsg = super::msg::rmw::Property;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        description: msg.description.as_str().into(),
        type_: msg.type_.as_str().into(),
        value: msg.value.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        description: msg.description.as_str().into(),
        type_: msg.type_.as_str().into(),
        value: msg.value.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
      description: msg.description.to_string(),
      type_: msg.type_.to_string(),
      value: msg.value.to_string(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__Solution
/// id of generating task

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Solution {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,

    /// planning scene of start state
    pub start_scene: moveit_msgs::msg::PlanningScene,

    /// set of all sub solutions involved
    pub sub_solution: Vec<super::msg::SubSolution>,

    /// (ordered) sequence of actual trajectories
    pub sub_trajectory: Vec<super::msg::SubTrajectory>,

}



impl Default for Solution {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Solution::default())
  }
}

impl rosidl_runtime_rs::Message for Solution {
  type RmwMsg = super::msg::rmw::Solution;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        start_scene: moveit_msgs::msg::PlanningScene::into_rmw_message(std::borrow::Cow::Owned(msg.start_scene)).into_owned(),
        sub_solution: msg.sub_solution
          .into_iter()
          .map(|elem| super::msg::SubSolution::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        sub_trajectory: msg.sub_trajectory
          .into_iter()
          .map(|elem| super::msg::SubTrajectory::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        start_scene: moveit_msgs::msg::PlanningScene::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start_scene)).into_owned(),
        sub_solution: msg.sub_solution
          .iter()
          .map(|elem| super::msg::SubSolution::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        sub_trajectory: msg.sub_trajectory
          .iter()
          .map(|elem| super::msg::SubTrajectory::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_id: msg.task_id.to_string(),
      start_scene: moveit_msgs::msg::PlanningScene::from_rmw_message(msg.start_scene),
      sub_solution: msg.sub_solution
          .into_iter()
          .map(super::msg::SubSolution::from_rmw_message)
          .collect(),
      sub_trajectory: msg.sub_trajectory
          .into_iter()
          .map(super::msg::SubTrajectory::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__SolutionInfo
/// unique id within task

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolutionInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,

    /// associated cost
    pub cost: f32,

    /// associated comment, usually providing failure hint
    pub comment: std::string::String,

    /// id of stage that created this trajectory
    pub stage_id: u32,

    /// name of the planner that created this solution
    pub planner_id: std::string::String,

    /// markers, e.g. providing additional hints or illustrating failure
    pub markers: Vec<visualization_msgs::msg::Marker>,

}



impl Default for SolutionInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SolutionInfo::default())
  }
}

impl rosidl_runtime_rs::Message for SolutionInfo {
  type RmwMsg = super::msg::rmw::SolutionInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        cost: msg.cost,
        comment: msg.comment.as_str().into(),
        stage_id: msg.stage_id,
        planner_id: msg.planner_id.as_str().into(),
        markers: msg.markers
          .into_iter()
          .map(|elem| visualization_msgs::msg::Marker::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      cost: msg.cost,
        comment: msg.comment.as_str().into(),
      stage_id: msg.stage_id,
        planner_id: msg.planner_id.as_str().into(),
        markers: msg.markers
          .iter()
          .map(|elem| visualization_msgs::msg::Marker::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      cost: msg.cost,
      comment: msg.comment.to_string(),
      stage_id: msg.stage_id,
      planner_id: msg.planner_id.to_string(),
      markers: msg.markers
          .into_iter()
          .map(visualization_msgs::msg::Marker::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__StageDescription
/// static description of a stage

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StageDescription {
    /// unique id within task
    pub id: u32,

    /// parent id, parent_id == id means root
    pub parent_id: u32,

    /// name of this stage
    pub name: std::string::String,

    /// flags: interface, ...
    pub flags: u32,

    /// properties
    pub properties: Vec<super::msg::Property>,

}



impl Default for StageDescription {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StageDescription::default())
  }
}

impl rosidl_runtime_rs::Message for StageDescription {
  type RmwMsg = super::msg::rmw::StageDescription;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        parent_id: msg.parent_id,
        name: msg.name.as_str().into(),
        flags: msg.flags,
        properties: msg.properties
          .into_iter()
          .map(|elem| super::msg::Property::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      parent_id: msg.parent_id,
        name: msg.name.as_str().into(),
      flags: msg.flags,
        properties: msg.properties
          .iter()
          .map(|elem| super::msg::Property::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      parent_id: msg.parent_id,
      name: msg.name.to_string(),
      flags: msg.flags,
      properties: msg.properties
          .into_iter()
          .map(super::msg::Property::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__StageStatistics
/// dynamically changing information for a stage

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StageStatistics {
    /// unique id within task
    pub id: u32,

    /// successful solution IDs of this stage, sorted by increasing cost
    pub solved: Vec<u32>,

    /// (optional) failed solution IDs of this stage
    pub failed: Vec<u32>,

    /// number of failed solutions (if failed is empty)
    pub num_failed: u32,

    /// total computation time in seconds
    pub total_compute_time: f64,

}



impl Default for StageStatistics {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StageStatistics::default())
  }
}

impl rosidl_runtime_rs::Message for StageStatistics {
  type RmwMsg = super::msg::rmw::StageStatistics;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        solved: msg.solved.into(),
        failed: msg.failed.into(),
        num_failed: msg.num_failed,
        total_compute_time: msg.total_compute_time,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        solved: msg.solved.as_slice().into(),
        failed: msg.failed.as_slice().into(),
      num_failed: msg.num_failed,
      total_compute_time: msg.total_compute_time,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      solved: msg.solved
          .into_iter()
          .collect(),
      failed: msg.failed
          .into_iter()
          .collect(),
      num_failed: msg.num_failed,
      total_compute_time: msg.total_compute_time,
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__SubSolution
/// generic solution information

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SubSolution {

    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::msg::SolutionInfo,

    /// IDs of subsolutions
    pub sub_solution_id: Vec<u32>,

}



impl Default for SubSolution {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SubSolution::default())
  }
}

impl rosidl_runtime_rs::Message for SubSolution {
  type RmwMsg = super::msg::rmw::SubSolution;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: super::msg::SolutionInfo::into_rmw_message(std::borrow::Cow::Owned(msg.info)).into_owned(),
        sub_solution_id: msg.sub_solution_id.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: super::msg::SolutionInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.info)).into_owned(),
        sub_solution_id: msg.sub_solution_id.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      info: super::msg::SolutionInfo::from_rmw_message(msg.info),
      sub_solution_id: msg.sub_solution_id
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__SubTrajectory
/// generic solution information

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SubTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::msg::SolutionInfo,

    /// trajectory execution information, like controller configuration
    pub execution_info: super::msg::TrajectoryExecutionInfo,

    /// trajectory
    pub trajectory: moveit_msgs::msg::RobotTrajectory,

    /// planning scene of end state as diff w.r.t. start state
    pub scene_diff: moveit_msgs::msg::PlanningScene,

}



impl Default for SubTrajectory {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SubTrajectory::default())
  }
}

impl rosidl_runtime_rs::Message for SubTrajectory {
  type RmwMsg = super::msg::rmw::SubTrajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: super::msg::SolutionInfo::into_rmw_message(std::borrow::Cow::Owned(msg.info)).into_owned(),
        execution_info: super::msg::TrajectoryExecutionInfo::into_rmw_message(std::borrow::Cow::Owned(msg.execution_info)).into_owned(),
        trajectory: moveit_msgs::msg::RobotTrajectory::into_rmw_message(std::borrow::Cow::Owned(msg.trajectory)).into_owned(),
        scene_diff: moveit_msgs::msg::PlanningScene::into_rmw_message(std::borrow::Cow::Owned(msg.scene_diff)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        info: super::msg::SolutionInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.info)).into_owned(),
        execution_info: super::msg::TrajectoryExecutionInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.execution_info)).into_owned(),
        trajectory: moveit_msgs::msg::RobotTrajectory::into_rmw_message(std::borrow::Cow::Borrowed(&msg.trajectory)).into_owned(),
        scene_diff: moveit_msgs::msg::PlanningScene::into_rmw_message(std::borrow::Cow::Borrowed(&msg.scene_diff)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      info: super::msg::SolutionInfo::from_rmw_message(msg.info),
      execution_info: super::msg::TrajectoryExecutionInfo::from_rmw_message(msg.execution_info),
      trajectory: moveit_msgs::msg::RobotTrajectory::from_rmw_message(msg.trajectory),
      scene_diff: moveit_msgs::msg::PlanningScene::from_rmw_message(msg.scene_diff),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__TaskDescription
/// unique id of this task

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskDescription {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,

    /// list of all stages, including the task stage itself
    pub stages: Vec<super::msg::StageDescription>,

}



impl Default for TaskDescription {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TaskDescription::default())
  }
}

impl rosidl_runtime_rs::Message for TaskDescription {
  type RmwMsg = super::msg::rmw::TaskDescription;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        stages: msg.stages
          .into_iter()
          .map(|elem| super::msg::StageDescription::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        stages: msg.stages
          .iter()
          .map(|elem| super::msg::StageDescription::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_id: msg.task_id.to_string(),
      stages: msg.stages
          .into_iter()
          .map(super::msg::StageDescription::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__TaskStatistics
/// unique id of generating task

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TaskStatistics {

    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,

    /// list of all stages, including the task stage itself
    pub stages: Vec<super::msg::StageStatistics>,

}



impl Default for TaskStatistics {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TaskStatistics::default())
  }
}

impl rosidl_runtime_rs::Message for TaskStatistics {
  type RmwMsg = super::msg::rmw::TaskStatistics;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        stages: msg.stages
          .into_iter()
          .map(|elem| super::msg::StageStatistics::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        stages: msg.stages
          .iter()
          .map(|elem| super::msg::StageStatistics::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_id: msg.task_id.to_string(),
      stages: msg.stages
          .into_iter()
          .map(super::msg::StageStatistics::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to moveit_task_constructor_msgs__msg__TrajectoryExecutionInfo
/// List of controllers to use when executing the trajectory

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrajectoryExecutionInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub controller_names: Vec<std::string::String>,

}



impl Default for TrajectoryExecutionInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TrajectoryExecutionInfo::default())
  }
}

impl rosidl_runtime_rs::Message for TrajectoryExecutionInfo {
  type RmwMsg = super::msg::rmw::TrajectoryExecutionInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        controller_names: msg.controller_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        controller_names: msg.controller_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      controller_names: msg.controller_names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


