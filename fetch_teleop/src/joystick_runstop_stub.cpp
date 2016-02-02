
// Runstop Teleop
class RunstopTeleop : public TeleopComponent
{
  //typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client_t;

public:
  RunstopTeleop(const std::string& name, ros::NodeHandle& nh) :
    //req_close_(false),
    //req_open_(false)
    disable_breakers_(false)
  {
    ros::NodeHandle pnh(nh, name);

    // Button mapping; require both right trigger buttons
    pnh.param("button_deadman_a", deadman_a_, 11);
    pnh.param("button_deadman_b", deadman_b_, 9);

    arm_breaker_client_ = nh.serviceClient<power_msgs::BreakerCommand>("/arm_breaker");
    base_breaker_client_ = nh.serviceClient<power_msgs::BreakerCommand>("/base_breaker");
    gripper_breaker_client_ = nh.serviceClient<power_msgs::BreakerCommand>("/gripper_breaker");

    //std::string action_name = "gripper_controller/gripper_action";
    //client_.reset(new client_t(action_name, true));
    //if (!client_->waitForServer(ros::Duration(2.0)))
    //{
    //  ROS_ERROR("%s may not be connected.", action_name.c_str());
    //}
  }

  void robotStateCallback(const fetch_driver_msgs::RobotStateConstPtr& msg)
  {
  }

  // This gets called whenever new joy message comes in
  // returns whether lower priority teleop components should be stopped
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    bool deadman_pressed_a = joy->buttons[deadman_a_];
    bool deadman_pressed_b = joy->buttons[deadman_b_];

    if (deadman_pressed_a && deadman_pressed_b)
    {
      arm_breaker_srv_.enable = true;
      base_breaker_srv_.enable = true;
      gripper_breaker_srv_.enable = true;
      //if (joy->buttons[open_button_])
      //  req_open_ = true;
      //else if (joy->buttons[close_button_])
      //  req_close_ = true;
      // disable breakers
      disable_breakers_ = true;
      hw_runstop_toggled_ = false;
    }
    else if (!hw_runstop_toggled)
    {
      // Check for runstop

    }
    // hw_runstop has been toggled; re-enable breakers
    else if (disable_breakers_
    {
      
    }

    return false;
  }

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt)
  {
    if (disable_breakers_ && !soft_runstop_enabled_)
    {
      arm_breaker_srv_.enable = false;
      base_breaker_srv_.enable = false;
      gripper_breaker_srv_.enable = false;
      arm_breaker_client_.call(arm_breaker_srv_);
      base_breaker_client_.call(base_breaker_srv_);
      gripper_breaker_client_.call(gripper_breaker_srv_);
      //client_.call(srv_)
      disable_breakers_ = false;
      soft_runstop_enabled_ = true;
      
    }
    else if (enable_breakers_ && soft_runstop_enabled_)
    {
      arm_breaker_srv_.enable = true;
      base_breaker_srv_.enable = true;
      gripper_breaker_srv_.enable = true;
      enable_breakers_ = false;
      soft_runstop_enabled_ = false;
      //robot_state_sub_.shutdown(); // TODO tradeoff of this vs. quick-exiting callback function?
    }
    else if (soft_runstop_enabled_)
    {
      // hardware runstop is currently triggered
      if (hw_runstopped_)
      {
        hw_runstop_status_ = true;
      }
      // hardware runstop is released and was previous pressed
      else if (!hw_runstopped_ && hw_runstop_status_)
      {
        enable_breakers_ = true;
        hw_runstopped_ = false;
      }
    }
  }

private:
  int deadman_a_, deadman_b_;
  double min_position_, max_position_, max_effort_;
  bool hw_runstop_status_, hw_runstop_toggled_, disable_breakers_, soft_runstop_enabled_;
  ros::Subscriber robot_state_sub_;
  // TODO should the clients be shared_ptrs? examples of both exist in teleop currently
  ros::ServiceClient arm_breaker_client_;
  ros::ServiceClient base_breaker_client_;
  ros::ServiceClient gripper_breaker_client_;
  // TODO (cleanup) are multiple srv objects actually needed? These just store the value of enable...
  // TODO (potential_bug) since the clients' call() method is blocking, is the delay significant
  // between breakers being disabled?
  power_msgs::BreakerCommand arm_breaker_srv_;
  power_msgs::BreakerCommand base_breaker_srv_;
  power_msgs::BreakerCommand gripper_breaker_srv_;
  boost::shared_ptr<client_t> client_;
  
};
