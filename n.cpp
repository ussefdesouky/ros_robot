  void FourWheelSteeringController::updateOdometry(const ros::Time& time)
  {
    // COMPUTE AND PUBLISH ODOMETRY
    const double fl_speed = front_wheel_joints_[0].getVelocity();
    const double fr_speed = front_wheel_joints_[1].getVelocity();
    const double rl_speed = rear_wheel_joints_[0].getVelocity();
    const double rr_speed = rear_wheel_joints_[1].getVelocity();
    if (std::isnan(fl_speed) || std::isnan(fr_speed)
        || std::isnan(rl_speed) || std::isnan(rr_speed))
      return;

    const double fl_steering = front_steering_joints_[0].getPosition();
    const double fr_steering = front_steering_joints_[1].getPosition();
    const double rl_steering = rear_steering_joints_[0].getPosition();
    const double rr_steering = rear_steering_joints_[1].getPosition();

    if (std::isnan(fl_steering) || std::isnan(fr_steering)
        || std::isnan(rl_steering) || std::isnan(rr_steering))
      return;

    double front_steering_pos = 0.0;
    if(fabs(fl_steering) > 0.001 || fabs(fr_steering) > 0.001)
    {
      front_steering_pos = atan(2*tan(fl_steering)*tan(fr_steering)/(tan(fl_steering) + tan(fr_steering)));
    }
    
    double rear_steering_pos = 0.0;
    if(fabs(rl_steering) > 0.001 || fabs(rr_steering) > 0.001)
    {
      rear_steering_pos = atan(2*tan(rl_steering)*tan(rr_steering)/(tan(rl_steering) + tan(rr_steering)));
    }

    ROS_DEBUG_STREAM_THROTTLE(1, "rl_steering "<<rl_steering<<" rr_steering "<<rr_steering<<" rear_steering_pos "<<rear_steering_pos);
    // Estimate linear and angular velocity using joint information
    odometry_.update(fl_speed, fr_speed, rl_speed, rr_speed,
                     front_steering_pos, rear_steering_pos, time);

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }
      if (odom_4ws_pub_->trylock())
      {
        odom_4ws_pub_->msg_.header.stamp = time;
        odom_4ws_pub_->msg_.data.speed = odometry_.getLinear();
        odom_4ws_pub_->msg_.data.acceleration = odometry_.getLinearAcceleration();
        odom_4ws_pub_->msg_.data.jerk = odometry_.getLinearJerk();
        odom_4ws_pub_->msg_.data.front_steering_angle = front_steering_pos;
        odom_4ws_pub_->msg_.data.front_steering_angle_velocity = odometry_.getFrontSteerVel();
        odom_4ws_pub_->msg_.data.rear_steering_angle = rear_steering_pos;
        odom_4ws_pub_->msg_.data.rear_steering_angle_velocity = odometry_.getRearSteerVel();
        odom_4ws_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }
  }

  void FourWheelSteeringController::updateCommand(const ros::Time& time, const ros::Duration& period)
  {
    // Retreive current velocity command and time step:
    Command* cmd;
    CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());
    Command4ws curr_cmd_4ws = *(command_four_wheel_steering_.readFromRT());

    if(curr_cmd_4ws.stamp >= curr_cmd_twist.stamp)
    {
      cmd = &curr_cmd_4ws;
      enable_twist_cmd_ = false;
    }
    else
    {
      cmd = &curr_cmd_twist;
      enable_twist_cmd_ = true;
    }

    const double dt = (time - cmd->stamp).toSec();
    // Brake if cmd_vel has timeout://
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd_twist.lin_x = 0.0;
      curr_cmd_twist.lin_y = 0.0;
      curr_cmd_twist.ang = 0.0;
      curr_cmd_4ws.lin = 0.0;
      curr_cmd_4ws.front_steering = 0.0;
      curr_cmd_4ws.rear_steering = 0.0;
    }

    const double cmd_dt(period.toSec());

    const double angular_speed = odometry_.getAngular();
    const double steering_track = track_-2*wheel_steering_y_offset_;

    ROS_DEBUG_STREAM("angular_speed "<<angular_speed<< " wheel_radius_ "<<wheel_radius_);
    double vel_left_front = 0, vel_right_front = 0;
    double vel_left_rear = 0, vel_right_rear = 0;
    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;

    if(enable_twist_cmd_ == true)
    {
      // Limit velocities and accelerations:
      limiter_lin_.limit(curr_cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
      limiter_ang_.limit(curr_cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
      last1_cmd_ = last0_cmd_;
      last0_cmd_ = curr_cmd_twist;

      // Compute wheels velocities
      if(abs(curr_cmd_twist.lin_x) > 0.001 && fabs(curr_cmd_twist.lin_y) < 0.05)
      {
          const double vel_steering_offset = (curr_cmd_twist.ang*wheel_steering_y_offset_)/wheel_radius_;
          const double sign = copysign(1.0, curr_cmd_twist.lin_x);
          vel_left_front  = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                            (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                          - vel_steering_offset;
          vel_right_front = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                            (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                          + vel_steering_offset;
          vel_left_rear = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                          (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                        - vel_steering_offset;
          vel_right_rear = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                           (wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_
                         + vel_steering_offset;

          if(fabs(2.0*curr_cmd_twist.lin_x) > fabs(curr_cmd_twist.ang*steering_track) && fabs(curr_cmd_twist.lin_y) < 0.001) 
          {
            front_left_steering = atan(curr_cmd_twist.ang*wheel_base_ /
                                    (2.0*curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track));
            front_right_steering = atan(curr_cmd_twist.ang*wheel_base_ /
                                     (2.0*curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track));
             rear_left_steering = -front_left_steering;
             rear_right_steering = -front_right_steering;
          }
          else if(fabs(curr_cmd_twist.lin_x) > 0.001 && fabs(curr_cmd_twist.lin_y) < 0.001)
          {
            front_left_steering = copysign(M_PI_2, curr_cmd_twist.ang);
            front_right_steering = copysign(M_PI_2, curr_cmd_twist.ang);
            rear_left_steering = -front_left_steering;
            rear_right_steering = -front_right_steering;
          }
      }
      else if(abs(curr_cmd_twist.ang) > 0.001 && abs(curr_cmd_twist.lin_x) < 0.001 && abs(curr_cmd_twist.lin_y) < 0.001) //pivot turn
      {
            double omega_const = 6; //to make robot rotates a little bit slower

            front_right_steering = atan((wheel_base_ /2)/(0 + steering_track/2));
            rear_right_steering = atan(-(wheel_base_ /2)/(0 + steering_track/2));
            front_left_steering = atan((wheel_base_ /2)/(0 - steering_track/2));
            rear_left_steering = atan(-(wheel_base_ /2)/(0 - steering_track/2));

            vel_left_front = -curr_cmd_twist.ang / (omega_const*wheel_radius_);
            vel_right_front = curr_cmd_twist.ang / (omega_const*wheel_radius_);
            vel_left_rear = vel_left_front ;
            vel_right_rear = vel_right_front;
      }

    }
    else
    {
      // Limit velocities and accelerations:
      limiter_lin_.limit(curr_cmd_4ws.lin, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
      last1_cmd_ = last0_cmd_;
      last0_cmd_.lin_x = curr_cmd_4ws.lin;
      curr_cmd_4ws.front_steering = clamp(curr_cmd_4ws.front_steering, -M_PI_2, M_PI_2);
      curr_cmd_4ws.rear_steering = clamp(curr_cmd_4ws.rear_steering, -M_PI_2, M_PI_2);

      // Compute steering angles
      const double tan_front_steering = tan(curr_cmd_4ws.front_steering);
      const double tan_rear_steering  = tan(curr_cmd_4ws.rear_steering);

      const double steering_diff =  steering_track*(tan_front_steering - tan_rear_steering)/2.0;
      if(fabs(wheel_base_ - fabs(steering_diff)) > 0.001)
      {
        front_left_steering = atan(wheel_base_*tan_front_steering/(wheel_base_-steering_diff));
        front_right_steering = atan(wheel_base_*tan_front_steering/(wheel_base_+steering_diff));
        rear_left_steering = atan(wheel_base_*tan_rear_steering/(wheel_base_-steering_diff));
        rear_right_steering = atan(wheel_base_*tan_rear_steering/(wheel_base_+steering_diff));
      }

      // Compute wheels velocities:
      if(fabs(curr_cmd_4ws.lin) > 0.001)
      {
        //Virutal front and rear wheelbase
        // distance between the projection of the CIR on the wheelbase and the front axle
        double l_front = 0;
        if(fabs(tan(front_left_steering) - tan(front_right_steering)) > 0.01)
        {
          l_front = tan(front_right_steering) * tan(front_left_steering) * steering_track
              / (tan(front_left_steering) - tan(front_right_steering));
        }
        // distance between the projection of the CIR on the wheelbase and the rear axle
        double l_rear = 0;
        if(fabs(tan(rear_left_steering) - tan(rear_right_steering)) > 0.01)
        {
          l_rear = tan(rear_right_steering) * tan(rear_left_steering) * steering_track
              / (tan(rear_left_steering) - tan(rear_right_steering));
        }

        const double angular_speed_cmd = curr_cmd_4ws.lin * (tan_front_steering-tan_rear_steering)/wheel_base_;
        const double vel_steering_offset = (angular_speed_cmd*wheel_steering_y_offset_)/wheel_radius_;
        const double sign = copysign(1.0, curr_cmd_4ws.lin);

        vel_left_front  = sign * std::hypot((curr_cmd_4ws.lin - angular_speed_cmd*steering_track/2),
                                            (l_front*angular_speed_cmd))/wheel_radius_
                          - vel_steering_offset;
        vel_right_front = sign * std::hypot((curr_cmd_4ws.lin + angular_speed_cmd*steering_track/2),
                                            (l_front*angular_speed_cmd))/wheel_radius_
                          + vel_steering_offset;
        vel_left_rear = sign * std::hypot((curr_cmd_4ws.lin - angular_speed_cmd*steering_track/2),
                                          (l_rear*angular_speed_cmd))/wheel_radius_
                        - vel_steering_offset;
        vel_right_rear = sign * std::hypot((curr_cmd_4ws.lin + angular_speed_cmd*steering_track/2),
                                           (l_rear*angular_speed_cmd))/wheel_radius_
                         + vel_steering_offset;
      }
    }

    ROS_DEBUG_STREAM_THROTTLE(1, "vel_left_rear "<<vel_left_rear<<" front_right_steering "<<front_right_steering);
    // Set wheels velocities:
    if(front_wheel_joints_.size() == 2 && rear_wheel_joints_.size() == 2)
    {
      front_wheel_joints_[0].setCommand(vel_left_front);
      front_wheel_joints_[1].setCommand(vel_right_front);
      rear_wheel_joints_[0].setCommand(vel_left_rear);
      rear_wheel_joints_[1].setCommand(vel_right_rear);
    }

    /// TODO check limits to not apply the same steering on right and left when saturated !
    if(front_steering_joints_.size() == 2 && rear_steering_joints_.size() == 2)
    {
      front_steering_joints_[0].setCommand(front_left_steering);
      front_steering_joints_[1].setCommand(front_right_steering);
      rear_steering_joints_[0].setCommand(rear_left_steering);
      rear_steering_joints_[1].setCommand(rear_right_steering);
    }
  }

  void FourWheelSteeringController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < front_wheel_joints_.size(); ++i)
    {
      front_wheel_joints_[i].setCommand(vel);
      rear_wheel_joints_[i].setCommand(vel);
    }

    const double pos = 0.0;
    for (size_t i = 0; i < front_steering_joints_.size(); ++i)
    {
      front_steering_joints_[i].setCommand(pos);
      rear_steering_joints_[i].setCommand(pos);
    }
  }

  void FourWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      if(std::isnan(command.angular.z) || std::isnan(command.linear.x))
      {
        ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
        return;
      }
      command_struct_twist_.ang   = command.angular.z;
      command_struct_twist_.lin_x   = command.linear.x;
      command_struct_twist_.lin_y   = command.linear.y;
      command_struct_twist_.stamp = ros::Time::now();
      command_twist_.writeFromNonRT (command_struct_twist_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_twist_.ang << ", "
                             << "Lin x: " << command_struct_twist_.lin_x << ", "
                             << "Lin y: " << command_struct_twist_.lin_y << ", "
                             << "Stamp: " << command_struct_twist_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  void FourWheelSteeringController::cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteering& command)
  {
    if (isRunning())
    {
      if(std::isnan(command.front_steering_angle) || std::isnan(command.rear_steering_angle)
         || std::isnan(command.speed))
      {
        ROS_WARN("Received NaN in four_wheel_steering_msgs::FourWheelSteering. Ignoring command.");
        return;
      }
      command_struct_four_wheel_steering_.front_steering   = command.front_steering_angle;
      command_struct_four_wheel_steering_.rear_steering   = command.rear_steering_angle;
      command_struct_four_wheel_steering_.lin   = command.speed;
      command_struct_four_wheel_steering_.stamp = ros::Time::now();
      command_four_wheel_steering_.writeFromNonRT (command_struct_four_wheel_steering_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Steering front : "   << command_struct_four_wheel_steering_.front_steering << ", "
                             << "Steering rear : "   << command_struct_four_wheel_steering_.rear_steering << ", "
                             << "Lin: "   << command_struct_four_wheel_steering_.lin << ", "
                             << "Stamp: " << command_struct_four_wheel_steering_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  bool FourWheelSteeringController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
          //ROS_INFO_STREAM("wheel name "<<i<<" " << wheel_names[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }
      return true;
  }
