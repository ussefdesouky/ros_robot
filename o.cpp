  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    last_update_timestamp_ = time;
  }

  bool Odometry::update(const double &fl_speed, const double &fr_speed,
                        const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, const ros::Time &time)
  {
    const double front_tmp = cos(front_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double front_left_tmp = front_tmp/sqrt(1-steering_track_*front_tmp*cos(front_steering)
                                               +pow(steering_track_*front_tmp/2,2));
    const double front_right_tmp = front_tmp/sqrt(1+steering_track_*front_tmp*cos(front_steering)
                                                +pow(steering_track_*front_tmp/2,2));
    const double fl_speed_tmp = fl_speed * (1/(1-wheel_steering_y_offset_*front_left_tmp));
    const double fr_speed_tmp = fr_speed * (1/(1-wheel_steering_y_offset_*front_right_tmp));
    const double front_linear_speed = wheel_radius_ * copysign(1.0, fl_speed_tmp+fr_speed_tmp)*
        sqrt((pow(fl_speed,2)+pow(fr_speed,2))/(2+pow(steering_track_*front_tmp,2)/2.0));

    const double rear_tmp = cos(rear_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double rear_left_tmp = rear_tmp/sqrt(1-steering_track_*rear_tmp*cos(rear_steering)
                                               +pow(steering_track_*rear_tmp/2,2));
    const double rear_right_tmp = rear_tmp/sqrt(1+steering_track_*rear_tmp*cos(rear_steering)
                                                +pow(steering_track_*rear_tmp/2,2));
    const double rl_speed_tmp = rl_speed * (1/(1-wheel_steering_y_offset_*rear_left_tmp));
    const double rr_speed_tmp = rr_speed * (1/(1-wheel_steering_y_offset_*rear_right_tmp));
    const double rear_linear_speed = wheel_radius_ * copysign(1.0, rl_speed_tmp+rr_speed_tmp)*
        sqrt((pow(rl_speed_tmp,2)+pow(rr_speed_tmp,2))/(2+pow(steering_track_*rear_tmp,2)/2.0));

    bool pivot_turn = false;

    if(abs(front_steering) < 1.571 && abs(front_steering) > 1.568 && abs(rear_steering) < 1.571 && abs(rear_steering) > 1.568){
        double R = sqrt(pow(wheel_base_/2,2)+pow(steering_track_/2,2)) + wheel_steering_y_offset_;
        angular_ = -wheel_radius_*fl_speed/R; //fl_speed:rad/s
        linear_x_ = 0.00001; //idialy this should be 0, but it seems that amcl don't calculate pose with 0
        linear_y_ = 0.00001;
        linear_ = 0.00001;
        pivot_turn = true;
    }
    else{
        angular_ = (front_linear_speed*front_tmp + rear_linear_speed*rear_tmp)/2.0;
        linear_x_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/2.0;
        linear_y_ = (front_linear_speed*sin(front_steering) - wheel_base_*angular_/2.0
                + rear_linear_speed*sin(rear_steering) + wheel_base_*angular_/2.0)/2.0;
        linear_ =  copysign(1.0, rear_linear_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));
        pivot_turn = false;
    }

    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    last_update_timestamp_ = time;
    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt, pivot_turn);

    linear_accel_acc_((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_((linear_accel_prev_ - bacc::rolling_mean(linear_accel_acc_))/dt);
    linear_accel_prev_ = bacc::rolling_mean(linear_accel_acc_);
    front_steer_vel_acc_((front_steer_vel_prev_ - front_steering)/dt);
    front_steer_vel_prev_ = front_steering;
    rear_steer_vel_acc_((rear_steer_vel_prev_ - rear_steering)/dt);
    rear_steer_vel_prev_ = rear_steering;
    return true;
  }
