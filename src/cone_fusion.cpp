#include <cone_fusion/cone_fusion.hpp>

ConeFusion::ConeFusion() : rclcpp::Node("cone_fusion_node") {
  /* Load node parameters */
  this->loadParameters();

  /* Create publisher */
  this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(this->output_odom_topic, 1);
  this->conesPositionsMarkerPub = this->create_publisher<visualization_msgs::msg::Marker>(this->mapped_cones_topic, 1);

  /* Create subscriptions */
  this->cones_sub = this->create_subscription<visualization_msgs::msg::Marker>(this->cones_topic, 100,std::bind(&ConeFusion::conesCallback, this, std::placeholders::_1));
  this->fast_limo_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(this->input_odom_topic, 1,std::bind(&ConeFusion::fastLimoDataCallback, this, std::placeholders::_1));
  this->race_status_sub = this->create_subscription<mmr_base::msg::RaceStatus>(this->race_status_topic, 1, std::bind(&ConeFusion::raceStatusCallback, this, std::placeholders::_1));

  /* Initialize the transform broadcaster */
  this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  /* Initialize vehicle pose */
  // 0 for now, this can change if a relocalisations is implemented using fast limo pose as global reference
  this->act_position << 0.0, 0.0, 0.0;
  this->act_orientation.set__w(1.0);
  this->act_orientation.set__x(0.0);
  this->act_orientation.set__y(0.0);
  this->act_orientation.set__z(0.0);

  /* Create EKF SLAM filter object */
  this->ekf_odom = std::make_shared<EKFOdom>(this->proc_noise, this->meas_noise, this->min_new_cone_distance);

  /* Init mapped cones markers */
  this->initConesMarker(this->conesMarker);
  this->initConesMarker(this->correctedConesMarker);

  /* Init race status */
  this->race_status = mmr_base::msg::RaceStatus();
  this->race_status.current_lap = 0;
}

void ConeFusion::loadParameters() {
  std::vector<double> tmp_proc_noise(2), tmp_meas_noise(3);
  declare_parameter("is_colorblind", true);

  declare_parameter("generic.cones_topic", "/clusters");
  declare_parameter("generic.cones_frame_id", "hesai_lidar");

  declare_parameter("generic.mapped_cones_topic", "/slam/cones_positions");
  declare_parameter("generic.mapped_cones_frame_id", "track");
  
  declare_parameter("generic.imu_topic", "/imu/data");
  
  declare_parameter("generic.input_odom_topic", "/fast_limo/state");
  declare_parameter("generic.input_odom_frame_id", "track");

  declare_parameter("generic.output_odom_topic", "/Odometry");
  declare_parameter("generic.output_odom_frame_id", "track");
  declare_parameter("generic.output_odom_child_frame_id", "imu_link");

  declare_parameter("generic.gps_speed_topic", "/speed/gps");
  declare_parameter("generic.gps_data_topic", "/gps/data");
  declare_parameter("generic.race_status_topic", "/planning/race_status");
  
  declare_parameter("generic.enable_logging", false);
  declare_parameter("generic.cone_time_seen_th", 2);
  declare_parameter("generic.is_skidpad_mission", false);
  declare_parameter("generic.cones_pub_for_debug", false);

  /* Declare Sensor Noise parameters */
  declare_parameter<std::vector<double>>("noises.proc_noise", std::vector<double>{0.0, 0.0});
  declare_parameter<std::vector<double>>("noises.meas_noise", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter("noises.min_new_cone_distance", 2.0);

  get_parameter("is_colorblind", this->is_colorblind);

  get_parameter("generic.cones_topic", this->cones_topic);
  get_parameter("generic.cones_frame_id", this->cones_frame_id);
  
  get_parameter("generic.mapped_cones_topic", this->mapped_cones_topic);
  get_parameter("generic.mapped_cones_frame_id", this->mapped_cones_frame_id);
  
  get_parameter("generic.imu_topic", this->imu_topic);
  
  get_parameter("generic.input_odom_topic", this->input_odom_topic);
  get_parameter("generic.input_odom_frame_id", this->input_odom_frame_id);
  
  get_parameter("generic.output_odom_topic", this->output_odom_topic);
  get_parameter("generic.output_odom_frame_id", this->output_odom_frame_id);
  get_parameter("generic.output_odom_child_frame_id", this->output_odom_child_frame_id);

  get_parameter("generic.gps_speed_topic", this->gps_speed_topic);
  get_parameter("generic.gps_data_topic", this->gps_data_topic);
  get_parameter("generic.race_status_topic", this->race_status_topic);
  get_parameter("generic.enable_logging", this->enable_logging);
  get_parameter("generic.is_skidpad_mission", this->is_skidpad_mission);
  get_parameter("generic.cones_pub_for_debug", this->cones_pub_for_debug);

  RCLCPP_INFO(this->get_logger(), "IS_SKIDPAD: %u", this->is_skidpad_mission);

  /* Get Sensor Noise parameters */
  get_parameter("noises.proc_noise", tmp_proc_noise);
  get_parameter("noises.meas_noise", tmp_meas_noise);
  get_parameter("noises.min_new_cone_distance", this->min_new_cone_distance);
  get_parameter("generic.cone_time_seen_th", this->cone_time_seen_th);


  /* Copy noise parameters */
  for (size_t i = 0; i < 3; i++) {
    if (i != 2) {
      this->proc_noise(i) = (float)tmp_proc_noise[i];
    }
    this->meas_noise(i) = (float)tmp_meas_noise[i];
  }
}

void ConeFusion::initConesMarker(visualization_msgs::msg::Marker &cones) {
  cones.header.frame_id = this->mapped_cones_frame_id;
  cones.ns = "ConesAbsolutePos";
  cones.id = 0;
  cones.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  cones.action = visualization_msgs::msg::Marker::ADD;
  cones.scale.x = 0.35;
  cones.scale.y = 0.35;
  cones.scale.z = 0.35;
  cones.pose.position.x = 0.0;
  cones.pose.position.y = 0.0;
  cones.pose.position.z = 0.0;
  // Initialize with identity quaternion (no rotation)
  cones.pose.orientation.w = 1.0;
  cones.pose.orientation.x = 0.0;
  cones.pose.orientation.y = 0.0;
  cones.pose.orientation.z = 0.0;
}

/* Callbacks */

void ConeFusion::conesCallback(
    const visualization_msgs::msg::Marker::SharedPtr cones_data) {
  /* If corrected cons are created -> Skip callback */
  // if(this->corrected_cones_created)
  // {
  //     return;
  // }
  /* By refusing to updates the cones (the center line from the local planner 
    won't be updated anyway) the ekf should work as an achor for possible fast limo drift */
  size_t detected_cones = cones_data->points.size();

  Vector3f *z = (Vector3f *)malloc(detected_cones * sizeof(Vector3f));
  for (size_t i = 0; i < detected_cones; i++) {
    z[i](0) = sqrt(pow(cones_data->points[i].x, 2) +
                   pow(cones_data->points[i].y, 2)); // + coneRadius;
    z[i](1) = atan2(cones_data->points[i].y, cones_data->points[i].x);

    z[i](2) = this->is_colorblind ? yellowCone : this->getConeColor(cones_data->colors[i]);
    /* Normalize angle */
    z[i](1) = this->ekf_odom->normalizeAngle(z[i](1));
  }

  rclcpp::Time start, end;
  start = this->now();

  this->ekf_odom->correct(z, detected_cones);

  end = this->now();
  rclcpp::Duration exe_time = end - start;
  
  if (this->enable_logging)
    RCLCPP_INFO(this->get_logger(), "CORRECT Exe time (ms): %lf",
                exe_time.nanoseconds() * 1e-6);

  free(z);

  /* Publish cones */
  if (!this->corrected_cones_created || this->cones_pub_for_debug) {
    size_t mapped_cones = this->ekf_odom->getActMappedLandmarks();
    conesMarker.points.reserve(mapped_cones);
    conesMarker.colors.reserve(mapped_cones);

    for (size_t i = 0; i < mapped_cones; i++) {
      /* Get how many times the i-th cone has been seen */
      std::pair<ColorId, uint32_t> cone_info =
          this->ekf_odom->getSignatures()(i).getConeColorAndCount();
      // uint16_t seen_threshold = cone_time_seen_th *
      // (this->race_status.current_lap);

      // RCLCPP_INFO(this->get_logger(), "SEEN TH: %u", seen_threshold);

      /* If less than 5 times, don't map it */
      if (cone_info.second < cone_time_seen_th)
        continue;
      Vector2f cone = this->ekf_odom->getState().segment(3 + (i * 2), 2);
      geometry_msgs::msg::Point p;
      std_msgs::msg::ColorRGBA c;
      this->setConeColor(c, static_cast<uint8_t>(cone_info.first));
      p.x = cone(0);
      p.y = cone(1);
      p.z = 0.0;
      conesMarker.points.push_back(p);
      conesMarker.colors.push_back(c);
    }

    if (this->race_status.current_lap > 1 && !this->corrected_cones_created) {
      this->ekf_odom->setFirstLapCompleted(true);
      RCLCPP_INFO(this->get_logger(), "Creating corrected cones markers with %zu cones", conesMarker.points.size());
      this->corrected_cones_created = true;
      this->corrected_cones_size = mapped_cones;
      this->correctedConesMarker = conesMarker;
    }
  }

  /* Publish vehicle pose and cones position */
  if (!this->corrected_cones_created || this->cones_pub_for_debug) {
    this->pubConesMarkers(conesMarker);
    conesMarker.points.clear();
    conesMarker.colors.clear();
  } else {
    this->pubConesMarkers(correctedConesMarker);
  }
}

// void ConeFusion::imuDataCallback(    // lidar_imu??
//     const sensor_msgs::msg::Imu::SharedPtr imu_data) {
//   // this->ekf_odom->setActAngVel(imu_data->angular_velocity.z);
//   tf2::Quaternion q;
//   q.setX(imu_data->orientation.x);
//   q.setY(imu_data->orientation.y);
//   q.setZ(imu_data->orientation.z);
//   q.setW(imu_data->orientation.w);
//   tf2::Matrix3x3 m(q);
// 
//   double r, p, y;
//   m.getRPY(r, p, y);
// 
//   this->act_yaw = y;
// 
//   q.setRPY(0.0, 0.0, this->act_yaw);
//   this->act_orientation.set__x(q.getX());
//   this->act_orientation.set__y(q.getY());
//   this->act_orientation.set__z(q.getZ());
//   this->act_orientation.set__w(q.getW());
// }

void ConeFusion::fastLimoDataCallback(
    const nav_msgs::msg::Odometry::SharedPtr fast_limo_data) {
  // RCLCPP_INFO(this->get_logger(), "FAST LIMO CB");
  tf2::Quaternion q;
  q.setX(fast_limo_data->pose.pose.orientation.x);
  q.setY(fast_limo_data->pose.pose.orientation.y);
  q.setZ(fast_limo_data->pose.pose.orientation.z);
  q.setW(fast_limo_data->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  double r, p, y;
  m.getRPY(r, p, y);

  Vector3f pose;
  pose << fast_limo_data->pose.pose.position.x,
      fast_limo_data->pose.pose.position.y, y;
  Vector3f pose_cov;

  /* Retrieve FAST-LIMO Pose covariance. The covariance of FAST-LIMO is a 6x6. The
     necessary pose covariance is in the main diagonal at indexes 0 (X), 7 (Y),
     35 (Yaw).
  */
  pose_cov(0) = fast_limo_data->pose.covariance.at(0);  /* X covariance */
  pose_cov(1) = fast_limo_data->pose.covariance.at(7);  /* Y covariance */
  pose_cov(2) = fast_limo_data->pose.covariance.at(35); /* Yaw covariance */

  this->ekf_odom->setPose(pose);
  this->ekf_odom->setPoseCovariance(pose_cov);

  /* Publish the EKF state (corrected by the cones), NOT the raw FAST-LIMO pose.
     This is what lets the cone corrections survive on the wire. */
  Vector3f ekf_pose = this->ekf_odom->getState().head(3);

  /* If this is the second lap, retrieve ONLY vehicle position, and publish
   * already mapped cones. */
  if (this->corrected_cones_created || this->is_skidpad_mission) {
    /* Get vehicle pose */
    this->act_position << ekf_pose(0), ekf_pose(1), 0.0;
    this->act_yaw = ekf_pose(2);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, this->act_yaw);
    this->act_orientation.set__x(q.getX());
    this->act_orientation.set__y(q.getY());
    this->act_orientation.set__z(q.getZ());
    this->act_orientation.set__w(q.getW());

    this->updatePose();
  }else{
    /* If this is the first lap, just publish FAST-LIMO pose and cones position if
     * seen. */
    this->act_position << ekf_pose(0), ekf_pose(1), 0.0;
    this->act_yaw = ekf_pose(2);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, this->act_yaw);
    this->act_orientation.set__x(q.getX());
    this->act_orientation.set__y(q.getY());
    this->act_orientation.set__z(q.getZ());
    this->act_orientation.set__w(q.getW());

    this->updatePose();
  }
}

void ConeFusion::pubConesMarkers(visualization_msgs::msg::Marker &cones) {
  cones.header.stamp = this->now();
  this->conesPositionsMarkerPub->publish(cones);
  std::cout << "Published " << cones.points.size() << " cones.\n";
}

void ConeFusion::updatePose() {
  geometry_msgs::msg::TransformStamped t;
  nav_msgs::msg::Odometry _odom;

  /* Update header */
  t.header.stamp = this->now();
  t.header.frame_id = this->output_odom_frame_id;
  t.child_frame_id = this->output_odom_child_frame_id;

  /* Update TF location */
  t.transform.translation.x = act_position[0];
  t.transform.translation.y = act_position[1];
  t.transform.translation.z = 0.0f;

  /* Update TF Orientation */
  t.transform.rotation = this->act_orientation;

  _odom.header.stamp = this->now();
  _odom.header.frame_id = this->output_odom_frame_id;
  _odom.child_frame_id = this->output_odom_child_frame_id;

  /* Update Odom position */
  _odom.pose.pose.position.x = act_position[0];
  _odom.pose.pose.position.y = act_position[1];
  _odom.pose.pose.position.z = 0.0f;

  /* Update Odom orientation */
  _odom.pose.pose.orientation = this->act_orientation;

  /* Update Odom covariance (just the main diagonal)*/
  Vector3f pose_cov = this->ekf_odom->getPoseCovariance();

  _odom.pose.covariance.at(0) = pose_cov(0);  /* X Covariance */
  _odom.pose.covariance.at(7) = pose_cov(1);  /* Y Covariance */
  _odom.pose.covariance.at(35) = pose_cov(2); /* Yaw Covariance */

  this->tf_broadcaster_->sendTransform(t);
  this->odom_pub->publish(_odom);
}

void ConeFusion::raceStatusCallback(
    const mmr_base::msg::RaceStatus::SharedPtr race_status_data) {
  /* Update race status */
  this->race_status = *race_status_data;
}