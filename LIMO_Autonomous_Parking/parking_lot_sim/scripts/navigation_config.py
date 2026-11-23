"""Project Nav2 configuration, generated without changing installed ROS files."""

import json
import math


def parameters(project, patrol, robot):
    axle = robot["wheelbase_m"]/2
    cx = robot["footprint_center_offset_m"][0]+axle
    length = robot["footprint_size_m"][0]
    width = robot["steered_envelope_width_m"]
    footprint = json.dumps([[cx-length/2, -width/2], [cx+length/2, -width/2],
                            [cx+length/2, width/2], [cx-length/2, width/2]])
    scan = {"topic": "/scan", "data_type": "LaserScan", "marking": True,
            "clearing": True, "max_obstacle_height": 3.0,
            "obstacle_min_range": 0.12, "obstacle_max_range": 20.0,
            "raytrace_min_range": 0.12, "raytrace_max_range": 30.0,
            "inf_is_valid": True}
    obstacle = {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True,
                "observation_sources": "scan", "scan": scan}
    # Smac's potential-field shortcut requires inflation to cover the complete
    # rear-axle-referenced footprint, including costmap padding.
    circumscribed = max(math.hypot(x, y) for x in (cx-length/2-0.12, cx+length/2+0.12)
                        for y in (-width/2-0.12, width/2+0.12))
    inflate = {"plugin": "nav2_costmap_2d::InflationLayer",
               "inflation_radius": math.ceil((circumscribed+0.15)*10)/10,
               # A steep decay can quantize the cost at the circumscribed
               # radius to zero even when inflation_radius is large enough.
               "cost_scaling_factor": 1.0}
    common = {"use_sim_time": True, "robot_base_frame": "rear_axle",
              "transform_tolerance": 0.5, "resolution": 0.05,
              "footprint": footprint, "footprint_padding": 0.12,
              "always_send_full_costmap": True,
              "obstacle_layer": obstacle, "inflation_layer": inflate}
    local = dict(common, global_frame="odom", rolling_window=True, width=14, height=14,
                 update_frequency=8.0, publish_frequency=4.0,
                 plugins=["obstacle_layer", "inflation_layer"])
    global_map = dict(common, global_frame="map", track_unknown_space=True,
                      update_frequency=2.0, publish_frequency=1.0,
                      plugins=["static_layer", "obstacle_layer", "inflation_layer"],
                      static_layer={"plugin": "nav2_costmap_2d::StaticLayer",
                                    "map_topic": "/navigation_map", "map_subscribe_transient_local": True,
                                    "subscribe_to_updates": False})
    controller = {
        "use_sim_time": True, "controller_frequency": 15.0, "odom_topic": "/odom",
        "enable_stamped_cmd_vel": False, "publish_zero_velocity": True,
        "min_x_velocity_threshold": 0.001, "min_y_velocity_threshold": 0.001,
        "min_theta_velocity_threshold": 0.001, "failure_tolerance": 0.5,
        "controller_plugins": ["FollowPath"], "progress_checker_plugins": ["progress_checker"],
        "goal_checker_plugins": ["goal_checker"],
        "progress_checker": {"plugin": "nav2_controller::SimpleProgressChecker",
                             "required_movement_radius": 0.2, "movement_time_allowance": 30.0},
        "goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker", "stateful": True,
                         "xy_goal_tolerance": patrol["goal_position_tolerance_m"],
                         "yaw_goal_tolerance": patrol["goal_heading_tolerance_rad"]},
        "FollowPath": {
            "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            "desired_linear_vel": patrol["cruise_speed_m_s"],
            "lookahead_dist": 2.5, "min_lookahead_dist": 1.5, "max_lookahead_dist": 3.5,
            "lookahead_time": 2.5, "use_velocity_scaled_lookahead_dist": True,
            "use_rotate_to_heading": False, "allow_reversing": False,
            "use_collision_detection": True, "max_allowed_time_to_collision_up_to_carrot": 2.5,
            "use_regulated_linear_velocity_scaling": True,
            "regulated_linear_scaling_min_radius": patrol["corner_radius_m"]
                    *patrol["cruise_speed_m_s"]/patrol["corner_speed_m_s"],
            "regulated_linear_scaling_min_speed": 0.25,
            "use_cost_regulated_linear_velocity_scaling": True,
            "cost_scaling_dist": 1.5, "cost_scaling_gain": 1.0,
            "inflation_cost_scaling_factor": inflate["cost_scaling_factor"],
            "min_approach_linear_velocity": 0.12, "approach_velocity_scaling_dist": 1.5,
            "transform_tolerance": 0.5, "max_robot_pose_search_dist": 10.0,
        },
    }
    planner = {
        "use_sim_time": True, "expected_planner_frequency": 1.0, "planner_plugins": ["GridBased"],
        "GridBased": {"plugin": "nav2_smac_planner::SmacPlannerHybrid", "tolerance": 0.3,
                      "allow_unknown": True, "downsample_costmap": True, "downsampling_factor": 2,
                      "motion_model_for_search": "DUBIN", "angle_quantization_bins": 72,
                      "minimum_turning_radius": patrol["planner_min_turning_radius_m"],
                      "reverse_penalty": 3.0, "non_straight_penalty": 1.2,
                      "cost_penalty": 2.0, "max_planning_time": 2.0,
                      "max_iterations": 1000000, "max_on_approach_iterations": 1000,
                      "analytic_expansion_ratio": 3.5, "analytic_expansion_max_length": 24.0,
                      "lookup_table_size": 30.0, "smooth_path": True,
                      "cache_obstacle_heuristic": False},
    }
    navigator = {
        "use_sim_time": True, "global_frame": "map", "robot_base_frame": "rear_axle",
        "odom_topic": "/odom", "bt_loop_duration": 10, "default_server_timeout": 20,
        "wait_for_service_timeout": 1000, "transform_tolerance": 0.5,
        "navigators": ["navigate_to_pose"],
        "navigate_to_pose": {"plugin": "nav2_bt_navigator::NavigateToPoseNavigator"},
        "default_nav_to_pose_bt_xml": str(project/"config"/"navigate.xml"),
    }
    behavior = {
        "use_sim_time": True, "global_frame": "map", "local_frame": "odom",
        "robot_base_frame": "rear_axle", "cycle_frequency": 10.0,
        "local_costmap_topic": "/local_costmap/costmap_raw",
        "global_costmap_topic": "/global_costmap/costmap_raw",
        "local_footprint_topic": "/local_costmap/published_footprint",
        "global_footprint_topic": "/global_costmap/published_footprint",
        "behavior_plugins": ["wait"], "wait": {"plugin": "nav2_behaviors::Wait"},
        "enable_stamped_cmd_vel": False,
    }
    return {
        "controller_server": {"ros__parameters": controller},
        "planner_server": {"ros__parameters": planner},
        "bt_navigator": {"ros__parameters": navigator},
        "behavior_server": {"ros__parameters": behavior},
        "local_costmap": {"local_costmap": {"ros__parameters": local}},
        "global_costmap": {"global_costmap": {"ros__parameters": global_map}},
    }
