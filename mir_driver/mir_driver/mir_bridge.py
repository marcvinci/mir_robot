#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, QoSDurabilityPolicy

import time
import copy
import sys
from collections.abc import Iterable
from collections import OrderedDict
import threading
from typing import Any

import mir_driver.rosbridge
from rclpy_message_converter import message_converter

from nav2_msgs.action import NavigateToPose
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from mir_actions.action import MirMoveBase
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import mir_msgs.msg
import visualization_msgs.msg
from action_msgs.msg import GoalStatusArray, GoalStatus

tf_prefix = ''


class TopicConfig(object):
    def __init__(self, topic: str, topic_type: Any, topic_renamed: str | None = None, latch: bool = False, dict_filter: Any | None = None,
                 qos_profile: Any | None = None) -> None:
        self.topic = topic
        if (topic_renamed):
            self.topic_ros2_name = topic_renamed
        else:
            self.topic_ros2_name = topic
        self.topic_type = topic_type
        self.latch = latch
        self.dict_filter = dict_filter
        if qos_profile is not None:
            self.qos_profile = qos_profile
        else:
            self.qos_profile = qos_profile_system_default
        if latch:
            self.qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL


class ActionConfig(object):
    def __init__(self, mir_action_topic: str, action_type: Any, action_topic: str | None = None,
                 goal_dict_filter: Any | None = None, cancel_dict_filter: Any | None = None, 
                 feedback_dict_filter: Any | None = None, status_dict_filter: Any | None = None,
                 result_dict_filter: Any | None = None, qos_profile: Any | None = None) -> None:
        self.mir_action_topic = mir_action_topic
        if action_topic is not None:
            self.action_topic = action_topic
        else:
            self.action_topic = mir_action_topic
        self.action_type = action_type
        self.goal_dict_filter = goal_dict_filter
        self.cancel_dict_filter = cancel_dict_filter
        self.feedback_dict_filter = feedback_dict_filter
        self.status_dict_filter = status_dict_filter
        self.result_dict_filter = result_dict_filter

        if qos_profile is not None:
            self.qos_profile = qos_profile
        else:
            self.qos_profile = qos_profile_system_default


def _odom_dict_filter(msg_dict: dict,  to_ros2: bool) -> dict:
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['child_frame_id'] = tf_prefix + \
        filtered_msg_dict['child_frame_id'].strip('/')
    return filtered_msg_dict


def _tf_dict_filter(msg_dict: dict, to_ros2: bool) -> dict:
    filtered_msg_dict = copy.deepcopy(msg_dict)

    for transform in filtered_msg_dict['transforms']:
        transform['child_frame_id'] = tf_prefix + transform['child_frame_id'].strip('/')
        transform['header'] = _convert_ros_header(transform['header'], to_ros2)
    return filtered_msg_dict


def _map_dict_filter(msg_dict: dict, to_ros2: bool) -> dict:
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(
        filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['info'] = _map_meta_data_dict_filter(filtered_msg_dict['info'], to_ros2)
    return filtered_msg_dict


def _map_meta_data_dict_filter(msg_dict: dict, to_ros2: bool) -> dict:
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['map_load_time'] = _convert_ros_time(
        filtered_msg_dict['map_load_time'], to_ros2)
    return filtered_msg_dict


def _marker_dict_filter(msg_dict: dict, to_ros2: bool) -> dict:
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['header'] = _convert_ros_header(filtered_msg_dict['header'], to_ros2)
    filtered_msg_dict['lifetime'] = _convert_ros_time(filtered_msg_dict['lifetime'], to_ros2)
    return filtered_msg_dict

def _convert_ros_time(time_msg_dict: dict, to_ros2: bool) -> dict:
    time_dict = copy.deepcopy(time_msg_dict)
    if to_ros2:
        # Conversion from MiR (ros1) to sys (ros2)
        time_dict['nanosec'] = time_dict.pop('nsecs')
        time_dict['sec'] = time_dict.pop('secs')
    else:
        # Conversion from sys (ros2) to MiR (ros1)
        time_dict['nsecs'] = time_dict.pop('nanosec')
        time_dict['secs'] = time_dict.pop('sec')

    return time_dict


def _convert_ros_header_recursive(header_msg_dict: dict, to_ros2: bool) -> dict:
    if not isinstance(header_msg_dict, dict):
        return header_msg_dict
    filtered_msg_dict = copy.deepcopy(header_msg_dict)
    for (key, value) in filtered_msg_dict.items():
        if key == 'header':
            try:
                value['stamp'] = _convert_ros_time(value['stamp'], to_ros2)
                if to_ros2:
                    del value['seq']
                    frame_id = value['frame_id'].strip('/')
                    value['frame_id'] = tf_prefix + frame_id
                else:
                    value['seq'] = 0
            except (TypeError, KeyError):
                pass   # value is not a dict or doesn't have key 'frame_id' or 'stamp'
        elif isinstance(value, dict):
            filtered_msg_dict[key] = _convert_ros_header_recursive(value, to_ros2)
        elif isinstance(value, list):
            new_list = []
            for item in value:
                new_list.append(_convert_ros_header_recursive(item, to_ros2))
            filtered_msg_dict[key] = new_list
    return filtered_msg_dict


def _convert_ros_header(header_msg_dict, to_ros2):
    header_dict = copy.deepcopy(header_msg_dict)
    header_dict['stamp'] = _convert_ros_time(header_dict['stamp'], to_ros2)
    if to_ros2:
        del header_dict['seq']
        frame_id = header_dict['frame_id'].strip('/')
        header_dict['frame_id'] = tf_prefix + frame_id
    else:  # to ros1
        header_dict['seq'] = 0
        # remove tf_prefix to frame_id

    return header_dict


def _prepend_tf_prefix_dict_filter(msg_dict: dict) -> dict | None:
    # filtered_msg_dict = copy.deepcopy(msg_dict)
    if not isinstance(msg_dict, dict):   # can happen during recursion
        return
    for (key, value) in msg_dict.items():
        if key == 'header':
            try:
                # prepend frame_id
                frame_id = value['frame_id'].strip('/')
                if (frame_id != 'map'):
                    # prepend tf_prefix, then remove leading '/' (e.g., when tf_prefix is empty)
                    value['frame_id'] = frame_id.strip('/')
                else:
                    value['frame_id'] = frame_id

            except TypeError:
                pass   # value is not a dict
            except KeyError:
                pass   # value doesn't have key 'frame_id'
        elif isinstance(value, dict):
            _prepend_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):    # an Iterable other than dict (e.g., a list)
            for item in value:
                _prepend_tf_prefix_dict_filter(item)
    return msg_dict


def _remove_tf_prefix_dict_filter(msg_dict: dict) -> dict | None:
    # filtered_msg_dict = copy.deepcopy(msg_dict)
    if not isinstance(msg_dict, dict):   # can happen during recursion
        return
    for (key, value) in msg_dict.items():
        if key == 'header':
            try:
                # remove frame_id
                s = value['frame_id'].strip('/')
                if s.find(tf_prefix) == 0:
                    # strip off tf_prefix, then strip leading '/'
                    value['frame_id'] = (s[len(tf_prefix):]).strip('/')
            except TypeError:
                pass   # value is not a dict
            except KeyError:
                pass   # value doesn't have key 'frame_id'
        elif isinstance(value, dict):
            _remove_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):    # an Iterable other than dict (e.g., a list)
            for item in value:
                _remove_tf_prefix_dict_filter(item)
    return msg_dict


def _navigate_to_pose_goal_dict_filter(msg_dict: dict, goal_id_str: str) -> OrderedDict:
    filtered_msg_dict = OrderedDict(goal=OrderedDict(), goal_id=OrderedDict())
    filtered_msg_dict['goal_id']['id'] = goal_id_str
    filtered_msg_dict['goal']['target_pose'] = copy.deepcopy(msg_dict['pose'])
    filtered_msg_dict['goal']['target_pose']['header'] = _convert_ros_header(filtered_msg_dict['goal']['target_pose']['header'], False)
    filtered_msg_dict['goal']['move_task'] = MirMoveBase.Goal.GLOBAL_MOVE
    filtered_msg_dict['goal']['goal_dist_threshold'] = 0.25
    filtered_msg_dict['goal']['clear_costmaps'] = True
    return filtered_msg_dict


def _navigate_to_pose_feedback_dict_filter(msg_dict: dict) -> OrderedDict:
    filtered_msg_dict = OrderedDict()
    filtered_msg_dict['current_pose'] = copy.deepcopy(msg_dict['feedback']['base_position'])
    filtered_msg_dict['current_pose']['header'] = _convert_ros_header(filtered_msg_dict['current_pose']['header'], True)
    return filtered_msg_dict


def _navigate_to_pose_status_dict_filter(msg_dict: dict) -> None:
    filtered_msg_dict = OrderedDict(status_list=[OrderedDict(),])
    goal_status_code = 0
    goal_id = ''
    stamp = OrderedDict()
    if len(msg_dict['status_list']) > 0:
        goal_status_code = msg_dict['status_list'][0]['status']
        goal_id = msg_dict['status_list'][0]['goal_id']['id']
        stamp = _convert_ros_time(msg_dict['status_list'][0]['goal_id']['stamp'], True)
    status = GoalStatus.STATUS_UNKNOWN

    if goal_status_code == 0: # PENDING
        status = GoalStatus.STATUS_UNKNOWN
    elif goal_status_code == 1: # ACTIVE
        status = GoalStatus.STATUS_EXECUTING
    elif goal_status_code == 2: # PREEMPTED
        status = GoalStatus.STATUS_CANCELED
    elif goal_status_code == 3: # SUCCEEDED
        status = GoalStatus.STATUS_SUCCEEDED
    elif goal_status_code == 4: # ABORTED
        status = GoalStatus.STATUS_ABORTED
    elif goal_status_code == 5: # REJECTED
        status = GoalStatus.STATUS_CANCELED
    elif goal_status_code == 6: # PREEMPTING
        status = GoalStatus.STATUS_CANCELING
    elif goal_status_code == 7: # RECALLING
        status = GoalStatus.STATUS_CANCELING
    elif goal_status_code == 8: # RECALLED
        status = GoalStatus.STATUS_CANCELED
    elif goal_status_code == 9: # LOST
        status = GoalStatus.STATUS_UNKNOWN
    else:
        status = GoalStatus.STATUS_UNKNOWN

    filtered_msg_dict['status_list'][0]['status'] = status
    filtered_msg_dict['status_list'][0]['goal_info'] = OrderedDict()
    filtered_msg_dict['status_list'][0]['goal_info']['goal_id'] = OrderedDict()
    filtered_msg_dict['status_list'][0]['goal_info']['goal_id']['uuid'] = list(bytes.fromhex(goal_id))
    filtered_msg_dict['status_list'][0]['goal_info']['stamp'] = stamp
    
    return filtered_msg_dict


def _navigate_to_pose_result_dict_filter(msg_dict: dict) -> OrderedDict:
    filtered_msg_dict = OrderedDict()
    if msg_dict['status']['status'] == 3: # SUCCEEDED
        filtered_msg_dict['error_code'] = 0
        filtered_msg_dict['error_msg'] = ''
    else:
        filtered_msg_dict['error_code'] = 1
        filtered_msg_dict['error_msg'] = msg_dict['status']['text']
    return filtered_msg_dict


# topics we want to publish to ROS (and subscribe to from the MiR)
PUB_TOPICS = [
    # TopicConfig('LightCtrl/bms_data', mir_msgs.msg.BMSData),
    # TopicConfig('LightCtrl/charging_state', mir_msgs.msg.ChargingState),
    # TopicConfig('LightCtrl/us_list', sensor_msgs.msg.Range),
    # TopicConfig('MC/battery_currents', mir_msgs.msg.BatteryCurrents),
    # TopicConfig('MC/battery_voltage', std_msgs.msg.Float64),
    # TopicConfig('MC/currents', sdc21x0.msg.MotorCurrents),
    # TopicConfig('MC/encoders', sdc21x0.msg.StampedEncoders),
    # TopicConfig('MissionController/CheckArea/visualization_marker',
    #   visualization_msgs.msg.Marker),
    # TopicConfig('MissionController/goal_position_guid', std_msgs.msg.String),
    # TopicConfig('MissionController/prompt_user', mir_msgs.msg.UserPrompt),
    # TopicConfig('SickPLC/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('SickPLC/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('active_mapping_guid', std_msgs.msg.String),
    # TopicConfig('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped),
    TopicConfig('b_raw_scan', sensor_msgs.msg.LaserScan, dict_filter=_convert_ros_header_recursive,
                qos_profile=qos_profile_sensor_data),
    TopicConfig('b_scan', sensor_msgs.msg.LaserScan, dict_filter=_convert_ros_header_recursive,
                qos_profile=qos_profile_sensor_data),
    # TopicConfig('camera_floor/background', sensor_msgs.msg.PointCloud2),
    # TopicConfig('camera_floor/depth/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('camera_floor/depth/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('camera_floor/depth/points', sensor_msgs.msg.PointCloud2),
    # TopicConfig('camera_floor/filter/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('camera_floor/floor', sensor_msgs.msg.PointCloud2),
    # TopicConfig('camera_floor/obstacles', sensor_msgs.msg.PointCloud2),
    # TopicConfig('check_area/polygon', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('check_pose_area/polygon', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('data_events/area_events', mir_data_msgs.msg.AreaEventEvent),
    # TopicConfig('data_events/maps', mir_data_msgs.msg.MapEvent),
    # TopicConfig('data_events/positions', mir_data_msgs.msg.PositionEvent),
    # TopicConfig('data_events/registers', mir_data_msgs.msg.PLCRegisterEvent),
    # TopicConfig('data_events/sounds', mir_data_msgs.msg.SoundEvent),
    # TopicConfig('diagnostics', diagnostic_msgs.msg.DiagnosticArray),
    # TopicConfig('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray),
    # TopicConfig('diagnostics_toplevel_state', diagnostic_msgs.msg.DiagnosticStatus),
    TopicConfig('f_raw_scan', sensor_msgs.msg.LaserScan, dict_filter=_convert_ros_header_recursive,
                qos_profile=qos_profile_sensor_data),
    TopicConfig('f_scan', sensor_msgs.msg.LaserScan, dict_filter=_convert_ros_header_recursive,
                qos_profile=qos_profile_sensor_data),
    # TopicConfig('imu_data', sensor_msgs.msg.Imu),
    # TopicConfig('laser_back/driver/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('laser_back/driver/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('laser_front/driver/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('laser_front/driver/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('localization_score', std_msgs.msg.Float64),
    TopicConfig('map', nav_msgs.msg.OccupancyGrid, latch=True, dict_filter=_map_dict_filter),
    TopicConfig('map_metadata', nav_msgs.msg.MapMetaData, dict_filter=_map_meta_data_dict_filter),
    # TopicConfig('marker_tracking_node/feedback',
    #   mir_marker_tracking.msg.MarkerTrackingActionFeedback),
    # TopicConfig('marker_tracking_node/laser_line_extract/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('marker_tracking_node/laser_line_extract/parameter_updates',
    #   dynamic_reconfigure.msg.Config),
    # TopicConfig('marker_tracking_node/laser_line_extract/visualization_marker',
    #   visualization_msgs.msg.Marker),
    # TopicConfig('marker_tracking_node/result',
    #   mir_marker_tracking.msg.MarkerTrackingActionResult),
    # TopicConfig('marker_tracking_node/status', actionlib_msgs.msg.GoalStatusArray),
    # TopicConfig('mirEventTrigger/events', mir_msgs.msg.Events),
    # TopicConfig('mir_amcl/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('mir_amcl/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('mir_amcl/selected_points', sensor_msgs.msg.PointCloud2),
    # TopicConfig('mir_log', rosgraph_msgs.msg.Log),
    # TopicConfig('mir_sound/sound_event', mir_msgs.msg.SoundEvent),
    # TopicConfig('mir_status_msg', std_msgs.msg.String),
    # TopicConfig('mirspawn/node_events', mirSpawn.msg.LaunchItem),
    # TopicConfig('mirwebapp/grid_map_metadata', mir_msgs.msg.LocalMapStat),
    # TopicConfig('mirwebapp/laser_map_metadata', mir_msgs.msg.LocalMapStat),
    # TopicConfig('mirwebapp/web_path', mir_msgs.msg.WebPath),
    # really mir_actions/MirMoveBaseActionFeedback:
    # TopicConfig('move_base/feedback', move_base_msgs.msg.MoveBaseActionFeedback,
    #   dict_filter=_move_base_feedback_dict_filter),
    # really mir_actions/MirMoveBaseActionResult:
    # TopicConfig('move_base/result', move_base_msgs.msg.MoveBaseActionResult,
    #   dict_filter=_move_base_result_dict_filter),
    # TopicConfig('move_base/status', actionlib_msgs.msg.GoalStatusArray),
    # TopicConfig('move_base_node/MIRPlannerROS/cost_cloud', sensor_msgs.msg.PointCloud2),
    TopicConfig('move_base_node/MIRPlannerROS/global_plan', nav_msgs.msg.Path, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/MIRPlannerROS/len_to_goal', std_msgs.msg.Float64),
    TopicConfig('move_base_node/MIRPlannerROS/local_plan', nav_msgs.msg.Path, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/MIRPlannerROS/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/MIRPlannerROS/parameter_updates',
    #   dynamic_reconfigure.msg.Config),
    TopicConfig('move_base_node/MIRPlannerROS/updated_global_plan', mir_msgs.msg.PlanSegments, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/MIRPlannerROS/visualization_marker',
    #   visualization_msgs.msg.MarkerArray),
    TopicConfig('move_base_node/SBPLLatticePlanner/plan', nav_msgs.msg.Path, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/SBPLLatticePlanner/sbpl_lattice_planner_stats',
    #   sbpl_lattice_planner.msg.SBPLLatticePlannerStats),
    # TopicConfig('move_base_node/SBPLLatticePlanner/visualization_marker',
    #   visualization_msgs.msg.MarkerArray),
    TopicConfig('move_base_node/current_goal', geometry_msgs.msg.PoseStamped, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/global_costmap/inflated_obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/global_costmap/obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/global_costmap/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/global_costmap/parameter_updates',
    #   dynamic_reconfigure.msg.Config),
    # TopicConfig('move_base_node/global_costmap/robot_footprint',
    #   geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/global_costmap/unknown_space', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/global_plan', nav_msgs.msg.Path),
    TopicConfig('move_base_node/local_costmap/inflated_obstacles', nav_msgs.msg.GridCells, dict_filter=_convert_ros_header_recursive),
    TopicConfig('move_base_node/local_costmap/obstacles', nav_msgs.msg.GridCells, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/local_costmap/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/local_costmap/parameter_updates',
    #   dynamic_reconfigure.msg.Config),
    TopicConfig('move_base_node/local_costmap/robot_footprint', geometry_msgs.msg.PolygonStamped, dict_filter=_convert_ros_header_recursive),
    # TopicConfig('move_base_node/local_costmap/safety_zone', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/local_costmap/unknown_space', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/mir_escape_recovery/visualization_marker',
    #   visualization_msgs.msg.Marker),
    # TopicConfig('move_base_node/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('move_base_node/time_to_coll', std_msgs.msg.Float64),
    # TopicConfig('move_base_node/traffic_costmap/inflated_obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/traffic_costmap/obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/traffic_costmap/parameter_descriptions',
    #   dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/traffic_costmap/parameter_updates',
    #  dynamic_reconfigure.msg.Config),
    # TopicConfig('move_base_node/traffic_costmap/robot_footprint',
    #   geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/traffic_costmap/unknown_space', nav_msgs.msg.GridCells),
    TopicConfig('move_base_node/visualization_marker', visualization_msgs.msg.Marker, dict_filter=_marker_dict_filter),
    TopicConfig('move_base_simple/visualization_marker', visualization_msgs.msg.Marker, dict_filter=_marker_dict_filter),
    TopicConfig('odom', nav_msgs.msg.Odometry, dict_filter=_odom_dict_filter),
    # TopicConfig('odom_enc', nav_msgs.msg.Odometry),
    # TopicConfig('one_way_map', nav_msgs.msg.OccupancyGrid),
    # TopicConfig('param_update', std_msgs.msg.String),
    # TopicConfig('particlevizmarker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('resource_tracker/needed_resources', mir_msgs.msg.ResourcesState),
    # TopicConfig('robot_mode', mir_msgs.msg.RobotMode),
    # TopicConfig('robot_pose', geometry_msgs.msg.Pose),
    # TopicConfig('robot_state', mir_msgs.msg.RobotState),
    # TopicConfig('robot_status', mir_msgs.msg.RobotStatus),
    # TopicConfig('/rosout', rosgraph_msgs.msg.Log),
    # TopicConfig('/rosout_agg', rosgraph_msgs.msg.Log),
    # TopicConfig('scan', sensor_msgs.msg.LaserScan),
    # TopicConfig('scan_filter/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('scan_filter/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('scan_filter/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('session_importer_node/info', mirSessionImporter.msg.SessionImportInfo),
    # TopicConfig('set_mc_PID', std_msgs.msg.Float64MultiArray),
    # let /tf be /tf if namespaced
    TopicConfig('tf', TFMessage, dict_filter=_tf_dict_filter, topic_renamed='/tf'),
    # TopicConfig('/tf_static', tf2_msgs.msg.TFMessage, dict_filter=_tf_static_dict_filter,
    #             latch=True),
    # TopicConfig('traffic_map', nav_msgs.msg.OccupancyGrid),
    # TopicConfig('wifi_diagnostics', diagnostic_msgs.msg.DiagnosticArray),
    # TopicConfig('wifi_diagnostics/cur_ap', mir_wifi_msgs.msg.APInfo),
    # TopicConfig('wifi_diagnostics/roam_events', mir_wifi_msgs.msg.WifiRoamEvent),
    # TopicConfig('wifi_diagnostics/wifi_ap_interface_stats',
    #  mir_wifi_msgs.msg.WifiInterfaceStats),
    # TopicConfig('wifi_diagnostics/wifi_ap_rssi', mir_wifi_msgs.msg.APRssiStats),
    # TopicConfig('wifi_diagnostics/wifi_ap_time_stats', mir_wifi_msgs.msg.APTimeStats),
    # TopicConfig('wifi_watchdog/ping', mir_wifi_msgs.msg.APPingStats),
]


# topics we want to subscribe to from ROS (and publish to the MiR)
SUB_TOPICS = [
    TopicConfig('cmd_vel', geometry_msgs.msg.TwistStamped, 'cmd_vel_stamped'),
    # TopicConfig('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped),
    # TopicConfig('light_cmd', std_msgs.msg.String),
    # TopicConfig('mir_cmd', std_msgs.msg.String),
    # TopicConfig('move_base/cancel', actionlib_msgs.msg.GoalID),
    # really mir_actions/MirMoveBaseActionGoal:
    # TopicConfig('move_base/goal', move_base_msgs.msg.MoveBaseActionGoal,
    #   dict_filter=_move_base_goal_dict_filter),
]


ACTIONS = [
    ActionConfig('move_base', NavigateToPose, 'nav2/navigate_to_pose',
        goal_dict_filter=_navigate_to_pose_goal_dict_filter,
        feedback_dict_filter=_navigate_to_pose_feedback_dict_filter,
        status_dict_filter=_navigate_to_pose_status_dict_filter,
        result_dict_filter=_navigate_to_pose_result_dict_filter),
    # TopicConfig('move_base/cancel', actionlib_msgs.msg.GoalID),
    # TopicConfig('move_base/feedback', move_base_msgs.msg.MoveBaseActionFeedback,
    #   dict_filter=_move_base_feedback_dict_filter),
    # really mir_actions/MirMoveBaseActionResult:
    # TopicConfig('move_base/result', move_base_msgs.msg.MoveBaseActionResult,
    #   dict_filter=_move_base_result_dict_filter),
    # TopicConfig('move_base/status', actionlib_msgs.msg.GoalStatusArray),
]


class PublisherWrapper(object):
    def __init__(self, topic_config: TopicConfig, nh: Node) -> None:
        self.topic_config = topic_config
        self.robot = nh.robot
        self.connected = False

        self.pub = nh.create_publisher(
            msg_type=topic_config.topic_type,
            topic=topic_config.topic_ros2_name,
            qos_profile=topic_config.qos_profile
        )

        nh.get_logger().info("Publishing topic '%s' [%s]" %
                             (topic_config.topic_ros2_name, topic_config.topic_type.__module__))
        # latched topics must be subscribed immediately
        # if topic_config.latch:
        self.peer_subscribe(nh)

    def peer_subscribe(self, nh: Node) -> None:
        if not self.connected:
            self.connected = True
            nh.get_logger().info("Starting to stream messages on topic '%s'" %
                                 self.topic_config.topic)
            self.robot.subscribe(
                topic=('/' + self.topic_config.topic), callback=self.callback)

    def callback(self, msg_dict: dict) -> None:
        if not isinstance(msg_dict, dict):   # can happen during recursion
            return
        msg_dict = _prepend_tf_prefix_dict_filter(msg_dict)
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict, to_ros2=True)
        msg = message_converter.convert_dictionary_to_ros_message(
            self.topic_config.topic_type, msg_dict)
        self.pub.publish(msg)


class SubscriberWrapper(object):
    def __init__(self, topic_config: TopicConfig, nh: Node) -> None:
        self.topic_config = topic_config
        self.robot = nh.robot
        self.sub = nh.create_subscription(
            msg_type=topic_config.topic_type,
            topic=topic_config.topic_ros2_name,
            callback=self.callback,
            qos_profile=topic_config.qos_profile
        )

        nh.get_logger().info("Subscribing to topic '%s' [%s]" % (
            topic_config.topic, topic_config.topic_type.__module__))

    def callback(self, msg: Any) -> None:
        if msg is None:
            return

        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_dict = _remove_tf_prefix_dict_filter(msg_dict)

        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict, to_ros2=False)
        self.robot.publish('/' + self.topic_config.topic, msg_dict)


class ActionWrapper(object):
    def __init__(self, action_config: ActionConfig, nh: Node) -> None:
        self.action_config = action_config
        self.robot = nh.robot
        self.server = ActionServer(
            nh,
            action_type=action_config.action_type,
            action_name=action_config.action_topic,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        nh.get_logger().info(f"Creating action server at '{action_config.action_topic}' [{action_config.action_type.__module__}]")

        # Subscribe to result/status/feedback topics of MiR base
        self.robot.subscribe(topic=('/' + self.action_config.mir_action_topic + '/result'), callback=self.result_callback)
        self.robot.subscribe(topic=('/' + self.action_config.mir_action_topic + '/status'), callback=self.status_callback)
        self.robot.subscribe(topic=('/' + self.action_config.mir_action_topic + '/feedback'), callback=self.feedback_callback)

        # Create publisher for status of MiR base
        self.status_pub = nh.create_publisher(
            msg_type=GoalStatusArray,
            topic=action_config.action_topic + '/status',
            qos_profile=action_config.qos_profile
        )

        self._lock = threading.Lock()
        self.goal_events = {}
        self.goal_results = {}
        self.goal_handles = {}

    def execute_callback(self, goal_handle: ServerGoalHandle) -> NavigateToPose.Result:
        goal_id_str = bytearray(goal_handle.goal_id.uuid).hex()
        event = threading.Event()
        with self._lock:
            self.goal_events[goal_id_str] = event
            self.goal_handles[goal_id_str] = goal_handle

        msg_dict = message_converter.convert_ros_message_to_dictionary(goal_handle.request)
        msg_dict = _remove_tf_prefix_dict_filter(msg_dict)

        if self.action_config.goal_dict_filter is not None:
            filtered_goal_msg = self.action_config.goal_dict_filter(msg_dict, goal_id_str)
        
        self.robot.publish('/' + self.action_config.mir_action_topic + '/goal', filtered_goal_msg)

        while rclpy.ok() and not event.is_set():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._cleanup_goal(goal_id_str)
                return NavigateToPose.Result()
            event.wait()  # Wait for the event

        result = NavigateToPose.Result()

        with self._lock:
            result_dict = self.goal_results.get(goal_id_str, {})

        if self.action_config.result_dict_filter is not None:
            result_dict = self.action_config.result_dict_filter(result_dict)

        result = message_converter.convert_dictionary_to_ros_message(
                self.action_config.action_type.Result, result_dict)

        if event.is_set():
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._cleanup_goal(goal_id_str)
        return result
        

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        goal_id_str = bytearray(goal_handle.goal_id.uuid).hex()

        cancel_msg_dict = {'id': goal_id_str, 'stamp': {'secs': 0, 'nsecs': 0}}
        self.robot.publish('/' + self.action_config.mir_action_topic + '/cancel', cancel_msg_dict)

        return CancelResponse.ACCEPT

    def result_callback(self, msg_dict: dict) -> None:
        if not isinstance(msg_dict, dict):
            return

        try:
            goal_id_str = msg_dict['status']['goal_id']['id']
        except KeyError:
            return

        with self._lock:
            event = self.goal_events.get(goal_id_str)
            if event:
                self.goal_results[goal_id_str] = msg_dict
                event.set()

    def feedback_callback(self, msg_dict: dict) -> None:
        if not isinstance(msg_dict, dict):
            return
        try:
            goal_id_str = msg_dict['status']['goal_id']['id']
        except KeyError:
            return

        with self._lock:
            goal_handle = self.goal_handles.get(goal_id_str)

        if goal_handle and goal_handle.is_active:
            msg_dict = _prepend_tf_prefix_dict_filter(msg_dict)
            if self.action_config.feedback_dict_filter is not None:
                feedback_filtered = self.action_config.feedback_dict_filter(msg_dict)
                feedback_msg = message_converter.convert_dictionary_to_ros_message(
                    self.action_config.action_type.Feedback, feedback_filtered)
                goal_handle.publish_feedback(feedback_msg)

    def status_callback(self, msg_dict: dict) -> None:
        if not isinstance(msg_dict, dict):
            return
    
        msg_dict = _prepend_tf_prefix_dict_filter(msg_dict)
        
        if self.action_config.status_dict_filter is not None:
            msg_dict = self.action_config.status_dict_filter(msg_dict)
        
        msg = message_converter.convert_dictionary_to_ros_message(
            GoalStatusArray, msg_dict)

        self.status_pub.publish(msg)

    def _cleanup_goal(self, goal_id_str: str) -> None:
        with self._lock:
            self.goal_events.pop(goal_id_str, None)
            self.goal_results.pop(goal_id_str, None)
            self.goal_handles.pop(goal_id_str, None)


class MiR100BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('mir_bridge')

        self.mir_bridge_ready = False  # state
        self.srv_mir_ready = self.create_service(
            Trigger, 'mir_bridge_ready', self.mir_bridge_ready_poll_callback)

        try:
            hostname = self.declare_parameter(
                'hostname', '192.168.12.20').value
        except KeyError:
            self.get_logger().fatal('Parameter "hostname" is not set!')
            sys.exit(-1)
        port = self.declare_parameter('port', 9090).value
        assert isinstance(port, int), 'port parameter must be an integer'

        global tf_prefix
        self.declare_parameter('tf_prefix', '')
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value.strip('/')
        if tf_prefix != "":
            tf_prefix = tf_prefix + '/'

        self.get_logger().info('Trying to connect to %s:%i...' % (hostname, port))
        self.robot = mir_driver.rosbridge.RosbridgeSetup(hostname, port)

        i = 1
        while not self.robot.is_connected():
            if not rclpy.ok():
                sys.exit(0)
            if self.robot.is_errored():
                self.get_logger().fatal('Connection error to %s:%i, giving up!'
                                        % (hostname, port))
                sys.exit(-1)
            if i % 10 == 0:
                self.get_logger().warn('Still waiting for connection to %s:%i...'
                                       % (hostname, port))
            i += 1
            time.sleep(1)
        self.get_logger().info('Connected to %s:%i...' % (hostname, port))

        topics = self.get_topics()
        published_topics = [topic_name for (topic_name, _, has_publishers, _)
                            in topics if has_publishers]
        subscribed_topics = [topic_name for (topic_name, _, _, has_subscribers)
                             in topics if has_subscribers]

        for pub_topic in PUB_TOPICS:
            PublisherWrapper(pub_topic, self)
            if ('/' + pub_topic.topic) not in published_topics:
                self.get_logger().warn("Topic '%s' is not published by the MiR!" % pub_topic.topic)

        for sub_topic in SUB_TOPICS:
            SubscriberWrapper(sub_topic, self)
            if ('/' + sub_topic.topic) not in subscribed_topics:
                self.get_logger().warn(
                    "Topic '%s' is not yet subscribed to by the MiR!" % sub_topic.topic)
                
        for action in ACTIONS:
            ActionWrapper(action, self)
            if ('/' + action.mir_action_topic + '/goal') not in subscribed_topics:
                self.get_logger().warn(
                    f"Action {'/' + action.mir_action_topic + '/goal'} is not yet subscribed to by the MiR"
                )

        self.mir_bridge_ready = True

    def get_topics(self) -> list:
        srv_response = self.robot.callService('/rosapi/topics', msg={})
        topic_names = sorted(srv_response['topics'])
        topics = []

        for topic_name in topic_names:
            srv_response = self.robot.callService(
                "/rosapi/topic_type", msg={'topic': topic_name})
            topic_type = srv_response['type']

            srv_response = self.robot.callService(
                "/rosapi/publishers", msg={'topic': topic_name})
            has_publishers = True if len(
                srv_response['publishers']) > 0 else False

            srv_response = self.robot.callService(
                "/rosapi/subscribers", msg={'topic': topic_name})
            has_subscribers = True if len(
                srv_response['subscribers']) > 0 else False

            topics.append(
                [topic_name, topic_type, has_publishers, has_subscribers])

        self.get_logger().info('Publishers:')
        for (topic_name, topic_type, has_publishers, has_subscribers) in topics:
            if has_publishers:
                self.get_logger().info((' * %s [%s]' % (topic_name, topic_type)))

        self.get_logger().info('\nSubscribers:')
        for (topic_name, topic_type, has_publishers, has_subscribers) in topics:
            if has_subscribers:
                self.get_logger().info((' * %s [%s]' % (topic_name, topic_type)))

        return topics

    def mir_bridge_ready_poll_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('Checked for readiness')
        response.success = self.mir_bridge_ready
        response.message = ""
        return response
    
    def destroy_node(self) -> None:
        self.robot.close()
        super().destroy_node()


from rclpy.executors import MultiThreadedExecutor

def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = MiR100BridgeNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
