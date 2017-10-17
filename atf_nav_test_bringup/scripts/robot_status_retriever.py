#!/usr/bin/python

import rospy
import math
import urllib2

import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
#from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from cob_msgs.msg import PowerState, EmergencyStopState
from msh_msgs.msg import RobotStatus
from actionlib_msgs.msg import GoalStatusArray
from msh_msgs.msg import ErrorState

class RobotStatusRetriever():

  def __init__(self):
    self.from_frame = 'map'
    self.to_frame = 'base_link'
    self.tf_buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf_buffer)
    self.robot_status = RobotStatus()
    self.navigation_active = False
    self.last_time_moving = rospy.Time.now()
    self.localized = False
    self.localization_lost = False
    self.emergency_button_stop = False
    self.scanner_stop = False
    self.actuator_error = False
    self.sensor_error = False
    self.navigation_error = False
    self.sm_error = False
    self.power_error = False
    self.general_diagnostics_error = False
    self.localization_quality = 0.0

    self.stehle_pos = None
    self.charger_pos = None
    self.home_threshold = rospy.get_param('home_threshold', 3.0)
    if not rospy.has_param('/script_server/base/stehle'):
      rospy.logerr('Could not find [/script_server/base/stehle] on ParameterServer')
    else:
      self.stehle_pos = rospy.get_param("/script_server/base/stehle")
    if not rospy.has_param('/script_server/base/charger'):
      rospy.logerr('Could not find [/script_server/base/charger] on ParameterServer')
    else:
      self.charger_pos = rospy.get_param("/script_server/base/charger")

    self.pub_robot_status = rospy.Publisher('robot_status', RobotStatus, queue_size=1)
    rospy.Subscriber("/power_state", PowerState, self.power_state_cb)
    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_cb)
#    rospy.Subscriber("/sis/smach/container_status", SmachContainerStatus, self.smach_cb)
    rospy.Subscriber("/localization_quality_ekf", Float32, self.localization_quality_cb)
    rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.odometry_cb)
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_cb)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.localization_cb)
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emergency_stop_cb)

    rospy.sleep(1.0) # let rospy time to setup communication

  def power_state_cb(self, msg):
    power_error = False
    if msg.relative_remaining_capacity < 5: # less than 5%
      power_error = True
    self.robot_status.power_state = msg
    self.power_error = power_error

  def localization_quality_cb(self, msg):
    self.localization_quality = msg.data

  def diagnostics_cb(self, msg):
    actuator_error = False
    sensor_error = False
    navigation_error = False
    general_diagnostics_error = False
    sm_error = False
    diagnostics_errors = []
    for status in msg.status:
      if status.level > 0:
        diagnostics_errors.append(status.name)
        if "Actuators" in status.name:
          actuator_error = True

        if "Sensors" in status.name:
          sensor_error = True

        if "Move Base" in status.name:
          navigation_error = True
        else:
          if not status.name == "/Scenario":
            navigation_error = False

        if "State Machine" in status.name:
          sm_error = True

        general_diagnostics_error = True

    self.actuator_error = actuator_error
    self.sensor_error = sensor_error
    self.navigation_error = navigation_error
    self.general_diagnostics_error = general_diagnostics_error
    self.diagnostics_errors = diagnostics_errors
    self.sm_error = sm_error

  def odometry_cb(self, msg):
    if math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2) >= 0.1:
      self.last_time_moving = rospy.Time.now()
    if not self.navigation_active:
      self.last_time_moving = rospy.Time.now()

  def move_base_status_cb(self, msg):
    if len(msg.status_list) == 0:
      self.navigation_active = False
      return
    if msg.status_list[len(msg.status_list)-1].status < 2:
      self.navigation_active = True
    else:
      self.navigation_active = False

  def localization_cb(self, msg):
    self.localized = True
    self.localization_lost = False

  def is_localization_lost(self):
    if rospy.Time.now() - self.last_time_moving >= rospy.Duration(5*60) and self.navigation_active:
        self.localization_lost = True
    return self.localization_lost

  def emergency_stop_cb(self, msg):
    self.emergency_button_stop = msg.emergency_button_stop
    self.scanner_stop = msg.scanner_stop

  def update_tf(self):
    #rospy.loginfo("update tf")
    try:
      time = rospy.Time.now()
      transform = self.tf_buffer.lookup_transform(self.from_frame, self.to_frame, time, rospy.Duration(0.5))
      trans = transform.transform.translation
      rot = transform.transform.rotation
      self.robot_status.pose_robot.header.stamp = time
      self.robot_status.pose_robot.header.frame_id = self.to_frame
      self.robot_status.pose_robot.pose.position.x = trans.x
      self.robot_status.pose_robot.pose.position.y = trans.y
      self.robot_status.pose_robot.pose.position.z = trans.z
      self.robot_status.pose_robot.pose.orientation.x = rot.x
      self.robot_status.pose_robot.pose.orientation.y = rot.y
      self.robot_status.pose_robot.pose.orientation.z = rot.z
      self.robot_status.pose_robot.pose.orientation.w = rot.w

      self.robot_status.pose_stehle.header.stamp = time
      self.robot_status.pose_stehle.header.frame_id = self.to_frame
      self.robot_status.pose_stehle.pose.position.x = self.stehle_pos[0]
      self.robot_status.pose_stehle.pose.position.y = self.stehle_pos[1]
      self.robot_status.pose_stehle.pose.position.z = 0.0
      rot = tf.transformations.quaternion_from_euler(0.0, 0.0, self.stehle_pos[2])
      self.robot_status.pose_stehle.pose.orientation.x = rot[0]
      self.robot_status.pose_stehle.pose.orientation.y = rot[1]
      self.robot_status.pose_stehle.pose.orientation.z = rot[2]
      self.robot_status.pose_stehle.pose.orientation.w = rot[3]

      self.robot_status.at_home = self.is_at_home(self.robot_status.pose_robot.pose)
      self.localized = True
    except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      rospy.logerr(str(e))

  def is_at_home(self, pose):
    #if not self.stehle_pos == None and not self.charger_pos == None: # TODO use this as soon as #msh/462 is fixed
    if not self.stehle_pos == None:
      dist_stehle = math.sqrt((self.stehle_pos[0] - pose.position.x)**2 + (self.stehle_pos[1] - pose.position.y)**2)
      dist_charger = math.sqrt((self.charger_pos[0] - pose.position.x)**2 + (self.charger_pos[1] - pose.position.y)**2)
      if dist_stehle < self.home_threshold or dist_charger < self.home_threshold:
        return True
    return False

  def is_online(self, hostname):
    try:
      urllib2.urlopen(hostname, timeout=1)
      return True
    except:
      return False

  def check_for_error(self):
    call_us = "Rufe meine Erbauer an (Tel: 071577348966)."
    restart = "Starte mich neu."
    back_to_charger = "Nimm den Joystick und fahre mich zurueck vor die Ladestation."

    if self.emergency_button_stop:
      self.robot_status.error_state.id = ErrorState.EMERGENCY_STOP_BUTTON
      self.robot_status.error_state.message = "Notaus: Notausknoepfe oder Bremsenknopf gedrueckt"
      self.robot_status.error_state.trouble_shooting = ["Loese die Knoepfe auf meiner Vorder- und Rueckseite.", call_us]
    elif self.scanner_stop:
      self.robot_status.error_state.id = ErrorState.EMERGENCY_STOP_LASER
      self.robot_status.error_state.message = "Notaus: Zu nahes Hindernis"
      self.robot_status.error_state.trouble_shooting = ["Falls moeglich: Entferne das Hindernis.", "Falls das nicht moeglich ist: Loese die Bremsen und schiebe mich vom Hindernis weg. (Denke daran danach den Bremsenknopf wieder zu loesen)", call_us]
    elif self.actuator_error:
      self.robot_status.error_state.id = ErrorState.MOTOR_ERROR
      self.robot_status.error_state.message = "Motorfehler"
      self.robot_status.error_state.trouble_shooting = ["Fehler in: " + str(self.diagnostics_errors) + "), druecke den <<recover all>> Knopf", restart, call_us]
    elif self.sensor_error:
      self.robot_status.error_state.id = ErrorState.SENSOR_ERROR
      self.robot_status.error_state.message = "Sensorfehler" # "sensor error"
      self.robot_status.error_state.trouble_shooting = ["Fehler in: " + str(self.diagnostics_errors) + ")", restart, call_us]
    elif not self.localized:
      self.robot_status.error_state.id = ErrorState.NOT_LOCALIZED
      self.robot_status.error_state.message = "Nicht lokalisiert" #"Robot not localised"
      self.robot_status.error_state.trouble_shooting = [back_to_charger, restart, call_us]
    elif self.navigation_error:
      self.robot_status.error_state.id = ErrorState.NAVIGATION_ERROR
      self.robot_status.error_state.message = "Fehler in Navigation, ich kann nicht weiterfahren"
      self.robot_status.error_state.trouble_shooting = [restart, call_us]
    elif self.is_localization_lost():
      self.robot_status.error_state.id = ErrorState.LOCALIZATION_LOST
      self.robot_status.error_state.message = "Lokalisierung verloren"
      self.robot_status.error_state.trouble_shooting = [back_to_charger, restart, call_us, "Qualitaet: " + str(self.localization_quality)]
    elif not (self.is_online("http://google.de") or self.is_online("http://github.com") or self.is_online("http://heise.de")):
      self.robot_status.error_state.id = ErrorState.OK # ErrorState.NO_INTERNET # if we set this error we might get into a deadlock when driving back to the home base, see msh#809
      self.robot_status.error_state.message = "Kein Internet" # "No internet"
      self.robot_status.error_state.trouble_shooting = [back_to_charger, restart, call_us]
    #elif not self.is_online("http://10.4.2.1"): #TODO: update to ip of stehle (http://10.0.3.20)
    #  self.robot_status.error_state.id = ErrorState.NO_CONNECTION_TO_STEHLE
    #  self.robot_status.error_state.message = "Keine Verbindung zur Stehle"
    #  self.robot_status.error_state.trouble_shooting = ["Starte die Stehle neu", restart, call_us]
    elif self.sm_error:
      self.robot_status.error_state.id = ErrorState.STATE_MACHINE_CRASHED
      self.robot_status.error_state.message = "State Machine abgestuerzt"
      self.robot_status.error_state.trouble_shooting = [restart, call_us]
    elif self.power_error:
      self.robot_status.error_state.id = ErrorState.POWER_ERROR
      self.robot_status.error_state.message = "Ladezustand kritisch"
      self.robot_status.error_state.trouble_shooting = ["Docke mich an die Ladestation an", call_us]
    elif self.general_diagnostics_error:
      self.robot_status.error_state.id = ErrorState.GENERAL_DIAGNOSTIC_ERROR
      self.robot_status.error_state.message = "Allgemeiner Diagnosefehler"
      self.robot_status.error_state.trouble_shooting = ["Fehler in: " + str(self.diagnostics_errors) + ")", restart, call_us]
    else: # reset error
      self.robot_status.error_state.id = ErrorState.OK
      self.robot_status.error_state.message = "Alles OK"
      self.robot_status.error_state.trouble_shooting = ["Hab Spass mit mir!"]

  def publish_robot_status(self):
    #rospy.loginfo("publish robot status")
    self.update_tf()
    self.check_for_error()
    self.pub_robot_status.publish(self.robot_status)

if __name__ == "__main__":

  rospy.init_node("robot_status_retriever")
  rss = RobotStatusRetriever()
  rospy.loginfo("robot_status_retriever running")
  r = rospy.Rate(0.25)
  while not rospy.is_shutdown():
    rss.publish_robot_status()
    try:
      r.sleep()
    except rospy.exceptions.ROSInterruptException as e:
      pass
