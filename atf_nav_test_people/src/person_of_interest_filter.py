#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import math

import cob_perception_msgs
from cob_perception_msgs.msg import DetectionArray, Detection
import emotion_detection
from emotion_detection.msg import DetectionArray
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3, Quaternion

class poi():
    def __init__(self):
        rospy.init_node('person_of_interest_filter')
        self.distance_factor = rospy.get_param('~distance_factor', 3.0)
        self.angle_factor = rospy.get_param('~angle_factor', 0.1)
        self.min_distance = rospy.get_param('~min_distance', 0.6)
        self.max_distance = rospy.get_param('~max_distance', 1.5)
        self.emotion_distance_offset = rospy.get_param('~emotion_distance_offset', 0.0)
        self.people_distance_offset = rospy.get_param('~people_distance_offset', 0.0)
        #rospy.Subscriber("/people_detection_sensorring/face_detector/face_detections_cartesian", cob_perception_msgs.msg.DetectionArray, self.people_callback)
        rospy.Subscriber("face_detections", cob_perception_msgs.msg.DetectionArray, self.people_callback)
        #rospy.Subscriber("/emotion_detection/detections", emotion_detection.msg.DetectionArray, self.emotion_callback)
        rospy.Subscriber("emotion_detections", emotion_detection.msg.DetectionArray, self.emotion_callback)
        self.pub = rospy.Publisher('detection', Detection, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()
        rospy.loginfo("people of interest filter started")
        self.interest_old = 0
        self.poi = Detection()

    def calculate_interest(self, detection, distance_offset):
        try:
            pose_bl = self.tfBuffer.transform(tf2_ros.convert(detection.pose, tf2_geometry_msgs.PoseStamped),"torso_rotated_base_link")
            detection.pose = pose_bl
        except tf2_ros.TransformException as e:
            rospy.logerr("person_of_interest, tf error: %s"%str(e))
            return 0

        distance = math.sqrt(math.pow(detection.pose.pose.position.x,2) + math.pow(detection.pose.pose.position.y,2))
        distance += distance_offset
        if(distance > self.max_distance):
            distance = self.max_distance
            return 0

        if(detection.pose.pose.position.x > 0):
            angle = math.atan(detection.pose.pose.position.y/detection.pose.pose.position.x)
        else:
            angle = math.pi/2

        # Interst should be normalized between 0 and 1. 1 = hight interest, 0 = low interest
        if detection.pose.pose.position.x > 0:
            interest = 1/(math.pow(distance, 2)/self.distance_factor + math.pow(angle, 2)/self.angle_factor)
        else:
            interest = 0

        # normalize interest between 0 and 1. 1=high interest, 0=low interest
        max_interest = 1/(math.pow(self.min_distance, 2)/self.distance_factor + math.pow(0.0, 2)/self.angle_factor)
        interest_norm = interest / max_interest

        #print "interest=", interest, "max_interest=", max_interest, "interest_norm=", interest_norm, "distance=", distance, "angle=", angle
        rospy.logdebug("interest=%s distance=%s angle=%s",interest_norm, distance, angle)
        return interest_norm

    def emotion_callback(self, detection_array):
        if len(detection_array.detections) > 0:
            rospy.logdebug("got emotion detection array")
            poi_emotion = detection_array.detections[0]
            poi_emotion.score = self.calculate_interest(detection_array.detections[0], self.emotion_distance_offset)

            for detection in detection_array.detections:
                detection.score = self.calculate_interest(detection, self.emotion_distance_offset)
                rospy.loginfo("emotion_callback: detection.score %f"%(detection.score))

                if (detection.score > poi_emotion.score):
                    poi_emotion = detection

            if poi_emotion.score == 0:
                return

            # copy into cob_perception_msgs/Detection
            poi = cob_perception_msgs.msg.Detection()
            poi.header = poi_emotion.header
            poi.pose = poi_emotion.pose
            poi.score = poi_emotion.score

            if poi.score > self.poi.score:
                rospy.logdebug("emotion_callback: new poi")
                self.poi = poi


    def people_callback(self, detection_array):
        if len(detection_array.detections) > 0:
            rospy.logdebug("got people detection array")
            poi = detection_array.detections[0]
            poi.score = self.calculate_interest(detection_array.detections[0], self.people_distance_offset)

            for detection in detection_array.detections:
                detection.score = self.calculate_interest(detection, self.people_distance_offset)
                rospy.loginfo("people_callback: detection.score %f"%(detection.score))

                if (detection.score > poi.score):
                    poi = detection

            if poi.score == 0:
                return

            if poi.score > self.poi.score:
                rospy.logdebug("people_callback: new poi")
                self.poi = poi

    def execute(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # reset poi if no more detections
            if (rospy.Time.now() - self.poi.header.stamp) > rospy.Duration(5):
                self.poi = Detection()
            if self.poi.score != 0:
                t = TransformStamped()
                t.header = self.poi.pose.header
                t.child_frame_id = "person_of_interest"
                t.transform.translation = Vector3(self.poi.pose.pose.position.x, self.poi.pose.pose.position.y, self.poi.pose.pose.position.z)
                t.transform.rotation = Quaternion(0,0,0,1)
                self.br.sendTransform(t)
                self.pub.publish(self.poi)
            r.sleep()

if __name__ == '__main__':
    try:
        t = poi()
        t.execute()
    except rospy.ROSInterruptException:
        pass
