#!/usr/bin/env python

import rospy

from std_srvs import *
from msh_msgs.srv import ShowEmotion, ShowProductList, ShowEmotionResponse, ShowProductListResponse
from msh_msgs.msg import Emotion, ProductDetails, Product, GuiOutcome, GuiStatus

class FakeGui(object):
    def __init__(self):
        self.srv_emotion_stehle = rospy.Service('/gui_stehle/show_emotion', ShowEmotion, self.emotion_cb)
        self.srv_showProductList_stehle = rospy.Service('/gui_stehle/show_product_list', ShowProductList, self.productlist_cb)
        rospy.loginfo('FakeGui is running')

    def emotion_cb(self, req):
        rospy.loginfo('Requested Emotion to: %s', self.emotion_to_string(req.emotion.emotion))
        res = ShowEmotionResponse()
        res.success = True
        res.message = "emotion set to " + self.emotion_to_string(req.emotion.emotion)
        return res

    def productlist_cb(self, req):
        res = ShowProductListResponse()
        if len(req.product_list.product_details_list) == 0:
            res.outcome = GuiOutcome.ABORT
        else:
            res.product_details = req.product_list.product_details_list[0]
            res.outcome.outcome = GuiOutcome.NAVIGATE
        return res

    def emotion_to_string(self, emotion):
        if emotion == Emotion.IDLE:
            return 'Idle'
        elif emotion == Emotion.PLEASED:
            return 'Pleased'
        elif emotion == Emotion.SEEKING:
            return 'Seeking'
        elif emotion == Emotion.CURIOUS:
            return 'Curious'
        elif emotion == Emotion.QUESTIONING:
            return 'Questioning'
        elif emotion == Emotion.EAGER:
            return 'Eager'
        elif emotion == Emotion.ASTONISHED:
            return 'Astonished'
        elif emotion == Emotion.SAD:
            return 'Sad'
        elif emotion == Emotion.DANCING:
            return 'Dancing'
        elif emotion == Emotion.SLEEPING:
            return 'Sleeping'
        elif emotion == Emotion.DELIGHTED:
            return 'Delighted'
        elif emotion == Emotion.EYES_SHUT:
            return 'Eyes shut'
        elif emotion == Emotion.EYES_WIDEN:
            return 'Eyes widen'
        elif emotion == Emotion.EYES_LEFT:
            return 'Eyes left'
        elif emotion == Emotion.EYES_RIGHT:
            return 'Eyes right'
        return 'unknown'


if __name__ == '__main__':
  rospy.init_node('fake_gui')
  FakeGui()
  rospy.spin()
