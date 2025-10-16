#!/usr/bin/env python

import os
import rospy
from sound_play.libsoundplay import SoundClient
from robot_vision_msgs.msg import BoundingBoxes

class DeliveryDetectionNode:
  def __init__(self):
    # initializing node, subscriber and sound client object
    rospy.init_node('delivery_detection_node')
    # register a callback function upon the node is shutting down
    rospy.on_shutdown(self.cleanup)
    
    self.subscriber = rospy.Subscriber('/yolo_ros/bounding_boxes', BoundingBoxes, self.detect)
    self.soundhandler = SoundClient(blocking=True)
    self.processing = False

    # pause execution of the node for 1 second to let the sound client to connect to the sound_play server
    rospy.sleep(1)

    # stop if any sound_play processes are in the queue
    self.soundhandler.stopAll()

  def cleanup(self):
    self.soundhandler.stopAll()
    rospy.signal_shutdown('Detection process completed')

  def detect(self, msg):
    boxes = msg.bounding_boxes
    if not self.processing and len(boxes) > 1:
      self.processing = True
      
      # due to the limitation of the yolo3 object detection, a box is hardly detected as a box but 'book', 'laptop', 'cell phone', and 'tvmonitor',
      # so the conditions check whether a person and the items are detected to identify whether a person has a box to deliver.
      if self.check_if_has(boxes, 'person') and (self.check_if_has(boxes, 'book') or self.check_if_has(boxes, 'laptop') or self.check_if_has(boxes, 'cell phone') or self.check_if_has(boxes, 'tvmonitor')):
        self.subscriber.unregister()
        
        # speak out the instructions
        self.soundhandler.say('Welcome to Faculty of Computer Science and Information Tectnology, University Ma-la-ya.')
        self.soundhandler.say('Please leave the parcel on the reception table. Thank you.')
        
        # pause execution of the node for 1 second
        rospy.sleep(1)
        # re-subscribe to yolo object detection topic
        #self.subscriber = rospy.Subscriber('/yolo_ros/bounding_boxes', BoundingBoxes, self.detect)
        
        # terminate this delivery detection and the yolo object detection nodes
        os.system('rosnode kill delivery_detection_node && rosnode kill yolo_ros')
      
      self.processing = False
  
  def check_if_has(self, boxes, target):
    if boxes:
      for item in boxes:
        #rospy.loginfo('Item detected: %s', item)
        if item.Class == target:
          return True
    return False 

if __name__ == '__main__':
  try:
    DeliveryDetectionNode()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

