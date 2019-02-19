#!/usr/bin/env python

import keras, numpy, pdb # Import Keras, Tensorflow, and NumPY (API calls, low-level deep learning software, and math package)
import tensorflow as tf
from keras.applications import mobilenet_v2 # A mobilenet is useful because our VMs are limited computationally -- this network is designed to run on a mobile device, like a cell phone
from keras.utils.vis_utils import plot_model # We use this to print the model description
from keras.preprocessing.image import load_img, img_to_array
from keras.applications.mobilenet_v2 import preprocess_input, decode_predictions
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class chirashi():

    def __init__(self):
        self.graph = tf.get_default_graph()
        self.model = mobilenet_v2.MobileNetV2(weights='imagenet') # The first time you run this script, Google's pretrained network weights are downloaded. This may take a few minutes so don't worry if it seems to "stall out"
        print(self.model.summary()) # Print a summary of the model's architecture
        plot_model(self.model, to_file='mobilenet.png') # This creates a .png showing the model's architecture

        #pdb.set_trace() # You may want to use PDB to step through code here
        self.bridge = CvBridge()
        self.image_sub=rospy.Subscriber('image_topic_2',Image,self.image_callback)
        self.pub = rospy.Publisher("label",String,queue_size=10)
        #pdb.set_trace()
        

    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(3)

        x = 224
        y = 224
        res = cv2.resize(self.cv_image,(x, y), interpolation = cv2.INTER_CUBIC)
        image = img_to_array(res)

        # The model expects data in a particular format: (Samples, rows, columns, channels)
        # We only have one file, so use reshape to add the sample dimension
        image = image.reshape((1, x, y, 3))

        # Many models require color resampling, e.g. subtracting the average R, G, and B values from the image before classification
        # Keras handles this automatically using 
       
        image = preprocess_input(image)

        # Finally, make a prediction
        with self.graph.as_default():
            model_output = self.model.predict(image)

        # The output must be decoded
        possible_labels = decode_predictions(model_output)

        # Up to you: select the highest probability prediction
        #rospy.loginfo("{}".format(possible_labels))
        
        real_label = possible_labels[0][0]

        label_text = real_label[0]
        label_probability = real_label[1]
    
        rospy.loginfo("{} {}".format(label_text, label_probability))
        self.pub.publish("{} {}".format(label_text, label_probability))


def main():
    potato = chirashi()
    rospy.init_node("Classifier")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()