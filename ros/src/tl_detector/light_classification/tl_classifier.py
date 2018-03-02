from styx_msgs.msg import TrafficLight
from tlclassifier import TrafficLightClassifier

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.classifier = self.classifier = TrafficLightClassifier('./light_classification/tensor/linux_tensor0.999')
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.classifier is None:
          #path = os.path.abspath('./tensor/linux_tensor0.999')
          #rospy.logwarn("Tensor_path {0}".format(path))
          self.classifier = TrafficLightClassifier('./light_classification/tensor/linux_tensor0.999')
        
        lights = (TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN)
        return lights[self.classifier.classifyImage(image)]
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
