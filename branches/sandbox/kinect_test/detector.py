
class Detection:
    """
    represents a detection in an image,
    along with its position and bounding box.
    """
    def __init__(self,(u,v),(min_u,max_u,min_v,max_v),value):
        """
        The detection is centered at pixel position (u,v), and
        it has a bounding box with the pixels
        min_u <= u < max_u and
        min_v <= v < max_v

        Higher values mean a better (more likely) detection.

        A detection may be labeled as True or False if
        its actual value is known.
        """
        self.position = (u,v)
        self.bounding_box = (min_u,max_u,min_v,max_v)
        self.value = value
    def __lt__(self,other):
        """
        Sorting a list of detections causes the highest-value detections
        to appear first in the list.
        """
        return self.value>other.value
    def __repr__(self):
        """
        Make the detection print out nicely on the shell.
        """
        return 'Detection(' +str(self.position)+','\
                            +str(self.bounding_box)+','\
                            +str(self.value)+')'


###this is just an example to show sorting and printing Detections
##detections = [];
##for v in range(480):
##    for u in range(640):
##        detections.append(Detection((u,v),(u-10,u+10,v-10,v+10),u*480+v))
##
##detections.sort()
##
##count = 0
##for d in detections:
##    count += 1
##    if count==10:
##        break
##    print d
            

class Detector:
    def detect(self,image):
        """
        Returns an iterable over detections for all pixels in the image.

        >>> import cv
        >>> width = 640
        >>> height = 480
        >>> image = cv.CreateMat(height,width,cv.CV_8UC3)
        ...

        fill in the image from some image source

        ...
        >>> d = Detector()
        >>> detections = d.detect(image)
        >>> print detections[0]
        Detection((324,298),(320,330,290,310),98.3)
        ...
        the first detection always has the highest value.
        """
        return detect_in_rectangle(image,(0,image.cols,0,image.rows))

    def detect_in_rectangle(self,image,(min_x,max_x,min_y,max_y)):
        """
        Returns an iterable over detections within a bounding box in the image.

        Should run the detector over all the pixels (u,v) for
        min_u <= u < max_u and
        min_v <= v < max_v
        """
        pass #call your sphere detector here


class ROC_curve:
    def __init__(detector,positive_label_file,bounding_box,negative_image_file):
        """
        Takes a Detector,
              a file with labeled positive examples,
              a bounding-box specifying the tolerance to localization error, and
              a file with negative examples
        creates an ROC_curve that can be queried and graphed.      
        """
        pass #fill this definition in
    def equal_error_TPR(self):
        """
        Calculates the equal-error ROC point and returns the true positive rate
        at that point.  The corresponding false positive rate is
        equal to 1.0-TPR.
        """
        pass #fill this definition in
    def FPR(self,TPR):
        """
        Calculates the false-positive rate for a given true-positive rate.
        """
        pass #fill this definition in
    def FP_per_image(self,TPR)
        """
        Calculates the average number of false-positives per image
        based on the average image size for a given true-positive rate.
        """
        pass #fill this definition in
    def plot(self):
        pass #either print an output that can be easily imported into excel,
             #OR make your own ROC curve plot
