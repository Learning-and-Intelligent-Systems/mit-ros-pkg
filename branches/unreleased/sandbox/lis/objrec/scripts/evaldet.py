import os
class bounding_box:
    def __init__(this,list_of_strings):
        bounds = [float(x) for x in list_of_strings]
        this.x0 = bounds[0]
        this.x1 = bounds[1]
        this.y0 = bounds[2]
        this.y1 = bounds[3]
    def area(this):
        return (this.x1-this.x0)*(this.y1-this.y0)
    def overlap(bbox1, bbox2):
        dx = min(bbox1.x1,bbox2.x1) - max(bbox1.x0,bbox2.x0) + 1
        dy = min(bbox1.y1,bbox2.y1) - max(bbox1.y0,bbox2.y0) + 1
        has_overlap = dx>0 and dy>0
        if has_overlap:
            intersection = dx*dy
        else:
            intersection = 0
        union = bbox1.area() + bbox2.area() - intersection
        return float(intersection)/float(union)

class detection:
    def __init__(this,detection_string):
        this.detection_string = detection_string
        tokens = detection_string.split()
        if len(tokens)>0 and tokens[0] is not "":
            try:
                this.confidence = float(tokens[0])
                this.bounding_box = bounding_box(tokens[1:5])
                this.image_file = tokens[-1]
                ground_truth_file = this.image_file[:-4]+'.bbox'
                this.ground_truth_exists = os.path.isfile(ground_truth_file)
                if this.ground_truth_exists:
                    this.ground_truth_file = ground_truth_file
                    with open(this.ground_truth_file) as f:
                        lines = f.readlines()
                        this.ground_truth = bounding_box(lines[1].split())
                    this.overlap= this.ground_truth.overlap(this.bounding_box)
            except ValueError as e:
                print e.args
                print 'tokens="'+str(tokens)+'"'
                print 'detection_string="'+detection_string+'"'


def non_maximum_suppress_detections(detections,min_overlap=0.5):
    detections.sort(key=lambda d: d.confidence) #from worst to best
    non_maximum_suppressed_detections = []
    for j in range(len(detections)):
        found_overlap = False
        for k in range(j+1,len(detections)):
            if detections[j].image_file==detections[k].image_file and detections[j].bounding_box.overlap(detections[k].bounding_box)>=min_overlap:
                found_overlap = True
                break
        if not found_overlap:
            non_maximum_suppressed_detections.append(detections[j])
    non_maximum_suppressed_detections.reverse() #from best to worst
    return non_maximum_suppressed_detections    

def detections(object_name,detection_dir,warn_empty_filename_condition=lambda f: False,min_overlap=0.5):
    detections = []
    detection_files = [f for f \
                       in os.listdir(os.path.expanduser(detection_dir))\
                       if f[0:len(object_name)]==object_name]
    for filename in detection_files:
        with open(os.path.expanduser(detection_dir+'/'+filename)) as f:
            lines = f.readlines()
            if lines==[]:
                image_name = filename[-8:-4]+'.png'
                #detections.append(detection('NaN 0 0 0 0 '+image_name))
                if warn_empty_filename_condition(filename):
                    print 'no detection in file "'+detection_dir+'/'+filename+'"'
            for line in lines:
                if len(line.split())!=21:
                    print 'bad line in file "'+detection_dir+'/'+filename+'"'
                    print line
                detections.append(detection(line))
    
    return non_maximum_suppress_detections(detections,min_overlap)

def tp_fp(detections,min_overlap=0.5):
    detections.sort(key=lambda d: d.confidence,reverse=True)
    tp_count = 0.
    fp_count = 0.
    tp = []
    fp = []
    tp_detections = []
    fp_detections = []
    gt = set()
    for d in detections:
        if d.ground_truth_exists and d.ground_truth_file not in gt \
           and d.overlap>=min_overlap:
            tp_count = tp_count + 1
            gt.add(d.ground_truth_file)
            tp_detections.append(d)
        else:
            fp_count = fp_count + 1
            fp_detections.append(d)
        tp.append(tp_count)
        fp.append(fp_count)
    return (tp,fp,tp_detections,fp_detections)

def margin(detections,min_overlap=0.5):
    (tp,fp,tp_detections,fp_detections) = tp_fp(detections,min_overlap)
    tp_confidences = [d.confidence for d in tp_detections]
    if len(tp_confidences)==0:
        return float('NaN')
    min_pos = min(tp_confidences)
    fp_confidences = [d.confidence for d in fp_detections]
    if len(fp_confidences)==0:
        return float('NaN')
    max_neg = max(fp_confidences)
    return min_pos-max_neg

def average_precision(detections,min_overlap=0.5):
    (tp,fp,tp_detections,fp_detections) = tp_fp(detections,min_overlap)
    if tp[-1]==0:
        return 0.0
    recall = [float(t)/tp[-1] for t in tp]
    precision = [float(tp[j])/(fp[j]+tp[j]) for j in range(len(tp))]

    mr = [0.]+recall+[1.]
    mp = [0.]+precision+[0.]
    for j in range(len(mr)-2,-1,-1):
        mp[j]=max(mp[j],mp[j+1])
    ind = [j+1 for j in range(len(mr)-1) if mr[j]!=mr[j+1]]
    ap = sum([(mr[j]-mr[j-1])*mp[j] for j in ind])
    return ap

def leave_one_out_average_precisions(detections,min_overlap=0.5):
    image_files = list(set([d.image_file for d in detections]))
    average_precisions = []
    for j in range(len(image_files)):
        leave_out = image_files[j]
        remaining_detections = [d for d in detections if d.image_file!=leave_out]
        average_precisions.append(average_precision(remaining_detections,min_overlap))
    return average_precisions

