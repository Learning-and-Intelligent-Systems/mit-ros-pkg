class FacePair:
    def __init__(self,face1,face2):
        self.faces = [face1,face2]
        self.gripper_angle = None
        self.Target_T_center = None
        self.Target_dir = None
        self.mins = None
        self.maxs = None