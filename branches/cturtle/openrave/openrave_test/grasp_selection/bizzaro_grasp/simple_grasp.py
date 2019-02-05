class SimpleGrasp:
    def __init__(self,Tgrasp,gripper_angle):
        self.Tgrasp = Tgrasp
        self.gripper_angle = gripper_angle
        self.face_pair = None