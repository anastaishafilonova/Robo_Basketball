import sys, os, math, time, cv2
import numpy as np
from pathlib import Path
from multiprocessing import Process, Value
from scipy.special import comb
import Roki

sys.path.append('..')

from old.Vision.vision_pipeline                             import RokiCamera
from roki_modules.lowlevel.Hardware                         import Hardware
from roki_modules.lowlevel.JointCommunicator                import JointCommunicator
from roki_modules.engines.walk.StarkitIKWalkController      import StarkitIKWalkController
from services.Localisation.class_Local import *
from old.Vision.reload import KondoCameraSensor
import roki_modules.lowlevel.class_stm_channel         as stm

class Throw:
    def __init__(self):
        self.hw = Hardware()
        self.controller = StarkitIKWalkController()
        self.communicator = JointCommunicator()
        self.Roki = Roki
        self.stm_channel = stm.STM_channel(self)
        self.rcb = self.stm_channel.rcb
        
        # Point of trajectory in pixels - (target_x, target_y)
        self.target_x =  Value("d", 0.0)
        self.target_y =  Value("d", 0.0)

        # Process for Vision Pipeline
        # self.cam_proc = Process(target=self.process_vision, args=(self.target_x, self.target_y))

    def process_vision(self, target_x, target_y):
        camera = KondoCameraSensor("../old/Vision/camera_params.json")
        
        while True:
            img = camera.snapshot().img
            target_x.value, target_y.value = 0., 0.
   
            time.sleep(0.4)

    def moveServo(self, frames, angle, name):
        servoData =  self.Roki.Rcb4.ServoData()

        if name == "shoulder_roll":
            servoData.Id, servoData.Sio = 2, 1
        elif name == "shoulder_pitch":
            servoData.Id, servoData.Sio = 1, 1
        elif name == "claw":
            servoData.Id, servoData.Sio = 14, 2
        elif name == "wrist_yaw":
            servoData.Id, servoData.Sio = 11, 1
        else:
            print("I dont know this motors")

        servoData.Data = 7500 + int(angle * 1698)
        a = self.rcb.setServoPos([servoData], frames)

    def solve_for_w(self):
        # Формируем коэффициенты кубического уравнения k1*x^3 + k2*x^2 + k3*x - L = 0
        coeffs = [
            self.k1,  # x^3
            self.k2,  # x^2
            self.k3,  # x
            -self.distance  # Свободный член
        ]

        # Находим корни кубического уравнения
        roots = np.roots(coeffs)

        # Оставляем только вещественные корни (поскольку w^2 >= 0)
        real_roots = [r.real for r in roots if np.isreal(r) and r.real >= 0]

        # Находим w как sqrt(x)
        w_values = [np.sqrt(x) for x in real_roots]

        return w_values
    
    
    def run_throw(self, distance):
        frames = 40
        self.moveServo(frames, 1.37, "shoulder_roll")
        self.moveServo(frames, 1.57, "shoulder_pitch")
        self.moveServo(frames, 2.8, "shoulder_roll")
        time.sleep(1.5)
        self.moveServo(frames, -0.9, "claw")
        # time.sleep(2.5)
        self.moveServo(frames, -0.05, "wrist_yaw")
        self.moveServo(frames, 2.1, "shoulder_pitch")
        time.sleep(2.5)

        #Distance of throw
        self.distance = distance

        #Coeffs of line L = k1*w^6 + k2*w^4 + k3*w^2
        self.k1 = -0.000432400716
        self.k2 = 0.0244057355
        self.k3 = 0.892415612

        omega = self.solve_for_w()
        print(omega)
        if omega[0] > 7:
            omega = omega[1]
        velocity_frames = 25
        angle = round(omega * velocity_frames)
        while angle > 110:
            velocity_frames -= 1
            angle = round(omega * velocity_frames)
        while angle < 90:
            velocity_frames += 1
            angle = round(omega * velocity_frames)
        print("distance =", distance, "omega = ", omega, "velocity_frames = ", velocity_frames, "angle = ", angle)
        self.moveServo(velocity_frames, (90 - angle) / 180 * math.pi, "shoulder_pitch")
        self.moveServo(frames, -1, "claw")
        time.sleep(3)

        while True: 
            break
            # TODO This place for logic throw ball

    def __del__(self):
        pass
        # self.stopFlag = True  
        # self.cam_proc.join()

if __name__=="__main__":
    throw = Throw()
    distance = int(sys.argv[1])
    if 10 <= distance <= 60:
        throw.run_throw(distance)
