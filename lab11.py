"""
Tool to plot a given vector graphics file (YAML).

Run "python3 lab11_plot.py lab11_img1.yaml" to plot the
bezier curves and lines defined in lab11_img1.yaml.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import math
import odometry
import pd_controller2
import pid_controller
from pyCreate2 import create2


import lab11_image

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

        self.tracker = factory.create_tracker(1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)

        self.nodes = []

        self.alpha = 0.9999
        self.beta = 1 - self.alpha
        # read file
        self.img = lab11_image.VectorImage("lab11_img1.yaml")
        self.penholder = factory.create_pen_holder()
        
        self.offset = 0.18

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # This is an example on how to check if a certain point in the given map is an obstacle
        # Each pixel corresponds to 1cm
            # setup figure
        fig = plt.figure()
        ax = fig.add_subplot(111,aspect='equal')

        
        # draw all bezier curves
        for path in self.img.paths:
            ts = np.linspace(0, 1.0, 100)
            result = np.empty((0,3))
            for i in range(0, path.num_segments()):
                for t in ts[:-2]:
                    s = path.eval(i, t)
                    result = np.vstack([result, s])

            ax.plot(result[:,0], result[:,1], path.color)

        # draw lines
        for line in self.img.lines:
            plt.plot([line.u[0], line.v[0]], [line.u[1], line.v[1]], line.color)

        # # show the output
        # plt.show()

        reOrderedLines = []
        reOrderedLines.append(self.img.lines[0])
        reOrderedLines.append(self.img.lines[1])
        reOrderedLines.append(self.img.lines[2])
        reOrderedLines.append(self.img.lines[5])
        reOrderedLines.append(self.img.lines[6])
        reOrderedLines.append(self.img.lines[7])
        reOrderedLines.append(self.img.lines[3])
        reOrderedLines.append(self.img.lines[4])

        count = 0
        for line in reOrderedLines:
            offset = self.offset
            if(count == 0):
                waypoints = list(getEquidistantPoints((line.u[0] - offset,line.u[1]), (line.v[0] - offset,line.v[1]), 20))
            if(count == 1):
                waypoints = list(getEquidistantPoints((line.v[0] + offset,line.v[1]), (line.u[0] + offset,line.u[1]), 20))
            if(count == 2):
                waypoints = list(getEquidistantPoints((line.v[0],line.v[1]-offset), (line.u[0],line.u[1] - offset), 20))
            if(count == 6):
                waypoints = list(getEquidistantPoints((line.u[0] - offset * math.cos(math.pi/4),line.u[1] + offset * math.sin(math.pi/4)), (line.v[0] - offset * math.cos(math.pi/4),line.v[1] + offset * math.sin(math.pi/4)), 20))
            if(count == 7):
                waypoints = list(getEquidistantPoints((line.u[0] + offset * math.cos(math.pi/4),line.u[1] + offset * math.sin(math.pi/4)), (line.v[0] + offset * math.cos(math.pi/4),line.v[1] + offset * math.sin(math.pi/4)), 20))
            if(count == 3):
                waypoints = list(getEquidistantPoints((line.u[0] - offset,line.u[1]), (line.v[0] - offset,line.v[1]), 20))
            if(count == 4):
                waypoints = list(getEquidistantPoints((line.u[0], line.u[1] + offset), (line.v[0],line.v[1] + offset), 20))
            if(count == 5):
                waypoints = list(getEquidistantPoints((line.u[0] + offset,line.u[1]), (line.v[0] + offset,line.v[1]), 20))

            if(line.color == "black"):
                self.penholder.set_color(0.0, 0.0, 0.0)
            elif(line.color == "blue"):
                self.penholder.set_color(0.0, 0.0, 1.0)
            elif(line.color == "red"):
                self.penholder.set_color(1.0, 0.0, 0.0)
            elif(line.color == "green"):
                self.penholder.set_color(0.0, 1.0, 0.0)
            




            goal_x = waypoints[0][0]
            goal_y = waypoints[0][1]

            while True:
                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        r = self.tracker.query()
                        if r is not None:
                            trackx = r["position"]["x"]
                            tracky = r["position"]["y"]
                            trackz = r["position"]["z"]
                            yaw = r["orientation"]["y"]

                            goal_theta = math.atan2(goal_y - self.alpha * self.odometry.y + self.beta * tracky, goal_x - self.alpha*self.odometry.x + self.beta * trackx)
                            output_theta = self.pidTheta.update(self.alpha * self.odometry.theta + self.beta * yaw, goal_theta, self.time.time())
                        else: 
                            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                        

                        # base version:
                        # self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                        # improved version 1: stop if close enough to goal
                        # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                        # if distance < 0.1:
                        #     break
                        # improved version 2: fuse with velocity controller
                        distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

                        if(distance < 0.05):
                            break

                        output_distance = self.pidDistance.update(0, distance, self.time.time())
                        self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

            if(count == 0):
                goal_theta = math.pi/2
            if(count == 1):
                goal_theta = 0-math.pi/2
            if(count == 2):
                goal_theta = math.pi
            if(count == 3):
                goal_theta = math.pi/2
            if(count == 4):
                goal_theta = 0
            if(count == 5):
                goal_theta = 0-math.pi/2
            if(count == 6):
                goal_theta = math.pi/4
            if(count == 7):
                goal_theta = 0-math.pi/4



            end_time = self.time.time() + 3
            while self.time.time() < end_time:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(output_theta), int(-output_theta))



            for waypoint in waypoints:
                if(waypoint == waypoints[1]):
                    self.penholder.go_to(-0.025)
                
                goal_x = waypoint[0]
                goal_y = waypoint[1]
                base_speed = 140
                result = np.empty((0,5))
                # end_time = self.time.time() + 10
                # while self.time.time() < end_time:
                while True:
                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        r = self.tracker.query()
                        if r is not None:
                            trackx = r["position"]["x"]
                            tracky = r["position"]["y"]
                            trackz = r["position"]["z"]
                            yaw = r["orientation"]["y"]
                            goal_theta = math.atan2(goal_y - self.alpha * self.odometry.y + self.beta * tracky, goal_x - self.alpha*self.odometry.x + self.beta * trackx)
                            output_theta = self.pidTheta.update(self.alpha * self.odometry.theta + self.beta * yaw, goal_theta, self.time.time())
                        else: 
                            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                        # goal_theta = math.atan2(goal_y - self.alpha * self.odometry.y + self.beta * tracky, goal_x - self.alpha*self.odometry.x + self.beta * trackx)
                        # theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                        # output_theta = self.pidTheta.update(self.alpha * self.odometry.theta + self.beta * yaw, goal_theta, self.time.time())

                        # base version:
                        # self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                        # improved version 1: stop if close enough to goal
                        # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                        # if distance < 0.1:
                        #     break
                        # improved version 2: fuse with velocity controller
                        distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

                        if(distance < 0.05):
                            break

                        output_distance = self.pidDistance.update(0, distance, self.time.time())
                        self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
            self.penholder.go_to(0.0)
            count = count + 1


            
        self.penholder.go_to(0.0)
        for path in self.img.paths:
            ts = np.linspace(0, 1.0, 100)
            result = np.empty((0,3))
            for i in range(0, path.num_segments()):
                for t in ts[:-2]:
                    s = path.eval(i, t)
                    result = np.vstack([result, s])


                    for waypoint in s:
                        if(waypoint == s[1]):
                            self.penholder.go_to(-0.025)
                        
                        goal_x = waypoint[0]
                        goal_y = waypoint[1]
                        base_speed = 140
                        result = np.empty((0,5))
                        # end_time = self.time.time() + 10
                        # while self.time.time() < end_time:
                        while True:
                            state = self.create.update()
                            if state is not None:
                                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                                # goal_theta = math.atan2(goal_y - self.alpha * self.odometry.y + self.beta * tracky, goal_x - self.alpha*self.odometry.x + self.beta * trackx)
                                # theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                                # output_theta = self.pidTheta.update(self.alpha * self.odometry.theta + self.beta * yaw, goal_theta, self.time.time())

                                # base version:
                                # self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                                # improved version 1: stop if close enough to goal
                                # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                                # if distance < 0.1:
                                #     break
                                # improved version 2: fuse with velocity controller
                                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

                                if(distance < 0.05):
                                    break

                                output_distance = self.pidDistance.update(0, distance, self.time.time())
                                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                    
                    


def getEquidistantPoints(p1, p2, parts):
    return zip(np.linspace(p1[0], p2[0], parts+1), np.linspace(p1[1], p2[1], parts+1))




