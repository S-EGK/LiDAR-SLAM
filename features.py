import math
import numpy as np
from scipy.odr import *
from fractions import Fraction

# landmarks
Landmarks = []

class FeaturesDetection:
    def __init__(self):
        # variables
        self.epsilon = 10
        self.delta =501
        self.snum = 6
        self.pmin = 20
        self.gmax = 20
        self.seed_segments = []
        self.line_segments = []
        self.laser_points = []
        self.line_params = None
        self.np = len(self.laser_points) - 1
        self.lmin = 20  # min len of a line segment
        self.lr = 0     # real length of a line segment
        self.pr = 0     # number of laser points contained in a line segment
        self.features = []

    # euclidian distance from point1 to point2
    @staticmethod
    def dist_point2point(point1, point2):
        px = (point1[0] - point2[0]) ** 2
        py = (point1[1] - point2[1]) ** 2
        return math.sqrt(px + py)
    
    # distance from point to line written in the general form
    def dist_point2line(self, params, point):
        A, B, C = params
        distance = abs(A * point[0] + B * point[1] + C) / math.sqrt(A ** 2 + B ** 2)
        return distance 
    
    # extract two points from a line equation under the slope intercepts form
    def line2points(self, m, b):
        x = 5
        y = m*x + b
        x2 = 2000
        y2 = m*x2 + b
        return [(x,y), (x2,y2)]
    
    # general form to slope intercepts form
    def lineform_g2si(self, A, b, C):
        m = -A/b
        b = -C/b
        return m, b
    
    # slope intercepts form to general form
    def lineform_si2g(self, m, b):
        A, b, C = -m, 1, -b
        if A < 0:
            A, b, C = -A, -b, -C

        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        A = A*lcm
        b = b*lcm
        C = C*lcm
        return A, b, C
    
    def line_intersect_general(self, params1, params2):
        A1, b1, C1 = params1
        A2, b2, C2 = params2
        x = (b2*C1 - b1*C2) / (A2*b1 - A1*b2 + np.finfo(np.float64).eps)
        y = (A1*C2 - A2*C1) / (A2*b1 - A1*b2 + np.finfo(np.float64).eps)
        return (x,y)
    
    def points_2line(self, point1, point2):
        m, b = 0, 0
        if point2[0] == point1[0]:
            pass
        else:
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b
    
    def projection_point2line(self, point, m, b):
        x, y = point
        m2 = -1/m
        c2 = y - m2*x
        intersection_x = -(b-c2)/(m-m2)
        intersection_y = m2*intersection_x + c2
        return intersection_x, intersection_y
    
    def ad2pos(self, distance, angle, robot_position):
        x = robot_position[0] + distance * math.cos(angle)
        y = robot_position[1] - distance * math.sin(angle)
        return (int(x), int(y))
    
    def laser_points_set(self, data):
        self.laser_points = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.ad2pos(point[0], point[1], point[2])
                self.laser_points.append([coordinates, point[1]])
        self.np = len(self.laser_points) - 1
    
    # define a funcrtion (quadratic) to fit the laser points
    def linear_func(self, p, x):
        m, b = p
        return m*x + b
    
    def odr_fit(self, laser_points):
        x = [point[0][0] for point in laser_points]
        y = [point[0][1] for point in laser_points]

        # create a model for fitting
        model = Model(self.linear_func)
        # create a RealData object using our initiated data from above
        data = RealData(x, y)
        # set up odr with the model and data
        odr = ODR(data, model, beta0=[0., 0.])
        # run the regression
        out = odr.run()
        m, b = out.beta
        
        return m, b
    
    def predict_point(self, line_params, sensed_point, robot_pos):
        m, b = self.points_2line(robot_pos, sensed_point)
        params1 = self.lineform_si2g(m, b)
        predx, predy = self.line_intersect_general(line_params, params1)
        return predx, predy

    def seed_segment_detection(self, robot_position, break_point_ind):
        flag = True
        self.np = max(0, self.np)
        self.seed_segments = []
        for i in range(break_point_ind, (self.np - self.pmin)):
            predicyed_points_to_draw = []
            j = i + self.snum
            m, c = self.odr_fit(self.laser_points[i:j])

            params = self.lineform_si2g(m, c)

            for k in range(i, j):
                predicted_point = self.predict_point(params, self.laser_points[k][0], robot_position)
                predicyed_points_to_draw.append((predicted_point))
                d1 = self.dist_point2point(predicted_point, self.laser_points[k][0])

                if d1 > self.delta:
                    flag = False
                    break
                d2 = self.dist_point2line(params, predicted_point)

                if d2 > self.epsilon:
                    flag = False
                    break
            if flag:
                self.line_params = params
                return [self.laser_points[i:j], predicyed_points_to_draw, (i, j)]
        return False
    
    def seed_segment_growing(self, indices, break_point):
        line_eq = self.line_params
        i, j = indices
        # beginning and final points in the line segment
        PB, PF = max(break_point, i-1), min(j+1, len(self.laser_points)-1)

        while self.dist_point2line(line_eq, self.laser_points[PF][0]) < self.epsilon:
            if PF > self.np-1:
                break
            else:
                m, b = self.odr_fit(self.laser_points[PB:PF])
                line_eq = self.lineform_si2g(m, b)

                POINT = self.laser_points[PF][0]

            PF = PF + 1
            NEXTPOINT = self.laser_points[PF][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.gmax:
                break

        PF = PF - 1

        while self.dist_point2line(line_eq, self.laser_points[PB][0]) < self.epsilon:
            if PB < break_point:
                break
            else:
                m, b = self.odr_fit(self.laser_points[PB:PF])
                line_eq = self.lineform_si2g(m, b)
                POINT = self.laser_points[PB][0]

            PB = PB - 1
            NEXTPOINT = self.laser_points[PB][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.gmax:
                break
        PB = PB + 1

        LR = self.dist_point2point(self.laser_points[PB][0], self.laser_points[PF][0])
        PR = len(self.laser_points[PB:PF])

        if (LR >= self.lmin) and (PR >= self.pmin):
            self.line_params = line_eq
            m, b = self.lineform_g2si(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line2points(m, b)
            self.line_segments.append((self.laser_points[PB+1][0], self.laser_points[PF-1][0]))
            return [self.laser_points[PB:PF], self.two_points, (self.laser_points[PB+1][0], self.laser_points[PF-1][0]), PF, line_eq, (m,b)]
        else:
            return False
        
    def linefeats2point(self):
        new_rep = []    # new representation of the features

        for feature in self.features:
            projection = self.projection_point2line((0,0), feature[0][1], feature[0][1])
            new_rep.append([feature[0], feature[1], projection])
        return new_rep
    
def lankmark_association(landmarks):
    thresh = 10
    for l in landmarks:
        flag = False
        for i, Landmark in enumerate(Landmarks):
            dist = FeaturesDetection.dist_point2point(l[2], Landmark[2])
            if dist < thresh:
                if not is_overlap(l[1], Landmark[1]):
                    continue
                else:
                    Landmarks.pop(i)
                    Landmarks.insert(i, l)
                    flag = True
                    break
        if not flag:
            Landmarks.append(l)

def is_overlap(seg1, seg2):
    length1 = FeaturesDetection.dist_point2point(seg1[0], seg1[1])
    length2 = FeaturesDetection.dist_point2point(seg2[0], seg2[1])
    center1 = ((seg1[0][0] + seg1[1][0])/2, (seg1[0][1] + seg1[1][1])/2)
    center2 = ((seg2[0][0] + seg2[1][0])/2, (seg2[0][1] + seg2[1][1])/2)
    dist = FeaturesDetection.dist_point2point(center1, center2)
    if dist > (length1 + length2)/2:
        return False
    else:
        return True
