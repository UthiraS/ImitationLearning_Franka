#!/usr/bin/env python3
import numpy as np
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint
from shapely.geometry import Polygon, LineString, MultiPoint
from shapely.geometry import Point as sPoint
import scipy.optimize as opt
from math import*
import time
import os
import datetime
import queue
from object_definitions import get_OBJ
from object_geom_func import generate_map, translate_point, rotate_vector, place_finger, pivot_finger, get_finger_param, find_contact_center
from object_surf_func import plot_3d_object, get_neighbors, unfold_surface, unfold_object
from finger_params import*
from experiments import get_experimental_setup

def solve_t1_slide_left(variables, coords):
    solt2 = variables[0]
    eqn = coords[1]*cos(solt2) - coords[0]*sin(solt2) + FINGER_WIDTH
    return eqn

def calculate_th1(th2, d2):
    x_coord = PALM_WIDTH + (d2 + OBJECT_LENGTH / 2.0) * np.cos(th2) - (OBJECT_WIDTH/2.0 + FINGER_WIDTH) * np.sin(th2)
    y_coord = (d2 + OBJECT_LENGTH / 2.0) * np.sin(th2) + (OBJECT_WIDTH / 2.0 + FINGER_WIDTH) * np.cos(th2)
    n = len(X_vertices)
    R = np.array([[cos(-GAMMA + th2), -sin(-GAMMA + th2), x_coord],
                  [sin(-GAMMA + th2), cos(-GAMMA + th2), y_coord],
                  [0, 0, 1]])
    coords = np.dot(R, np.concatenate(([X_vertices], [Y_vertices], np.ones((1, n)))))

    beta = -9999
    contact_left = -1

    for i in range(n):
        initial_guess_t1 = np.pi / 2
        solution = opt.fsolve(solve_t1_slide_left, initial_guess_t1, args=coords[:, i], full_output=True)
        if (solution[2] == 1 and solution[0] > 0 and solution[0] < np.pi and solution[0] > beta):
            beta = solution[0]
            contact_left = i

    if beta == -9999:
        th1 = None
    else:
        th1 = beta[0]

    d1 = sqrt((coords[0, contact_left] - PALM_WIDTH) ** 2 + coords[1, contact_left] ** 2 - FINGER_WIDTH** 2)

    return th1

def solve_t2_slide_right(variables, coords):
    solt2 = variables[0]
    eqn = coords[1]*cos(solt2) - (coords[0] - PALM_WIDTH)*sin(solt2) - FINGER_WIDTH
    return eqn

def calculate_th2(th1, d1):
    x_coord = (d1 + OBJECT_LENGTH/2.0) * np.cos(th1) + (FINGER_WIDTH + OBJECT_WIDTH/2) * np.sin(th1)
    y_coord = (d1 + OBJECT_LENGTH/2.0) * np.sin(th1) - (FINGER_WIDTH + OBJECT_WIDTH/2) * np.cos(th1)
    n = len(X_vertices)
    R = np.array([[cos(-GAMMA + th1), -sin(-GAMMA + th1), x_coord], [sin(-GAMMA + th1), cos(-GAMMA + th1), y_coord], [0, 0, 1]])
    coords = np.dot(R, np.concatenate(([X_vertices], [Y_vertices], np.ones((1, n)))))

    beta = 99999
    contact_right = -1

    for i in range(n):
        initial_guess_t2 = np.pi / 2
        solution = opt.fsolve(solve_t2_slide_right, initial_guess_t2, args=coords[:, i], full_output=True)
        if (solution[2] == 1 and solution[0] > 0 and solution[0] < np.pi and solution[0] < beta):
            beta = solution[0]
            contact_right = i

    if beta == 99999:
        th2 = None
    else:
        th2 = beta[0]
    d2 = sqrt((coords[0, contact_right] - PALM_WIDTH) ** 2 + coords[1, contact_right] ** 2 - FINGER_WIDTH ** 2)

    return th2

def action_right_equations(variables) :
    (th1,th2) = variables
    eqn1 = FINGER_WIDTH*sin(th1)+FINGER_WIDTH*sin(th2)+left_position * cos(th1) + OBJECT_WIDTH * sin(th1) - PALM_WIDTH - right_position * cos(th2)
    eqn2 =-FINGER_WIDTH*cos(th1)-FINGER_WIDTH*cos(th2)+left_position * sin(th1) - OBJECT_WIDTH * cos(th1) - right_position * sin(th2)
    return [eqn1, eqn2]

def action_left_equations(variables) :
    (th1, th2) = variables
    eqn1 = FINGER_WIDTH * sin(th1) + FINGER_WIDTH * sin(th2) + left_position * cos(th1) + OBJECT_WIDTH * sin(th2) - PALM_WIDTH - right_position * cos(th2)
    eqn2 = -FINGER_WIDTH * cos(th1) - FINGER_WIDTH * cos(th2) + left_position * sin(th1) - OBJECT_WIDTH * cos(th2) - right_position * sin(th2)
    return [eqn1, eqn2]

def theta_conversion(left, right, action_name):
    """
    Given d_l, d_r and the manipulation primitive return the finger angle/angles

    Parameters
    ----------
    left: float
        d_l
    right: float
        d_r
    action_name: string
        manipulation primitive
    Returns
    ----------
    solution: tuple or float
        th1 and th2 for sliding actions or corresponding finger angle for rotation actions
    """
    global left_position
    global right_position

    left_position =left
    right_position=right

    # Solve for finger angles given a sliding action
    # Slide on right finger
    if (action_name == "Slide right up" or action_name == "Slide right down"):
        for i in range(31):
            initial_guess=(i/10.0,i/10.0)
            solution = opt.fsolve(action_right_equations, initial_guess, full_output=True)
            if solution[2]==1 and solution[0][0]>0 and solution[0][0]<np.pi and solution[0][1]<np.pi and solution[0][1]>0:
                return solution[0]

        return (None,None)
    # Slide on left finger
    elif (action_name == "Slide left up" or action_name == "Slide left down"):
        for i in range(31):
            initial_guess=(i/10.0,i/10.0)
            solution = opt.fsolve(action_left_equations, initial_guess, full_output=True)
            if solution[2] == 1 and solution[0][0] > 0 and solution[0][0] < np.pi and solution[0][1] < np.pi and solution[0][1] > 0:
                return solution[0]

        return (None,None)

    # Solve for finger angle given the rotation actions
    # Counter clockwise: Left finger angle
    elif (action_name=="Rotate ccw"):
        solution= np.pi - np.arccos((((right_position)**2 + OBJECT_WIDTH**2 - PALM_WIDTH**2 -
                                      (left_position)**2)/(2*PALM_WIDTH*(left_position))))
        return (solution)
    # Clockwise: Right finger angle
    elif (action_name=="Rotate cw"):
        solution= np.arccos(((left_position)**2 + OBJECT_WIDTH**2 -
                             (right_position)**2 - PALM_WIDTH**2)/(2*PALM_WIDTH*(right_position)))
        return (solution)

class Node:

    """
    Node(left finger, right finger, parent node, primitive resolution, primitive action)

    Node class for A* search algorithm representing finger and object states, determining and storing available actions

    Attributes
    ----------
    parent: Node
        Parent node object
    action: str
        Previous primitive that results in this node
    finger_l: tuple, (float,float,Geometry3D.Vector,Geometry3D.Vector)
        A tuple that stores the finger state given as (left d,elevation z,normal vector,distal vector)
    finger_r: tuple, (float,float,Geometry3D.Vector,Geometry3D.Vector)
        A tuple that stores the finger state given as (right d,elevation z,normal vector,distal vector)
    resolution: float
        Resolution for sliding actions and moving up & down
    hand_normal: Geometry3D.Vector
        Vector normal to the manipulation plane of the hand
    finger_l_poly: Geometry3D.ConvexPolygon
        Finger geometry defined as a convex polygon - including pose information (position and orientation)
    finger_r_poly: Geometry3D.ConvexPolygon
        Finger geometry defined as a convex polygon - including pose information (position and orientation)
    surf_l: int
        Number assigned to the surface in contact with the left finger
    c_l: Geometry3D.ConvexPolygon (expected)
        Geometry definition of the intersection between left finger and object surface - currently expected to be a convex polygon (could also be a line in the future)
    surf_r: int
        Number assigned to the surface in contact with the right finger
    c_r: Geometry3D.ConvexPolygon (expected)
        Geometry definition of the intersection between right finger and object surface - currently expected to be a convex polygon (could also be a line in the future)
    pivot_angle: float
        Pivoting angle of the object at current state
    x_vertices: array-like
        Vertices of the convex-hull that covers the cross-section of the object along the hand normal
    y_vertices: array-like
        Vertices of the convex-hull that covers the cross-section of the object along the hand normal
    angle_cw_l: float
        Angle for the clockwise rotation according to left finger
    angle_ccw_l: float
        Angle for the counterclockwise rotation according to left finger
    angle_cw_r: float
        Angle for the clockwise rotation according to right finger
    angle_ccw_r: float
        Angle for the counterclockwise rotation according to right finger
    obj_left_l: float
        Object length along the surface in contact with left finger
    obj_left_h: float
        Object height along the surface in contact with left finger
    obj_right_l: float
        Object length along the surface in contact with right finger
    obj_right_r: float
        Object height along the surface in contact with right finger
    obj_w: float
        Object width - distance between finger contact surfaces
    map_l: dict
        A dictionary similar to object dictionary, where the center surface remains the same but other surfaces are modified through unfolding. Center surface is the surface in contact with the left finger.
    map_r: dict
        A dictionary similar to object dictionary, where the center surface remains the same but other surfaces are modified through unfolding. Center surface is the surface in contact with the right finger.
    surf_list_l: dict
        A dictionary that stores shapely.geometry.Polygon's for each surface of the object, modified through unfolding on left finger contact
    goal_list_l: dict
        A dictionary that stores shapely.geometry.Polygon's for each goal region on the surfaces of the object, modified through unfolding on left finger contact
    sfing_l: shapely.geometry.Polygon
        Left finger geometry defined in 2D through shapely Polygon
    surf_list_r: dict
        A dictionary that stores shapely.geometry.Polygon's for each surface of the object, modified through unfolding on right finger contact
    goal_list_r: dict
        A dictionary that stores shapely.geometry.Polygon's for each goal region on the surfaces of the object, modified through unfolding on right finger contact
    sfing_r: shapely.geometry.Polygon
        Right finger geometry defined in 2D through shapely Polygon
    contact_l: shapely.geometry.Polygon (expected)
        2D geometry definition of the intersection beteween left finger and the object - currently a polygon (could be a line in the future)
    contact_r: shapely.geometry.Polygon (expected)
        2D geometry definition of the intersection beteween right finger and the object - currently a polygon (could be a line in the future)
    obj_h: float
        Object height - generalized
    g: float
        Action cost for the node
    h: float
        Heuristic cost for the node
    f: float
        Total cost for the node
    theta: array-like
        Left and right finger position commands for the manipulation primitive (sliding or rotation)
    command: array-like
        Information regarding the execution of the manipulation primitive (finger angles for slides and rotations, other parameters for other primitives)
    available_actions: array-like
        List of strings, available manipulation primitives at this node

    Methods
    ----------
    __init__(self,finger_l,finger_r,parent=None,resolution=0.5,action=None)
        Initialization function of the node given the finger states, parent node, and leading action
    __eq__(self,other)
        Equality check between states of two nodes
    __lt__(self,other)
        Inequality check between costs of two nodes
    at_goal(self)
        Checks whether contact regions are within goal regions
    plot_state(self)
        Plots the hand-object state in 3D
    """

    def __init__(self,finger_l,finger_r,parent=None,resolution=0.5,action=None):
        """
        Initialization function of the node given the finger states, parent node, and leading action

        Parameters
        ----------
        finger_l: tuple, (float,float,Geometry3D.Vector,Geometry3D.Vector)
            A tuple that stores the finger state given as (left d,elevation z,normal vector,distal vector)
        finger_r: tuple, (float,float,Geometry3D.Vector,Geometry3D.Vector)
            A tuple that stores the finger state given as (right d,elevation z,normal vector,distal vector)
        parent: Node
            Parent node object, defaulted to None for the initial nodes
        resolution: float
            Resolution for sliding actions and moving up & down, defaulted to 0.5
        action: str
            Previous primitive that results in this node (leading action)
        """
        # Assign parameters to attributes
        self.parent = parent
        self.action = action
        self.finger_l = finger_l
        self.finger_r = finger_r
        self.resolution = resolution

        # Find hand normal as the cross product of left finger distal vector and normal vector
        self.hand_normal = finger_l[3].cross(finger_l[2])

        # Set finger geometries with pose information using place_finger function
        self.finger_l_poly = place_finger(OBJ,finger_l[0],finger_l[1],finger_l[2],finger_l[3],self.hand_normal)
        try:
            self.finger_r_poly = place_finger(OBJ,finger_r[0],finger_r[1],finger_r[2],finger_r[3],self.hand_normal)
        except:
            print(self.action)
            print(finger_l)
            print(finger_r)

        # Check to find contact
        for surf in OBJ:
            # Find the intersection (if there is) between finger polygon and each of the object surfaces
            # Currently, this intersection should be a convex polygon according to the assumptions
            check = intersection(self.finger_l_poly,OBJ[surf][0])
#           if check is not None and OBJ[surf][1].angle(finger_l[2])<1e-3:
            # Finger will be in a line contact with the neighboring surfaces
            # We also check the angle between surface and finger normals
            if OBJ[surf][1].angle(finger_l[2])<1e-3:
                self.surf_l = surf
                self.c_l = check

        # Some debugging material
        try:
#             print(self.surf_l)
            pass
        except:
            self.plot_state()
            for surf in OBJ:
                print("Finger:",self.finger_l_poly)
                print("Surf:",OBJ[surf][0])
                check = intersection(self.finger_l_poly,OBJ[surf][0])
                print(check)

        # Repeat for right finger
        for surf in OBJ:
            check = intersection(self.finger_r_poly,OBJ[surf][0])
#             if check is not None and OBJ[surf][1].angle(finger_r[2])<1e-3:
            if OBJ[surf][1].angle(finger_r[2])<1e-3:
                self.surf_r = surf
                self.c_r = check

        # Select object surface in contact with the left finger to make computations for pivoting
        pivot_side = OBJ[self.surf_l][0]

        # Select an edge of the surface
        for a in pivot_side.segments():
            # Select another edge
            for b in pivot_side.segments():
                # Continue if it is the same edge
                if a == b:
                    continue
                else:
                    # Check if they are neighboring edges (if they have an intersecting point)
                    if a.intersection(b) is not None:
                        # Pivoting angle is the outer angle between two edges
                        # (assuming that the cross section is a regular polygon we can use any two neighboring edges)
                        self.pivot_angle = Vector(a[0],a[1]).angle(Vector(b[0],b[1]))
                        break

        # To find convex hull of the cross section along hand normal
        a,b,c = self.hand_normal

        # Find rotation axis and angle for projecting all vertices on z=0 plane
        axis = self.hand_normal.cross(Vector(0,0,1))
        angle = self.hand_normal.angle(Vector(0,0,1))

        points = []
        for surf in OBJ:
            for point in OBJ[surf][0].points:
                # Required translation
                t = (-a*point.x - b*point.y - c*point.z)/(a**2 + b**2 + c**2)
                xn = point.x + a*t
                yn = point.y + b*t
                zn = point.z + c*t
                vec = rotate_vector(Vector(xn,yn,zn),axis,angle)
                x,y,z = vec
                # Add projected point to list
                points.append(sPoint(x,y))

        # Multipoint entity
        mpoints = MultiPoint(points)
        global X_vertices
        global Y_vertices

        # Vertices of the convex hull of the cross section
        self.x_vertices,self.y_vertices = mpoints.convex_hull.exterior.xy
#         self.cross_section = list(zip(x[:-1],y[:-1]))

        # Remove last elements
        X_vertices = self.x_vertices[:-1]
        Y_vertices = self.y_vertices[:-1]

        # Initialization for debugging
        self.angle_cw_l = None

        # Finding rotation angles for in-plane rotation
        # Check the neighbors of the left contact surface
        for neighbor in OBJ_neighbors[self.surf_l]:
            if OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal).length()<1e-3:
                # Exception case for the bugs in the geometry library
                try:
                    q = self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)
                except:
                    ac = self.finger_l[2].cross(OBJ[neighbor][1])*self.hand_normal/(self.finger_l[2].cross(OBJ[neighbor][1]).length()*self.hand_normal.length())
                    if ac < -1:
                        q = np.pi
                    if ac > 1:
                        q = 0

                # Rotation angle is the angle between finger normal and neighboring surface normals
                if q<1e-3:
                    self.angle_cw_l = self.finger_l[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("cw: ", self.angle_cw_l)
                if abs(q-np.pi)<1e-3:
                    self.angle_ccw_l = self.finger_l[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("ccw: ", self.angle_ccw_l)

        # Repeat for right finger
        for neighbor in OBJ_neighbors[self.surf_r]:
            if OBJ_neighbors[self.surf_r][neighbor][1].cross(self.hand_normal).length()<1e-3:
                try:
                    q = self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)
                except:
                    ac = self.finger_r[2].cross(OBJ[neighbor][1])*self.hand_normal/(self.finger_r[2].cross(OBJ[neighbor][1]).length()*self.hand_normal.length())
                    if ac < -1:
                        q = np.pi
                    if ac > 1:
                        q = 0
                if q<1e-3:
                    self.angle_cw_r = self.finger_r[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("cw: ", self.angle_cw_r)
                if abs(q-np.pi)<1e-3:
                    self.angle_ccw_r = self.finger_r[2].angle(OBJ[neighbor][1])
#                     print("Neighbor: ", neighbor)
#                     print("ccw: ", self.angle_ccw_r)


        # debugging material
        if self.angle_cw_l == None:
            print(self.surf_l)
            for neighbor in OBJ_neighbors[self.surf_l]:
                print("edge vector: ",OBJ_neighbors[self.surf_l][neighbor][1])
                print("hand normal: ",self.hand_normal)
                print("cross: ",OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal))
                if OBJ_neighbors[self.surf_l][neighbor][1].cross(self.hand_normal).length()<1e-3:
                    print(self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal))
                    if self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)<1e-3:
                        self.angle_cw_l = self.finger_l[2].angle(OBJ[neighbor][1])
                    if abs(self.finger_l[2].cross(OBJ[neighbor][1]).angle(self.hand_normal)-np.pi)<1e-3:
                        self.angle_ccw_l = self.finger_l[2].angle(OBJ[neighbor][1])
            current = self
            while current is not None:
                print(current.action)
                current = current.parent


        # exception (both fingers are on the same surface)
        if self.surf_l==self.surf_r:
            print("Error")
            current = self
            while current is not None:
                print(current.action)
                print(current.finger_l[2],current.finger_l[3])
                print(current.finger_r[2],current.finger_r[3])
                current = current.parent


        # Finding object lenght and height along the surfaces in contact with left and right fingers

        # Left finger
        for point in OBJ[self.surf_l][0].points:
            for other_point in OBJ[self.surf_l][0].points:
                if point==other_point:
                    continue
                else:
                    edge = Vector(point,other_point)
                    angle = edge.angle(finger_l[3])
                    # Distance along the distal direction is the length
                    if angle<1e-3:
                        self.objleft_l = edge.length()
                    # Distance along the direction perpendicular to distal vector is the height
                    if abs(angle- np.pi/2)<1e-3:
                        self.objleft_h = edge.length()

        # Right finger
        for point in OBJ[self.surf_r][0].points:
            for other_point in OBJ[self.surf_r][0].points:
                if point==other_point:
                    continue
                else:
                    edge = Vector(point,other_point)
                    angle = edge.angle(finger_r[3])
                    if angle<1e-3:
                        self.objright_l = edge.length()
                    if abs(angle - np.pi/2)<1e-3:
                        self.objright_h = edge.length()

        # Finding the object width
        for point in OBJ[self.surf_l][0].points:
            for other_point in OBJ[self.surf_r][0].points:
                if point==other_point:
                    continue
                else:
                    # Distance between the contact surfaces along the finger normals is the object width
                    edge = Vector(point,other_point)
                    angle = edge.angle(finger_r[2])
                    if angle <1e-3:
                        self.obj_w = edge.length()
                        break

        # Dictionaries for left and right fingers that will be used as the map for path search
        self.map_l = unfold_object(self.surf_l,OBJ,OBJ_neighbors)
        self.map_r = unfold_object(self.surf_r,OBJ,OBJ_neighbors)

        # Converting the map, goal regions, and fingers to shapely.geometry items for further calculations (heuristics)
        self.surf_list_l,self.goal_list_l,self.sfing_l = generate_map(self.map_l,self.surf_l,self.finger_l_poly)
        self.surf_list_r,self.goal_list_r,self.sfing_r = generate_map(self.map_r,self.surf_r,self.finger_r_poly)

        # Contact regions (shapely)
        self.contact_l = self.surf_list_l[self.surf_l].intersection(self.sfing_l)
        self.contact_r = self.surf_list_r[self.surf_r].intersection(self.sfing_r)

        # global variables will be used in kinematics equations
        global GAMMA
        global OBJECT_LENGTH
        global OBJECT_WIDTH

        # Assuming width and height will be uniform
        OBJECT_WIDTH = self.obj_w
        self.obj_h = self.objright_h

        # Initialize all costs as 0
        self.g = 0
        self.h = 0
        self.f = 0

        # Initialize finger position commands
        self.theta = [np.pi/2,np.pi/2]

        # Initialize primitive command
        self.command = []

        # List of all manipulation primitives
        action_list = ["Slide left down","Slide left up","Slide right down","Slide right up",
                       "Rotate ccw","Rotate cw","Move down","Move up","Pivot"]

        # Initialize a list of available actions
        self.available_actions = list()

        # Loop through the primitives to eliminate the unavailable ones
        for action in action_list:
            # There will not be any primitives after "Pivot"
            if self.action is not None and self.action == "Pivot":
                continue

            # Sliding primitives

            if action == "Slide left down":
                # Transform the state parameters according to the selected primitive
                d_l = self.finger_l[0] - resolution
                d_r = self.finger_r[0]
                # Check state-space limits
                if d_l > FINGER_END or d_l < FINGER_START:
                    continue
                else:
                    # Find solutions for finger angles
                    sol=theta_conversion(d_l, d_r, action)
                    # Assign length and gamma for kinematics
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    # Check workspace limits
                    TH2_MAX = calculate_th2(TH1_MAX, d_l)
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    # Check workspace limits
                    TH1_MIN = calculate_th1(TH2_MIN, d_r)
                    th1=sol[0]
                    th2 = sol[1]
                    if th1==None or th2==None:
                        continue
                    else:
                        if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                            pass
                        else:
                            continue
                    # Append primitive to available actions if all checks are passed
                    self.available_actions.append(action)

            # Repeat for other sliding primitives
            elif action == "Slide left up":
                d_l = self.finger_l[0] + resolution
                d_r = self.finger_r[0]
                if d_l > FINGER_END or d_l < FINGER_START:
                    continue
                else:
                    sol=theta_conversion(d_l, d_r, action)
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    TH2_MAX = calculate_th2(TH1_MAX, d_l)
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    TH1_MIN = calculate_th1(TH2_MIN, d_r)
                    th1=sol[0]
                    th2 = sol[1]
                    if th1==None or th2==None:
                        continue
                    else:
                        if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                            pass
                        else:
                            continue
                    self.available_actions.append(action)

            elif action == "Slide right down":
                d_r = self.finger_r[0] - resolution
                d_l = self.finger_l[0]
                if d_r > FINGER_END or d_r < FINGER_START:
                    continue
                else:
                    sol=theta_conversion(d_l, d_r, action)
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    TH2_MAX = calculate_th2(TH1_MAX, d_l)
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    TH1_MIN = calculate_th1(TH2_MIN, d_r)
                    th1=sol[0]
                    th2 = sol[1]
                    if th1==None or th2==None:
                        continue
                    else:
                        if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                            pass
                        else:
                            continue
                    self.available_actions.append(action)

            elif action == "Slide right up":
                d_r = self.finger_r[0] + resolution
                d_l = self.finger_l[0]
                if d_r > FINGER_END or d_r < FINGER_START:
                    continue
                else:
                    sol=theta_conversion(d_l, d_r, action)
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    TH2_MAX = calculate_th2(TH1_MAX, d_l)
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    TH1_MIN = calculate_th1(TH2_MIN, d_r)
                    th1=sol[0]
                    th2 = sol[1]
                    if th1==None or th2==None:
                        continue
                    else:
                        if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                            pass
                        else:
                            continue
                    self.available_actions.append(action)

            # Rotation primitives

            elif action == "Rotate ccw":
                # Get the rotation angle (Assuming uniform)
                rot_angle = self.angle_ccw_l
                # Transform finger states (including distal and normal vectors, contact faces, d_l and d_r) according to the selected primitive
                axis_l = self.finger_l[3].cross(self.finger_l[2])
                normal_l = rotate_vector(self.finger_l[2],axis_l,-rot_angle,)
                distal_l = rotate_vector(self.finger_l[3],axis_l,-rot_angle)
                axis_r = self.finger_r[3].cross(self.finger_r[2])
                normal_r = rotate_vector(self.finger_r[2],axis_r,rot_angle)
                distal_r = rotate_vector(self.finger_r[3],axis_r,rot_angle)
                # Find new contact surfaces,
                for surf in OBJ:
#                     print(square[surf][1].angle(normal_l))
                    if OBJ[surf][1].angle(normal_l)<1e-3:
                        surf_l = surf
                        break
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_r)<1e-3:
                        surf_r = surf
                        break
                for point in OBJ[surf_l][0].points:
                    for other_point in OBJ[surf_l][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_l)
                            if angle<1e-3:
                                length_l = edge.length()
                for point in OBJ[surf_r][0].points:
                    for other_point in OBJ[surf_r][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_r)
                            if angle<1e-3:
                                length_r = edge.length()
                d_l = self.finger_l[0] + length_l
                d_r = self.finger_r[0] - length_r
                # Check state-space limits
                if d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START:
                    continue
                else:
                    # Find solutions for finger angles and check workspace limits
#                     th1=theta_conversion(self.finger_l[0], self.finger_r[0], action)
                    th1=theta_conversion(d_l, d_r, action)
#                     th2=calculate_th2(th1,self.finger_l[0])
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    th2=calculate_th2(th1,d_l)
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    TH2_MAX=calculate_th2(TH1_MAX,self.finger_l[0])
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    TH1_MIN=calculate_th1(TH2_MIN,self.finger_r[0])
                    if th1==None or th2==None:
                        continue
                    else:
                        if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                            pass
                        else:
                            continue
                    # Append primitive to available actions if all checks are passed
                    self.available_actions.append(action)

            # Repeat for other rotation primitives
            elif action == "Rotate cw":
                rot_angle = self.angle_cw_l
                axis_l = self.finger_l[3].cross(self.finger_l[2])
                normal_l = rotate_vector(self.finger_l[2],axis_l,rot_angle)
                distal_l = rotate_vector(self.finger_l[3],axis_l,rot_angle)
                axis_r = self.finger_r[3].cross(self.finger_r[2])
                normal_r = rotate_vector(self.finger_r[2],axis_r,-rot_angle)
                distal_r = rotate_vector(self.finger_r[3],axis_r,-rot_angle)
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_l)<1e-3:
                        surf_l = surf
                        break
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_r)<1e-3:
                        surf_r = surf
                        break
                for point in OBJ[surf_l][0].points:
                    for other_point in OBJ[surf_l][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_l)
                            if angle <1e-3:
                                length_l = edge.length()
                for point in OBJ[surf_r][0].points:
                    for other_point in OBJ[surf_r][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_r)
                            if angle <1e-3:
                                length_r = edge.length()
                d_l = self.finger_l[0] - length_l
                d_r = self.finger_r[0] + length_r
                if d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START:
                    continue
                else:
#                     th2=theta_conversion(self.finger_l[0], self.finger_r[0], action)
                    th2=theta_conversion(d_l, d_r, action)
#                     th1 = calculate_th1(th2, self.finger_r[0])
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    th1 = calculate_th1(th2, d_r)
                    OBJECT_LENGTH = self.objleft_l
                    GAMMA = self.angle_ccw_l
                    TH2_MAX = calculate_th2( TH1_MAX,self.finger_l[0])
                    OBJECT_LENGTH = self.objright_l
                    GAMMA = self.angle_cw_l
                    TH1_MIN = calculate_th1(TH2_MIN,self.finger_r[0])
                    if th1==None or th2==None:
                        continue
                    else:
                        if(th1<=TH1_MAX and th1>=TH1_MIN and th2>=TH2_MIN and th2<=TH2_MAX):
                            pass
                        else:
                            continue
                    self.available_actions.append(action)

            # Moving up and down
            elif action == "Move down":
                # Transform finger state according to the selected primitive
                z = self.finger_l[1] - resolution
                # Check state-space limits
                if z > self.obj_h/2 - finger_h/2 or z < 4-self.obj_h/2:
                    continue
                else:
                    self.available_actions.append(action)
            # Repeat
            elif action == "Move up":
                z = self.finger_l[1] + resolution
                if z > self.obj_h/2 - finger_h/2 or z < 4-self.obj_h/2:
                # if z > self.obj_h/2 or z < -self.obj_h/2:
                    continue
                else:
                    self.available_actions.append(action)

            # Pivoting
            elif action == "Pivot":
                # Check state-space limits (prior)
                if z < 6-self.obj_h/2:
                    continue
                # Assign pivoting angle
                pivot_angle = self.pivot_angle
                # Determine pivoting center for left and right fingers
                left_center = find_contact_center(self.finger_l_poly,self.finger_l[2],self.finger_l[3],OBJ[self.surf_l][0],self.finger_l[0],self.finger_l[1],self.objleft_l)
                right_center = find_contact_center(self.finger_r_poly,self.finger_r[2],self.finger_r[3],OBJ[self.surf_r][0],self.finger_r[0],self.finger_r[1],self.objright_l)

                # Apply pivoting to the finger polygons
                finger_poly_left = pivot_finger(pivot_angle,self.finger_l_poly,self.finger_l[2],left_center,self.finger_l[3])
                finger_poly_right = pivot_finger(-pivot_angle,self.finger_r_poly,self.finger_r[2],right_center,self.finger_r[3])

                # Get transformed finger parameters from pivoted finger polygons
                d_l,z = get_finger_param(finger_poly_left,OBJ)
                d_r,z = get_finger_param(finger_poly_right,OBJ)

                # Reassign object height and length
                new_obj_h = self.objleft_l
                new_obj_l = self.objleft_h

                # Check state-space limits
                if z > new_obj_h/2 or z < -new_obj_h/2 or d_r > FINGER_END or d_r < FINGER_START or d_l > FINGER_END or d_l < FINGER_START or d_l+new_obj_l<finger_w or d_r+new_obj_l<finger_w:
                    continue
                else:
                    self.available_actions.append(action)

        # Assign primitive command for the parenting action
        # For slides and rotations we use finger angles
        if(self.action=="Slide left down"):
            (self.theta)= theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Slide left up"):
            (self.theta) = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Slide right down"):
            (self.theta) = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Slide right up"):
            (self.theta) = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            self.command = [self.theta[0],self.theta[1]]

        elif(self.action=="Rotate ccw"):
#             self.theta[0] = theta_conversion(self.parent.finger_l[0], self.parent.finger_r[0], self.action)
            self.theta[0] = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            OBJECT_LENGTH = self.objleft_l
            GAMMA = self.angle_ccw_l
            self.theta[1] = calculate_th2(self.theta[0], self.finger_l[0])
#             self.theta[0] = theta_conversion(self.finger_l[0]-OBJECT_SIZE, self.finger_r[0]+OBJECT_SIZE, self.action)
#             self.theta[1] = calculate_th2(self.theta[0], self.finger_l[0]-OBJECT_SIZE)
            self.command = self.theta

        elif(self.action=="Rotate cw"):
#             self.theta[1] = theta_conversion(self.parent.finger_l[0], self.parent.finger_r[0], self.action)
            self.theta[1] = theta_conversion(self.finger_l[0], self.finger_r[0], self.action)
            OBJECT_LENGTH = self.objright_l
            GAMMA = self.angle_cw_l
            self.theta[0] = calculate_th1(self.theta[1], self.finger_r[0])
#             self.theta[1] = theta_conversion(self.finger_l[0]+OBJECT_SIZE, self.finger_r[0]-OBJECT_SIZE, self.action)
#             self.theta[0] = calculate_th1(self.theta[1], self.finger_r[0]+OBJECT_SIZE)
            self.command = self.theta

        # For moving up and down, finger angles remain same, we add the resolution for motion
        elif (self.action=="Move down"):
            self.theta = self.parent.theta
            # self.command = self.finger_l[1]
            self.command = [self.parent.theta[0],self.parent.theta[1],resolution]

        elif (self.action=="Move up"):
            self.theta = self.parent.theta
            # self.command = self.finger_l[1]
            self.command = [self.parent.theta[0],self.parent.theta[1],resolution]

        # For pivoting
        elif (self.action=="Pivot"):
#             start_p = translate_point(OBJ[self.parent.surf_l][0].center_point,-self.parent.finger_l[3]*(self.parent.finger_l[0]+self.parent.objleft_l/2))
#             end_p = translate_point(OBJ[self.surf_l][0].center_point,-self.finger_l[3]*(self.finger_l[0]+self.objleft_l/2))
#             vec = Vector(start_p,end_p)

            # Finger angles remain the same
            self.theta=self.parent.theta

            # Parameters for frame transformations during pivoting
            if finger_w - self.parent.finger_l[0] - self.parent.objleft_l > 0:
                xl = self.parent.objleft_l/2
            else:
                xl = (finger_w - self.parent.finger_l[0])/2

            # Command: theta_l, dx, theta_pivot, dz, dl
            self.command = [self.parent.theta[0],self.parent.finger_l[0]+xl,self.parent.pivot_angle,self.parent.finger_l[1]+self.objleft_h,xl]

    def __eq__(self,other):
        """
        Equivalance operator for nodes. Check whether the nodes represent the same states

        Parameters
        ----------
        self: Node
            Instance on the left
        other: Node
            Instance on the right

        Returns
        ----------
        bool: True if equal, false if not equal
        """
        return (self.finger_l[0]==other.finger_l[0] and self.finger_r[0]==other.finger_r[0] and self.finger_l[1]==other.finger_l[1] and self.finger_r[1]==other.finger_r[1]
                    and self.surf_l==other.surf_l)

    def __lt__(self,other):
        """
        Comparison operator for nodes. Compares the costs associated with the nodes.

        Parameters
        ----------
        self: Node
            Instance on the left
        other: Node
            Instance on the right

        Returns
        ----------
        bool: True if Node on the left has lower cost, false if higher
        """
        return (self.f < other.f)


    def at_goal(self):
        """
        Function to check whether the Node is in one of the goal states. Checks if all contact regions are within the goal regions.

        Parameters
        ----------
        self: Node
            Node to check

        Returns
        ----------
        bool: True if at goal, false if not
        """
        # Left finger flag initialized
        flag_l = False
        # Check if the intersection between the goal and the contact has the same area as the contact area (complex definition due to geometry bugs)
        try:
            if abs(self.goal_list_l[self.surf_l].intersection(self.contact_l).area-self.contact_l.area)<1e-3:
                    flag_l = True
        except:
            pass
        # Repeat for right finger
        flag_r = False
        try:
            if abs(self.goal_list_r[self.surf_r].intersection(self.contact_r).area-self.contact_r.area)<1e-3:
                    flag_r = True
        except:
            pass

        # Previous versions that are prone to the bugs
#         flag_l = False
#         for goal in self.goal_list_l:
#             if abs(goal.intersection(self.contact_l).area-self.contact_l.area)<1e-3:
#                 flag_l = True
#                 break
#         flag_r = False
#         for goal in self.goal_list_r:
#             if abs(goal.intersection(self.contact_r).area-self.contact_r.area)<1e-3:
#                 flag_r = True
#                 break
#         flag_l = False
#         for goal in self.goal_list_l:
#             if goal.contains(self.contact_l):
#                 flag_l = True
#                 break
#         flag_r = False
#         for goal in self.goal_list_r:
#             if goal.contains(self.contact_r):
#                 flag_r = True
#                 break
#         flag_l = False
#         for goal in self.goal_list_l:
#             if not goal.disjoint(self.contact_l):
#                 flag_l = True
#                 break
#         flag_r = False
#         for goal in self.goal_list_r:
#             if not goal.disjoint(self.contact_r):
#                 flag_r = True
#                 break
        return flag_l and flag_r

    def plot_state(self):
        """
        Function to plot the object and finger states in 3D

        Parameters
        ----------
        self: Node
            Node to plot
        """
        # Initialize the renderer
        r = Renderer()
        # Add object surfaces, and goal regions if there are any
        for surf in OBJ:
            r.add((OBJ[surf][0],'b',1))
            if len(OBJ[surf])>2:
                r.add((OBJ[surf][2],'r',1))
            if len(OBJ[surf])>3:
                r.add((OBJ[surf][3],'g',1))
        # Add fingers
        r.add((self.finger_l_poly,'k',1))
        r.add((self.finger_r_poly,'k',1))

        # For scaling the plot
        r.add((gPoint(-15,-15,-15),'k',1))
#         r.add((gPoint(15,15,15),'k',1))
        r.show()

def search(start_node,cost_list):
    """
    A* search function. Conducts an A* search given the starting node and a cost list for the manipulation primitives.

    Parameters
    ----------
    start_node: Node
        Starting node for the search
    cost_list: dict
        A dictionary storing costs for each manipulation primitive

    Returns
    ----------
    path: list
        List of right and left finger polygons (Geometry3D.ConvexPolygon) as they navigate from start to goal
    actions: list
        List of manipulation primitives and corresponding commands to navigate the fingers from start to goal
    dt: float
        Time taken to complete the search
    """

    # Record starting time for the search
    start_time = time.time()

    # Initialize an open list for nodes to be visited as queue.PriorityQueue instance
    yet_to_visit_list = queue.PriorityQueue()

    # Initialize the list for visited nodes
    visited_list = []
#     yet_to_visit_list.append(start_node)

    # Input the starting node to the open list, using its cost as the priority number
    yet_to_visit_list.put((start_node.f,start_node))

    # Iteration no
    outer_iterations = 0

    # Iteration limit
    max_iterations = 100000

    # Check if there are Nodes to be visited in the queue
    while not yet_to_visit_list.empty():
        # Increase the iteration no
        outer_iterations += 1
        # Print out the progress
        if outer_iterations%1000 == 0:
            print("{}/{}".format(outer_iterations,max_iterations))

        # Current node is picked as the node with the highest priority (lowest cost) from the queue
        current_node = yet_to_visit_list.get()[1]

        # Iteration limit is reached, end the search
        if outer_iterations > max_iterations:
            end_time = time.time()
            dt = end_time - start_time
            print("Time taken: {} s".format(dt))
            print("giving up on pathfinding too many iterations")
            path, actions = return_path(current_node)
            return path, actions, dt

        # Add the current node to the visited node list
        visited_list.append(current_node)

        # Check if the goal is reached
        if current_node.at_goal():
            # Record the final time
            end_time = time.time()
            # Find the time taken
            dt = end_time - start_time
            print("Time taken: {} s".format(dt))
            # Extract the path and actions starting from the end node
            path, actions = return_path(current_node)
            return path, actions, dt

        # Initialize the list for children nodes
        children = []
        # Loop through the available manipulation primitives at current node
        for action in current_node.available_actions:
            if action == "Slide left down":
                # Transform the finger states
                d_l = current_node.finger_l[0] - current_node.resolution
                d_r = current_node.finger_r[0]
                # Generate the fingers
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            # Repeat for other primitives
            elif action == "Slide left up":
                d_l = current_node.finger_l[0] + current_node.resolution
                d_r = current_node.finger_r[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Slide right down":
                d_r = current_node.finger_r[0] - current_node.resolution
                d_l = current_node.finger_l[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Slide right up":
                d_r = current_node.finger_r[0] + current_node.resolution
                d_l = current_node.finger_l[0]
                finger_l = (d_l,current_node.finger_l[1],current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (d_r,current_node.finger_r[1],current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Rotate ccw":
                rot_angle = current_node.angle_ccw_l
                axis_l = current_node.finger_l[3].cross(current_node.finger_l[2])
                normal_l = rotate_vector(current_node.finger_l[2],axis_l,-rot_angle)
                distal_l = rotate_vector(current_node.finger_l[3],axis_l,-rot_angle)
                axis_r = current_node.finger_r[3].cross(current_node.finger_r[2])
                normal_r = rotate_vector(current_node.finger_r[2],axis_r,rot_angle)
                distal_r = rotate_vector(current_node.finger_r[3],axis_r,rot_angle)
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_l)<1e-3:
                        surf_l = surf
                        break
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_r)<1e-3:
                        surf_r = surf
                        break
                for point in OBJ[surf_l][0].points:
                    for other_point in OBJ[surf_l][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_l)
                            if angle<1e-3:
                                length_l = edge.length()
                for point in OBJ[surf_r][0].points:
                    for other_point in OBJ[surf_r][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_r)
                            if angle<1e-3:
                                length_r = edge.length()
                d_l = current_node.finger_l[0] + length_l
                d_r = current_node.finger_r[0] - length_r
                finger_l = (d_l,current_node.finger_l[1],normal_l,distal_l)
                finger_r = (d_r,current_node.finger_r[1],normal_r,distal_r)
            elif action == "Rotate cw":
                rot_angle = current_node.angle_cw_l
                axis_l = current_node.finger_l[3].cross(current_node.finger_l[2])
                normal_l = rotate_vector(current_node.finger_l[2],axis_l,rot_angle)
                distal_l = rotate_vector(current_node.finger_l[3],axis_l,rot_angle)
                axis_r = current_node.finger_r[3].cross(current_node.finger_r[2])
                normal_r = rotate_vector(current_node.finger_r[2],axis_r,-rot_angle)
                distal_r = rotate_vector(current_node.finger_r[3],axis_r,-rot_angle)
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_l)<1e-3:
                        surf_l = surf
                        break
                for surf in OBJ:
                    if OBJ[surf][1].angle(normal_r)<1e-3:
                        surf_r = surf
                        break
                for point in OBJ[surf_l][0].points:
                    for other_point in OBJ[surf_l][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_l)
                            if angle<1e-3:
                                length_l = edge.length()
                for point in OBJ[surf_r][0].points:
                    for other_point in OBJ[surf_r][0].points:
                        if point==other_point:
                            continue
                        else:
                            edge = Vector(point,other_point)
                            angle = edge.angle(distal_r)
                            if angle<1e-3:
                                length_r = edge.length()
                d_l = current_node.finger_l[0] - length_l
                d_r = current_node.finger_r[0] + length_r
                finger_l = (d_l,current_node.finger_l[1],normal_l,distal_l)
                finger_r = (d_r,current_node.finger_r[1],normal_r,distal_r)
            elif action == "Move down":
                z = current_node.finger_l[1] - current_node.resolution
                finger_l = (current_node.finger_l[0],z,current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (current_node.finger_r[0],z,current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Move up":
                z = current_node.finger_l[1] + current_node.resolution
                finger_l = (current_node.finger_l[0],z,current_node.finger_l[2],current_node.finger_l[3])
                finger_r = (current_node.finger_r[0],z,current_node.finger_r[2],current_node.finger_r[3])
            elif action == "Pivot":
                pivot_angle = current_node.pivot_angle
                left_center = find_contact_center(current_node.finger_l_poly,current_node.finger_l[2],current_node.finger_l[3],
                                                  OBJ[current_node.surf_l][0],current_node.finger_l[0],current_node.finger_l[1],
                                                  current_node.objleft_l)
                right_center = find_contact_center(current_node.finger_r_poly,current_node.finger_r[2],current_node.finger_r[3],
                                                   OBJ[current_node.surf_r][0],current_node.finger_r[0],current_node.finger_r[1],
                                                   current_node.objright_l)
                finger_poly_left = pivot_finger(pivot_angle,current_node.finger_l_poly,current_node.finger_l[2],left_center,current_node.finger_l[3])
                finger_poly_right = pivot_finger(-pivot_angle,current_node.finger_r_poly,current_node.finger_r[2],right_center,current_node.finger_r[3])

                d_l,z = get_finger_param(finger_poly_left,OBJ)
                d_r,z = get_finger_param(finger_poly_right,OBJ)
                finger_l = (d_l,z,finger_poly_left[1],finger_poly_left[2])
                finger_r = (d_r,z,finger_poly_right[1],finger_poly_right[2])


            # Generate the child node
            new_node = Node(finger_l,finger_r,parent=current_node,action=action)

            # Append the node to the list
            children.append(new_node)

        # Loop through the child nodes
        for child in children:

            # Check if the child is already visited
            if len([visited_child for visited_child in visited_list if
                   visited_child == child]) > 0:
                # print(child.action)
                continue

            # Action/primitive cost
            child.g = current_node.g + cost_list[child.action]

            # Compute the heuristic cost

            # Distance between each corner of the contact and goal region

            # Corners of the contact region
            x,y = child.contact_l.exterior.xy
            a = list(zip(x[:-1],y[:-1]))
            dists = []
            # Loop through goal regions
            for goal_key in child.goal_list_l:
                goal = child.goal_list_l[goal_key]
                sum = 0
                # Sum the distances between each corner and goal region
                for point in a:
                    sum += sPoint(point).distance(goal)**2
                dists.append(sum)
            # Heuristic is the minimum of the summed distances
            h_l = min(dists)

            # Repeat for the right finger
            x,y = child.contact_r.exterior.xy
            a = list(zip(x[:-1],y[:-1]))
            dists = []

            for goal_key in child.goal_list_r:
                goal = child.goal_list_r[goal_key]
                sum = 0
                for point in a:
                    sum += sPoint(point).distance(goal)**2
                dists.append(sum)

            h_r = min(dists)

            # Final heuristic is the average of the left and right heuristics
            child.h = (h_l + h_r)/2

            # A discount for repeating the same action consecutively
            if child.action == current_node.action:
                child.h *= 0.7 #My beta: 0.5 Joshua's beta: 0.7

            # Computing the cost, using a weighted heuristic for faster search
            child.f = child.g + child.h*100 #My epsilon: 5 Joshua's epsilon: 2

            # If there is a same node with less cost in the queue, discard the current child node
            if len([i[1] for i in yet_to_visit_list.queue if child == i[1] and
                    child.f > i[1].f]) > 0:
                continue

            # print("Action: {}, Cost: {}".format(child.action,child.f))
            # Put the child node into priority queue
            yet_to_visit_list.put((child.f,child))


def return_path(current_node):
    """
    Function to extract the path (finger polygons) and actions (manipulation primitives) between start and goal nodes

    Parameters
    ----------
    current_node: Node
        End node that will be unraveled until the start node using the .parent attribute

    Returns
    ----------
    path: list
        List of right and left finger polygons (Geometry3D.ConvexPolygon) as they navigate from start to goal
    actions: list
        List of manipulation primitives and corresponding commands to navigate the fingers from start to goal
    """
    # Initialize an empty list for path and actions
    path = []
    actions = []
    # Current node is the end node
    current = current_node
    # Loop until the first node
    while current is not None:
        # current.plot_state()
        # Append the current finger polygons to path list as a tuple
        path.append((current.finger_l_poly,current.finger_r_poly))
        # Append the current manipulation primitive and corresponding command to action list as a tuple
        actions.append((current.action,current.command))
        # Assign the parent node as the current node
        current = current.parent

    # Reverse the order of the lists end:start -> start:end
    path = path[::-1]
    actions = actions[::-1]

    return path, actions

if __name__=="__main__":

    # Provide object name to be used:
    obj_name = "square_prism"
    # Experiment no: corresponds to a initial region - goal region pair
    exp_no = 1

    # Get object type, initial and goal state information according to the provided parameters
    obj_type, goal_no, z, d_l, d_r, normal_l, distal_l, normal_r, distal_r = get_experimental_setup(obj_name,exp_no)

    # Get object geometry: Defined by Geometry 3D library
    OBJ = get_OBJ(obj_type,goal_no)

    # Get surface neighboring dictionary
    OBJ_neighbors = get_neighbors(OBJ)

    # print(OBJ_neighbors)

    # Define fixed action costs: corresponding to each manipulation primitive
    costs = {"Slide left down":1, "Slide left up":1, "Slide right down":1, "Slide right up":1,
     "Rotate ccw":3, "Rotate cw":3, "Move up":10, "Move down": 10, "Pivot":25}

    # Initialize finger states:
    finger_l = (d_l,z,normal_l,distal_l)
    finger_r = (d_r,z,normal_r,distal_r)

    # Initial node:
    ex = Node(finger_l,finger_r)

    #print(ex.available_actions)

    # Plot initial state:
    ex.plot_state()

    # Run search algorithm:
    path, actions, dt = search(ex,costs)



    # Define and create folder at home directory for saving results:
    home_dir = os.path.expanduser("~")
    output_folder = home_dir + "/reg_based_plans"
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)
    file_path = "/plan_" + obj_type + "_" + str(exp_no) + "_" + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M')
    os.mkdir(output_folder+file_path)

    # Define and create folder in region_based_planning package for saving results:
    dir_path = os.path.dirname(os.path.realpath(__file__))
    output_folder = dir_path + "/../output_plans/"
    file_path = "/plan_" + obj_type + "_" + str(exp_no) + "_" + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M')
    os.mkdir(output_folder+file_path)

    # Saving generated manipulation plan (headers):
    plan_file = open(output_folder+file_path+"/data.txt","w")
    data_name = ["Step","command"]
    plan_file.write(str(data_name)+"\n")

    # Saving experiment information:
    info_file = open(output_folder+file_path+"/info.txt","w")
    info_name = ["Object","Goal","z","d_l","d_r","normal_l","distal_l","normal_r","distal_r","Plan time"]
    info_file.write(str(info_name)+"\n")
    info = [obj_name,goal_no,z,d_l,d_r,normal_l,distal_l,normal_r,distal_r,dt]
    info_file.write(str(info)+"\n")

    # Going through the planned manipulation sequence:
    for i in range(1,len(path)):

        if actions[i][0]==None:
            continue
        else:
            data = [i,actions[i]]

        # Save manipulation primitive & corresponding command to file
        plan_file.write(str(data)+"\n")
