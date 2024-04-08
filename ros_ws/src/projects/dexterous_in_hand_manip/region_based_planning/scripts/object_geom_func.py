# Geometry operations on the object and finger system

import numpy as np
from shapely.geometry import Polygon, LineString, MultiPoint
from shapely.geometry import Point as sPoint
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint
from finger_params import*

def generate_map(unfolded_surfaces,base,contact):
    """
    Function to convert Geometry3D polygons for the object and fingers to shapely.geometry entities representing elements of the search map

    Parameters
    ----------
    unfolded_surfaces: dict
        A dictionary similar to object dictionary, where the center surface remains the same but other surfaces are modified through unfolding.
    base: int
        Integer indicating the contact surface number (which is the base surface for the unfolding)
    contact: Geometry3D.ConvexPolygon
        Finger geometry defined as a convex polygon - including pose information (position and orientation)

    Returns
    ----------
    surf_map: dict
        A dictionary that stores shapely.geometry.Polygon's for each surface of the object, modified through unfolding
    goal_map: dict
        A dictionary that stores shapely.geometry.Polygon's for each goal region on the surfaces of the object, modified through unfolding
    contact_map: shapely.geometry.Polygon
        Finger geometry defined in 2D through shapely Polygon
    """
    # Initialize empty dictionaries for goal and surface polygons
    goal_map = dict()
    surf_map = dict()
    # Main surface (center surface for the unfolding)
    main = unfolded_surfaces[base]
    # Transform (translate and project) the 3D polygon definitions onto xy-plane and convert to 2D for shapely

    # Translation required assuming the center of the main surface is the origin
    translation = [-main[0].center_point.x,-main[0].center_point.y,-main[0].center_point.z]
    # Determine required axis and rotation angles for the transformation
    axis = Vector(0,0,1).cross(main[1])
    angle = Vector(0,0,1).angle(main[1])
    A,B,C = axis
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    if L == 0:
        R_y = np.eye(4)
    else:
        R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x
    # Transform object surfaces and goal regions using the computed transformation matrix
    for item in unfolded_surfaces:
        surf_init = unfolded_surfaces[item][0]
        P_init = np.empty((4,0))
        for point in surf_init.points:
            point_vec = np.array([point.x+translation[0],point.y+translation[1],point.z+translation[2],1]).reshape(4,1)
            P_init = np.concatenate([P_init,point_vec],axis=1)
        P_final = T@P_init
        new_points = list()
        for i in range(P_final.shape[1]):
            new_points.append(list(P_final[:2,i]))
        surfs = Polygon(new_points)
        surf_map[item] = surfs
        if len(unfolded_surfaces[item])>2:
            goal_init = unfolded_surfaces[item][2]
            P_init = np.empty((4,0))
            for point in goal_init.points:
                point_vec = np.array([point.x+translation[0],point.y+translation[1],point.z+translation[2],1]).reshape(4,1)
                P_init = np.concatenate([P_init,point_vec],axis=1)
            P_final = T@P_init
            new_points = list()
            for i in range(P_final.shape[1]):
                new_points.append(list(P_final[:2,i]))
            goal = Polygon(new_points)
            goal_map[item] = goal
    # Transform the finger polygon similarly
    C_init = np.empty((4,0))
    for point in contact.points:
        point_vec = np.array([point.x+translation[0],point.y+translation[1],point.z+translation[2],1]).reshape(4,1)
        C_init = np.concatenate([C_init,point_vec],axis=1)
    C_final = T@C_init
    new_points = list()
    for i in range(C_final.shape[1]):
        new_points.append(list(C_final[:2,i]))
    contact_map = Polygon(new_points)
    
    return surf_map,goal_map,contact_map

def translate_point(point,vector):
    """
    Translate a Geometry3D.Point along a given Geometry.3D Vector

    Parameters
    ----------
    point: Geometry3D.Point
        Point to be translated
    vector: Geometry3D.Vector
        Vector to translate along

    Returns
    ----------
    new point: Geometry3D.Point
        Translated point
    """
    
    # Generate an array of coordinates using the given point
    p = np.array([point.x,point.y,point.z])
    
    # Generate an array of values for the vector
    v = np.array([vector[0],vector[1],vector[2]])

    # Generate and return the point resulting from the translation
    return gPoint(p+v)

def rotate_vector(vector,axis,angle):
    """
    Function to rotate a given vector around the given axis by a given angle

    Parameters
    ----------
    vector: Geometry3D.Vector
        Vector to be rotated
    axis: Geometry3D.Vector
        Rotation axis
    angle: float
        Rotation angle

    Returns
    ----------
    Geometry3D.Vector
        Rotated vector
    """

    # Compute the required transformation (rotation) matrix
    A,B,C = axis
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    if L == 0:
        R_y = np.eye(4)
    else:
        R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x
    # Apply the transformation
    C_init = np.array([vector[0],vector[1],vector[2],1]).reshape(4,1)
    C_final = T@C_init
    for i in range(C_final.shape[1]):
        new_points = np.round(C_final[:3,i],decimals=3)
    return Vector(new_points)

def place_finger(obj,d,z,normal,distal,hand_normal):
    """
    Function to generate Geometry3D.ConvexPolygon definitions of fingers (including finger geometry and pose)

    Parameters
    ----------
    obj: dict
        Dictionary including information about each surface on the object in the following form:
                     Geometry3D ConvexPolygon  , Geometry3D Vector    , Geometry3D ConvexPolygon
        {surface_no:(surface_polygon_definition, surface_normal_vector, goal_region_polygon(if available))}
    d: float
        Parameter d for given finger
    z: float
        Elevation z for given finger
    normal: Geometry3D.Vector
        Normal vector pointing out of the finger (backhand)
    distal: Geometry3D.Vector
        Distal vector pointing out of the finger (from palm to fingertip)
    hand_normal: Geometry3D.Vector
        Vector normal to the manipulation plane of the hand

    Returns
    ----------
    finger: Geometry3D.ConvexPolygon
        Finger geometry defined as a convex polygon - including pose information (position and orientation)
    """
    # Loop through the object surfaces to find the one that has the same (similar) normal vector with the finger
    for surf in obj:
        if obj[surf][1].angle(normal)<1e-3:
            # Contact surface found
            surface = obj[surf][0]
            break
    
    # Find object length as the distance between two corners of the surface in the distal direction
    for point in surface.points:
        for other_point in surface.points:
            if point==other_point:
                continue
            else:
                edge = Vector(point,other_point)
                angle = edge.angle(distal)
                if angle <1e-3:
                    obj_length = edge.length()
        
    # Find finger center by translating the surface center along z and distal direction using given finger parameters
    finger_center = translate_point(surface.center_point,(hand_normal*z - distal*(d+obj_length/2-finger_w/2)))

    # Find corner 1-4 of the finger by translating the finger center according to given finger parameters
    finger_p1 = translate_point(finger_center,(distal*(finger_w/2)+distal.cross(normal)*(finger_h/2)))
    finger_p2 = translate_point(finger_center,(-distal*(finger_w/2)+distal.cross(normal)*(finger_h/2)))
    finger_p3 = translate_point(finger_center,(-distal*(finger_w/2)-distal.cross(normal)*(finger_h/2)))
    finger_p4 = translate_point(finger_center,(distal*(finger_w/2)-distal.cross(normal)*(finger_h/2)))
    
    # Define Geometry3D.ConvexPolygon for the finger using finger corners
    finger = ConvexPolygon((finger_p1,finger_p2,finger_p3,finger_p4))
    return finger

def pivot_finger(angle,poly,normal,center,distal):
    """
    Given a finger polygon, corresponding vectors, pivoting angle and a center generate the pivoted finger polygon

    Parameters
    ----------
    angle: float
        Pivoting angle
    poly: Geometry3D.ConvexPolygon
        Finger polygon to be pivoted
    normal: Geometry3D.Vector
        Normal vector of the finger
    center: Geometry3D.Point
        Pivoting center
    distal: Geometry3D.Vector
        Distal vector of the finger

    Returns
    ----------
    (nsurf, normal, ndistal): tuple
        Transformed finger polygon (Geometry3D.ConvexPolygon), finger normal remains the same (Geometry3D.Vector), transformed distal vector (Geometry3D.Vector)
    """
    # Compute the transformation
    A,B,C = normal
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    D = np.array([[1,0,0,-center.x],[0,1,0,-center.y],[0,0,1,-center.z],[0,0,0,1]])
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    if L == 0:
        R_y = np.eye(4)
    else:
        R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(D)@np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x@D
    # Apply transformation
    P_init = np.empty((4,0))
    for point in poly.points:
        point_vec = np.array([point.x,point.y,point.z,1]).reshape(4,1)
        P_init = np.concatenate([P_init,point_vec],axis=1)
    distal_vec = np.array([poly.points[-1].x+distal[0],poly.points[-1].y+distal[1],poly.points[-1].z+distal[2],1]).reshape(4,1)
    P_init = np.concatenate([P_init,distal_vec],axis=1)
    P_final = T@P_init
    new_points = list()
    for i in range(P_final.shape[1]-1):
        new_points.append(gPoint(np.round(P_final[:3,i],decimals=3)))
    
    ndistal = Vector(np.round(P_final[:3,-1]-P_final[:3,-2],decimals=3))
    nsurf = ConvexPolygon((new_points))
    return (nsurf,normal,ndistal)

def get_finger_param(finger_poly,obj):
    """
    Given the finger polygon and the object find the state parameters (finger parameters)

    Parameters
    ----------
    finger_poly: (surf, normal, distal): tuple
        Finger polygon (Geometry3D.ConvexPolygon), finger normal(Geometry3D.Vector), distal vector (Geometry3D.Vector)
    obj:  dict
        Dictionary including information about each surface on the object in the following form:
                     Geometry3D ConvexPolygon  , Geometry3D Vector    , Geometry3D ConvexPolygon
        {surface_no:(surface_polygon_definition, surface_normal_vector, goal_region_polygon(if available))}
    
    Returns
    ----------
    d: float
        Distance between finger joint and object start
    z: float
        Elevation of the finger on the object
    """
    # Find finger center point
    finger_center = finger_poly[0].center_point
    # Find contact surface
    for surf in obj:
        if obj[surf][1].angle(finger_poly[1])<1e-3:
            surface = obj[surf][0]
            break
    # Find object length alond finger distal vector
    for point in surface.points:
        for other_point in surface.points:
            if point==other_point:
                continue
            else:
                edge = Vector(point,other_point)
                angle = edge.angle(finger_poly[2])
                if angle <1e-3:
                    obj_length = edge.length()
    # Find the center of the contact surface
    obj_center = surface.center_point
    # Center point on the proximal object edge
    close_edge = translate_point(obj_center,(-finger_poly[2]*(obj_length/2)))
    # Generate the vector connecting the proximal edge center and finger center
    vec = Vector(finger_center,close_edge)
    # If the vector has a length of 0, centers align, thus elevation is the same and d is half of the finger length
    if vec.length()==0:
        d = finger_w/2
        z = 0
    # Else compute the angle between distal vector and the generated vector and find the distance components to determine d and z
    else:
        ang = vec.angle(finger_poly[2])
        if ang<np.pi/2:
            b = vec.length()*abs(np.cos(ang))
        else:
            b = -vec.length()*abs(np.cos(ang))
        d = np.round(b+finger_w/2,decimals=1)
        q = obj_center.distance(finger_center)**2-(b+obj_length/2)**2
        if q < 0 and abs(q) < 1e-3:
            q = 0
        if finger_poly[2].cross(finger_poly[1]).angle(vec)<np.pi/2:
            z = -np.round(np.sqrt(q),decimals=1)
        else:
            z = np.round(np.sqrt(q),decimals=1)

    # debugging material
    if np.isnan(z):
        print(obj_center.distance(finger_center))
        print((b+obj_length/2))
        print(obj_center)
        print(finger_center)
        print(b)
        print(vec)
        print(finger_poly[2])
        
        return None

    return d,z

def find_contact_center(finger_poly,normal,distal,surface,d,z,obj_l):
    """
    Given the finger polygon, corresponding vectors, finger and object parameters, determines the pivoting center for the finger

    Parameters
    ----------
    finger_poly: Geometry3D.ConvexPolygon
        Finger polygon
    normal: Geometry3D.Vector
        Finger normal
    distal: Geometry3D.Vector
        Finger distal vector
    surface: Geometry3D.ConvexPolygon
        Polygon corresponding to the contact surface
    d: float
        Finger parameter d
    z: float
        Finger parameter z
    obj_l: float
        Object length along distal direction

    Returns
    ----------
    center: Geometry3D.Point
        Pivoting center
    """
    # Check if the object exceeds the tip of the finger
    if finger_w - d - obj_l > 0:
        # if not no translation is necessary along distal
        v1 = distal*0
    else:
        v1 = (-(finger_w-d)/2 + (obj_l/2))*distal
    
    # Determine the translation along the hand normal
    v_temp = Vector(surface.center_point,finger_poly.center_point)
    hand_normal = distal.cross(normal)
    if v_temp.length() == 0:
        b = 0
    else:
        ang = v_temp.angle(hand_normal)
        if ang<np.pi/2:
            b = v_temp.length()*abs(np.cos(ang))
        else:
            b = -v_temp.length()*abs(np.cos(ang))
    v2 = b*hand_normal
    # Generate the pivoting center point by translating the surface center using the computed translation vectors
    center = translate_point(surface.center_point,v1+v2)
    return center