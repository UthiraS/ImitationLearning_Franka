# Geometrical definitions of prismatic objects using Geometry3D library

import numpy as np
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint


# This is a definition used when dealing with some bugs within the Geometry3D library
sqrt3 = np.sqrt(3)

def get_OBJ(object_type,goal_no):
    
    """
    Given the object name and goal region number provides geometry definitions for object and goal region

    Parameters
    ----------
    object_type: string
        Name of the object
    goal_no: int
        Number assigned to goal region

    Returns
    ----------
    prism: dict
        Dictionary including information about each surface on the object in the following form:
                     Geometry3D ConvexPolygon  , Geometry3D Vector    , Geometry3D ConvexPolygon
        {surface_no:(surface_polygon_definition, surface_normal_vector, goal_region_polygon(if available))}
    """

    if object_type == "square_prism":
        ####################################################### Square prism ########################################################################
        ##############################################################################################################################################
        # a = 2.5 cm, h = 8 cm

        p1 = gPoint(-1.25,-1.25,-4)
        p2 = gPoint(-1.25,-1.25,4)
        p3 = gPoint(1.25,-1.25,4)
        p4 = gPoint(1.25,-1.25,-4)
        p5 = gPoint(1.25,1.25,-4)
        p6 = gPoint(1.25,1.25,4)
        p7 = gPoint(-1.25,1.25,4)
        p8 = gPoint(-1.25,1.25,-4)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        
        if goal_no == 1:
            g1 = gPoint(-1.25,-1.25,4)
            g2 = gPoint(-1.25,-1.25,2)
            g3 = gPoint(1.25,-1.25,2)
            g4 = gPoint(1.25,-1.25,4)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.25,1.25,4)
            g2 = gPoint(-1.25,1.25,2)
            g3 = gPoint(1.25,1.25,2)
            g4 = gPoint(1.25,1.25,4)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(1.25,-1.25,-4)
            g2 = gPoint(1.25,-1.25,2)
            g3 = gPoint(1.25,0.75,2)
            g4 = gPoint(1.25,0.75,-4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.25,-1.25,-4)
            g2 = gPoint(-1.25,-1.25,2)
            g3 = gPoint(-1.25,1.25,2)
            g4 = gPoint(-1.25,1.25,-4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 3:
            g1 = gPoint(1.25,-1.25,4)
            g2 = gPoint(1.25,-1.25,2)
            g3 = gPoint(1.25,1.25,2)
            g4 = gPoint(1.25,1.25,4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.25,-1.25,4)
            g2 = gPoint(-1.25,-1.25,2)
            g3 = gPoint(-1.25,1.25,2)
            g4 = gPoint(-1.25,1.25,4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "rectangular_prism_small":
        # Rectangular prism:
        ##################
        # w = 2 cm, l = 3 cm, h = 4 cm

        p1 = gPoint(-1.5,-1,-2)
        p2 = gPoint(-1.5,-1,2)
        p3 = gPoint(1.5,-1,2)
        p4 = gPoint(1.5,-1,-2)
        p5 = gPoint(1.5,1,-2)
        p6 = gPoint(1.5,1,2)
        p7 = gPoint(-1.5,1,2)
        p8 = gPoint(-1.5,1,-2)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            goal2=surf2
            goal4=surf4
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            goal5=surf5
            goal6=surf6
            prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5,goal5),6:(surf6,n6,goal6)}
        elif goal_no == 4:
            g1 = gPoint(-1.5,-1,0)
            g2 = gPoint(-1.5,-1,2)
            g3 = gPoint(1.5,-1,2)
            g4 = gPoint(1.5,-1,0)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,1,0)
            g2 = gPoint(-1.5,1,2)
            g3 = gPoint(1.5,1,2)
            g4 = gPoint(1.5,1,0)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
    
    elif object_type == "rectangular_prism_curved":
        # Rectangular prism tall with flanged corners:
        ##################
        # w = 2 cm, l = 3 cm, h = 8 cm

        p1 = gPoint(-1.5,-1,-4)
        p2 = gPoint(-1.5,-1,4)
        p3 = gPoint(1.5,-1,4)
        p4 = gPoint(1.5,-1,-4)
        p5 = gPoint(1.5,1,-4)
        p6 = gPoint(1.5,1,4)
        p7 = gPoint(-1.5,1,4)
        p8 = gPoint(-1.5,1,-4)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-1.5,-1,4)
            g2 = gPoint(-1.5,-1,2)
            g3 = gPoint(1.5,-1,2)
            g4 = gPoint(1.5,-1,4)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,1,4)
            g2 = gPoint(-1.5,1,2)
            g3 = gPoint(1.5,1,2)
            g4 = gPoint(1.5,1,4)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(1.5,-1,-4)
            g2 = gPoint(1.5,-1,2)
            g3 = gPoint(1.5,1,2)
            g4 = gPoint(1.5,1,-4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,-1,-4)
            g2 = gPoint(-1.5,-1,2)
            g3 = gPoint(-1.5,1,2)
            g4 = gPoint(-1.5,1,-4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 3:
            g1 = gPoint(1.5,-1,4)
            g2 = gPoint(1.5,-1,2)
            g3 = gPoint(1.5,1,2)
            g4 = gPoint(1.5,1,4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,-1,4)
            g2 = gPoint(-1.5,-1,2)
            g3 = gPoint(-1.5,1,2)
            g4 = gPoint(-1.5,1,4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "hexagonal_prism_small":
        # Hexagonal prism:
        ##################
        # a = 2 cm, h = 3 cm
        h = 1.5*2

        p1 = gPoint(-1,-sqrt3,-h/2)
        p2 = gPoint(-1,-sqrt3,h/2)
        p3 = gPoint(1,-sqrt3,h/2)
        p4 = gPoint(1,-sqrt3,-h/2)
        p5 = gPoint(2,0,h/2)
        p6 = gPoint(2,0,-h/2)
        p7 = gPoint(1,sqrt3,h/2)
        p8 = gPoint(1,sqrt3,-h/2)
        p9 = gPoint(-1,sqrt3,h/2)
        p10 = gPoint(-1,sqrt3,-h/2)
        p11 = gPoint(-2,0,h/2)
        p12 = gPoint(-2,0,-h/2)
        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p5,p6))
        surf3 = ConvexPolygon((p6,p5,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p9,p10))
        surf5 = ConvexPolygon((p10,p9,p11,p12))
        surf6 = ConvexPolygon((p12,p11,p2,p1))
        surf7 = ConvexPolygon((p2,p11,p9,p7,p5,p3))
        surf8 = ConvexPolygon((p10,p12,p1,p4,p6,p8))
        n1 = Vector(0,-1,0)
        n2 = Vector(1.5,-sqrt3/2,0)
        n3 = Vector(1.5,sqrt3/2,0)
        n4 = Vector(0,1,0)
        n5 = Vector(-1.5,sqrt3/2,0)
        n6 = Vector(-1.5,-sqrt3/2,0)
        n7 = Vector(0,0,1)
        n8 = Vector(0,0,-1)

        if goal_no == 1:
            # ############ GOAL DEF 1 ############
            goal7 = surf7
            goal8 = surf8
            prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6),7:(surf7,n7,goal7),8:(surf8,n8,goal8)}
        elif goal_no == 2:
            # ############ GOAL DEF 2 ############
            goal2 = surf2
            goal5 = surf5
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5,goal5),6:(surf6,n6),7:(surf7,n7),8:(surf8,n8)}
        elif goal_no == 3:
            # ############ GOAL DEF 3 ############
            g1 = gPoint(-1,-sqrt3,h/2)
            g2 = gPoint(-1,-sqrt3,0)
            g3 = gPoint(1,-sqrt3,h/2)
            g4 = gPoint(1,-sqrt3,0)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1,sqrt3,h/2)
            g2 = gPoint(-1,sqrt3,0)
            g3 = gPoint(1,sqrt3,h/2)
            g4 = gPoint(1,sqrt3,0)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6),7:(surf7,n7),8:(surf8,n8)}
        elif goal_no == 4:
            g1 = gPoint(-2,0,h/2)
            g2 = gPoint(-1,sqrt3,h/2)
            g3 = gPoint(1,sqrt3,h/2)
            g4 = gPoint(2,0,h/2)
            goal7 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-2,0,-h/2)
            g2 = gPoint(-1,sqrt3,-h/2)
            g3 = gPoint(1,sqrt3,-h/2)
            g4 = gPoint(2,0,-h/2)
            goal8 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6),7:(surf7,n7,goal7),8:(surf8,n8,goal8)}
        elif goal_no == 5:
            pass

    elif object_type == "dome_tall":
        ####################################################### Dome Tall ########################################################################
        ##############################################################################################################################################
        # a = 3 cm, h = 8 cm

        p1 = gPoint(-1.5,-1.5,-4)
        p2 = gPoint(-1.5,-1.5,4)
        p3 = gPoint(1.5,-1.5,4)
        p4 = gPoint(1.5,-1.5,-4)
        p5 = gPoint(1.5,1.5,-4)
        p6 = gPoint(1.5,1.5,4)
        p7 = gPoint(-1.5,1.5,4)
        p8 = gPoint(-1.5,1.5,-4)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-1.5,-1.5,4)
            g2 = gPoint(-1.5,-1.5,2)
            g3 = gPoint(1.5,-1.5,2)
            g4 = gPoint(1.5,-1.5,4)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,1.5,4)
            g2 = gPoint(-1.5,1.5,2)
            g3 = gPoint(1.5,1.5,2)
            g4 = gPoint(1.5,1.5,4)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(1.5,-1.5,-4)
            g2 = gPoint(1.5,-1.5,2)
            g3 = gPoint(1.5,1,2)
            g4 = gPoint(1.5,1,-4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,-1.5,-4)
            g2 = gPoint(-1.5,-1.5,2)
            g3 = gPoint(-1.5,1.5,2)
            g4 = gPoint(-1.5,1.5,-4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 3:
            g1 = gPoint(1.5,-1.5,4)
            g2 = gPoint(1.5,-1.5,2)
            g3 = gPoint(1.5,1.5,2)
            g4 = gPoint(1.5,1.5,4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,-1.5,4)
            g2 = gPoint(-1.5,-1.5,2)
            g3 = gPoint(-1.5,1.5,2)
            g4 = gPoint(-1.5,1.5,4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "round_prism":
        ####################################################### Round prism ########################################################################
        ##############################################################################################################################################
        # a = 4 cm, h = 6 cm

        p1 = gPoint(-2,-2,-3)
        p2 = gPoint(-2,-2,3)
        p3 = gPoint(2,-2,3)
        p4 = gPoint(2,-2,-3)
        p5 = gPoint(2,2,-3)
        p6 = gPoint(2,2,3)
        p7 = gPoint(-2,2,3)
        p8 = gPoint(-2,2,-3)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-1.5,-2,-3)
            g2 = gPoint(-1.5,-2,3)
            g3 = gPoint(2,-2,3)
            g4 = gPoint(2,-2,-3)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,2,-3)
            g2 = gPoint(-1.5,2,3)
            g3 = gPoint(2,2,3)
            g4 = gPoint(2,2,-3)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "cube":
        ####################################################### Cube ########################################################################
        ##############################################################################################################################################
        # a = 5 cm

        p1 = gPoint(-2.5,-2.5,-2.5)
        p2 = gPoint(-2.5,-2.5,2.5)
        p3 = gPoint(2.5,-2.5,2.5)
        p4 = gPoint(2.5,-2.5,-2.5)
        p5 = gPoint(2.5,2.5,-2.5)
        p6 = gPoint(2.5,2.5,2.5)
        p7 = gPoint(-2.5,2.5,2.5)
        p8 = gPoint(-2.5,2.5,-2.5)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-1.5,-2.5,-2.5)
            g2 = gPoint(-1.5,-2.5,2.5)
            g3 = gPoint(2.5,-2.5,2.5)
            g4 = gPoint(2.5,-2.5,-2.5)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,2.5,-2.5)
            g2 = gPoint(-1.5,2.5,2.5)
            g3 = gPoint(2.5,2.5,2.5)
            g4 = gPoint(2.5,2.5,-2.5)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "rectangular_prism_large":
        ####################################################### Square prism ########################################################################
        ##############################################################################################################################################
        # a = 3 cm, b = 4 cm, h = 5 cm

        p1 = gPoint(-1.5,-2,-2.5)
        p2 = gPoint(-1.5,-2,2.5)
        p3 = gPoint(1.5,-2,2.5)
        p4 = gPoint(1.5,-2,-2.5)
        p5 = gPoint(1.5,2,-2.5)
        p6 = gPoint(1.5,2,2.5)
        p7 = gPoint(-1.5,2,2.5)
        p8 = gPoint(-1.5,2,-2.5)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-1,-2,-2.5)
            g2 = gPoint(-1,-2,2.5)
            g3 = gPoint(1.5,-2,2.5)
            g4 = gPoint(1.5,-2,-2.5)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1,2,-2.5)
            g2 = gPoint(-1,2,2.5)
            g3 = gPoint(1.5,2,2.5)
            g4 = gPoint(1.5,2,-2.5)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(1.5,-2,-1)
            g2 = gPoint(1.5,-2,2.5)
            g3 = gPoint(1.5,2,2.5)
            g4 = gPoint(1.5,2,-1)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,-2,-1)
            g2 = gPoint(-1.5,-2,2.5)
            g3 = gPoint(-1.5,2,2.5)
            g4 = gPoint(-1.5,2,-1)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 3:
            goal5 = surf5
            goal6 = surf6
            prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5,goal5),6:(surf6,n6,goal6)}
        elif goal_no == 4:
            g1 = gPoint(-1.5,-2,0)
            g2 = gPoint(-1.5,-2,2.5)
            g3 = gPoint(1.5,-2,2.5)
            g4 = gPoint(1.5,-2,0)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,2,0)
            g2 = gPoint(-1.5,2,2.5)
            g3 = gPoint(1.5,2,2.5)
            g4 = gPoint(1.5,2,0)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "dome_short":
        ####################################################### Dome Short ########################################################################
        ##############################################################################################################################################
        # a = 3 cm, h = 4 cm

        p1 = gPoint(-1.5,-1.5,-2)
        p2 = gPoint(-1.5,-1.5,2)
        p3 = gPoint(1.5,-1.5,2)
        p4 = gPoint(1.5,-1.5,-2)
        p5 = gPoint(1.5,1.5,-2)
        p6 = gPoint(1.5,1.5,2)
        p7 = gPoint(-1.5,1.5,2)
        p8 = gPoint(-1.5,1.5,-2)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-1,-1.5,-2)
            g2 = gPoint(-1,-1.5,2)
            g3 = gPoint(1.5,-1.5,2)
            g4 = gPoint(1.5,-1.5,-2)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1,1.5,-2)
            g2 = gPoint(-1,1.5,2)
            g3 = gPoint(1.5,1.5,2)
            g4 = gPoint(1.5,1.5,-2)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(1.5,-1.5,-1)
            g2 = gPoint(1.5,-1.5,2)
            g3 = gPoint(1.5,1.5,2)
            g4 = gPoint(1.5,1.5,-1)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,-1.5,-1)
            g2 = gPoint(-1.5,-1.5,2)
            g3 = gPoint(-1.5,1.5,2)
            g4 = gPoint(-1.5,1.5,-1)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 3:
            goal5 = surf5
            goal6 = surf6
            prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5,goal5),6:(surf6,n6,goal6)}
        elif goal_no == 4:
            g1 = gPoint(-1.5,-1.5,0)
            g2 = gPoint(-1.5,-1.5,2)
            g3 = gPoint(1.5,-1.5,2)
            g4 = gPoint(1.5,-1.5,0)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,1.5,0)
            g2 = gPoint(-1.5,1.5,2)
            g3 = gPoint(1.5,1.5,2)
            g4 = gPoint(1.5,1.5,0)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "hexagonal_prism_large":
        # Hexagonal prism large:
        ##################
        # a = 3 cm, h = 10 cm
        h = 1.5*2

        p1 = gPoint(-1,-sqrt3,-h/2)
        p2 = gPoint(-1,-sqrt3,h/2)
        p3 = gPoint(1,-sqrt3,h/2)
        p4 = gPoint(1,-sqrt3,-h/2)
        p5 = gPoint(2,0,h/2)
        p6 = gPoint(2,0,-h/2)
        p7 = gPoint(1,sqrt3,h/2)
        p8 = gPoint(1,sqrt3,-h/2)
        p9 = gPoint(-1,sqrt3,h/2)
        p10 = gPoint(-1,sqrt3,-h/2)
        p11 = gPoint(-2,0,h/2)
        p12 = gPoint(-2,0,-h/2)
        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p5,p6))
        surf3 = ConvexPolygon((p6,p5,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p9,p10))
        surf5 = ConvexPolygon((p10,p9,p11,p12))
        surf6 = ConvexPolygon((p12,p11,p2,p1))
        surf7 = ConvexPolygon((p2,p11,p9,p7,p5,p3))
        surf8 = ConvexPolygon((p10,p12,p1,p4,p6,p8))
        n1 = Vector(0,-1,0)
        n2 = Vector(1.5,-sqrt3/2,0)
        n3 = Vector(1.5,sqrt3/2,0)
        n4 = Vector(0,1,0)
        n5 = Vector(-1.5,sqrt3/2,0)
        n6 = Vector(-1.5,-sqrt3/2,0)
        n7 = Vector(0,0,1)
        n8 = Vector(0,0,-1)
        
        if goal_no == 1:
            g1 = gPoint(1.5,-sqrt3,h/2)
            g2 = gPoint(1.5,-sqrt3,2)
            g3 = gPoint(3,0,2)
            g4 = gPoint(3,0,h/2)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1.5,sqrt3,h/2)
            g2 = gPoint(-1.5,sqrt3,2)
            g3 = gPoint(-3,0,2)
            g4 = gPoint(-3,0,h/2)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(1.5,-sqrt3,-h/2)
            g2 = gPoint(1.5,-sqrt3,2)
            g3 = gPoint(2,sqrt3,2)
            g4 = gPoint(2,sqrt3,-h/2)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-2,sqrt3,-h/2)
            g2 = gPoint(-2,sqrt3,2)
            g3 = gPoint(-3,0,2)
            g4 = gPoint(-3,0,-h/2)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "rectangular_prism_2x4":
        # a = 2 cm, b = 4 cm, h = 8 cm

        p1 = gPoint(-2,-1,-4)
        p2 = gPoint(-2,-1,4)
        p3 = gPoint(2,-1,4)
        p4 = gPoint(2,-1,-4)
        p5 = gPoint(2,1,-4)
        p6 = gPoint(2,1,4)
        p7 = gPoint(-2,1,4)
        p8 = gPoint(-2,1,-4)

        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p6,p5))
        surf3 = ConvexPolygon((p5,p6,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p2,p1))
        surf5 = ConvexPolygon((p2,p7,p6,p3))
        surf6 = ConvexPolygon((p8,p1,p4,p5))

        n1 = Vector(0,-1,0)
        n2 = Vector(1,0,0)
        n3 = Vector(0,1,0)
        n4 = Vector(-1,0,0)
        n5 = Vector(0,0,1)
        n6 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(-2,-1,4)
            g2 = gPoint(-2,-1,2)
            g3 = gPoint(2,-1,2)
            g4 = gPoint(2,-1,4)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-2,1,4)
            g2 = gPoint(-2,1,2)
            g3 = gPoint(2,1,2)
            g4 = gPoint(2,1,4)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 2:
            g1 = gPoint(2,-1,-4)
            g2 = gPoint(2,-1,2)
            g3 = gPoint(2,1,2)
            g4 = gPoint(2,1,-4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-2,-1,-4)
            g2 = gPoint(-2,-1,2)
            g3 = gPoint(-2,1,2)
            g4 = gPoint(-2,1,-4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}
        elif goal_no == 3:
            g1 = gPoint(2,-1,4)
            g2 = gPoint(2,-1,2)
            g3 = gPoint(2,1,2)
            g4 = gPoint(2,1,4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-2,-1,4)
            g2 = gPoint(-2,-1,2)
            g3 = gPoint(-2,1,2)
            g4 = gPoint(-2,1,4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

    elif object_type == "hexagonal_prism_tall":
        # Hexagonal prism:
        ##################
        # a = 2 cm, h = 8 cm
        h = 4*2

        p1 = gPoint(-1,-sqrt3,-h/2)
        p2 = gPoint(-1,-sqrt3,h/2)
        p3 = gPoint(1,-sqrt3,h/2)
        p4 = gPoint(1,-sqrt3,-h/2)
        p5 = gPoint(2,0,h/2)
        p6 = gPoint(2,0,-h/2)
        p7 = gPoint(1,sqrt3,h/2)
        p8 = gPoint(1,sqrt3,-h/2)
        p9 = gPoint(-1,sqrt3,h/2)
        p10 = gPoint(-1,sqrt3,-h/2)
        p11 = gPoint(-2,0,h/2)
        p12 = gPoint(-2,0,-h/2)
        surf1 = ConvexPolygon((p1,p2,p3,p4))
        surf2 = ConvexPolygon((p4,p3,p5,p6))
        surf3 = ConvexPolygon((p6,p5,p7,p8))
        surf4 = ConvexPolygon((p8,p7,p9,p10))
        surf5 = ConvexPolygon((p10,p9,p11,p12))
        surf6 = ConvexPolygon((p12,p11,p2,p1))
        surf7 = ConvexPolygon((p2,p11,p9,p7,p5,p3))
        surf8 = ConvexPolygon((p10,p12,p1,p4,p6,p8))
        n1 = Vector(0,-1,0)
        n2 = Vector(1.5,-sqrt3/2,0)
        n3 = Vector(1.5,sqrt3/2,0)
        n4 = Vector(0,1,0)
        n5 = Vector(-1.5,sqrt3/2,0)
        n6 = Vector(-1.5,-sqrt3/2,0)
        n7 = Vector(0,0,1)
        n8 = Vector(0,0,-1)

        if goal_no == 1:
            g1 = gPoint(1,-sqrt3,4)
            g2 = gPoint(1,-sqrt3,2)
            g3 = gPoint(2,0,2)
            g4 = gPoint(2,0,4)
            goal2 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1,sqrt3,4)
            g2 = gPoint(-1,sqrt3,2)
            g3 = gPoint(-2,0,2)
            g4 = gPoint(-2,0,4)
            goal5 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4),5:(surf5,n5,goal5),6:(surf6,n6),7:(surf7,n7),8:(surf8,n8)}
        elif goal_no == 2:
            g1 = gPoint(2,0,-4)
            g2 = gPoint(2,0,2)
            g3 = gPoint(1,sqrt3,2)
            g4 = gPoint(1,sqrt3,-4)
            goal3 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-2,0,-4)
            g2 = gPoint(-2,0,2)
            g3 = gPoint(-1,-sqrt3,2)
            g4 = gPoint(-1,-sqrt3,-4)
            goal6 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3,goal3),4:(surf4,n4),5:(surf5,n5),6:(surf6,n6,goal6),7:(surf7,n7),8:(surf8,n8)}
        elif goal_no == 3:
            g1 = gPoint(-1,-sqrt3,4)
            g2 = gPoint(-1,-sqrt3,2)
            g3 = gPoint(1,-sqrt3,2)
            g4 = gPoint(1,-sqrt3,4)
            goal1 = ConvexPolygon((g1,g2,g3,g4))
            g1 = gPoint(-1,sqrt3,4)
            g2 = gPoint(-1,sqrt3,2)
            g3 = gPoint(1,sqrt3,2)
            g4 = gPoint(1,sqrt3,4)
            goal4 = ConvexPolygon((g1,g2,g3,g4))
            prism = {1:(surf1,n1,goal1),2:(surf2,n2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6),7:(surf7,n7),8:(surf8,n8)}


    return prism