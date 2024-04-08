# Set of experiments for prismatic objects

from Geometry3D import Vector

def get_experimental_setup(obj,exp_no):

    """
    For a given object and experiment number, provides initial finger states and goal region number
    
    Parameters
    ----------
    obj: string
        Name of the object
    exp_no: int
        Number assigned to a initial and goal pair

    Returns
    ----------
    obj_type: string
        Name of the object
    goal_no: int
        Number assigned to goal region
    z: float
        finger elevation
    d_l: float
        left finger d
    d_r: float
        right finger d
    normal_l: Geometry3D Vector
        Normal vector left finger
    distal_l: Geometry3D Vector
        Distal vector left finger
    normal_r: Geometry3D Vector
        Normal vector right finger
    distal_r: Geometry3D Vector
        Distal vector right finger
    """

    if obj == "square_prism":
        obj_type = obj

        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,1,0)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,1,0)

        elif exp_no == 2:
            goal_no = 2
            z=2
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)

        elif exp_no == 3:
            goal_no = 1
            z=2.5
            d_l=10
            d_r=10
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,1,0)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,1,0)

        elif exp_no == 4:
            goal_no = 1
            z=1
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)

    elif obj == "rectangular_prism_small":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 2:
            goal_no = 2
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,0,-1)
        elif exp_no == 3:
            goal_no = 2
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,1,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(0,-1,0)
            distal_r=Vector(0,0,-1)
        elif exp_no == 4:
            goal_no = 4
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(0,1,0)
            distal_r=Vector(0,0,-1)

    elif obj == "rectangular_prism_curved":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,1,0)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,1,0)
        elif exp_no == 2:
            goal_no = 2
            z=2
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 3
            z=2.5
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 4:
            goal_no = 1
            z=1
            d_l=9
            d_r=9
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)

    elif obj == "hexagonal_prism_small":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(0,1,0)
            distal_r=Vector(0,0,-1)
        elif exp_no == 2:
            goal_no = 2
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 3
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,0,-1)
            distal_l=Vector(0,-1,0)
            normal_r=Vector(0,0,1)
            distal_r=Vector(0,-1,0)
        elif exp_no == 4:
            goal_no = 4
            z=0
            d_l=8
            d_r=8
            # normal_l=Vector(0,0,-1)
            # distal_l=Vector(0,-1,0)
            # normal_r=Vector(0,0,1)
            # distal_r=Vector(0,-1,0)
            normal_l=Vector(0,-1,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(0,1,0)
            distal_r=Vector(0,0,-1)

    elif obj == "dome_tall":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,1,0)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,1,0)
        elif exp_no == 2:
            goal_no = 2
            z=2
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 3
            z=2.5
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 4:
            goal_no = 1
            z=1
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)

    elif obj == "round_prism":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,1,0)
            distal_l=Vector(1,0,0)
            normal_r=Vector(0,-1,0)
            distal_r=Vector(1,0,0)
        elif exp_no == 2:
            pass
        elif exp_no == 3:
            pass
        elif exp_no == 4:
            pass 

    elif obj == "cube":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,1,0)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,1,0)
        elif exp_no == 2:
            pass
        elif exp_no == 3:
            pass
        elif exp_no == 4:
            pass 

    elif obj == "rectangular_prism_large":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,1,0)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,1,0)
        elif exp_no == 2:
            goal_no = 2
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,0,-1)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,0,1)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 3
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,0,-1)
        elif exp_no == 4:
            goal_no = 4
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(0,1,0)
            distal_r=Vector(0,0,-1)

    elif obj == "dome_short":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,1,0)
            distal_l=Vector(1,0,0)
            normal_r=Vector(0,-1,0)
            distal_r=Vector(1,0,0)
        elif exp_no == 2:
            goal_no = 2
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,0,-1)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,0,1)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 3
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(-1,0,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(1,0,0)
            distal_r=Vector(0,0,-1)
        elif exp_no == 4:
            goal_no = 4
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(0,0,-1)
            normal_r=Vector(0,1,0)
            distal_r=Vector(0,0,-1)

    elif obj == "hexagonal_prism_large":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=1
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 2:
            pass
        elif exp_no == 3:
            pass
        elif exp_no == 4:
            pass 

    elif obj == "rectangular_prism_2x4":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 2:
            goal_no = 2
            z=2
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 1
            z=0
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 4:
            goal_no = 1
            z=1
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)

    elif obj == "hexagonal_prism_tall":
        obj_type = obj
        if exp_no == 1:
            goal_no = 1
            z=0
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 2:
            goal_no = 2
            z=2
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 3:
            goal_no = 1
            z=2.5
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 4:
            goal_no = 3
            z=1
            d_l=8
            d_r=8
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)
        elif exp_no == 5:
            goal_no = 1
            z=2.5
            d_l=10
            d_r=10
            normal_l=Vector(0,-1,0)
            distal_l=Vector(-1,0,0)
            normal_r=Vector(0,1,0)
            distal_r=Vector(-1,0,0)



    return obj_type, goal_no, z, d_l, d_r, normal_l, distal_l, normal_r, distal_r