# Function definitions regarding object surfaces

import numpy as np
# from shapely.geometry import Polygon, LineString, MultiPoint
# from shapely.geometry import Point as sPoint
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint

def plot_3d_object(object_):
    
    """
    Given the object geometry generate a 3D plot using Geometry3D library Renderer instance

    Parameters
    ----------
    object_: dict
        Object dictionary including information of surface polygons and goal regions
    """
    
    # Initialize renderer instance
    r = Renderer()

    # Add surfaces and goal regions to the renderer instance
    for surf in object_:
        r.add((object_[surf][0],'b',1))
        if len(object_[surf])>2:
            r.add((object_[surf][2],'r',1))
    r.add((gPoint(-15,-15,-15),'k',1))
    r.show()

def get_neighbors(object_):
    
    """
    Given the object geometry, determine neighbors for each surface

    Parameters
    ----------
    object_: dict
        Object dictionary including information of surface polygons, normal vectors, and goal regions

    Returns
    ----------
    neighbors: dict
        A nested dictionary in following form:
                     int        : Geometry3D ConvexPolygon, Geometry3D Vector             , Geometry3D Point               , float
        {surface_no:{neighbor_no:(neighbor_surface_polygon, vector_along_intersection_edge, one_of_the_intersecting_corners, angle_between_surfaces)}}
    """

    # Initialize neighbors dictionary
    neighbors = dict()

    # For each surface in object dictionary
    for surf in object_:

        # Selected surface
        current_surface = object_[surf][0]

        # Surface normal
        current_normal = object_[surf][1]

        # Rest of the surfaces
        dummy_surfaces = object_.copy()
        dummy_surfaces.pop(surf)

        # Initialize nested dictionary for selected surface
        neighbors[surf] = dict()

        # For each candidate surface (from rest of the surfaces)
        for n in dummy_surfaces:
            # Candidate surface
            candidate_surface = dummy_surfaces[n][0]
            # Candidate normal
            candidate_normal = dummy_surfaces[n][1]
            # Check if there is an intersection - there should be a line intersection if neighboring surfaces
            check = intersection(current_surface,candidate_surface)
            if check is not None:
                # Intersection vector
                rotation_axis = candidate_normal.cross(current_normal)
                # Angle between surfaces
                angle = candidate_normal.angle(current_normal)
                # Corner position
                axis_position = gPoint((check[0].x+check[1].x)/2,(check[0].y+check[1].y)/2,(check[0].z+check[1].z)/2)
                neighbors[surf][n] = ((candidate_surface,rotation_axis,axis_position,angle))
    return neighbors

def unfold_surface(surface_dict,neighbors_dict,surf_idx,other,neighbor,show=False):
    
    """
    Given the object surface dict and neighbors dict, generate an unfolded surface for the desired surface centering a selected surface

    Parameters
    ----------
    surface_dict: dict
        Object dictionary including information of surface polygons, normal vectors, and goal regions
    neighbors_dict: dict
        A nested dictionary including information of surfaces and corresponding neighboring relationships
    surf_idx: int
        Number assigned to the center surface
    other: tuple (Geometry3D.ConvexPolygon,Geometry3D.Vector,Geometry3D.ConvexPolygon)
        information of folded surface to be unfolded - surface polygon, normal vector, goal region
    neighbor: tuple (Geometry3D.ConvexPolygon,Geometry3D.Vector,Geometry3D.Point,float)
        neighboring information - neighbor surface polygon, vector along intersection edge, intersecting corners, angle between surfaces
    show: bool
        generates a plot if True

    Returns
    ----------
    (nsurf,nnormal,ngoal): tuple
        unfolded surface, unfolded normal, unfolded goal (if available)
    
    """

    # Visualization
    p = Renderer()
    p.add((surface_dict[surf_idx][0],'r',1))

    # Normal of the center surface
    current_normal = surface_dict[surf_idx][1]
    # Normal of the neighboring surface
    candidate_normal = other[1]

    # Angle between surfaces
    angle = candidate_normal.angle(current_normal)

    # Rotation calculations (Finding transformation matrix)
    A,B,C = neighbor[1]
    L = np.sqrt(A**2 + B**2 + C**2)
    V = np.sqrt(B**2 + C**2)
    D = np.array([[1,0,0,-neighbor[2][0]],[0,1,0,-neighbor[2][1]],[0,0,1,-neighbor[2][2]],[0,0,0,1]])
    if V == 0:
        R_x = np.eye(4)
    else:
        R_x = np.array([[1,0,0,0],[0,C/V,-B/V,0],[0,B/V,C/V,0],[0,0,0,1]])
    R_y = np.array([[V/L,0,-A/L,0],[0,1,0,0],[A/L,0,V/L,0],[0,0,0,1]])
    R_z = np.array([[np.cos(angle),-np.sin(angle),0,0],
            [np.sin(angle),np.cos(angle),0,0],
            [0,0,1,0],[0,0,0,1]])
    T = np.linalg.inv(D)@np.linalg.inv(R_x)@np.linalg.inv(R_y)@R_z@R_y@R_x@D

    # Applying transformation
    P_init = np.empty((4,0))
    for point in other[0].points:
        point_vec = np.array([point.x,point.y,point.z,1]).reshape(4,1)
        P_init = np.concatenate([P_init,point_vec],axis=1)
    normal_vec = np.array([other[0].points[-1].x+candidate_normal[0],other[0].points[-1].y+candidate_normal[1],other[0].points[-1].z+candidate_normal[2],1]).reshape(4,1)
    P_init = np.concatenate([P_init,normal_vec],axis=1)
    P_final = T@P_init
    new_points = list()
    for i in range(P_final.shape[1]-1):
        new_points.append(gPoint(np.round(P_final[:3,i],decimals=3)))
    
    # New normal vector
    nnormal = Vector(P_final[:3,-1]-P_final[:3,-2])

    # New surface definition as convex polygon
    nsurf = ConvexPolygon((new_points))

    # Transform goal region as well
    if len(other)>2:
        G_init = np.empty((4,0))
        for point in other[2].points:
            point_vec = np.array([point.x,point.y,point.z,1]).reshape(4,1)
            G_init = np.concatenate([G_init,point_vec],axis=1)
        G_final = T@G_init
        new_goal = list()
        for i in range(G_final.shape[1]):
            new_goal.append(gPoint(np.round(G_final[:3,i],decimals=3)))
        ngoal = ConvexPolygon((new_goal))
        
        p.add((nsurf,'k',1))
        p.add((ngoal,'k',1))
        if show:
            p.add((other[0],'k',1))
            p.show()
        return (nsurf,nnormal,ngoal)

    else:
        p.add((nsurf,'k',1))
        if show:
            p.add((other[0],'k',1))
            p.show()
        return (nsurf,nnormal)

def unfold_object(current_surface,surface_dict,neighbors_dict):

    """
    Given the object surface dictionary and neighbors dictionary, unfold rest of the surfaces with a selected surface at the center

    Parameters
    ----------
    current_surface: int
        Number assigned to centering surface
    surface_dict: dict
        Object dictionary including information of surface polygons, normal vectors, and goal regions
    neighbors_dict: dict
        A nested dictionary including information of surfaces and corresponding neighboring relationships

    Returns
    ----------
    unfolded_surfaces: dict
        A dictionary similar to object dictionary, where the center surface remains the same but other surfaces are modified through unfolding
    """

    # Initialize a dictionary for the unfolded surfaces
    unfolded_surfaces = dict()

    # Generate a list of surface numbers
    surface_list = list(surface_dict.keys())

    # Add center surface to dictionary without modification
    unfolded_surfaces[current_surface] = surface_dict[current_surface]

    # Remove center surface from surface numbers list
    surface_list.remove(current_surface)

    # Generate an open list that contains the neighboring surface numbers to the center surface
    open_list = [(neighbor,[current_surface]) for neighbor in neighbors_dict[current_surface]]

    # Run until all surfaces are added to the unfolding
    while len(surface_list) > 0:

        # Next item in open list
        item = open_list[0]

        # Initialize a list of parents
        parents = list()

        # If neighbor is not already added
        if item[0] in surface_list:
            # Get folded version of the surface
            folded_surf = surface_dict[item[0]]
            child = item[0]

            # while there are still other parents of neighboring surface 
            # (when more than one unfolding is required for a neighbor of the neighbor)
            while len(item[1])>0:
                # get parent surface no
                parent = item[1][-1]
                # unfold surface on the parent
                folded_surf = unfold_surface(surface_dict,neighbors_dict,item[1][-1],folded_surf,neighbors_dict[parent][child])
                # parent becomes the child
                child = item[1][-1]
                # remove parent and add to the parents list
                parents.append(item[1].pop())

            unfolded_surf = folded_surf

        else:
            open_list.pop(0)
            continue

        unfolded_surfaces[item[0]] = unfolded_surf
        
        surface_list.remove(item[0])

        parents.reverse()
        # for neighbors of the current neighbor
        for neighbor in neighbors_dict[item[0]]:
            # if not already added
            if neighbor in surface_list:
                # add to the open list with correct parenting order
                open_list.append((neighbor,parents+[item[0]]))
            else:
                continue
        open_list.pop(0)
    return unfolded_surfaces