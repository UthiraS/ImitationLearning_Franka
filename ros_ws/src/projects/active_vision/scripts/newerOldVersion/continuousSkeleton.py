if __name__ == "__main__":
    1. Load pointcloud
    2. Eliminate all parts of it which cannot contain grasps
        a. Too close to the edge
        b. Surface geometry too rough
    3. For all survivors, check for suitable grasp pairs
        a. For all grasp pairs, draw their visibility on the viewsphere
        b. If there are any grasp pairs, draw the point's visibility on
            the viewsphere. Calculate and record the shortest and farthest
            distances between point and counterpoint
            - Keep global records of that as well.
    4. Save the marked viewsphere
    5. For any start point, look at all points visible
        a. Calculate the true distance to their counterpoints. Take the shortest.
        b. While shortest > global optimal, expand by delta=(global optimal-shortest)
        c. If delta+distance < shortest, calculate true distance (how?)
        