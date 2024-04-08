#include "../include/optimality/point_relationships.h"
/*Stores a point cloud, attaches a list of complimentary points and distances to each point*/

// I wish there were words to express my feelings that this is not easier.
// Build the array with
// T arrayName[vec.size()];
// And then populate it with this function.
template <class T>
void vecToArray(std::vector<T> *vec, T arr[])
{
    std::copy(vec->begin(), vec->end(), arr);
}

//First try at a boost::graph visitor to update distances. It is supposed to 
// set weights = distance from the start point to the points visible from it, and
// weights = distance from those viewsphere points to other points. 
//Almost certainly needs to be refactored, loosely cribbed from https://www.boost.org/doc/libs/1_78_0/libs/graph/example/dave.cpp
template <class Tag>
struct update_distances
    : public boost::base_visitor<update_distances<Tag>>
{
    typedef Tag event_filter;

    PointRelationship *p;

    update_distances<Tag>(PointRelationship *in)
    {
        p = in;
    }

    template <class Edge, class Graph>
    void operator()(Edge e, Graph g)
    {
        int cWeight = get(boost::edge_weight, g, e);
        int firstPoint = source(e, g);
        int secondPoint = target(e, g);
        //Check if we've already updated this distance in either direction
        double explored = -1;
        std::unordered_map<int, double> firstDists = p->distPairs->at(firstPoint);
        std::unordered_map<int, double> secondDists = p->distPairs->at(secondPoint);
        try
        {
            explored = secondDists[firstPoint];
            if(-1 != explored){
                explored = firstDists[secondPoint];
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        //If we're starting with the source point, just do some sanity checks.
        if (p->start == firstPoint)
        {
            std::pair<double, int> update = p->singlePointDistance(firstPoint, secondPoint, true);
            std::cout << cWeight << " should = " << int(100 * update.first) << std::endl;
        }
        //Avoid backtracking. Probably needs to be refactored
        else if (p->start == secondPoint && explored != -1)
        {
            std::cout << "Backwards connection, skipping" << std::endl;
        }
        else
        {
            int viewIndex = -1;
            //Don't backtrack
            bool skip = false;
            for (auto pair : *(p->sourceVisList))
            {
                if (pair.first == firstPoint)
                {
                    viewIndex = pair.second;
                }
            }
            //Should never be called
            if (viewIndex < 0)
            {
                std::cerr << "viewIndex error " << firstPoint << std::endl;
            }
            if (!skip)
            {
                std::pair<double, int> update = p->singlePointDistance(viewIndex, secondPoint, false);
                std::cout << "Updated " << firstPoint << "(" << viewIndex << ") -> " << secondPoint << " (" << update.second << ") from " << cWeight << " to " << int(100 * update.first) << " originally " << update.first << std::endl;
                p->sourceVisList->push_back(std::pair<int, int>(secondPoint, update.second));
                //Set distance to -1 to mark the node as explored from this direction
                p->distPairs->at(secondPoint)[firstPoint] = -1;
                p->distPairs->at(firstPoint)[secondPoint] = -1;
                put(boost::edge_weight, g, e, int(100 * update.first));
            }
        }
    }
};
//Not sure why the tutorial does it this way, but it does. :(
template <class Tag>
inline update_distances<Tag>
dist_update(Tag, PointRelationship *p)
{
    return update_distances<Tag>(p);
}

void PointRelationship::setInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined, pcl::PointCloud<pcl::Normal>::Ptr cloudNormal, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr o)
{
    std::cout << "-1" << std::endl;
    // Hide my lack of cpp skill with this kludge to init viewer
    ptCldVis::Ptr tempViewer(new ptCldVis("PCL Viewer"));
    std::swap(viewer, tempViewer);
    //copy pointclouds and octree in in
    objCloud = cloud;
    ptrObjNormal = cloudNormal;
    sphereCloud = sphere;
    combinedCloud = combined;
    oct = o;
    //Set up octree
    oct->setInputCloud(combinedCloud);
    oct->addPointsFromInputCloud();
    //Start size of objCloud
    maxIndex = objCloud->points.size();
    //Final number of points being considered
    finalIndex = maxIndex;
    //Set up empty lists
    distPairs = new std::vector<std::unordered_map<int, double>>;
    sourceVisList = new std::vector<std::pair<int, int>>;
    //Set up viewer
    std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    addRGB(viewer, combinedCloud, "Object", 1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "Object");
    viewer->registerKeyboardCallback(&PointRelationship::keyInput, *this);
    setup();
    processPoints();
    // Remove empty vertices from the count
    for (int i = 0; i < maxIndex; i++)
    {
        if (getPartners(i).empty())
        {
            finalIndex--;
        }
    }
    //Display fraction of points that produce grasps
    std::cout << finalIndex << ":" << maxIndex << "=" << float(finalIndex) / float(maxIndex) << "%" << std::endl;
    //Build shortest path to grasp from source point graph
    buildGraph();
}

//Viewer controls- if the user presses '0', advance.
void PointRelationship::keyInput(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.keyUp())
    {
        if (event.getKeySym() == "KP_0")
        {
            keypress = true;
        }
        else if (event.getKeySym() == "0")
        {
            keypress = true;
        }
    }
}

//Pause until the viewer presses 0.
void PointRelationship::getUserInput()
{
    while (!keypress)
    {
        stepViewer();
    }
    keypress = false;
}

//Keep the viewer spinning so the user can maneuver around it.
void PointRelationship::stepViewer()
{
    viewer->spinOnce(100);
    viewer->updatePointCloud(combinedCloud, "Object");
}

//Create empty lists for each point in the object.
void PointRelationship::setup()
{
    for (int i = 0; i < maxIndex; i++)
    {
        std::unordered_map<int, double> *current = new std::unordered_map<int, double>;
        distPairs->push_back(*current);
    }
    visPairs = new std::unordered_map<int, std::vector<int>>();
}

void PointRelationship::processPoints()
{
    // Iterate through all points and calculate visibility
    for (int i = 0; i < maxIndex; i++)
    {
        calculateVisibility(i);
    }
    for (int i = 0; i < maxIndex; i++)
    {
        std::unordered_map<int, double> current = distPairs->at(i);
        //Loop over all later points- earlier ones are already handled.
        for (int j = i + 1; j < maxIndex; j++)
        {
            //Only calc distance for pairs that can yield grasps.
            if (validPair(i, j))
            {
                // Initial distance estimate, needs to get redone
                double distance = distanceFrom(i, j);
                if (distance < 99999)
                {
                    //Mark your distance to j
                    current.insert(std::pair<int, double>(j, distance));
                    //Mark j's distance to you
                    std::unordered_map<int, double> next = distPairs->at(j);
                    next.insert(std::pair<int, double>(i, distance));
                    distPairs->at(j) = next;

                    //Track shortest edge- deprecated, CUT
                    cShortestDist = std::min(distance, cShortestDist);
                    //Add valid edges to the graph
                    edgeArray.push_back(Edge(i, j));
                    weights.push_back(max(1, int(distance * 100)));
                }
            }
        }
        distPairs->at(i) = current;
    }
}

//Check which points are visible from the point with index i
int PointRelationship::calculateVisibility(int i)
{
    pcl::PointXYZRGB cPoint = combinedCloud->points[i];
    Eigen::Vector3f origin(cPoint.x, cPoint.y, cPoint.z);
    std::vector<int> outputIndices, results;
    results = {};
    for (int j = maxIndex; j < combinedCloud->size(); j++)
    {
        // Just look at the sphere points, which will be after
        //  the object points
        pcl::PointXYZRGB pt = combinedCloud->points[j];
        Eigen::Vector3f target(pt.x, pt.y, pt.z);
        //Raytracing from origin to target- only return the first two voxels (first always holds the origin)
        oct->getIntersectedVoxelIndices(origin, target - origin, outputIndices, 2);
        if (outputIndices.size() > 1)
        {
            //Check that i and j are in the first two voxels- std::find returns the iterator if it exists,
            // and the .end() if it doesn't, so these will both fail if i and j are in outputIndices.
            if (outputIndices.end() != std::find(outputIndices.begin(), outputIndices.end(), i) &&
                outputIndices.end() != std::find(outputIndices.begin(), outputIndices.end(), j))
            {
                //Account for artificial point being added
                results.push_back(j+1);
            }
        }
        outputIndices.clear();
    }
    visPairs->insert(std::make_pair(i, results));
    return results.size();
}

//Check if point j is the nearest member of its neighbor group to point i
bool PointRelationship::nearestGroupPoint(int i, int j)
{
    std::unordered_map<int, double> partners = getPartners(j);
    // If j has no partners, don't bother connecting to it
    if (partners.empty())
        return false;
    double distance = distanceFrom(i, j, true);
    std::cout << "Distance from " << i << " to " << j << "=" << distance << std::endl;
    // If j is not visible, don't bother connecting to it
    if (distance >= 99999)
        return false;
    // If j has partners, loop over them and return false if
    //  any are closer to i
    for (auto iter = partners.begin(); iter != partners.end(); iter++)
    {
        int index = iter->first;
        double d2 = distanceFrom(i, index, true);
        //Should never happen
        if (distance < 0.0 || d2 < 0.0)
        {
            std::cout << distance << " " << d2 << std::endl;
            std::cout << "--------!!!!!!!!!----------" << std::endl;
        }
        if (d2 < distance)
        {
            std::cout << "But " << index << " distance " << d2 << " smaller" << std::endl;
            return false;
        }
    }
    return true;
}

//Fully connect new source point
int PointRelationship::fullyConnectPoint(int startIndex)
{
    int i = startIndex;
    // Only add edges not already in the adjacency list
    std::unordered_map<int, double> partners = getPartners(i);
    for (int j = 0; j < maxIndex; j++)
    {
        //Only connect to the closest point in the group- otherwise you get
        // weird cycles
        if (nearestGroupPoint(startIndex, j))
        {
            std::pair<double, int> ret = singlePointDistance(i, j, true);
            double distance = ret.first;
            int index = ret.second;
            edgeArray.push_back(Edge(i, j));
            int trueDist = max(1, int(100 * distance));
            std::cout << trueDist << " -> point " << j << std::endl;
            if (0 >= trueDist)
                std::cout << trueDist << "!!!!" << j << std::endl;
            weights.push_back(trueDist);
            sourceVisList->push_back(std::pair<int, int>(j, index));
        }
    }
    return i;
}

//Build a source point and add it to the combined and object clouds
// Source point doesn't need to be normalized- the distance
// function handles that.
void PointRelationship::insertArtificialPoint()
{
    pcl::PointXYZRGB p;
    p.x = tX;
    p.y = tY;
    p.z = tZ;
    p.r = 0.0;
    p.g = 0.0;
    p.b = 0.0;
    pcl::Normal n;
    n.normal_x = 0.0;
    n.normal_y = 0.0;
    n.normal_z = 0.5;
    objCloud->push_back(p);
    auto it = combinedCloud->begin();
    combinedCloud->insert(std::next(it, objCloud->points.size() - 1), p);
    ptrObjNormal->push_back(n);
    std::unordered_map<int, double> *current = new std::unordered_map<int, double>;
    distPairs->push_back(*current);
}

// Lovingly "inspired" by
// https://www.boost.org/doc/libs/1_78_0/libs/graph/example/dijkstra-example.cpp
void PointRelationship::buildGraph()
{
    // clearSphere();
    // buildList(247, &firstList, 1);
    // buildList(271, &firstList, 2);
    // getUserInput();
    insertArtificialPoint();
    start = fullyConnectPoint(objCloud->points.size() - 1);
    Edge edgeFormatted[edgeArray.size()];
    vecToArray(&edgeArray, edgeFormatted);
    int weightFormatted[weights.size()];
    vecToArray(&weights, weightFormatted);
    int numArcs = sizeof(edgeFormatted) / sizeof(Edge);
    graph_t g(edgeFormatted, edgeFormatted + numArcs, weightFormatted, finalIndex);
    boost::property_map<graph_t, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, g);
    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    vertex_descriptor s = vertex(start, g);
    // clearSphere();
    // buildList(247, &firstList, 1);
    // buildList(271, &firstList, 2);
    // getUserInput();

    dijkstra_shortest_paths(g, s,
                            predecessor_map(boost::make_iterator_property_map(
                                                p.begin(), get(boost::vertex_index, g)))
                                .distance_map(boost::make_iterator_property_map(
                                    d.begin(), get(boost::vertex_index, g)))
                                .visitor(boost::make_dijkstra_visitor(dist_update(boost::on_examine_edge(), this))));

    std::ofstream output("/home/diyogon/Documents/WPI/Berk_Lab/mer_lab/ros_ws/src/projects/active_vision/src/temp.txt");
    output << "distances and parents:\n";
    boost::graph_traits<graph_t>::vertex_iterator vi, vend;
    int bestDist = 1410065406;
    std::string bestPoint = "-1";
    for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi)
    {
        // Only record connected points
        if ((d[*vi] <= 2147483646) && (p[*vi] != start) && (*vi != start))
        {
            output << "distance(" << std::to_string(*vi) << ") = " << d[*vi] << ", ";
            output << "parent(" << std::to_string(*vi) << ") = " << std::to_string(p[*vi])
                   << "\n";
            if (d[*vi] < bestDist)
            {
                bestDist = d[*vi];
                bestPoint = std::to_string(*vi);
            }
        }
    }
    output << std::endl;
    std::cout << bestPoint << ": " << bestDist << std::endl;
    int parentID = p[std::stoi(bestPoint)];
    bestPath.push_back(std::stoi(bestPoint));
    bestPath.push_back(parentID);
    bestPath.push_back(0);
    // Display the best path
    int viewIndex = -1;
    for (auto pair : *sourceVisList)
        {
            if (pair.first == parentID)
            {
                viewIndex = pair.second;
            }
        }
    viewer->addArrow(combinedCloud->points[parentID], combinedCloud->points[start], 255, 0, 0, true, "arrow3");
    viewer->addArrow(combinedCloud->points[viewIndex], combinedCloud->points[start], 255, 255, 255, true, "arrow4");
    cDistanceF(std::stoi(bestPoint), parentID, false, false);
    viewer->removeShape("arrow1");
    viewer->removeShape("arrow2");
    viewer->addArrow(objCloud->points[std::stoi(bestPoint)], objCloud->points[parentID], 0, 255, 0, true, "arrow1");
    // singlePointDistance(viewIndex, std::stoi(bestPoint), false, false);

    // Debugging, cut
    std::ofstream dot_file("/home/diyogon/Documents/WPI/Berk_Lab/mer_lab/ros_ws/src/projects/active_vision/src/dijkstra-eg.dot");
    dot_file << "graph D {\n"
             << "  rankdir=LR\n"
             << "  size=\"400,300\"\n"
             << "  ratio=\"compress\"\n"
             << "  edge[style=\"bold\"]\n"
             << "  node[shape=\"circle\"]\n";

    boost::graph_traits<graph_t>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
        boost::graph_traits<graph_t>::edge_descriptor e = *ei;
        boost::graph_traits<graph_t>::vertex_descriptor u = source(e, g),
                                                        v = target(e, g);
        dot_file << std::to_string(u) << " -- " << std::to_string(v) << "[label=\""
                 << get(weightmap, e) << "\"";
        if (p[v] == u || p[u] == v)
            dot_file << ", color=\"black\"";
        else
            dot_file << ", color=\"grey\"";
        dot_file << "]";
    }
    dot_file << "}";
}

bool PointRelationship::validPair(int i, int j, bool display)
{
    p1 = objCloud->points[i];
    p2 = objCloud->points[j];
    vectA = p1.getVector3fMap() - p2.getVector3fMap();
    vectB = p2.getVector3fMap() - p1.getVector3fMap();
    gripperWidth = vectA.norm() + voxelGridSize; // Giving a tolerance based on voxel grid size
    // If grasp width is greater than the limit then skip the rest
    if (gripperWidth > maxGripperWidth)
        return false;
    try
    {
        // Using normals to find the angle
        A = std::min(pcl::getAngle3D(vectA, ptrObjNormal->points[i].getNormalVector3fMap()),
                     pcl::getAngle3D(vectB, ptrObjNormal->points[i].getNormalVector3fMap())) *
            180 / M_PI;
        if (display)
            printf("v1:%f v2:%f\n", pcl::getAngle3D(vectA, ptrObjNormal->points[i].getNormalVector3fMap()) * 180 / M_PI,
                   pcl::getAngle3D(vectB, ptrObjNormal->points[i].getNormalVector3fMap()) * 180 / M_PI);
        B = std::min(pcl::getAngle3D(vectA, ptrObjNormal->points[j].getNormalVector3fMap()),
                     pcl::getAngle3D(vectB, ptrObjNormal->points[j].getNormalVector3fMap())) *
            180 / M_PI;
        if (display)
            printf("v1:%f v2:%f\n", pcl::getAngle3D(vectA, ptrObjNormal->points[j].getNormalVector3fMap()) * 180 / M_PI,
                   pcl::getAngle3D(vectB, ptrObjNormal->points[j].getNormalVector3fMap()) * 180 / M_PI);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    quality = 180 - (A + B);
    if (display)
        printf("A: %f, B:%f, quality:%f\n", A, B, quality);
    // If grasp quality is less than the min requirement then skip the rest
    if (quality < minGraspQuality)
        return false;
    return true;
}

//Based on obsolete code, refactor.
double PointRelationship::distanceFrom(int a, int b, bool artificialPoint, bool display)
{
    if (a != currentPointIndex)
    {
        setFirstPoint(a);
        setSecondPoint(b);
        currentPointIndex = a;
        return cDistanceF(a, b, artificialPoint, display);
    }
    else
    {
        setSecondPoint(b);
        return cDistanceF(a, b, artificialPoint, display);
    }
}

//Find the distance from a single point on the viewsphere (a) to a point with line of sight 
// to a single point on the object (b). I believe artificialPoint is redundant
std::pair<double, int> PointRelationship::singlePointDistance(int a, int b, bool artificialPoint, bool display)
{
    //Remove visualization
    clearSphere();
    //First list is just the point on the viewsphere,
    // second list is the points w/ line of sight to b.
    firstList = {combinedCloud->points[a]};
    secondList = {combinedCloud->points[b]};
    if (display)
        std::cout << a << " " << b << std::endl;
    try
    {
        buildList(b, &secondList, 2);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return std::pair<double, int>(999999, -1);
    }
    //Indicate points a and b
    viewer->removeShape("arrow1");
    viewer->addArrow(combinedCloud->points[b], combinedCloud->points[a], 255, 255, 255, true, "arrow1");
    double minDist = 1410065406;
    int minPoint = -1;
    int counter = 0;
    double p1[3], p2[3];
    for (pcl::PointXYZRGB startPoint : firstList)
    {
        populateArray(p1, startPoint);
        for (pcl::PointXYZRGB endPoint : secondList)
        {
            populateArray(p2, endPoint);
            double res = pointDistances(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], false);
            viewer->removeShape("arrow2");
            viewer->addArrow(endPoint, startPoint, 0, 255, 0, true, "arrow2");
            // Remove 0 false positives
            if (res < minDist && res > .0001)
            {
                minDist = res;
                // Record the index of the visible point
                minPoint = visPairs->at(b)[counter];
                // if (252 == b)
                // {
                //     std::cout << res << " minDist to " << minPoint << " from " << a << std::endl;
                //     getUserInput();
                // }
            }
            counter++;
        }
    }
    //Clear the viewsphere
    combinedCloud->points[a].r = 0;
    combinedCloud->points[a].g = 0;
    combinedCloud->points[a].b = 255;
    combinedCloud->points[b].r = 0;
    combinedCloud->points[b].g = 0;
    combinedCloud->points[b].b = 255;
    return std::pair<double, int>(minDist, minPoint);
}

//Find the distance along the viewsphere between the closest two points that can
// see object points a and b. artificialPoint is used here to indicate when
// you don't need to build list1, aka when you should use the other distance function.
// Refactor that.
double PointRelationship::cDistanceF(int a, int b, bool artificialPoint, bool display)
{
    clearSphere();
    // Clean last arrow- if no path exists, it'll stick around otherwise.
    viewer->removeShape("arrow2");
    firstList = {objCloud->points[a]};
    secondList = {objCloud->points[b]};
    if (display)
        std::cout << a << " " << b << std::endl;
    try
    {
        buildList(a, &firstList, 1);
    }
    catch (const std::exception &e)
    {
        if (!artificialPoint)
        {
            std::cerr << e.what() << '\n';
            return 999999;
        }
        else
        {
            std::cout << "Arificial point, it's fine" << std::endl;
            firstList = {objCloud->points[a]};
        }
    }
    try
    {
        buildList(b, &secondList, 2);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 999999;
    }
    viewer->removeShape("arrow1");
    viewer->addArrow(objCloud->points[b], objCloud->points[a], 255, 255, 255, true, "arrow1");

    double minDist = 1410065406;
    double p1[3], p2[3];
    for (pcl::PointXYZRGB startPoint : firstList)
    {
        populateArray(p1, startPoint);
        for (pcl::PointXYZRGB endPoint : secondList)
        {
            populateArray(p2, endPoint);
            double res = pointDistances(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], display);
            // Remove 0 false positives
            if (res < minDist && res > .0001)
            {
                minDist = res;
                viewer->removeShape("arrow2");
                viewer->addArrow(endPoint, startPoint, 0, 255, 0, true, "arrow2");
            }
        }
    }
    if (display)
        getUserInput();
    combinedCloud->points[a].r = 0;
    combinedCloud->points[a].g = 0;
    combinedCloud->points[a].b = 255;
    combinedCloud->points[b].r = 0;
    combinedCloud->points[b].g = 0;
    combinedCloud->points[b].b = 255;
    return minDist;
}

//Uncolor the viewsphere
void PointRelationship::clearSphere()
{
    for (int j = 0; j < sphereCloud->size(); j++)
    {
        combinedCloud->points[j + maxIndex].r = 0;
        combinedCloud->points[j + maxIndex].g = 0;
        combinedCloud->points[j + maxIndex].b = 255;
    }
}

void PointRelationship::buildList(int sourcePoint, std::vector<pcl::PointXYZRGB> *targetList, int color)
{
    targetList->clear();
    std::vector<int> visibleIndices = visPairs->at(sourcePoint);
    // Color the origin point to match its ROV
    if (1 == color)
    {
        combinedCloud->points[sourcePoint].r = 255;
    }
    else if (2 == color)
    {
        combinedCloud->points[sourcePoint].g = 255;
    }

    // Loop over the ROV, coloring the points and adding them
    //  to the list to be updated.
    //  std::cout << visibleIndices.size() << std::endl;
    for (int i : visibleIndices)
    {
        pcl::PointXYZRGB c = combinedCloud->points[i];
        targetList->push_back(c);
        if (1 == color)
        {
            combinedCloud->points[i].r = 255;
        }
        else if (2 == color)
        {
            combinedCloud->points[i].g = 255;
        }
    }
}

//Helper
void PointRelationship::populateArray(double target[], pcl::PointXYZRGB source)
{
    target[0] = source.x;
    target[1] = source.y;
    target[2] = source.z;
}

// Great circle distance f(x), see https://en.wikipedia.org/wiki/Great-circle_distance#Vector_version
double PointRelationship::pointDistances(double x1, double y1, double z1, double x2, double y2, double z2, bool display)
{
    Eigen::Vector3d v1(x1, y1, z1);
    Eigen::Vector3d v2(x2, y2, z2);
    v1.normalize();
    v2.normalize();
    double dotP = v1.dot(v2);
    double magCross = v1.cross(v2).norm();
    double ret = std::atan(magCross / dotP);
    if (ret < 0)
        ret += M_PI;
    if (display)
    {
        printf("A=(%f, %f, %f)\n", x1, y1, z1);
        printf("B=(%f, %f, %f)\n", x2, y2, z2);
        printf("%f->%f\n", dotP, ret);
    }
    return ret;
}

void PointRelationship::setFirstPoint(int a)
{
    calculatePoint(a, firstPoint);
}

void PointRelationship::setSecondPoint(int a)
{
    calculatePoint(a, secondPoint);
}

//Project the point onto the viewsphere. I believe this is now obsolete.
void PointRelationship::calculatePoint(int ind, Point_3 &setPoint)
{
    pcl::PointXYZRGB p = objCloud->points[ind];
    pcl::Normal n = ptrObjNormal->points[ind];
    n.normal_x *= -.61;
    n.normal_y *= -.61;
    n.normal_z *= -.61;
    temp.x = n.normal_x + p.x;
    temp.y = n.normal_y + p.y;
    temp.z = n.normal_z + p.z;
    SK::Point_3 cPoint(SK::Point_3(p.x, p.y, p.z));
    SK::Point_3 cNormal(SK::Point_3(temp.x, temp.y, temp.z));
    SK::Sphere_3 s1(SK::Point_3(0, 0, 0), 1.0);
    SK::Line_3 proj(SK::Line_3(cPoint, cNormal));
    CGAL::intersection(s1, proj, std::back_inserter(intersecs));
    for (int i = 0; i < intersecs.size(); i++)
    {
        if (const std::pair<SK::Circular_arc_point_3, unsigned> *cP1 = CGAL::object_cast<std::pair<SK::Circular_arc_point_3, unsigned>>(&intersecs.at(i)))
        {
            temp1.x = cP1->first.bbox().xmin();
            temp1.y = cP1->first.bbox().ymin();
            temp1.z = cP1->first.bbox().zmin();
            SK::Point_3 cP2(SK::Point_3(temp1.x, temp1.y, temp1.z));
            // Returns A->B's relationship to A->C
            CGAL::Comparison_result realIntersect = (CGAL::compare_distance_to_point(cNormal, cP2, cPoint));
            if (realIntersect == CGAL::SMALLER)
            {
                setPoint = cP2;
            }
            else if (realIntersect == CGAL::LARGER)
            {
                // printf("Wrong one %d\n", i);
            }
            else
            {
                printf("Huh?\n");
            }
        }
    }
}

//Return the indexes of the object points complimentary to object
// point ind.
std::unordered_map<int, double> PointRelationship::getPartners(int ind)
{
    return distPairs->at(ind);
}