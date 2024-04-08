#include "../include/optimality/opt_path_finder.h"


void PathVis::setup(ptCldColor::Ptr combined, ptCldColor::Ptr object, ptCldNormal::Ptr normals, string name, int numberOfViewpoints)
{
  AVVis::setup(name, numberOfViewpoints);
  _combined = combined;
  setRGB(_combined, "combined");
//   viewer->addCoordinateSystem(0.5, "reference", 0);
//   viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(object, normals, 1, 0.01, "normals", _vp->at(0));
}

void PathVis::stepVis(bool hardStop)
{
  if(!_initialized) return;
  if (hardStop)
  {
    _keypress = WAIT;
  }
  viewer->updatePointCloud(_combined, "combined");
  AVVis::stepVis();
}

// Creates the default PathFinder object with no logging or graphics
PathFinder::PathFinder()
{
    PathFinder(0, false);
}

// Allows the user to set log level and graphics
// Logging 0- Silent, no logging
//         1- Minimal logging
//         2- Verbose logging
//         3- Maximal logging. Do not try to print this to terminal
// TODO- better definition
PathFinder::PathFinder(int log, bool graph)
{
    logging = log;
    graphics = graph;
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "PathFinder");
    ros::NodeHandle nh;
    g = new graspSynthesis(&nh);
}

void PathFinder::updateLogging(int log){
    logging = log;
    graphics = (log >= 2);
    if(graphics){
        visualizer->setup(combinedCloud, objCloud, ptrObjNormal, "Planning View", views);
    }
}

// Sets up PathFinder with the user provided object and settings.
// This must be run before calling any other PathFinder functions
// cloud- The point cloud of the object to be analyzed
// sphere- The viewsphere points the camera is restricted to
// combined- The sum of cloud and sphere
// cloudNormal- The precalculated normals of the cloud object
// o- An octree to do collision checking with. It should be empty when passed in.
// angle- The maximum difference between the normal of a point on the object and a position that has line of sight to it
// runSetup- Kludge for discrete search, enables or disables the datastructures that continuous uses to save time
// discrete- whether to run continuous or discrete search
// TODO- retry calculating octree/combined/normals/sphere internally. Seperate out discrete search
void PathFinder::initialize(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr object, PointCloud<PointXYZRGB>::Ptr sphere, PointCloud<PointXYZRGB>::Ptr combined, PointCloud<PointNormal>::Ptr pObjNormal, PointCloud<PointXYZRGB>::Ptr unexpCloud, octree::OctreePointCloudSearch<PointXYZRGB>::Ptr o, double angle, bool runSetup, bool discrete)
{
    if (logging)
        printf("Starting init...\n");
    // copy pointclouds and octree in in
    viewAndObjectCloud = cloud;
    objCloud = object;
    ptrUnexpCloud = unexpCloud;
    sphereCloud = sphere;
    combinedCloud = combined;
    ptrObjNormal = pObjNormal;
    oct = o;
    viewAngle = angle;
    nonDiscrete = !discrete;
    KdTree->setInputCloud(object);
    boost::shared_ptr<PointCloud<PointXYZRGB>> tempCloud(new PointCloud<PointXYZRGB>(*object));
    boost::shared_ptr<PointCloud<PointXYZRGB>> tempCloud2(new PointCloud<PointXYZRGB>(*ptrUnexpCloud));
    g->setNormals(ptrObjNormal);
    g->preprocessing(tempCloud, tempCloud2);
    g->debugCollisionCheck = (2 < logging);
    g->calcGraspPairs();
    // while(ptrObjNormal->size() < viewAndObjectCloud->size()){
    //     pcl::PointNormal *n = new pcl::PointNormal();
    //     n->normal_x = 0.0;
    //     n->normal_y = 0.0;
    //     n->normal_z = 1.0;
    //     ptrObjNormal->push_back(*n);
    //     // refined->push_back(*n);
    // }
    // Set up octree
    oct->defineBoundingBox(2);
    oct->setResolution(voxelGridSize);
    oct->setInputCloud(combinedCloud);
    // oct->setInputCloud(object);
    oct->addPointsFromInputCloud();
    // Start size of viewAndObjectCloud
    maxObjectIndex = object->points.size();
    maxCollisionIndex = viewAndObjectCloud->points.size();
    maxCombinedIndex = combinedCloud->points.size();
    if(logging) cout << "Obj size = " << maxObjectIndex << "collision size = " << maxCollisionIndex << " total size = " << maxCombinedIndex << endl;
    // Final number of points being considered
    nObjectPoints = maxObjectIndex;
    // Set up empty lists
    graspPairs = new vector<vector<int>>;
    sourceVisList = new unordered_map<int, int>;
    distList = new unordered_map<int, double>;
    objMarginal = new unordered_map<int, bool>;
    distPartnerList = new unordered_map<int, int>;
    graspPartnerList = new unordered_map<int, pair<int,int>>;
    // Set up viewer
    visualizer = new PathVis();
    if(graphics){
        // Hide my lack of cpp skill with this kludge to init viewer
        visualizer->setup(combinedCloud, object, ptrObjNormal, "viewer", views);
        // showVoxelBounds(viewer, oct, 0);
        // std::cout << "Received points " << refined->points.size() << std::endl;
        // viewer->removeShape("normals");
        // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (viewAndObjectCloud, refined, 1, 0.01, "normals_ref");
        // getUserInput();
    } 
    if(runSetup)
    {
        setup();
        if (logging){
            cout << nObjectPoints << " points remaining of " << maxObjectIndex << endl;
            cout << int(18 * log10(nObjectPoints)) << " Finished init." << endl;
            if(graphics && 0 == nObjectPoints) visualizer->stepVis();
        }
        savedSourceVisList = new unordered_map<int, int>;
        savedVisPairs = new unordered_map<int, vector<int>>();
        savedDistList = new unordered_map<int, double>;
        savedObjMarginal = new unordered_map<int, bool>;
        savedDistPartnerList = new unordered_map<int, int>;
        savedGraspPartnerList = new unordered_map<int, pair<int,int>>;
        *savedSourceVisList = *sourceVisList;
        *savedVisPairs = *visPairs;
        *savedDistList = *distList;
        *savedObjMarginal = *objMarginal;
        *savedDistPartnerList = *distPartnerList;
        *savedGraspPartnerList = *graspPartnerList;
    }
    if(graphics){
        // getUserInput();
        visualizeOverViewpoints();
    }
 }


/// @brief Tests whether a grasp can be found in exactly depth steps
/// @param depth The BFS depth to search to
/// @return True IFF a grasp can be found in exactly depth steps
/// @details Assumes that the points in the viewsphere have been added by addContPoints in discrete search
bool PathFinder::getNSteps(int depth)
{
    //Calculate how many paths to check,
    // only checking paths of exactly length depth
    int finalNumber = pow(8, depth);
    for(int i=0; i<finalNumber; i++)
    {
        vector<int> cPath;
        int prev = 0;
        int cDepth = 0;
        cPath.push_back(0+maxCollisionIndex);
        //Converting to octal gives a good representation
        // of which o the 8 compass directions to take at each step.
        // e.g. 15=[0,1,7] would correspond to take direction 0, then 1, then 7.
        for(int j : decimalToOctal(i, depth)){
            cPath.push_back(prev*8+j+maxCollisionIndex);
            prev = prev*8+j;
            cDepth++;
        }
        if(graspVisibleFrom(cPath))
        {
            if(logging)
            {
                for(int i : cPath)
                {
                    cout << i << " <-";
                }
                cout << endl;
            }
            if(graphics)
                displayFullPath(cPath);
            return true;
        }
    }
    return false;
}

/// @brief Check if a valid grasp can be found among the object points visible from the viewsphere at any of viewIndices
/// @param viewIndices The indices of the viewsphere points to be checked
/// @return Whether or not a grasp is possible
bool PathFinder::graspVisibleFrom(vector<int> viewIndices)
{
    for (int i=0; i<maxObjectIndex; i++)
    {
        if(!singlePointVisibile(viewIndices, i))
        {
            continue;
        }
        vector<int> partners = getPartners(i);
        for(int partner : partners)
        {
            if(partner < i)
            {
                continue;
            } 
            else if(singlePointVisibile(viewIndices, partner))
            {
                return true;
            }
            //Can be optimized to reduce repeat calls
        }
    }
    return false;
}

//Checks if pointIndex is visible from any point in viewIndices
bool PathFinder::singlePointVisibile(vector<int> viewIndices, int pointIndex)
{
    //Take the viewsphere indices that can view pointIndex
    vector<int> cVis = visPairs->at(pointIndex);
    vector<int> v_intersection;

    //Check if viewIndices and those indices overlap at all
    set_intersection(viewIndices.begin(), viewIndices.end(),
                        cVis.begin(), cVis.end(),
                        back_inserter(v_intersection));
    return (v_intersection.size() > 0);
}

/*  1) Initialize data structures
    2) Build visibility map for object + pcd
    3) Build grasp pair groups */
// TODO split off discrete and continuous
void PathFinder::setup()
{
    if (logging)
        printf("Starting setup...\n");
    // *origVisPairs = *visPairs;
    buildVis();
    for (int i = 0; i < maxObjectIndex; i++)
    {
        vector<int> *current = new vector<int>;
        graspPairs->push_back(*current);
    }
    buildGroups();
    // Remove empty vertices from the count
    for (int i = 0; i < maxObjectIndex; i++)
    {
        if (getPartners(i).empty())
        {
            nObjectPoints--;
        }
    }
    if(nonDiscrete)
        findGroupDistances();
    if (logging)
    {
        cout << "Finished setup!" << endl;
    }
}

// Find visibility for all object points
void PathFinder::buildVis()
{
    for (int i = 0; i < maxObjectIndex; i++)
    {
        int s = calculateVisibility(i);
        // if(logging > 1)
        //     cout << i << " visible from " << s << endl;
    }
}
/*
/// @brief Check which viewsphere points are visible from object point i
/// @param i The index in combinedCloud of the object point to be examined
/// @return The number of viewpoints i is visible from
int PathFinder::calculateVisibility(int i)
{
    int N = 9;
    PointXYZRGB cPoint = combinedCloud->points[i];
    Eigen::Vector3f origin(cPoint.x, cPoint.y, cPoint.z);
    vector<int> outputIndices, results, resultsI;
    results = {};
    //First, check if the point can be grasped.
    bool internalPoint = (i >= maxObjectIndex);
    // if(internalPoint || !g->isContactPatchOk(i, 26)){
    // if(internalPoint || !g->validForGrasping(i)){
    // // if(internalPoint){
    //     if(graphics)
    //     {
    //         combinedCloud->points[i].b = 255;
    //     }
    //     //Every object pair needs an entry in visPairs- this just inserts
    //     // an empty vector
    //     visPairs->erase(i);
    //     visPairs->insert(make_pair(i, results));
    //     return 0;
    // }
    //Mark flat points which are valid, but only just
    // objMarginal->insert(make_pair(i, !g->isContactPatchOk(i, 39)));
    objMarginal->insert(make_pair(i, false));
    

    // if(visPairs->at(i).size() < minVisSize){
    //     visPairs->erase(i);
    //     visPairs->insert(make_pair(i, results));
    // }
    vector<float> distances = {};
    vector<int> indices = {};
    distances.resize(N);
    indices.resize(N);
    for(int viewIndex : visPairs->at(i)){
        if(!results.empty()) continue;
        distances.clear();
        indices.clear();
        oct->nearestKSearch(viewIndex, N, indices, distances);
        bool use = true;
        for(int index : indices){
            // AVLOG(to_string(index), 0, 0);
            if(use && !vectorContainsElement<int>(visPairs->at(i), index)){
                use = false;
            }
        }
        if(use){
            results = visPairs->at(i);
        }
    }
    visPairs->erase(i);
    visPairs->insert(make_pair(i, results));

    //Mark valid points which are occluded
    if(graphics && 0 >= visPairs->at(i).size()){
        combinedCloud->points[i].r = 0;
        combinedCloud->points[i].g = 0;
        combinedCloud->points[i].b = 0;
    }
    if(graphics && objMarginal->at(i)){
        combinedCloud->points[i].r = 0;
        combinedCloud->points[i].g = 255;
        combinedCloud->points[i].b = 0;
    }
    return visPairs->at(i).size();
}
*/

/// @brief Check which viewsphere points are visible from object point i
/// @param i The index in combinedCloud of the object point to be examined
/// @return The number of viewpoints i is visible from
int PathFinder::calculateVisibility(int i)
{
    int N = 0;
    PointXYZRGB cPoint = combinedCloud->points[i];
    Eigen::Vector3f origin(cPoint.x, cPoint.y, cPoint.z);
    vector<int> outputIndices, results, resultsI;
    results = {};
    //First, check if the point can be grasped.
    bool internalPoint = (i >= maxObjectIndex);
    // if(internalPoint || !g->isContactPatchOk(i, 26)){
    // if(internalPoint || !g->validForGrasping(i)){
    // // if(internalPoint){
    //     if(graphics)
    //     {
    //         combinedCloud->points[i].b = 255;
    //     }
    //     //Every object pair needs an entry in visPairs- this just inserts
    //     // an empty vector
    //     visPairs->erase(i);
    //     visPairs->insert(make_pair(i, results));
    //     return 0;
    // }
    //Mark flat points which are valid, but only just
    // objMarginal->insert(make_pair(i, !g->isContactPatchOk(i, 39)));
    objMarginal->insert(make_pair(i, false));
    

    // if(visPairs->at(i).size() < minVisSize){
    //     visPairs->erase(i);
    //     visPairs->insert(make_pair(i, results));
    // }
    vector<float> distances = {};
    vector<int> indices = {};
    distances.resize(N);
    indices.resize(N);
    for(int viewIndex : visPairs->at(i)){
        if(!results.empty()) break;
        distances.clear();
        indices.clear();
        oct->nearestKSearch(viewIndex, N, indices, distances);
        bool use = true;
        for(int index : indices){
            // AVLOG(to_string(index), 0, 0);
            if(use && !vectorContainsElement<int>(visPairs->at(i), index)){
                use = false;
            }
        }
        //If any points are still valid, use all of them.
        if(use){
            results = visPairs->at(i);
        }
    }
    visPairs->erase(i);
    visPairs->insert(make_pair(i, results));
    // N=3;
    // if(!results.empty()){
    //     for(int j=maxCollisionIndex; j<maxCombinedIndex; j++){
    //         distances.clear();
    //         indices.clear();
    //         oct->nearestKSearch(j, N, indices, distances);
    //         for(int index : indices){
    //             // AVLOG(to_string(index), 0, 0);
    //             if(vectorContainsElement<int>(visPairs->at(i), index) && !vectorContainsElement<int>(results, j) ){
    //                 results.push_back(j);
    //             }
    //         }
    //     }
    // }
    // visPairs->erase(i);
    // visPairs->insert(make_pair(i, results));

    //Mark valid points which are occluded
    if(graphics && 0 >= visPairs->at(i).size()){
        combinedCloud->points[i].r = 0;
        combinedCloud->points[i].g = 0;
        combinedCloud->points[i].b = 0;
    }
    if(graphics && objMarginal->at(i)){
        combinedCloud->points[i].r = 0;
        combinedCloud->points[i].g = 255;
        combinedCloud->points[i].b = 0;
    }
    return visPairs->at(i).size();
}


bool PathFinder::checkAngleMultiRequirement(int objectIndex, Eigen::Vector3f normal, Eigen::Vector3f origin, Eigen::Vector3f target)
{
    int k = 8;
    int cNeighbor = 0;
    bool ret = checkAngleRequirement(objectIndex, 1*normal, origin, target);
    Indices neighbors;
    std::vector<float> distances;
    KdTree->nearestKSearch(combinedCloud->points[objectIndex], k, neighbors, distances);
    while(ret && cNeighbor < k){
        int cIndex = neighbors.at(cNeighbor);
        PointXYZRGB pt = combinedCloud->points[cIndex];
        Eigen::Vector3f cOrigin(pt.x, pt.y, pt.z);
        // ret = checkAngleRequirement(cIndex, ptrObjNormal->points[cIndex].getNormalVector3fMap(), origin, cOrigin);
        ret = checkAngleRequirement(cIndex, -1*ptrObjNormal->points[cIndex].getNormalVector3fMap(), cOrigin, target) || checkAngleRequirement(cIndex, ptrObjNormal->points[cIndex].getNormalVector3fMap(), cOrigin, target);
        cNeighbor++;
    }
    return ret;
}

bool PathFinder::checkAngleRequirement(int objectIndex, Eigen::Vector3f normal, Eigen::Vector3f origin, Eigen::Vector3f target)
{
    
    Eigen::Vector3f viewPoint = origin - .01*normal;
    //This could infinte loop, fix eventually.
    while(abs(viewPoint.norm()-1) > .01)
    {
        viewPoint -= .01*normal;
    }
    double distance = radiansToDegrees(sphericalDistance(viewPoint, target));
    // if(objMarginal->at(objectIndex))
    //     return (distance < viewAngle-0);
    return (distance < viewAngle);
}

// Find complimentary groups for all visible object points
void PathFinder::buildGroups()
{
    graspPoint graspTemp;
    for (int i = 0; i < maxObjectIndex; i++)
    {
        // Exclude non-visible points
        if (!(visPairs->at(i).size() > 0)) continue;

        bool partners = false;
        vector<int> current = graspPairs->at(i);
        // Loop over all later points- earlier ones are already handled.
        for (int j = i + 1; j < maxObjectIndex; j++)
        {
            // Only add pairs that can yield grasps.
            if (0 >= visPairs->at(j).size()) continue;
            if (false == g->genGraspPoint(i, j, graspTemp)) continue;
            partners = true;
            //Check for collisions with the model
            g->clearGrasps();
            g->addGraspDetails(graspTemp);
            g->setupCollisionCheck();
            if(g->collisionCheckSingleGrasp(0))
            {
                // Mark i and j's relationship to eachother
                current.push_back(j);
                vector<int> next = graspPairs->at(j);
                next.push_back(i);
                // Update j
                graspPairs->at(j) = next;
                pathPresent = true;
            }
        }
        graspPairs->at(i) = current;
        if(current.size() > 0)
        {
            combinedCloud->points[i].b = 0;
        } 
        // else if(logging && partners) {
        //     cout << i << " had valid grasps, they just didn't work" << endl;
        // } else if(logging){
        //     cout << i << " had no potential partners" << endl;
        // }
    }
    if(logging) cout << "Pair construction finished!" << endl;
}

//Find shortest distances between all used pairs of viewpoints
void PathFinder::findGroupDistances()
{
    for (int i = 0; i < maxObjectIndex; i++)
    {
        //Only use non-empty points
        if (!getPartners(i).empty())
        {
            for(int viewPoint : visPairs->at(i))
            {
                findSingleGroupDistance(i, viewPoint);
            }
        }
    }
}

void PathFinder::findSingleGroupDistance(int objIndex, int viewPoint){
    double bestDist = 9999;
    double cDist = 9999;
    int bestPoint = -1;
    int partner = -1;
    int start = objIndex;
    //Check if we've used this key before- if so, 
    // we want to import its old settings.
    bool redo = (1 == distList->count(viewPoint));
    if(redo)
    {
        bestDist = distList->at(viewPoint);
        distList->erase(viewPoint);
        bestPoint = distPartnerList->at(viewPoint);
        distPartnerList->erase(viewPoint);
        start = graspPartnerList->at(viewPoint).first;
        partner = graspPartnerList->at(viewPoint).second;
        graspPartnerList->erase(viewPoint);
        AVLOG("Rechecking "+to_string(viewPoint), logging, 3);
        AVLOG(to_string(bestDist) + " " + to_string(bestPoint), logging, 3);
    } 
    //findGroupDistances checks that objIndex has partners, so getPartners can't be empty
    for(int j : getPartners(objIndex))
    {
        //partners need to be visible from somewhere, so visPairs can't be empty
        for(int viewPoint2 : visPairs->at(j))
        {
            cDist = doublePointDistance(viewPoint, viewPoint2);
            if(cDist < bestDist)
            {
                bestDist = cDist;
                bestPoint = viewPoint2;
                partner = j;
                start = objIndex;
            }
        }
    }
    if(9999 == bestDist){
        AVLOG(to_string(objIndex) + " has no compliment! Skipping\n", logging, 3);
        return;
    }
    distList->insert(make_pair(viewPoint, bestDist));
    distPartnerList->insert(make_pair(viewPoint, bestPoint));
    graspPartnerList->insert(make_pair(viewPoint, make_pair(start, partner)));
    AVLOG("Found distance for " + to_string(viewPoint) + " = " + to_string(distList->at(viewPoint)), logging, 3);
    if(bestPoint < 0){
        AVLOG(to_string(bestPoint) + "AAAAAAAAAAAA\n\n\n", logging, 3);
    }
    
}

vector<int> PathFinder::getPartners(int ind)
{
    return graspPairs->at(ind);
}

/// @brief Iterates over all points in the viewsphere, highlighting the object points they have line of sight to in red
void PathFinder::visualizeOverViewpoints(){
    for(int j=0; j<maxObjectIndex; j++){
        combinedCloud->points[j].r = 0;
    }
    for(int i=0; i<maxObjectIndex; i++){
        bool skip = true;
        for(int j=maxCollisionIndex; j<maxCombinedIndex; j++){
            combinedCloud->points[j].r = 0;
            combinedCloud->points[j].g = 0;
            combinedCloud->points[j].b = 0;
            if(vectorContainsElement<int>(origVisPairs->at(i), j))
            {
                combinedCloud->points[j].b = 255;
                skip = false;
            }
            if(vectorContainsElement<int>(visPairs->at(i), j))
            {
                combinedCloud->points[j].b = 0;
                combinedCloud->points[j].r = 255;
                // skip = false;
            }
        }
        visualizer->viewer->addSphere(combinedCloud->points[i], .045, 255, 0, 0, "cp");
        if(!skip)
            visualizer->stepVis();
        visualizer->viewer->removeAllShapes();
    }
    
    for(int j=maxCollisionIndex; j<maxCombinedIndex; j++){
        combinedCloud->points[j].r = 0;
        combinedCloud->points[j].b = 0;
    }
    for(int j=0; j<maxObjectIndex; j++){
        combinedCloud->points[j].r = 255;
    }
}

double PathFinder::searchFrom(double theta, double phi)
{
    playNext = false;
    insertArtificialPoint(1.0, theta, phi);
    double bestDist = 9999;
    double cDist = -1;
    int firstView = -1;
    int start = combinedCloud->points.size()-1;
    for(const pair<int, double>& pointDist : *distList)
    {
        cDist = pointDist.second + doublePointDistance(start, pointDist.first);
        if(cDist < bestDist)
        {
            firstView = pointDist.first;
            bestDist = cDist;
        }
    }
    if(logging)
    {
        cout << "Best distance = " << bestDist * 180 / M_PI << " from start to " << firstView << " to " << distPartnerList->at(firstView) << endl;
    }
    if(graphics)
    {
        //Iterate over the path
        pair<int, int> grasp = graspPartnerList->at(firstView);
        // TODO: Split into separate function
        graspPoint graspTemp;
        graspTemp.p1 = viewAndObjectCloud->points[grasp.first];
        graspTemp.p2 = viewAndObjectCloud->points[grasp.second];
        g->clearGrasps();
        g->addGraspDetails(graspTemp);
        g->setupCollisionCheck();
        g->collisionCheckSingleGrasp(0);
        //Clean the arrows
        visualizer->viewer->removeAllShapes();
        
        visualizer->viewer->addArrow(combinedCloud->points[firstView], combinedCloud->points[start], 0, 255, 0, false, "A");
        visualizer->viewer->addArrow(combinedCloud->points[distPartnerList->at(firstView)], combinedCloud->points[firstView], 0, 255, 0, false, "B");
        visualizer->viewer->addArrow(combinedCloud->points[grasp.first], combinedCloud->points[firstView], 0, 0, 0, false, "C");
        visualizer->viewer->addArrow(combinedCloud->points[grasp.second], combinedCloud->points[distPartnerList->at(firstView)], 0, 0, 0, false, "D");
        visualizer->viewer->addSphere(combinedCloud->points[grasp.first], .045, 255, 0, 0, "p1");
        visualizer->viewer->addSphere(combinedCloud->points[grasp.second], .045, 0, 255, 0, "p2");
        vector<int> visibleIndices1 = visPairs->at(grasp.first);
        vector<int> visibleIndices2 = visPairs->at(grasp.second);
        //Highlight ROV
        for (int i : visibleIndices1)
        {
            PointXYZRGB c = combinedCloud->points[i];
            combinedCloud->points[i].r = 255;
        }
        for (int i : visibleIndices2)
        {
            PointXYZRGB c = combinedCloud->points[i];
            combinedCloud->points[i].g = 255;
        }
        //Hard stop for user
        visualizer->stepVis(true);
        for (int i : visibleIndices1)
        {
            PointXYZRGB c = combinedCloud->points[i];
            combinedCloud->points[i].r = 0;
        }
        for (int i : visibleIndices2)
        {
            PointXYZRGB c = combinedCloud->points[i];
            combinedCloud->points[i].g = 0;
        }
        visualizer->viewer->removeAllShapes();
    }
    cleanup();
    return radiansToDegrees(bestDist);
}

// Add the start point of the search
void PathFinder::insertArtificialPoint(double r, double theta, double phi)
{
    vector<double> coords{r, theta, phi};
    PointXYZ p0 = sphericalToCartesian(coords, defautCentre);
    PointXYZRGB p;
    p.x = p0.x;
    p.y = p0.y;
    p.z = p0.z;
    p.r = 0.0;
    p.g = 0.0;
    p.b = 0.0;
    combinedCloud->push_back(p);
    //Add the new points to the octree
    oct->setInputCloud(combinedCloud);
    oct->addPointsFromInputCloud();
    int viewIndex = combinedCloud->points.size()-1;
    if(logging)
    {
        cout << "Checking distances..." << endl;
    }
    for (int i = 0; i < maxObjectIndex; i++)
    {
        //Assume that if an object point has no ROV or no partners, it's invalid.
        // This is wrong, but saves time.
        if(visPairs->at(i).size() > 0 && !getPartners(i).empty())
        {
            if(logging) cout << i << " being checked..." << endl;
            PointXYZRGB cPoint = combinedCloud->points[i];
            Eigen::Vector3f origin(cPoint.x, cPoint.y, cPoint.z);
            vector<int> start = visPairs->at(i);
            vector<int> end;
            // visibilityCalc(viewAndObjectCloud, ptrUnexpCloud, ptrObjNormal, &end, *oct);
            // if(end.size() == start.size()+1){
            //     visPairs->erase(i);
            //     visPairs->insert(make_pair(i, end));
            //     findSingleGroupDistance(i, viewIndex);
            //     if(logging)
            //     {
            //         cout << i << " connected" << endl;
            //     }
            // }
        }
    }
    if(logging) 
    {
        cout << "Start point index = " << viewIndex << endl;
    }
    sourceVisList->emplace(make_pair(viewIndex, viewIndex));
}

pair<double, int> PathFinder::singlePointDistance(int viewPoint, int objPoint)
{
    PointXYZRGB startPoint = combinedCloud->points[viewPoint];
    ROVList = {};
    if (logging > 2)
        cout << "viewpoint = " << viewPoint << " object point = " << objPoint << endl;
    try
    {
        buildList(objPoint, &ROVList);
    }
    catch (const exception &e)
    {
        cerr << e.what() << '\n';
        return pair<double, int>(999999, -1);
    }
    double minDist = 1410065406;
    int minPoint = -1;
    int counter = 0;
    double p1[3], p2[3];
    populateArray(p1, startPoint);
    for (PointXYZRGB endPoint : ROVList)
    {
        populateArray(p2, endPoint);
        // TODO: Remove res sanity checking
        double res = sphericalDistance(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
        double res2 = sphericalDistance(p2[0], p2[1], p2[2], p1[0], p1[1], p1[2]);
        if(res != res2)
            cout << "r1: " << res << " r2: " << res2 << endl;
        if (res < minDist)
        {
            minDist = res;
            // Record the index of the visible point
            minPoint = visPairs->at(objPoint)[counter];
        }
        counter++;
    }
    return pair<double, int>(minDist, minPoint);
}

//Find the largest gap in the viewsphere. Do not run
// with artificial points
// TODO: Move somewhere else
double PathFinder::maxViewsphereDist(){
    double cMax = 0.0;
    for(int i=0; i<sphereCloud->points.size(); i++)
    {
        double cMin = 9999;
        for(int j=0; j<sphereCloud->points.size(); j++)
        {
            if(i==j) continue;
            else 
            {
                cMin = min(cMin, doublePointDistance(maxCollisionIndex+i, maxCollisionIndex+j));
            }
        }
        cMax = max(cMax, cMin);
    }
    return radiansToDegrees(cMax);
}

//Find the distance between two points along the viewsphere by index.
double PathFinder::doublePointDistance(int vp1, int vp2){
    PointXYZRGB startPoint = combinedCloud->points[vp1];
    PointXYZRGB endPoint = combinedCloud->points[vp2];
    double p1[3], p2[3];
    populateArray(p1, startPoint);
    populateArray(p2, endPoint);
    return sphericalDistance(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
}

//Find ROV for sourcePoint
void PathFinder::buildList(int sourcePoint, vector<PointXYZRGB> *targetList)
{
    targetList->clear();
    vector<int> visibleIndices = visPairs->at(sourcePoint);
    // Loop over the ROV, adding them
    //  to the list to be updated.
    for (int i : visibleIndices)
    {
        PointXYZRGB c = combinedCloud->points[i];
        targetList->push_back(c);
        if(logging > 2){
            cout << i << endl;
        }
    }
}

// TODO: Cleanup
void PathFinder::displayFullPath(vector<int> path)
{
    //Clean the arrows
    visualizer->viewer->removeAllShapes();
    //Iterate over the path
    int cPoint = path[0];
    int nextPoint = -1;
    for(int i=1; i<path.size(); i++){
        nextPoint = path[i];
        visualizer->viewer->addArrow(combinedCloud->points[nextPoint], combinedCloud->points[cPoint], 0, 255, 0, false, to_string(nextPoint*-1));
        cPoint = path[i];
    }
    //Hard stop for user
    visualizer->stepVis(true);
    visualizer->viewer->removeAllShapes();
}

void PathFinder::cleanup()
{
    //Reset the lists
    if(logging){
        cout << "# of Pairs = " << sourceVisList->size() << endl;
    }
    sourceVisList->clear();
    *sourceVisList = *savedSourceVisList;
    visPairs->clear();
    *visPairs = *savedVisPairs;
    *distList = *savedDistList;
    *objMarginal = *savedObjMarginal;
    *distPartnerList = *savedDistPartnerList;
    *graspPartnerList = *savedGraspPartnerList;
    //delete the artificial point(s)
    while(combinedCloud->points.size() > maxCombinedIndex)
    {
        combinedCloud->points.pop_back();
    }
    if(logging){
        cout << "# of Pairs now = " << sourceVisList->size() << endl;
    }
    visualizer->endStep();
    playNext = true;
}

//Uncolor the viewsphere
void PathFinder::clearSphere()
{
    for (int j = 0; j < sphereCloud->size(); j++)
    {
        combinedCloud->points[j + maxCollisionIndex].r = 0;
        combinedCloud->points[j + maxCollisionIndex].g = 0;
        combinedCloud->points[j + maxCollisionIndex].b = 255;
    }
}

void PathFinder::drawPath(int objPoint, int startView, int endView){
    if(logging>2) cout << startView << " -> " << endView << endl;
    visualizer->viewer->removeShape(to_string(endView));
    visualizer->viewer->removeShape(to_string(objPoint));
    vector<int> visibleIndices = visPairs->at(objPoint);
    //Highlight ROV
    for (int i : visibleIndices)
    {
        PointXYZRGB c = combinedCloud->points[i];
        combinedCloud->points[i].r = 255;
    }
    visualizer->viewer->addArrow(combinedCloud->points[endView], combinedCloud->points[startView], 0, 255, 0, false, to_string(endView));
    visualizer->viewer->addArrow(combinedCloud->points[objPoint], combinedCloud->points[endView], 255, 255, 255, false, to_string(objPoint));
    if(graphics) visualizer->stepVis();
    //Remove ROV
    for (int i : visibleIndices)
    {
        PointXYZRGB c = combinedCloud->points[i];
        combinedCloud->points[i].r = 0;
    }
}