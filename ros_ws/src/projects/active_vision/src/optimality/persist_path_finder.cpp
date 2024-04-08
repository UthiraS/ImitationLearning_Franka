#include "../include/optimality/persist_path_finder.h"
int logging = 1;
bool aDexterb(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n){
    return (a.cross(b).dot(n) < 0.0);
}


Eigen::Vector3d project(Eigen::Vector3d a, Eigen::Vector3d b){
    Eigen::Vector3d ret = b - ((b).dot(a)*a);
    ret.normalize();
    return ret;
}

double aAngleb(Eigen::Vector3d a, Eigen::Vector3d b){
    Eigen::Vector3d z(0,0,1);
    Eigen::Vector3d bProj = project(a, b);
    Eigen::Vector3d zProj = project(a, z);
    double angle = acos(bProj.dot(zProj)/(bProj.norm()*zProj.norm()));
    return std::fmod(angle+M_PI, (2*M_PI));
    // if(aDexterb(a, b)){
    //     std::cout << "A dexter B" << std::endl;
    //     return acos(a.dot(b)/(a.norm()*b.norm()));
    // } else {
    //     std::cout << "A sinester B" << std::endl;
    //     return (2*M_PI) - acos(a.dot(b)/(a.norm()*b.norm()));
    // }
}

PersistentPathFinder::PersistentPathFinder(int log, bool graph):PathFinder(log, graph){};

void PersistentPathFinder::resetViewsphere()
{
    firstRequest = true;
    runPastCentroid = false;
    runPastCentroid2 = false;
    prevDist = 9999;
    lookedFor.clear();
    cleanup();
}

double PersistentPathFinder::findDistToCentroid(int start, int graspPoint, float &dist, bool vis){
    std::vector<int> visPoints = visPairs->at(graspPoint);
    //Iterate over the vis points and average their spherical coordinates
    std::vector<double> avg = {0,0,0};
    
    int points = 0;
    while(points < visPoints.size()){
        if(visPoints[points] > maxCombinedIndex-1){
            points++;
        } else {
            // PointXYZ cPoint(combinedCloud->points[visPoints[points]].x, combinedCloud->points[visPoints[points]].y, combinedCloud->points[visPoints[points]].z);
            // std::vector<double> cSphere = cartesianToSpherical(cPoint, defautCentre);
            // avg[0] += cSphere[0];
            // avg[1] += cSphere[1];
            // avg[2] += cSphere[2];
            avg[0] += combinedCloud->points[visPoints[points]].x;
            avg[1] += combinedCloud->points[visPoints[points]].y;
            avg[2] += combinedCloud->points[visPoints[points]].z;
            points++;
        }
    }
    avg[0] /= float(points);
    avg[1] /= float(points);
    avg[2] /= float(points);
    PointXYZ cPoint(avg[0], avg[1], avg[2]);
    std::vector<double> cSphere = cartesianToSpherical(cPoint, defautCentre);
    //Normalize the new point's radius to the viewsphere
    cSphere[0] = 1.0;
    PointXYZ centroid = sphericalToCartesian(cSphere, defautCentre);
    PointXYZRGB temp;
    temp.x = centroid.x,
    temp.y = centroid.y,
    temp.z = centroid.z,
    combinedCloud->points.push_back(temp);
    float retDir;
    
    AVLOG("Centroid = ("+to_string(centroid.x)+","+to_string(centroid.y)+","+to_string(centroid.z)+"), avg = ("+to_string(avg[0])+","+to_string(avg[1])+","+to_string(avg[2])+")", logging, 3);
    retDir = (aAngleofB(start, combinedCloud->size()-1));
    AVLOG("Visible "+to_string(aAngleofB(start, combinedCloud->size()-1))+" "+to_string(aAngleofB(combinedCloud->size()-1, start))+" "+to_string(aRightofB(start, combinedCloud->size()-1))+" "+to_string(retDir),logging, 3);
    dist = sphericalDistance(temp.x, temp.y, temp.z, combinedCloud->points[start].x, combinedCloud->points[start].y, combinedCloud->points[start].z);
    if(graphics && vis) visualizeStep(start, firstView, true, dist * 180 / M_PI);
    combinedCloud->points.pop_back();
    return retDir;
}

void PersistentPathFinder::findDistToPoints(int start, int graspPoint, float &dist){
    std::vector<int> visPoints = visPairs->at(graspPoint);
    
    int points = 0;
    double minDist = 9999;
    while(points < visPoints.size()){
        if(visPoints[points] > maxCombinedIndex-1){
            points++;
        } else {
            minDist = min(minDist, sphericalDistance(combinedCloud->points[visPoints[points]].x, combinedCloud->points[visPoints[points]].y, combinedCloud->points[visPoints[points]].z, combinedCloud->points[start].x, combinedCloud->points[start].y, combinedCloud->points[start].z));
            points++;
        }
    }
    dist = minDist;
    return;
}

void PersistentPathFinder::sortROV(pair<int, int> &grasp, int index){
    firstGraspPoint = grasp.first;
    secondGraspPoint = grasp.second;
    float firstDist=0;
    float secondDist=0;
    AVLOG("Checking region "+to_string(grasp.first)+" "+to_string(grasp.second), logging, 1);
    findDistToPoints(maxCombinedIndex, grasp.first, firstDist);
    findDistToPoints(maxCombinedIndex, grasp.second, secondDist);
    if(firstDist > secondDist){
        AVLOG("Flipping ROVs", logging, 1);
        firstGraspPoint = grasp.second;
        secondGraspPoint = grasp.first;
    }
}

double PersistentPathFinder::searchFrom(double theta, double phi, vector<double> vis, double rotation)
{
    AVLOG("Starting searchFrom", logging, 1);
    playNext = false;
    AVLOG("Received vis", logging, 1);
    // cout << vis[0] << " " << vis[1] << " " << vis[2] << endl;
    vector<int> visIndices = unpackVis(vis, rotation);
    AVLOG("Unpacked vis", logging, 1);
    if(graphics)    visualizeNewViews(visIndices);
    insertPersistentPoint(1.0, theta, phi, visIndices);
    double bestDist = 9999;
    double cDist = 9999;
    int start = combinedCloud->points.size()-1;
    firstView = -1;
    AVLOG("Starting iteration", logging, 1);
    for(const std::pair<int, double>& pointDist : *distList)
    {
        AVLOG("Point dists "+to_string(pointDist.first)+":"+to_string(pointDist.second), logging, 3);
        //If the other point is artificial, there's no distance between the two.
        if(pointDist.first > maxCombinedIndex-1)
        {
            cDist = pointDist.second;
            //Add the current point to the list. It will be visible from cPoint's
            // object points and have cPoint's viewsphere partners.
            // AVLOG(to_string(pointDist.first), logging, 1);
            try{
                std::pair<int, int> grasps = graspPartnerList->at(pointDist.first);
                graspPartnerList->insert(std::make_pair(start, std::make_pair(grasps.first, grasps.second)));
                std::vector<int> startVis = visPairs->at(grasps.first);
                startVis.push_back(start);
                visPairs->erase(grasps.first);
                visPairs->insert(std::make_pair(grasps.first, startVis));
                int oldPartner = distPartnerList->at(pointDist.first);
                //Check if we're closer now.
                cDist = min(cDist, doublePointDistance(start, oldPartner));
                distPartnerList->insert(std::make_pair(start, oldPartner));
            } catch (...)
            {
                AVLOG("Partner has no compliment! Skipping", logging, 1);
                cDist = 9999;
            }
        } else 
        {
            cDist = pointDist.second + doublePointDistance(start, pointDist.first);
        }
        if(cDist < bestDist)
        {
            firstView = pointDist.first;
            bestDist = cDist;
        }
    }
    distList->insert(std::make_pair(start, cDist));
    // cleanup();
    std::pair<int, int> grasp = graspPartnerList->at(firstView);
    sortROV(grasp, firstView);
    firstRequest = false;
    bool foundFirstPoint = vectorContainsElement<int>(visPairs->at(firstGraspPoint), start);
    bool firstTime = !vectorContainsElement<int>(visPairs->at(firstGraspPoint), start-1);
    bool foundPath = vectorContainsElement<int>(visPairs->at(secondGraspPoint), start);
    AVLOG("Direction = "+to_string(nextDirection)+" "+to_string(bestDist), logging, 1);
    if(foundPath || (bestDist+0.087 >= prevDist)){
        AVLOG("Distance hasn't decreased by a step! This shouldn't happen..", logging, 1);
        float distanceToCentroid = 0;
        if(foundFirstPoint && !firstTime){
            AVLOG("First point is visible- skipping to second", logging, 1);
            // foundPath = true;
            int tentativeDirection = findDistToCentroid(start, secondGraspPoint, distanceToCentroid, true);
            AVLOG(to_string(distanceToCentroid * 36/M_PI) + " steps from the centroid", logging, 1);
            if(!runPastCentroid2 && (distanceToCentroid >= 0.087)){
                nextDirection = tentativeDirection;
            } else {
                runPastCentroid2 = true;
            }
            if(graphics) visualizeStep(start, firstView, true, bestDist * 180 / M_PI);
            return bestDist * 180 / M_PI;
        }
        if(!foundFirstPoint){
            AVLOG("First point is hidden- trying to find it", logging, 0);
            int tentativeDirection = findDistToCentroid(start, firstGraspPoint, distanceToCentroid, true);
            if(!runPastCentroid && (distanceToCentroid >= 0.087)){
                nextDirection = tentativeDirection;
            } else {
                runPastCentroid = true;
            }
            prevDist = bestDist;
            return bestDist * 180 / M_PI;
        }
        AVLOG("First point has just been found after backgracking- continuing.", logging, 1);
    }

    prevDist = bestDist;
    if(logging)
    {   
        string message = "Best distance = ";
        message += to_string(bestDist * 180 / M_PI) + " from start to " + to_string(firstView) + " to ";
        message += to_string(distPartnerList->at(firstView)) += "\nObj points = " + to_string(firstGraspPoint);
        message += " " + to_string(secondGraspPoint);
        AVLOG(message, logging, 1);
    }
    //Check if we've seen both points- if so, that's an error. Just return the bestdist and retain the old direction.
    // if(foundPath){
    //     AVLOG("Path already visible- this shouldn't happen but once. "+to_string(phi)+" "+to_string(nextDirection), logging, 1);
    //     // //Corner case where we're off by one, and at the very top of the viewsphere. 
    //     // if((.05 > phi) && (30 > nextDirection || 330 < nextDirection)){
    //     //     std::cout << "Close to top and continuing to go up- switching to going down instead." << std::endl;
    //     //     if(30 > nextDirection){
    //     //         nextDirection = 180 + nextDirection;
    //     //     } else {
    //     //         nextDirection = nextDirection - 180;
    //     //     }
    //     // }
    //     // findDistToCentroid(start, firstView, firstGraspPoint);
    //     nextDirection = findDistToCentroid(start, firstView, secondGraspID);
    //     return bestDist * 180 / M_PI;
    // }
    // if(0 < cRepeatSteps){
    //     AVLOG("Taking a second step in the same direction", logging, 1);
    //     if(graphics) visualizeStep(start, firstView, true, bestDist * 180 / M_PI);
    //     cRepeatSteps -= 1;
    //     prevDist = bestDist;
    //     return bestDist * 180 / M_PI;
    // }
    // for(int index : visPairs->at(secondGraspPoint)){
    //     if(start == index)
    //     {
    //         AVLOG("Second point visible- this shouldn't happen but once.", logging, 0);
    //         foundPath = true;
    //         //Save the 2nd grasp point to prevent ROV flipping.
            
    //         // if(phi >= 0.05)
    //         //     cRepeatSteps = -1;
    //         // visPairs->at(secondGraspPoint).pop_back();
    //         if(graphics) visualizeStep(start, firstView, true, bestDist * 180 / M_PI);
    //         // nextDirection = findDistToCentroid(start, firstView, secondGraspID);
    //         return bestDist * 180 / M_PI;
    //     }
    // }
    //Check if start can see the first grasp point- if not, go to the firstView
    bool pointVisible = false;
    for(int index : visPairs->at(firstGraspPoint)){
        if(start == index)
            pointVisible = true;
    }
    string message;
    if(!pointVisible){
        if(aRightofB(start, firstView)){
            nextDirection = (aAngleofB(start, firstView));
        } else {
            nextDirection = (aAngleofB(start, firstView));
        }

        message += "Not visible "+to_string(aAngleofB(start, firstView))+" "+to_string(aRightofB(start, firstView));
        message += " " + to_string(nextDirection);
    } else {
        if(aRightofB(start, distPartnerList->at(firstView))){
            nextDirection = (aAngleofB(start, distPartnerList->at(firstView)));
        } else {
            nextDirection = (aAngleofB(start, distPartnerList->at(firstView)));
        }
        message += "Visible "+to_string(aAngleofB(start, distPartnerList->at(firstView)))+" "+to_string(aRightofB(start, distPartnerList->at(firstView)));
        message += " " + to_string(nextDirection);
    }
    // AVLOG(message, logging, 0);
    //If the distance is increasing, try going the opposite way to break out of the loop
    // if(bestDist > prevDist){
    //     AVLOG("Distance increasing! This shouldn't happen..", logging, 0);
    //     cRepeatSteps = maxRepeatSteps;
    //     maxRepeatSteps += 1;
    // }
    

    if(graphics) visualizeStep(start, firstView, pointVisible, bestDist*180/M_PI);
    
    
    //Radians->degrees
    return bestDist * 180 / M_PI;
}

void PersistentPathFinder::visualizeStep(int start, int firstView, bool pointVisible, float bestDist)
{
    //Clean the arrows
    visualizer->viewer->removeAllShapes();
    //Re-add the object + viewsphere
    // addRGB(viewer, combinedCloud, "Object", 1);
    //Iterate over the path
    std::pair<int, int> grasp = graspPartnerList->at(firstView);
    visualizer->viewer->addArrow(combinedCloud->points[firstView], combinedCloud->points[start], 0, 255, 0, false, "A");
    visualizer->viewer->addArrow(combinedCloud->points[distPartnerList->at(firstView)], combinedCloud->points[firstView], 0, 255, 0, false, "B");
    visualizer->viewer->addArrow(combinedCloud->points[grasp.first], combinedCloud->points[firstView], 0, 0, 0, false, "C");
    visualizer->viewer->addArrow(combinedCloud->points[grasp.second], combinedCloud->points[distPartnerList->at(firstView)], 0, 0, 0, false, "D");
    visualizer->viewer->addSphere(combinedCloud->points[combinedCloud->size()-1], .045, 255, 0, 0, "last");
    std::vector<int> visibleIndices1 = visPairs->at(firstGraspPoint);
    std::vector<int> visibleIndices2 = visPairs->at(secondGraspPoint);
    // AVLOG(to_string(bestDist), logging, 1);
    if(5 > bestDist)
        visualizer->viewer->addText("Next step should find a grasp",5,65,25,0,1,0,"GID",0);
    else
        visualizer->viewer->addText(to_string(bestDist/5.0)+" steps to grasp",5,65,25,1,1,0,"GID",0);
    //Add the angle arrow:
    if(!pointVisible){
        visualizer->viewer->addArrow(combinedCloud->points[firstView], combinedCloud->points[start], 0, 0, 255, false, "E");
    } else {
        visualizer->viewer->addArrow(combinedCloud->points[distPartnerList->at(firstView)], combinedCloud->points[start], 0, 0, 255, false, "E");
    }
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
    visualizer->stepVis();
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

void PersistentPathFinder::insertPersistentPoint(double r, double theta, double phi, vector<int> vis){
    AVLOG("insertPersistentPoint starting", logging, 1);
    std::vector<double> coords{r, theta, phi};
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
    oct->deleteTree();
    oct->defineBoundingBox(2);
    oct->setInputCloud(combinedCloud);
    oct->addPointsFromInputCloud();
    int viewIndex = combinedCloud->points.size()-1;
    AVLOG("Checking distances... w/out octree", logging, 1);
    
    for(int i : vis){
        AVLOG("Checking point "+to_string(i), logging, 3);
        vector<int> start = visPairs->at(i);
        start.push_back(viewIndex);
        visPairs->erase(i);
        visPairs->insert(make_pair(i, start));
        findSingleGroupDistance(i, viewIndex);
    }
    /*
    for (int i = 0; i < maxObjectIndex; i++)
    {
        //Assume that if an object point has no ROV or not partners, it's invalid.
        // This is wrong, but saves time.
        if(visPairs->at(i).size() > 0 && !getPartners(i).empty())
        {
            if(logging) std::cout << i << " being checked..." << std::endl;
            PointXYZRGB cPoint = combinedCloud->points[i];
            Eigen::Vector3f origin(cPoint.x, cPoint.y, cPoint.z);
            std::vector<int> start = visPairs->at(i);
            
            if(end.size() == start.size()+1){
                visPairs->erase(i);
                visPairs->insert(std::make_pair(i, end));
                findSingleGroupDistance(i, viewIndex);
                if(logging)
                {
                    std::cout << i << " connected" << std::endl;
                }
            }
            else {
                cout << "Non visible, " << start.size() << " should == " << end.size() << endl;
            }
        }
    }*/
    // ptrObjNormal = g->getNormals();
}

void PersistentPathFinder::findRotation(double rotation){
    if(!firstRequest) return;
    Eigen::Affine3f center = getTransformation(-0.45, 0.0, -0.125, 0, 0, 0);
    // Eigen::Affine3f rotate = getTransformation(0, 0, 0, 0, 0, 0);
    Eigen::Affine3f rotate = getTransformation(0, 0, 0, 0, 0, -rotation);
    // Eigen::Affine3f offset = getTransformation(0.45, 0.0, 0.125, 0, 0, 0);
    Eigen::Affine3f net = rotate;
    rotateObject = net.matrix();
}

vector<double> PersistentPathFinder::fakeVis(){
    vector<double> ret;
    for(float i=-7*voxelGridSize; i< 7*voxelGridSize; i+= voxelGridSize){
        for(float j=-7*voxelGridSize; j< 7*voxelGridSize; j+= voxelGridSize){
            for(float k=-7*voxelGridSize; k< 7*voxelGridSize; k+= voxelGridSize){
                ret.push_back(centroid.x + i + .00375);
                ret.push_back(centroid.y + j + .00375);
                ret.push_back(centroid.z + k + .00375);
            }
        }
    }
    return ret;
}

void PersistentPathFinder::addEigenPoint(ptCldColor::Ptr dest, Eigen::Vector4f coordinates){
    PointXYZRGB cPoint;
    coordinates = rotateObject*coordinates;
    cPoint.x = coordinates[0];
    cPoint.y = coordinates[1];
    cPoint.z = coordinates[2];
    cPoint.r = 255;
    cPoint.b = 255;
    dest->push_back(cPoint);
}

vector<int> PersistentPathFinder::unpackVis(vector<double> vis, double rotation){
    findRotation(rotation);
    vector<int> visIndices;
    ptCldColor::Ptr fails(new ptCldColor());
    ptCldColor::Ptr receivedCloud(new ptCldColor());
    double xOff, yOff, zOff;
    vector<bool> visible;
    //Calculate the offset between the first received point and the first object point
    // to align the received point to the voxel centers
    for(int i=0; i<vis.size()/4; i++){
        Eigen::Vector4f unrotated(vis[i*4] - centroid.x, vis[1+(i*4)] - centroid.y, vis[2+(i*4)] - centroid.z, 1);
        visible.push_back(bool(vis[3+(i*4)]));
        addEigenPoint(receivedCloud, unrotated);
        // for(float xO : {.45,-.45}){
        //     for(float yO : {.45,-.45}){
        //         for(float zO : {.45,-.45}){
        //             Eigen::Vector4f corner(xO *  voxelGridSize, yO *  voxelGridSize, zO *  voxelGridSize, 1);
        //             corner += unrotated;
        //             addEigenPoint(receivedCloud, corner);
        //         }
        //     }
        // }
    }
    // xOff = fmod((objCloud->points[0].x - receivedCloud->points[0].x), voxelGridSize);
    // yOff = voxelGridSize+fmod((objCloud->points[0].y - receivedCloud->points[0].y), voxelGridSize);
    // zOff = fmod((objCloud->points[0].z - receivedCloud->points[0].z), voxelGridSize);
    // AVLOG(to_string(objCloud->points[0].x) + " " + to_string(objCloud->points[0].y) + " " + to_string(objCloud->points[0].z) + " ", logging, 1);
    // AVLOG(to_string(receivedCloud->points[0].x) + " " + to_string(receivedCloud->points[0].y) + " " + to_string(receivedCloud->points[0].z) + " ", logging, 1);
    // AVLOG(to_string(receivedCloud->points[0].x+xOff) + " " + to_string(receivedCloud->points[0].y+yOff) + " " + to_string(receivedCloud->points[0].z+zOff) + " ", logging, 1);
    // AVLOG(to_string(xOff) + " " + to_string(yOff) + " " + to_string(zOff) + " ", logging, 1);
    // for(int i=0; i<receivedCloud->points.size(); i++){
    //     receivedCloud->points[i].x += (xOff);
    //     receivedCloud->points[i].y += (yOff);
    //     receivedCloud->points[i].z += (zOff);
    // }
    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    icp.setInputSource(receivedCloud);
    icp.setInputTarget(objCloud);
    icp.setMaxCorrespondenceDistance(5*voxelGridSize);

    icp.setMaximumIterations(50);
    icp.align(*receivedCloud);
    // cout << icp.getFinalTransformation() << endl;
    octree::OctreePointCloudSearch<PointXYZRGB> occupancyOct(voxelGridSize);

    // occupancyOct.defineBoundingBox(2);
    // // double x1, y1, z1, x2, y2, z2;
    // // occupancyOct.getBoundingBox(x1, y1, z1, x2, y2, z2);
    // // AVLOG(to_string(x1)+","+to_string(y1)+","+to_string(z1)+","+to_string(x2)+","+to_string(y2)+","+to_string(z2), logging, 1);
    // occupancyOct.setInputCloud(receivedCloud);
    // occupancyOct.addPointsFromInputCloud();
    // // occupancyOct.getBoundingBox(x1, y1, z1, x2, y2, z2);
    // // AVLOG(to_string(x1)+","+to_string(y1)+","+to_string(z1)+","+to_string(x2)+","+to_string(y2)+","+to_string(z2), logging, 1);
    
    
    // for(int i=0; i<objCloud->points.size(); i++){
    //     if(occupancyOct.isVoxelOccupiedAtPoint(objCloud->points[i].x, objCloud->points[i].y, objCloud->points[i].z)){
    //         visIndices.push_back(i);
    //     }
    // }
    occupancyOct.deleteTree();
    occupancyOct.defineBoundingBox(2);
    occupancyOct.setInputCloud(objCloud);
    occupancyOct.addPointsFromInputCloud();
    for(int i=0; i<receivedCloud->points.size(); i++){
        vector<int> indices{0};
        vector<float> distances{0.0};
        if(visible[i]){
            occupancyOct.nearestKSearch(receivedCloud->points[i], 1, indices, distances);
            visIndices.push_back(indices[0]);
        } else {
            receivedCloud->points[i].g = 255;
        }
        // if(!occupancyOct.isVoxelOccupiedAtPoint(receivedCloud->points[i].x, receivedCloud->points[i].y, receivedCloud->points[i].z)){
        //     receivedCloud->points[i].g = 255;
        //     fails->points.push_back(receivedCloud->points[i]);
        // }
    }
    visualizer->setRGB(fails, "Failures", 3, 0);
    visualizer->setRGB(receivedCloud, "NewPoints", 3, 1);
    return visIndices;
}
/*
vector<int> PersistentPathFinder::unpackVis(vector<double> vis, double rotation){
    findRotation(rotation);
    vector<int> visIndices;
    ptCldColor::Ptr fails(new ptCldColor());
    ptCldColor::Ptr receivedCloud(new ptCldColor());
    for(int i=0; i<vis.size()/3; i++){
        AVLOG("Unpacking "+to_string(i*3)+"/"+to_string(vis.size()), logging, 1);
        PointXYZRGB cPoint;
        Eigen::Vector4f unrotated(vis[i*3]-centroid.x, vis[1+(i*3)]-centroid.y, vis[2+(i*3)]-centroid.z, 1);
        unrotated = rotateObject*unrotated;
        cPoint.x = unrotated[0];
        // cPoint.x = 0.0075; //vis[i*3]-centroid.x;
        cPoint.y = unrotated[1];
        // cPoint.y = 0.1875; //vis[1+(i*3)]-centroid.y;
        cPoint.z = unrotated[2];
        // cPoint.z = 0.015; //vis[2+(i*3)]-centroid.z;
        vector<int> dest;
        double x1, y1, z1, x2, y2, z2;
        oct->getBoundingBox(x1, y1, z1, x2, y2, z2);
        // AVLOG(to_string(x1)+","+to_string(y1)+","+to_string(z1)+","+to_string(x2)+","+to_string(y2)+","+to_string(z2), logging, 1);
        bool s1 = oct->isVoxelOccupiedAtPoint(cPoint);
        // AVLOG(to_string(s1), logging, 1);
        if(!s1){
            cPoint.r = 255;
            cPoint.b = 255;
            fails->push_back(cPoint);
            continue;
        }
        AVLOG("About to octree search..."+to_string(cPoint.x)+","+to_string(cPoint.y)+","+to_string(cPoint.z), logging, 1);
        bool success = oct->voxelSearch(cPoint, dest);
        // AVLOG("Voxel search ->"+to_string(success)+","+to_string(dest[0]), logging, 1);
        visIndices.push_back(dest[0]);
    }
    visualizer->setRGB(fails, "Failures", 5, 0);
    return visIndices;
}*/

bool PersistentPathFinder::aRightofB(int indexA, int indexB){
    PointXYZRGB A = combinedCloud->points[indexA];
    PointXYZRGB B = combinedCloud->points[indexB];
    return aDexterb(Eigen::Vector3d(A.x, A.y, 0.0), Eigen::Vector3d(B.x, B.y, 0.0), Eigen::Vector3d(A.x, A.y, 0.0));
}

double PersistentPathFinder::aAngleofB(int indexA, int indexB){
    PointXYZRGB A = combinedCloud->points[indexA];
    PointXYZRGB B = combinedCloud->points[indexB];
    Eigen::Vector3d z(0,0,1);
    Eigen::Vector3d a(A.x, A.y, A.z);
    Eigen::Vector3d b(B.x, B.y, B.z);
    Eigen::Vector3d bProj = project(a, b);
    Eigen::Vector3d zProj = project(a, z);
    double angle = acos(bProj.dot(zProj));
    if(aDexterb(zProj, bProj, a)){
        // cout << "Fixing sign " << angle << " to ";
        angle = 2*M_PI-angle;
        // cout << angle << endl;
    }
        
    return radiansToDegrees(fmod(angle, 2*M_PI));
}

void PersistentPathFinder::visualizeNewViews(vector<int> vis){
    AVLOG("", logging, 1);
    for(int j=0; j<maxObjectIndex; j++){
        combinedCloud->points[j].r = 0;
    }
    visualizer->viewer->removeAllShapes();
    for(int i=0; i<maxObjectIndex; i++){
        combinedCloud->points[i].r = 0;
        if(vectorContainsElement<int>(vis, i))
        {
            combinedCloud->points[i].r = 255;
        }
    }
    AVLOG("", logging, 1);
}