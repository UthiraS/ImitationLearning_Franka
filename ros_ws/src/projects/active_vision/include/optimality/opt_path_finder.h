#ifndef OPTP_H
#define OPTP_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <active_vision/toolGraspSynthesis.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/toolViewPointCalc.h>
#include <pcl/visualization/cloud_viewer.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <boost/config.hpp>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <active_vision_tools/mathConvenience.h>
#include <active_vision_tools/visualization.h>

using namespace pcl;
using namespace std;

typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
        boost::property< boost::edge_weight_t, int > >
        graph_t;
typedef boost::graph_traits< graph_t >::vertex_descriptor vertex_descriptor;
typedef pair< int, int > Edge;


class PathVis : public AVVis{
public:

    /// @brief Initializes the viewer
    /// @param combined The entire point cloud to display
    /// @param object Point cloud containing only the object points
    /// @param normals Point cloud containing the object normals
    /// @param name Viewer's name
    /// @param numberOfViewpoints Number of sub-windows to open for the viewer
    void setup(ptCldColor::Ptr combined, ptCldColor::Ptr object, ptCldNormal::Ptr normals, string name="viewer", int numberOfViewpoints=1);


    /// @brief Takes a single step in the visualization
    /// @param hardStop Whether to reset keypress and force the user to interact to continue
    void stepVis(bool hardStop=false);
protected:
    ptCldColor::Ptr _combined;
};

class PathFinder
{
    public:
        PathFinder();
        PathFinder(int log, bool graph);
        void updateLogging(int log);
        bool graphics;
        int logging;
        int minVisSize;
        //Sends all needed data to pathfinder. Only call it once.
        void initialize(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr object, PointCloud<PointXYZRGB>::Ptr sphere, PointCloud<PointXYZRGB>::Ptr combined, PointCloud<PointNormal>::Ptr pObjNormal,  PointCloud<PointXYZRGB>::Ptr unexpCloud, octree::OctreePointCloudSearch<PointXYZRGB>::Ptr o, double angle = 90, bool runSetup = true, bool discrete = false);

        //Finds the shortest paths from theta, phi on the viewsphere
        virtual double searchFrom(double theta, double phi);
        double oldSearch(double theta, double phi);
        bool artificialInUse = false;

        //Find the spherical distance between a point on the viewsphere and a
        // point in objPoint's ROV
        pair<double, int> singlePointDistance(int viewPoint, int objPoint);
        vector<PointXYZRGB> ROVList;
        PointXYZRGB centroid;

        //object + viewsphere pcd
        PointCloud<PointXYZRGB>::Ptr viewAndObjectCloud;
        //Just object pcd
        PointCloud<PointXYZRGB>::Ptr objCloud;
        //Just unexplored pcd
        PointCloud<PointXYZRGB>::Ptr ptrUnexpCloud;
        //object normals
        PointCloud<PointNormal>::Ptr ptrObjNormal;
        //Just viewsphere pcd
        PointCloud<PointXYZRGB>::Ptr sphereCloud;
        //object+viewsphere+(optional temp point) in that order
        PointCloud<PointXYZRGB>::Ptr combinedCloud;
        //User defined octree for raytracing
        octree::OctreePointCloudSearch<PointXYZRGB>::Ptr oct;
        //For visualization
        PathVis* visualizer;

        //Maps object point indexes to complimentary points
        vector<vector<int>> *graspPairs;
        //Maps object point indexes to list of point indexes on the
        // viewsphere that can see that point
        unordered_map<int, vector<int>> *visPairs, *savedVisPairs, *origVisPairs;
        //Maps object point indexes to point index on the viewsphere
        // closest to the current search source.
        unordered_map<int, int> *sourceVisList, *savedSourceVisList;

        //Maps viewpoint indexes to distances to closest compliment
        unordered_map<int, double> *distList, *savedDistList;

        //Records whether each object point is "marginal" for grasping
        unordered_map<int, bool> *objMarginal, *savedObjMarginal;
        //Maps viewpoint indexes to their partner
        unordered_map<int, int> *distPartnerList, *savedDistPartnerList;
        //For record keeping- maps viewpoint indexes to <their grasp point, their partner's grasp point>
        unordered_map<int, pair<int, int>> *graspPartnerList, *savedGraspPartnerList;
        


        //Helper- draws an arrow from startView->endView and 
        // endView->objPoint
        void drawPath(int objPoint, int startView, int endView);
        
        //Returns true iff a grasp can be found in exactly depth steps
        // Assumes that the points in the viewsphere have been added by
        // addContPoints in discrete search
        bool getNSteps(int depth);

        //Remove artificial point and prep for next search
        void cleanup();

        //Signals user that the current run is finished.
        bool playNext = false;

        //Max index of point cloud + viewsphere- used to decide if a point is artificial
        int maxCombinedIndex;

        double maxViewsphereDist();

        //Display function- shows the visible surface from each viewpoint
        void visualizeOverViewpoints();

    protected:
        int maxObjectIndex, maxCollisionIndex, nObjectPoints;
        bool nonDiscrete;
        float avgCurvature;
        int views=2;
        vector<Eigen::Vector3f>* cardinalDirection = new vector<Eigen::Vector3f>({Eigen::Vector3f(1.0,0.0,0.0), Eigen::Vector3f(-1.0,0.0,0.0), Eigen::Vector3f(0.0, 1.0,0.0), Eigen::Vector3f(0.0,-1.0,0.0), Eigen::Vector3f(0.0,0.0,1.0), Eigen::Vector3f(0.0,0.0,-1.0)});
        search::Search<PointXYZRGB>::Ptr KdTree{new search::KdTree<PointXYZRGB>};
        //------Setup-------------
        //Run all preliminary work before a path can be generated. Only call it once
        void setup();
        //Finds visibility for all object points
        void buildVis();
        //Finds visibility for single object point
        int calculateVisibility(int i);
        //Checks that all k nearest points meet angle requirements.
        bool checkAngleMultiRequirement(int objectIndex, Eigen::Vector3f normal, Eigen::Vector3f origin, Eigen::Vector3f target);
        //Checks if the normal is within max degrees of the target
        bool checkAngleRequirement(int objectIndex, Eigen::Vector3f normal, Eigen::Vector3f origin, Eigen::Vector3f target);
        //Finds all complimentary grasp pairs
        void buildGroups();
        //Assign the distances for each viewpoint
        void findGroupDistances();
        //Assign the distance for a single viewpoint from a single object point
        void findSingleGroupDistance(int objIndex, int viewPoint);

        //Returns object indices of complimentary grasps to ind
        vector<int> getPartners(int ind);

        //For validity calculations
        int patchWidth = 26;         // min radius of acceptable patch
        double viewAngle = 60;     // max legal angle between a point on the viewsphere and a point it sees
        double maxGripperWidth = .08;                           // Gripper max width (Actual is 8 cm)
        double minGraspQuality = 150;
        //TODO: let user specify
        double voxelGridSize = 0.0075;

        graspSynthesis *g;// = graspSynthesis(maxGripperWidth, minGraspQuality);

        //-------Graph handling-----------
        //Add the start point of the search
        virtual void insertArtificialPoint(double r, double theta, double phi);
        //Find the spherical distance between two points on the viewsphere
        double doublePointDistance(int vp1, int vp2);
        
        //Find sourcePoint's ROV and put it into targetList.
        void buildList(int sourcePoint, vector<PointXYZRGB> *targetList);
        
        //Tracks if any grasp points are complimentary for discrete search
        bool pathPresent = false;
        //Resets viewsphere colors
        void clearSphere();

        //Show the final path on the viewsphere
        // Connects each viewsphere point whose index is in path
        void displayFullPath(vector<int> path);

        //Discrete functions
        bool graspVisibleFrom(vector<int> viewIndices);
        bool singlePointVisibile(vector<int> viewIndices, int pointIndex);
        

};
//Find the spherical distance between two 3D points along the viewsphere
// based on https://en.wikipedia.org/wiki/Great-circle_distance
// BOTH POINTS ARE ASSUMED TO ACTUALLY BE ON THE VIEWSPHERE

// double sphericalDistance(Eigen::Vector3d v1, Eigen::Vector3d v2);
#endif