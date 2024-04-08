//Useful CLI tools:
//Convert .obj to .pcd:
//  pcl_mesh2pcd textured.obj output.pcd -no_vis_result
//Convert dot file to graphics
//  dot -Tps dijkstra-eg.dot -o outfile.ps

#ifndef POINTRELATIONSHIP_H
#define POINTRELATIONSHIP_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include "active_vision/toolGraspSynthesis.h"
#include <active_vision/toolVisualization.h>
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

typedef CGAL::Exact_spherical_kernel_3 SK;
typedef CGAL::Sphere_3<SK> Sphere_3;
typedef CGAL::Plane_3<SK> Plane_3;
typedef CGAL::Point_3<SK> Point_3;
typedef CGAL::Circle_3<SK> Circle_3;
typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
        boost::property< boost::edge_weight_t, int > >
        graph_t;
typedef boost::graph_traits< graph_t >::vertex_descriptor vertex_descriptor;
typedef std::pair< int, int > Edge;

template <class T>
void vecToArray(std::vector<T> *vec, T arr[]);


class PointRelationship
{
    public:
        // Constructor
        PointRelationship(){};

        std::vector<int> bestPath;

        ptCldVis::Ptr viewer;

        double tX, tY, tZ;

        int start = -1;
        
        void setInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere, pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined, pcl::PointCloud<pcl::Normal>::Ptr cloudNormal, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr);

        // Destructor
        virtual ~PointRelationship(){};

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloud;
        pcl::PointCloud<pcl::Normal>::Ptr ptrObjNormal;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphereCloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr oct;

        //Maps object point indexes to min distances
        std::vector<std::unordered_map<int, double>> *distPairs;
        //Maps object point indexes to list of point indexes on the
        // viewsphere that can see that point
        std::unordered_map<int, std::vector<int>> *visPairs;
        //Maps object point indexes to point index on the viewsphere
        // closest to the current search source.
        std::vector<std::pair<int, int>> *sourceVisList;

        void processPoints();

        void buildGraph();

        std::string getIndexName(int index);

        bool validPair(int a, int b, bool display=false);

        double distanceFrom(int a, int b, bool artificialPoint=false, bool display=false);

        std::pair<double, int> singlePointDistance(int a, int b, bool artificialPoint=false, bool display=false);

        double pointDistances(double x1, double y1, double z1, double x2, double y2, double z2, bool display=false);

        double cDistanceF(int a, int b, bool artificialPoint=false, bool display=false);

        void setFirstPoint(int a);

        void setSecondPoint(int a);
        
        void calculatePoint(int ind, Point_3 &setPoint);

        int calculateVisibility(int i);

        void stepViewer();

        std::unordered_map<int, double> getPartners(int ind);

    private:
        void setup();
        int fullyConnectPoint(int start);
        void insertArtificialPoint();
        bool nearestGroupPoint(int i, int j);
        void buildList(int sourcePoint, std::vector<pcl::PointXYZRGB> *targetList, int color=1);
        void populateArray(double target[], pcl::PointXYZRGB source);
        void getUserInput();
        void keyInput(const pcl::visualization::KeyboardEvent &event, void*);
        void clearSphere();
        int maxIndex, finalIndex;
        double cShortestDist = 999999.9;
        double maxGripperWidth = 5;                           // Gripper max width (Actual is 8 cm)
        double minGraspQuality = 170;
        double voxelGridSize = 0.5;
        Eigen::Vector3f vectA, vectB;
        double A,B;
        pcl::PointXYZRGB p1, p2;
        double gripperWidth, quality;
        int currentPointIndex = 0;
        Point_3 firstPoint, secondPoint;
        std::vector< CGAL::Object > intersecs;
        std::vector<pcl::PointXYZRGB> firstList, secondList;
        pcl::PointXYZ temp, temp1;
        std::vector<Edge> edgeArray;
        std::vector<int> weights;
        int debugLength = 300;
        bool keypress = false;

};

#endif