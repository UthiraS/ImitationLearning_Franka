#ifndef P_OPTP_H
#define P_OPTP_H

#include "optimality/opt_path_finder.h"
#include <pcl/registration/icp.h>

bool aDexterb(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d n);

double aAngleb(Eigen::Vector3d a, Eigen::Vector3d b);

class PersistentPathFinder : public PathFinder
{
    public:
        PersistentPathFinder();
        PersistentPathFinder(int log, bool graph);
        //As in the parent class, but once you're done add the point
        // to the viewsphere so later measurements can use it
        double searchFrom(double theta, double phi, vector<double> vis, double rotation);
        int nSteps = 2;
        //Removes the added points from the viewsphere
        void resetViewsphere();
        //Kludge- lists the next direction in the optimal path (3/7)
        int nextDirection = -1;
    protected:
        double findDistToCentroid(int start, int graspPoint, float &dist, bool vis=false);
        void findDistToPoints(int start, int graspPoint, float &dist);
        void visualizeStep(int start, int firstView, bool pointVisible, float bestDist);
        void visualizeNewViews(vector<int> vis);
        void sortROV(pair<int, int> &grasp, int index);
        vector<double> fakeVis();
        vector<int> unpackVis(vector<double> vis, double rotation);
        //Adds previously visited points to the viewsphere
        void insertPersistentPoint(double r, double theta, double phi, vector<int> vis);
        //Checks if point A on the viewsphere is to the right of point B,
        // also on the viewsphere
        bool aRightofB(int indexA, int indexB);
        double aAngleofB(int indexA, int indexB);
        void findRotation(double rotation);
        void addEigenPoint(ptCldColor::Ptr dest, Eigen::Vector4f coordinates);
        double prevDist = false;
        bool firstRequest = true;
        bool runPastCentroid = false;
        bool runPastCentroid2 = false;
        int firstGraspPoint;
        int secondGraspPoint;
        int firstView = -1;
        vector<int> lookedFor = {};
        Eigen::Matrix4f rotateObject;
};

#endif