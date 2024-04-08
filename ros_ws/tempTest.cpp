
#include <vector>
#include <map>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

using namespace Eigen;
using namespace std;

bool aDexterb(Vector3d a, Vector3d b, Vector3d n){
    cout << "Dexterity " << a.cross(b).sum() << " " << a.cross(b).dot(n) << endl;
    return (a.cross(b).dot(n) > 0.0);
}

double radiansToDegrees(double radians){
    return (180 * radians / M_PI);
}

//https://stackoverflow.com/questions/2782647/how-to-get-yaw-pitch-and-roll-from-a-3d-vector
void calc2(vector<double> &startPose, int angle){
    double radians = angle * M_PI / 180.0;
    double step = 15 * M_PI / 180.0;
    double sPhi = asin(startPose[2]);
    double sTheta = atan2(startPose[1], startPose[0]);
    cout << "Start angles" << endl;
    cout << sPhi << " " << sTheta << endl;
    double fPhi = step * cos(radians) + sPhi;
    double fTheta = step * sin(radians) + sTheta;
    cout << "End angles" << endl;
    cout << fPhi << " " << fTheta << endl;
    vector<double> ret = {
        cos(fTheta)*cos(fPhi), 
        sin(fTheta)*cos(fPhi),
        sin(fPhi)};
    cout << "Delta from " << radians << ":" << endl;
    cout << startPose[0] << " " << startPose[1] << " " << startPose[2] << " " << endl;
    cout << ret[0] << " " << ret[1] << " " << ret[2] << " " << endl;
    //Error checking: Left/Right
    if(0 < angle && 180 > angle){
        assertm((sTheta <= fTheta + 1e-6), "Moving left should increase theta");
    } else if(180 < angle && 360 > angle){
        assertm((fTheta <= sTheta + 1e-6), "Moving right should decrease theta");
    }
    //Error checking: Up/Down
    if(90 > angle || 270 < angle){
        assertm((sPhi <= fPhi+1e-6), "Moving up should never decrease phi");
    } else if(90 < angle && 270 > angle){
        assertm((fPhi <= sPhi+1e-6), "Moving down should never increase phi");
    }
}

void calcExplorationPose(vector<double> &startPose, int angle){

    double radians = angle * M_PI / 180.0;
    double step = 15 * M_PI / 180.0;
  

//   pcl::PointXYZ centre(0,0,0);
//   pcl::PointXYZ stPoint = sphericalToCartesian(startPose);
//   pcl::PointXYZ endPoint;

//   zAxis = stPoint.getVector3fMap(); zAxis.normalize();
//   xAxis = zAxis.cross(xyPlane);
//   if(xAxis.norm() == 0) xAxis << sin(startPose[1]),cos(startPose[1]+M_PI),0;
//   xAxis.normalize();
//   yAxis = zAxis.cross(xAxis);
    Vector3f rot(cos(step),0,sin(step));
    Vector3f stR(1, 0, 0);
    Matrix3f rotateAboutX, rotateAboutZ;
    rotateAboutX = AngleAxisf(radians,Vector3f(1,0,0));
    rot = rotateAboutX * rot;
    cout << "Initial rotation " << radians << " about x:" << endl;
    cout << rot[0] << " " << rot[1] << " " << rot[2] << " " << endl;
    //Rotate in the first quadrant
    Matrix3f R;
    R = Quaternionf().setFromTwoVectors(Vector3f(1,0,0),Vector3f(startPose[0], startPose[1], startPose[2]));
    // R = Quaternionf().setFromTwoVectors(Vector3f(1,0,0),Vector3f(abs(startPose[0]), abs(startPose[1]), abs(startPose[2])));
    rot = R * rot;
    stR = R * stR;
    cout << "Translated to positions:" << endl;
    cout << rot[0] << " " << rot[1] << " " << rot[2] << " " << endl;
    cout << stR[0] << " " << stR[1] << " " << stR[2] << " " << endl;
    //Translate to the correct quadrant
    bool XNeg(0.0 > startPose[0]);
    bool YNeg(0.0 > startPose[1]);
    cout << "XNeg? " << XNeg << " YNeg? " << YNeg << endl;
    rotateAboutZ = AngleAxisf(0,Vector3f(0,0,1));
    if(XNeg && YNeg)
        rotateAboutZ = AngleAxisf(M_PI,Vector3f(0,0,1));
    else if(XNeg)
        rotateAboutZ = AngleAxisf(M_PI/2,Vector3f(0,0,1));
    else if(YNeg)
        rotateAboutZ = AngleAxisf(3*M_PI/2,Vector3f(0,0,1));
    cout << "Moved to quadrant:" << endl;
    rot = rotateAboutZ * rot;
    cout << rot[0] << " " << rot[1] << " " << rot[2] << " " << endl;
    //Error checking: Left/Right
    if(0 < angle && 180 > angle){
        assertm((rot[1] <= stR[1]+1e-6), "Moving left should never increase y");
    } else if(180 < angle && 360 > angle){
        assertm((stR[1] <= rot[1]+1e-6), "Moving right should never decrease y");
    }
    //Error checking: Up/Down
    // if(90 > angle || 270 < angle){
    //     assertm((stR[2] <= rot[2]+1e-6), "Moving up should never decrease z");
    // } else if(90 < angle && 270 > angle){
    //     assertm((rot[2] <= stR[2]+1e-6), "Moving down should never increase z");
    // }

}

vector<vector<double>> genSphere(float scale = 1.0, int nPoints = 20){
  double r, theta, phi;
  r = scale;
  int cPoint = 0;
  double goldenRatio = (1 + pow(5, 0.5))/2;
  double epsilon = 27;
  vector<vector<double>> ret;
  while(cPoint < nPoints){
    // theta = 2 * M_PI * cPoint / goldenRatio;
    // phi = acos(1 - 2*(cPoint+epsilon)/(nPoints-1+2*epsilon));
    double cI = ((double(cPoint)+.5)/double(nPoints));
    phi = acos(1-(2*cI));
    theta = M_PI * (1 + pow(5.0, .5)) * (double(cPoint)+.5);
    vector<double> c = {
        cos(theta) * sin(phi) * scale,
        sin(theta) * sin(phi) * scale,
        cos(phi) * scale};
    // cPt.g = 255;
    ret.push_back(c);
    cPoint++;
  }
  return ret;
}

vector<vector<double>> genCircle(double scale = 1.0, int nPoints = 20)
{
  double r, theta, phi;
  r = scale;
  int cPoint = 0;
  vector<vector<double>> ret;
  while(cPoint < nPoints){
    double cI = ((double(cPoint)+.5)/double(nPoints));
    phi = 45 * (M_PI / 180.0);
    theta = 2* M_PI * (double(cPoint)/double(nPoints));
    vector<double> c = {
        cos(theta) * sin(phi) * scale,
        sin(theta) * sin(phi) * scale,
        cos(phi) * scale
    };
    ret.push_back(c);
    cPoint++;
  }
  return ret;
}

Vector3d project(Vector3d a, Vector3d b){
    Vector3d ret = b - ((b).dot(a)*a);
    ret.normalize();
    return ret;
}

double FullCheck1(Vector3d a, Vector3d b){
    Vector3d z(0,0,1);
    Vector3d bProj = project(a, b);
    Vector3d zProj = project(a, z);
    // cout << bProj << "\n---\n" << zProj << endl;
    double angle = acos(bProj.dot(zProj));
    // cout << "raw dot: " << bProj.dot(zProj) << endl;
    if(aDexterb(zProj, bProj, a)){
        cout << "Fixing sign" << endl;
        angle = 2*M_PI-angle;
    }
        
    return radiansToDegrees(fmod(angle, 2*M_PI));
}

void FullCheck(Vector3d a, Vector3d b){
    double c = FullCheck1(a, b);
    double d = FullCheck1(b, a);
    double diff = abs(c - d);
    cout << c << " " << d << " " << abs(diff - 180) << endl;
    
    assertm((abs(diff - 180))< 12, "Opposites should be 180 degrees apart");
}

void TestRelationship(Vector3d a, Vector3d b, double expectedAngle){
    FullCheck(a, b);
    double angle = FullCheck1(a,b);
    cout << angle << " expected " << expectedAngle << endl;
    double diff = min(abs(angle-expectedAngle), abs(angle-expectedAngle+360));
    diff = min(diff, abs(angle-360-expectedAngle));
    assertm(diff<12, "Angle should be within 10 degrees of the expected angle");
}

void TestCompass(){
    Vector3d center(.5, .5, .71);
    Vector3d north(.35, .35, .87);
    Vector3d ne(.5, .46, .74);
    Vector3d east(.54, .45, .71);
    Vector3d se(.54, .5, .68);
    Vector3d south(.54, .54, .64);
    Vector3d sw(.5, .54, .68);
    Vector3d west(.45, .54, .71);
    Vector3d nw(.46, .5, .74);
    Vector3d directions[] = {north, ne, east, se, south, sw, west, nw};
    double angles[] = {0.0, 315, 270.0, 225, 180.0, 135, 90.0, 45};
    int signX[] = {1,-1,-1,1};
    int signY[] = {1,1,-1,-1};
    bool invert[] = {false, true, false, true};
    for(int i=0; i<8; i++){
        double angle = angles[i];
        Vector3d position = directions[i];
        for(int j=0; j<4; j++){
            double target = angle;
            if(invert[j])
                target = 360 - target;
            cout << target << " - sign (" << signX[j] << "," << signY[j] << ")" << endl;
            Vector3d center_1(center[0]*signX[j], center[1]*signY[j], center[2]);
            Vector3d position_1(position[0]*signX[j], position[1]*signY[j], position[2]);
            TestRelationship(center_1, position_1, target);
        }
        
    }
}

int main(int argc, char **argv)
{
    TestCompass();
    // Vector3d a(.35, .35, .87);
    // Vector3d b(.5, .5, .71);
    // Vector3d c(.54, .45, .71);
    // Vector3d d(.45, .54, .71);
    // Vector3d e(.49, .41, .77);
    // FullCheck(a, b);
    // FullCheck(c, b);
    // FullCheck(d, b);
    // FullCheck(e, b);
    // FullCheck(a, c);
    // FullCheck(a, d);
    // FullCheck(a, e);
    // FullCheck(c, d);
    // FullCheck(c, e);
    // FullCheck(e, e);
    
}