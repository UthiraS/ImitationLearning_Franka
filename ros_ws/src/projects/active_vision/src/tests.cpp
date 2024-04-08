#include <gtest/gtest.h>
#include "optimality/opt_path_finder.h"
#include <active_vision_tools/mathConvenience.h>
#define random double(rand())/RAND_MAX * 2 - 1
using namespace std;


vector<vector<double>> genPairs(int numpairs) {
    vector<vector<double>> ret = {};
    srand(0);
    for (int i = 0; i<numpairs; i++) {
        ret.push_back({random, random, 
        random, random, 
        random, random, 
        2*M_PI*(random), 
        random, random, random});
    }
    ret.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    return ret;
}
class sphereTestParam : public ::testing::TestWithParam<vector<double>> {
};

TEST_P(sphereTestParam, CommutativeDistance) {
    vector<double> pair = GetParam();
    Eigen::Vector3d v1(pair[0], pair[1], pair[2]);
    Eigen::Vector3d v2(pair[3], pair[4], pair[5]);
    ASSERT_EQ(sphericalDistance(v2, v1), sphericalDistance(v1, v2));
}

TEST_P(sphereTestParam, IdentityDistance) {
    vector<double> pair = GetParam();
    Eigen::Vector3d v1(pair[0], pair[1], pair[2]);
    if(0 != v1.norm()){
        ASSERT_EQ(sphericalDistance(v1, v1), 0);
    } else {
        ASSERT_EQ(sphericalDistance(v1, v1), -1);
    }
}

TEST_P(sphereTestParam, AnyRotationAnyAxis) {
    vector<double> pair = GetParam();
    Eigen::Vector3d v1(pair[0], pair[1], pair[2]);
    Eigen::Vector3d v2(pair[3], pair[4], pair[5]);
    double rotationangle = pair[6];
    Eigen::Vector3d randomaxis(pair[7], pair[8], pair[9]);
    randomaxis.normalize();
    Eigen::AngleAxis<double> rotation(rotationangle, randomaxis);
    ASSERT_NEAR(sphericalDistance(v2, v1), sphericalDistance(rotation*v2, rotation*v1), 0.00001);
}

//Check that rotation does not change relative distances for random points: DONE

INSTANTIATE_TEST_CASE_P(
    optimalityTests,
    sphereTestParam,
    ::testing::ValuesIn(genPairs(360))
);

class specialCoordinates {
    public:
        double x1, y1, z1, x2, y2, z2, rotation, axisX, axisY, axisZ;
        //vector<double>input = {};
        specialCoordinates(double _x1, double _y1, double _z1, double _x2, double _y2, double _z2, double _rotation, double _axisX, double _axisY, double _axisZ) {
            x1 = _x1;
            y1 = _y1;
            z1 = _z1;
            x2 = _x2;
            y2 = _y2;
            z2 = _z2;
            rotation = _rotation;
            axisX = _axisX;
            axisY = _axisY;
            axisZ = _axisZ;
            // input.push_back(x1);
            // input.push_back(y1);
            // input.push_back(z1);
            // input.push_back(x2);
            // input.push_back(y2);
            // input.push_back(z2);
            // input.push_back(rotation);
            // input.push_back(axisX);
            // input.push_back(axisY);
            // input.push_back(axisZ);
        }
         
};
std::vector<specialCoordinates> genList(bool useRandom) {
  std::vector<specialCoordinates> ret = {};
  std::vector<std::vector<double>> input = {};
  srand(0);
  input.push_back({1, 0, 0, -1, 0, 0, 0, 0, 0, 0});
  input.push_back({0, 1, 0, 0, -1, 0, 0, 0, 0, 0});
  input.push_back({0, 0, 1, 0, 0, -1, 0, 0, 0, 0});
  input.push_back({sin(M_PI/4), sin(M_PI/4), 0, -sin(M_PI/4), -sin(M_PI/4), 0, 0, 0, 0, 0});
  input.push_back({-sin(M_PI/4), sin(M_PI/4), 0, sin(M_PI/4), -sin(M_PI/4), 0, 0, 0, 0, 0});
  input.push_back({0, sin(M_PI/4), sin(M_PI/4), 0, -sin(M_PI/4), -sin(M_PI/4), 0, 0, 0, 0});
  input.push_back({0, -sin(M_PI/4), sin(M_PI/4), 0, sin(M_PI/4), -sin(M_PI/4), 0, 0, 0, 0});
  input.push_back({sin(M_PI/4), 0, sin(M_PI/4), -sin(M_PI/4), 0, -sin(M_PI/4), 0, 0, 0, 0});
  input.push_back({-sin(M_PI/4), 0, sin(M_PI/4), sin(M_PI/4), 0, -sin(M_PI/4), 0, 0, 0, 0});
  for (int i = 0; i < input.size(); i++) {
    if (useRandom) {
        input[i][6] = 2 * M_PI * random;
        input[i][7] = random;
        input[i][8] = random;
        input[i][9] = random;
    }
    specialCoordinates sc(input[i][0], input[i][1], input[i][2], input[i][3], input[i][4], input[i][5], input[i][6], input[i][7], input[i][8], input[i][9]);
    ret.push_back(sc);
  }
  return ret;
}

struct specialCoordinatestest : public ::testing::TestWithParam<specialCoordinates> {};

TEST_P(specialCoordinatestest, test) {
  specialCoordinates sc = GetParam();
  std::cout << "Input (" <<  sc.x1 << " " << sc.y1 << " " << sc.z1 << ")\n";
  std::cout << "(" << sc.x2 << " " << sc.y2 << " " << sc.z2 << ")\n";
  std::cout << "Rotation angle + axis: " << sc.rotation << " (" << sc.axisX << " " << sc.axisY << " " << sc.axisZ << ")\n";
  Eigen::Vector3d v1(sc.x1, sc.y1, sc.z1);
  Eigen::Vector3d v2(sc.x2, sc.y2, sc.z2);
  std::cout << "Dot product: " << v1.dot(v2) << "\n";
  Eigen::Vector3d randomaxis(sc.axisX, sc.axisY, sc.axisZ);
  randomaxis.normalize();
  Eigen::AngleAxis<double> rotation(sc.rotation, randomaxis);
  EXPECT_DOUBLE_EQ(sphericalDistance(v1, v2), M_PI);
}

INSTANTIATE_TEST_SUITE_P(
    norotation,
    specialCoordinatestest,
    ::testing::ValuesIn(genList(false))
  );

INSTANTIATE_TEST_SUITE_P(
    randomrotation,
    specialCoordinatestest,
    ::testing::ValuesIn(genList(true))
  );

// TEST(specialCoordinatesTest, AnyRotationAnyAxis) {
//    vector<specialCoordinates> input = specialCoordList();
//    for (specialCoordinates sc : input) {
//         Eigen::Vector3d v1(sc.x1, sc.y1, sc.z1);
//         Eigen::Vector3d v2(sc.x2, sc.y2, sc.z2);
//         double rotationangle = sc.rotation;
//         Eigen::Vector3d randomaxis(sc.axisX, sc.axisY, sc.axisZ);
//         Eigen::AngleAxis<double> rotation(rotationangle, randomaxis);
//         EXPECT_NEAR(sphericalDistance(v1, v2), M_PI, 0.0001);
//    }
// }


//Create a list of points with known, hand calculated distances 
//Should include a point in each octant, points at each corner 
// https://en.wikipedia.org/wiki/Antipodal_point , https://en.wikipedia.org/wiki/Great_circle
//Check that rotation does not change those distances
// These points below should all equal M_PI

/* Point list 
(1, 0, 0)
(-1, 0, 0)

(0, 1, 0)
(0, -1, 0)

(0, 0, 1)
(0, 0, -1)

(sin(pi/4), sin(pi/4), 0)
(-sin(pi/4), -sin(pi/4), 0)

(-sin(pi/4), sin(pi/4), 0)
(sin(pi/4), -sin(pi/4), 0)

(0, sin(pi/4), sin(pi/4))
(0, -sin(pi/4), -sin(pi/4))

(0, -sin(pi/4), sin(pi/4))
(0, sin(pi/4), -sin(pi/4))

(sin(pi/4), 0, sin(pi/4))
(-sin(pi/4), 0, -sin(pi/4))

(-sin(pi/4), 0, sin(pi/4))
(sin(pi/4), 0, -sin(pi/4))
*/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}