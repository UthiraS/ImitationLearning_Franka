#include <gtest/gtest.h>
#include <ros/master.h>
#include <active_vision/environment.h>
#define SLEEPTIME 0.00
using std::vector;
class environmentTest : public ::testing::TestWithParam<vector<int>> {
protected:

// Gazebo setup
    environment *env;
    ros::ServiceClient gazeboCheckModel;
    gazebo_msgs::GetModelState checkObj;
    bool spin = false;
    //static void SetUpTestSuite() {
    //    std::system("roslaunch active_vision workspace.launch &");
    //    while(std::system("pidof gzserver") != 0) {
    //       sleep(0.5);
           //std::cout << std::system("pidof gzserver") << "\n";
    //    }
    //}
    
    static void TearDownTestSuite() {
       //std::system("killall -9 rosmaster");
       ros::shutdown();
    }
// Passing in dummy varialbes to setup a ros environment and a gazebo model checker
    void SetUp() {
        const std::string s = "testNode";
        char **c = new char*;
        int i = 0;
        ros::init(i, c, s);
        ros::NodeHandle nh;
	    env = new environment(&nh);
        gazeboCheckModel = nh.serviceClient<gazebo_msgs::GetModelState> ("/gazebo/get_model_state");
        //gazebo_msgs::GetModelState checkObj;
        //checkObj.request.model_name = "Cordless Drill";
        checkObj.request.model_name = env->objectDict[GetParam()[0]].description;
    }
// deletes object after each test
    void TearDown() {
        env->deleteObject(GetParam()[0]);
        //ros::shutdown();
    }
// implentatiion of an extra delay -- if spin is set to false above and SLEEPTIME set to 0, this doesnt do anything
    void delay() {
        gazeboCheckModel.call(checkObj);
        if (spin) 
            ros::spinOnce();
        sleep(SLEEPTIME);
    }

public:
// Format of the Google Test test name by modifying the TestParamInfo with the ObjID, the Position and the Angle as well as the overall test number
    struct PrintToStringParamName
    {
        template <class ParamType>
        std::string operator()(const ::testing::TestParamInfo<ParamType>& info) const
        {
            std::stringstream ss;
            ss << "ObjID_" << info.param.at(0)
            << "_Position_" << info.param.at(1)
            << "_Angle_" << info.param.at(2) 
            << "_" << info.param.at(3);
            return ss.str();
        }
    };
};

// Generates a random angle integer between 0 and 359
int genAngle() {
    srand(time(0));
    int random = rand() % 360;
    return random;
}

// generates our test paramters by creating a dummy ros environment in order to return a vector of vectors each of which represents an object in the format:
// {objID, pose#, angle}
vector<vector<int>> genParam() {
    environment *env;
    const std::string s = "testNode";
    char **c = new char*;
    int i = 0;
    ros::init(i, c, s);
    ros::NodeHandle nh_;
	env = new environment(&nh_);
    vector<vector<int>> ret = {};
    //int angle = genAngle();
    int k = 0;
    // for (int objID = 1; objID < 14; objID++) {
    //     int poses = env->objectDict[objID].nPoses;
    //     for (int pose = 0; pose < poses; pose++ ){
    //         for (int angle = 0; angle < 360; angle++)
    //             ret.push_back({objID, pose, angle, k++});
    //     }
    // }
    // for (int objID = 1; objID < 14; objID++) {
    //     int poses = env->objectDict[objID].nPoses;
    //     for (int pose = 0; pose < poses; pose++) {
    //         ret.push_back({objID, pose, 240, k++});
    //     }
    // }
    for (int objID = 5; objID < 6; objID++) {
        int poses = env->objectDict[objID].nPoses;
        for (int pose = 0; pose < poses; pose++ ){
            for (int angle = 0; angle < 360; angle++)
                ret.push_back({objID, pose, angle, k++});
        }
    }
    //ros::shutdown();
    return ret;
}
// Essentially does the same thing as "EXPECT_NEAR" but in this scenario it is written to check whether the opposite signs of coordinate z and w will return true if the regular signs return false due to an error we were getting in testing that had to do w/ the rotation calculations from gazebo in our t ests referring to the same thing, but with oppsoite signs
bool OrEqual(vector<double> values, double z, double w) {
    if (abs(values[0] - values[2]) < 0.05+z * 0.05 && abs(values[1] - values[3]) < 0.05+w * 0.05) {
        return true;
    } else if (abs(-1*values[0] - values[2]) < 0.05+z * 0.05 && abs(-1*values[1] - values[3]) < 0.05+w * 0.05) {
        return true;
    } else {
        return false;
    }
}

// simply tests to make sure gazebo detects that an object was not added
TEST_P(environmentTest, noInit) {
    delay();
    EXPECT_FALSE(checkObj.response.success);
}

// test to make sure each model can be added correclty at each pose w/ no rotation
TEST_P(environmentTest, addModel) {
    env->moveObject(GetParam()[0], GetParam()[1], 0);
    delay();
    EXPECT_TRUE(checkObj.response.success);
}

// test to make sure each model is added correclty at each pose w/ no rotation, and that for 0-360 degrees, each obj at each pose is moved to the correct place
TEST_P(environmentTest, moveModel) {
   // std::cout << "Pose " << i+1 << std::endl;
    env->moveObject(GetParam()[0], GetParam()[1], 0);
    delay();

    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 m_starting_rot;
    m_starting_rot.setEulerYPR(0, env->objectDict[GetParam()[0]].poses[GetParam()[1]][2], env->objectDict[GetParam()[0]].poses[GetParam()[1]][1]);

    // Convert into quaternion
    tf::Quaternion start_quat;
    m_starting_rot.getRotation(start_quat);

    EXPECT_TRUE(checkObj.response.success);
    EXPECT_NEAR(checkObj.response.pose.orientation.x, start_quat.x(), 0.05+start_quat.x()*0.05);
    EXPECT_NEAR(checkObj.response.pose.orientation.y, start_quat.y(), 0.05+start_quat.y()*0.05);
    EXPECT_NEAR(checkObj.response.pose.orientation.z, start_quat.z(), 0.05+start_quat.z()*0.05);
    EXPECT_NEAR(checkObj.response.pose.orientation.w, start_quat.w(), 0.05+start_quat.w()*0.05);

    // EXPECT_NEAR(checkObj.response.pose.position.x, env->tableCentre[0], 0.01);
    // float angle = genAngle();

    env->moveObject(GetParam()[0], GetParam()[1], GetParam()[2]* M_PI/180.0);
    // std::cout << angle << " rotation radians" << std::endl;
    delay();

    tf::Matrix3x3 m_rotation;
    m_rotation.setEulerYPR(GetParam()[2]* M_PI/180.0, env->objectDict[GetParam()[0]].poses[GetParam()[1]][2], env->objectDict[GetParam()[0]].poses[GetParam()[1]][1]);

    tf::Quaternion rot_quat;
    m_rotation.getRotation(rot_quat);

    //std::cout << "\n" << "Quat Coordinate: (" << checkObj.response.pose.orientation.x << ", " << checkObj.response.pose.orientation.y << ", " 
    //    << checkObj.response.pose.orientation.z << ", " << checkObj.response.pose.orientation.w << ")\n";
    
    EXPECT_NEAR(checkObj.response.pose.orientation.x, rot_quat.x(), 0.05+rot_quat.x()*0.05);// or if w * -1, z * -1 = true
    EXPECT_NEAR(checkObj.response.pose.orientation.y, rot_quat.y(), 0.05+rot_quat.y()*0.05);
    // EXPECT_NEAR(checkObj.response.pose.orientation.z, rot_quat.z(), 0.05+rot_quat.z()*0.05);
    // if (testing::Test::HasFailure()) {
    //     std::cout << "\n Checking opposite sign for z \n";
    //     EXPECT_NEAR(checkObj.response.pose.orientation.z * -1, rot_quat.z(), 0.05+rot_quat.z() * 0.05);
    //     //testing::ADD_SUCCESS();
    // }
    // EXPECT_NEAR(checkObj.response.pose.orientation.w, rot_quat.w(), 0.05+rot_quat.w()*0.05);
    // if (testing::Test::HasFailure()) {
    //     std::cout << "\n Checking opposite sign for w \n";
    //     EXPECT_NEAR(checkObj.response.pose.orientation.w * -1, rot_quat.w(), 0.05+rot_quat.w() * 0.05);
    //     //testing::ADD_SUCCESS();
    // }

    // value vector, the gazebo calculated z and w as well as our calculated z and w
    vector<double> values = {checkObj.response.pose.orientation.z, checkObj.response.pose.orientation.w, rot_quat.z(), rot_quat.w()};
    // workaround to avoid error stated aboce at the OrEqual function
    EXPECT_TRUE(OrEqual(values, rot_quat.z(), rot_quat.w()));
    EXPECT_TRUE(checkObj.response.success);
}

// test to make sure each object at each pose can be added and deleted correctly
TEST_P(environmentTest, deleteModel) {
    env->moveObject(GetParam()[0], GetParam()[1], 0);
    delay();
    EXPECT_TRUE(checkObj.response.success);

    env->deleteObject(GetParam()[0]);
    delay();
    EXPECT_FALSE(checkObj.response.success);
}


// Instantiation of the environment test, using the values in the genparameters function return as our testing parameters, and using the StringParamName function to generate a custom test name for each test
INSTANTIATE_TEST_SUITE_P(allObjects, environmentTest, ::testing::ValuesIn(genParam()), environmentTest::PrintToStringParamName()
    //[](const ::testing::TestParamInfo<environmentTest::vector<int>>& info) {
    //   // Can use info.param here to generate the test suffix
    //   std::string name = environmentTest.checkObj.request.model_name + "\nPosition: " + environmentTest.checkObj.request.pose;
    //   return name;
    //}
);

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}