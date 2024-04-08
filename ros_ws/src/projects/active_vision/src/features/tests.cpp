#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../include/features/gasd_features.h"

using ::testing::AtLeast;
using ::testing::_;

class MockFeatures : GASD_Features {
    public:
    MOCK_METHOD(void, setInput, (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud), ());
    MOCK_METHOD(void, calcaulateFeatures, (), ());
    MOCK_METHOD(void, visualize, (), ());
    MOCK_METHOD(void, convertDesctoVec, (std::vector<float> &featVector), ());
    MOCK_METHOD(void, mergeFeaturesVectors, (), ());
};

TEST(inputtest, equality) {
    MockFeatures ft;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    EXPECT_CALL(ft, setInput(sceneCloud)).Times(1);
    ft.setInput(sceneCloud);
    


}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}