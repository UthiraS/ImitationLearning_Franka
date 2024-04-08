#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/copy_point.h>
#include <active_vision/getStatePCASRV.h>
#include <features/vfh_features.h>
#include <features/features.h>
#include <features/gasd_features.h>
#include <features/grsd_features.h>
#include <features/cvfh_features.h>
#include <features/esf_features.h>
#include <features/haf_features.h>
#include <features/fpfh_features.h>
#include <string>

std::vector<float> getStateVec(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obj, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_unexpobj, std::string feature_type, float c0, float c1, float c2)
{
	Features *features_obj;
	Features *features_unexp;
	std::vector<float> stateVector;
	stateVector.clear();
	try
	{
		if (feature_type == "HAF")
		{
			HAF_Features *features_haf = new HAF_Features();
			features_haf->setInput(cloud_obj);
			features_haf->setInputUnexp(cloud_unexpobj);
			features_haf->calculateFeatures();
			features_haf->convertDesctoVec(stateVector);

			delete features_haf;
		}
		else
		{
			if (feature_type == "VFH")
			{
				features_obj = new VFH_Features();
				features_unexp = new VFH_Features();
			}
			else if (feature_type == "CVFH")
			{
				features_obj = new CVFH_Features();
				features_unexp = new CVFH_Features();
			}
			else if (feature_type == "GASD")
			{
				features_obj = new GASD_Features();
				features_unexp = new GASD_Features();
			}
			else if (feature_type == "ESF")
			{
				features_obj = new ESF_Features();
				features_unexp = new ESF_Features();
			}
			else if (feature_type == "GRSD")
			{
				features_obj = new GRSD_Features();
				features_unexp = new GRSD_Features();
			}
			else if (feature_type == "FPFH")
			{
				features_obj = new FPFH_Features();
				features_unexp = new FPFH_Features();
			}

			features_obj->setInput(cloud_obj);
			features_obj->calculateFeatures();

			features_unexp->setInput(cloud_unexpobj);
			features_unexp->calculateFeatures();

			features_obj->convertDesctoVec(stateVector);
			features_unexp->convertDesctoVec(stateVector);

			delete features_obj;
			delete features_unexp;
		}
		stateVector.push_back(c0);
		stateVector.push_back(c1);
 		stateVector.push_back(c2);
	}
	catch (...)
	{
		std::cout << "-------------------------Wrong Data: Will return empty state vector----------------------" << std::endl;
		stateVector.clear();
	}

	return stateVector;
}

bool getState(active_vision::getStatePCASRV::Request &req,
			  active_vision::getStatePCASRV::Response &res)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obj(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(req.pcd_obj, *cloud_obj);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_unexpobj(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(req.pcd_unexpobj, *cloud_unexpobj);
		if (cloud_obj->points.size() < 20 || cloud_unexpobj->points.size() < 20)
		{
			res.done = false;
			std::cout << "-------------------------Wrong Data: Will return empty state vector(less data)----------------------" << std::endl;
		}
		else
		{
			res.stateVec.data = getStateVec(cloud_obj, cloud_unexpobj, req.feature_type, req.c0, req.c1, req.c2);
			res.done = true;
		}
	}
	catch (...)
	{
		res.done = false;
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "GetStateVectorPCA");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("/active_vision/getStateVectorPCA", getState);
	while (ros::ok())
	{
		ros::spin();
	}
	return (0);
}
