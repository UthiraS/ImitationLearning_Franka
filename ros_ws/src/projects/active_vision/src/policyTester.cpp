#include <active_vision/environment.h>
#include <active_vision/toolDataHandling.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/toolStateVector.h>
#include <active_vision/heuristicPolicySRV.h>
#include <active_vision/trainedPolicySRV.h>
#include <active_vision/getStatePCASRV.h>
#include <active_vision/restartObjSRV.h>
#include <active_vision/controlSRV.h>
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
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
//change featuretype

#define MAGICSTEP 0.05

int runMode;
bool testAll = false;
int maxSteps;
int HAFstVecGridSize;
string simulationMode;
bool recordPtCldEachStep;
string feature_type = "HAF";
int logging = 0;

void help()
{
	cout << "******* Policy Tester Node Help *******" << endl;
	cout << "Arguments : [RunMode]" << endl;
	cout << "RunMode : 0->Manual, 1->Heuristic, 2->Trained Policy" << endl;
	cout << "*******" << endl;
}

bool findGrasp(int object, int objPoseCode, int objYaw, ros::ServiceClient &policy, ros::ServiceClient &restartObjService, ros::ServiceClient &moveObjService, ros::Publisher &currentObjectAnglePublisher)
{
	//Start the object
	active_vision::restartObjSRV startSrv;
	startSrv.request.object = object;
	startSrv.request.objPoseCode = objPoseCode;
	startSrv.request.objYaw = objYaw;
	restartObjService.call(startSrv);
	std_msgs::Float64 angleMsg;
	angleMsg.data = objYaw;
	currentObjectAnglePublisher.publish(angleMsg);
	//while the object is not finished, repeatedly call the policy
	active_vision::trainedPolicySRV queryPolicy;
	queryPolicy.request.stateVec = startSrv.response.stateVec;
	queryPolicy.request.viz = startSrv.response.viz;
	policy.call(queryPolicy);
	int nextDirection = queryPolicy.response.direction;
	active_vision::controlSRV moveSrv;
	moveSrv.request.direction = nextDirection;
	moveSrv.request.step = MAGICSTEP;
	moveObjService.call(moveSrv);
	bool outOfTime = (maxSteps < moveSrv.response.steps);
	while(!moveSrv.response.done && !outOfTime)
	{
		queryPolicy.request.stateVec = moveSrv.response.stateVec;
		queryPolicy.request.viz = moveSrv.response.viz;
		policy.call(queryPolicy);
		nextDirection = queryPolicy.response.direction;
		moveSrv.request.direction = nextDirection;
		moveSrv.request.step = MAGICSTEP;
		moveObjService.call(moveSrv);
		outOfTime = (maxSteps < moveSrv.response.steps);
	}
	return !outOfTime;
}

void getGlobalParameters(ros::NodeHandle &nh){
	nh.getParam("/active_vision/policyTester/maxSteps", ::maxSteps);
	nh.getParam("/active_vision/policyTester/HAFstVecGridSize", ::HAFstVecGridSize);
	nh.getParam("/active_vision/policyTester/recordPtCldEachStep", ::recordPtCldEachStep);
	nh.getParam("/active_vision/simulationMode", ::simulationMode);
}

int getLocalParams(ros::NodeHandle &nh, string &dir, string &csvName, int &objID){
	bool relativePath;
	string temp;
	string time = getCurTime();
	nh.getParam("/active_vision/policyTester/relative_path", relativePath);
	nh.getParam("/active_vision/policyTester/directory", temp);
	if (relativePath == true)
		dir = ros::package::getPath("active_vision") + temp;
	else
		dir = temp;
	nh.getParam("/active_vision/policyTester/csvName", temp);
	if (temp == "default.csv")
		csvName = time + "_dataRec.csv";
	else
		csvName = temp;

	if (csvName.substr(csvName.size() - 4) != ".csv")
	{
		ROS_ERROR("Incorrect file name.");
		help();
		return (-1);
	}
	nh.getParam("/active_vision/policyTester/objID", objID);
	
	return 0;
}

int main(int argc, char **argv)
{
	chdir(getenv("HOME"));
	ros::init(argc, argv, "Policy_Tester");
	ros::NodeHandle nh;
	ros::ServiceClient policy;
	ros::ServiceClient getStateService;
	ros::ServiceClient restartObjService;
	ros::ServiceClient moveObjService;
	ros::Publisher currentObjectPublisher = nh.advertise<std_msgs::String>("/active_vision/cFileName",1);
	ros::Publisher currentObjectAnglePublisher = nh.advertise<std_msgs::Float64>("/active_vision/cRotation",1);
	ros::Publisher graspTestPublisher = nh.advertise<std_msgs::String>("testGrasp", 1);
	environment env(&nh);
	signal(SIGINT, dieWithGrace);
	sleep(1);

	
	string dir;
	string csvName;
	int objID;
	
	if(0 != getLocalParams(nh, dir, csvName, objID)){
		AVLOG("ERROR- parameter loading failed", logging, 0);
		return -1;
	}
	if (env.objectDict.count(objID) == 0)
	{
		AVLOG("ERROR. Incorrect Object ID.", logging, 1);
		return (-1);
	}
	getGlobalParameters(nh);

	restartObjService = nh.serviceClient<active_vision::restartObjSRV>("/active_vision/restartEnv");
	moveObjService = nh.serviceClient<active_vision::controlSRV>("/active_vision/moveKinect");

	::runMode = atoi(argv[1]);
	if (::runMode < MANUAL && ::runMode > TRAINED)
		::runMode = MANUAL;

	if (MANUAL == ::runMode)
	{
		ROS_INFO("Manual Mode Selected.");
	}
	else if (HEURISTIC == ::runMode)
	{
		while (!ROSCheck("SERVICE", "/active_vision/heuristic_policy"))
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		policy = nh.serviceClient<active_vision::heuristicPolicySRV>("/active_vision/heuristic_policy");
		ROS_INFO("Heuristic Policy Selected.");
	}
	else if (TRAINED == ::runMode)
	{
		while (!ROSCheck("SERVICE", "/active_vision/trained_policy"))
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		policy = nh.serviceClient<active_vision::trainedPolicySRV>("/active_vision/trained_policy");
		while (!ROSCheck("SERVICE", "/active_vision/getStateVectorPCA"))
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}
		getStateService = nh.serviceClient<active_vision::getStatePCASRV>("/active_vision/getStateVectorPCA");
		
		ROS_INFO("Trained Policy Selected.");
	}
	
	if (::simulationMode == "SIMULATION" && MANUAL != ::runMode)
	{
		printf("Do you want to test for all possible poses (0->No, 1->Yes) : ");
		string inputS;
		cin >> inputS;
		::testAll = (1 == atoi(inputS.c_str()));
	}

	env.spawnObject(objID, 0, 0);

	std_msgs::String objName;
	objName.data = to_string(objID);
	currentObjectPublisher.publish(objName);
	
	AVLOG("Testing for Object: "+env.objectDict[objID].fileName,logging,1);

	chrono::high_resolution_clock::time_point start, end;
	double elapsed;
	if (!testAll)
	{
		int objPoseCode = 0;
		int objYaw = 0;
		if (::simulationMode != "FRANKA")
		{
			printf("Enter the object pose code (0-%d) : ", int(env.objectDict[objID].nPoses - 1));
			cin >> objPoseCode;
			if (objPoseCode < 0 || objPoseCode > env.objectDict[objID].nPoses - 1)
				objPoseCode = 0;
			printf("Enter the object yaw (deg) (0-360) : ");
			cin >> objYaw;
		}

		bool res = findGrasp(objID, objPoseCode, objYaw, policy, restartObjService, moveObjService, currentObjectAnglePublisher);
		
		if (res){
			if (::simulationMode != "SIMULATION"){
				AVLOG("Checking grasp!", logging, 1);
				std_msgs::String msg;
				graspTestPublisher.publish(msg);
				AVLOG("Grasp checked", logging, 1);
			}
		}
		else
			AVLOG("Grasp not found, env.graspID = "+to_string(env.graspID), logging, 1);
		env.deleteObject(objID);
		return 0;
	}

	// Reading the yawValues csv file
	string yawAnglesCSVDir;
	nh.getParam("/active_vision/policyTester/yawAnglesCSVDir", yawAnglesCSVDir);
	yawAnglesCSVDir = ros::package::getPath("active_vision") + yawAnglesCSVDir;
	string yawAnglesCSV;
	nh.getParam("/active_vision/policyTester/yawAnglesCSV", yawAnglesCSV);
	vector<vector<string>> yawAngle = readCSV(yawAnglesCSVDir + yawAnglesCSV);

	// Converting it to a dictionary format (First column is the key)
	map<string, vector<int>> yawAngleDict;
	for (int row = 0; row < yawAngle.size(); row++)
	{
		yawAngleDict.insert({yawAngle[row][0], {}});
		for (int col = 1; col < yawAngle[row].size(); col++)
		{
			yawAngleDict[yawAngle[row][0]].push_back(stoi(yawAngle[row][col]));
		}
	}

	int uniformYawStepSize;
	nh.getParam("/active_vision/policyTester/uniformYawStepSize", uniformYawStepSize);
	int nDataPoints;
	nh.getParam("/active_vision/policyTester/nDataPoints", nDataPoints);
	if (::simulationMode == "FRANKASIMULATION")
		nDataPoints = 3;
	for (int objPoseCode = 0; objPoseCode < env.objectDict[objID].nPoses; objPoseCode += 1)
	{
		int pointsToGo = nDataPoints;

		// Checking for the uniform steps
		for (int objYaw = env.objectDict[objID].poses[objPoseCode][3]; objYaw < env.objectDict[objID].poses[objPoseCode][4]; objYaw += uniformYawStepSize)
		{
			printf("%d - Obj #%d, Pose #%d, Yaw %d \t", nDataPoints - pointsToGo + 1, objID, objPoseCode, objYaw);
			start = chrono::high_resolution_clock::now();
			findGrasp(objID, objPoseCode, objYaw, policy, restartObjService, moveObjService, currentObjectAnglePublisher);
			end = chrono::high_resolution_clock::now();
			elapsed = (chrono::duration_cast<chrono::milliseconds>(end - start)).count();
			AVLOG("\tTime(ms) : "+to_string(elapsed), logging, 1);
			pointsToGo--;
		}

		string key = to_string(int(env.objectDict[objID].poses[objPoseCode][3])) + "-" +
							to_string(int(env.objectDict[objID].poses[objPoseCode][4]));

		if (yawAngleDict.count(key) == 0)
			continue;

		// Checking for the random steps
		for (int ctr = 0; ctr < yawAngleDict[key].size() && pointsToGo > 0; ctr++)
		{
			printf("%d - Obj #%d, Pose #%d, Yaw %d \t", nDataPoints - pointsToGo + 1, objID, objPoseCode, yawAngleDict[key][ctr]);
			start = chrono::high_resolution_clock::now();
			findGrasp(objID, objPoseCode, yawAngleDict[key][ctr], policy, restartObjService, moveObjService, currentObjectAnglePublisher);
			end = chrono::high_resolution_clock::now();
			elapsed = (chrono::duration_cast<chrono::milliseconds>(end - start)).count();
			AVLOG("\tTime(ms) : "+to_string(elapsed), logging, 1);
			pointsToGo--;
		}
	}


	env.deleteObject(objID);
	printf("Data saved to : %s\n", csvName.c_str());
}
