#include <active_vision/environment.h>
#include <active_vision/toolDataHandling.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/toolGraspSynthesis.h>
#include <active_vision/toolStateVector.h>
#include <active_vision/heuristicPolicySRV.h>
#include <active_vision/trainedPolicySRV.h>
#include <active_vision/controlSRV.h>
#include <active_vision/restartObjSRV.h>
#include <active_vision/getStatePCASRV.h>
#include <features/vfh_features.h>
#include <features/features.h>
#include <features/gasd_features.h>
#include <features/grsd_features.h>
#include <features/cvfh_features.h>
#include <features/esf_features.h>
#include <features/haf_features.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

std::string feature_type = "GRSD";
int mode;
int recordCode;
int HAFstVecGridSize;
int visualizationLevel;
environment *env;
RouteData *home = NULL;
RouteData *cur = NULL;
std::vector<double> startPose;
int cObj;
int maxSteps;
float viewSphereRad;
ros::ServiceClient heuristicPolicyClient;
ros::ServiceClient stateGetservice;
ptCldVis::Ptr viewer;
static keyboardEvent keyPress;
std::vector<int> vp;
std::string dir;
std::string saveLocation;
std::string cPolicy;
int logging = 0;
graspSynthesis *g;
int iteration =0;
void updateRouteData(environment &env, RouteData &data, bool save, std::string name)
{
	AVLOG("UpdateRouteData start",logging,1);
	data.success = (env.graspID != -1);
	if (save == true)
	{
		// data.obj = *env.ptrPtCldObject;
		// data.unexp = *env.ptrPtCldUnexp;
		data.childID = env.saveConfiguration(name);
	}
	if (data.success == true)
	{
		if (save == true)
			data.goodInitialGrasp = true;
		data.graspQuality = env.graspData.quality;
		env.updateGripper(env.graspID, 0);
		// data.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
	}
	else
	{
		if (save == true)
			data.goodInitialGrasp = false;
		// data.env = *env.ptrPtCldEnv;
	}
	AVLOG("UpdateRouteData end",logging,1);
}

std::vector<int> nearbyDirections(int dir)
{
	std::vector<int> dirs = {1, 2, 3, 4, 5, 6, 7, 8};
	dir--;
	dirs[0] = (8 + dir) % 8 + 1;
	dirs[1] = (8 + dir + 1) % 8 + 1;
	dirs[2] = (8 + dir - 1) % 8 + 1;
	dirs[3] = (8 + dir + 2) % 8 + 1;
	dirs[4] = (8 + dir - 2) % 8 + 1;
	dirs[5] = (8 + dir + 3) % 8 + 1;
	dirs[6] = (8 + dir - 3) % 8 + 1;
	dirs[7] = (8 + dir + 4) % 8 + 1;
	return dirs;
}

void resetEnv()
{
	env->spawnObject(cObj, 0, 0);
	ROS_WARN("spawing object %d", cObj);
	startPose = {env->viewsphereRad, 180 * (M_PI / 180.0), 30 * (M_PI / 180.0)};
	env->reset();
	// Home Pose
	if(NULL != home){
		free(home);
		free(cur);
	}
	home = new RouteData;
	cur = new RouteData;
	home->reset();
	home->path = {startPose};
	home->type = 1;
	home->direction = 0;
	home->nSteps = 0;
	home->timer = {0};
	*cur = *home;
}

std::vector<float> readyStateVec()
{
	HAFStVec1 stVec;
	ptCldColor::Ptr tempPtCldObj{new ptCldColor};
	ptCldColor::Ptr tempPtCldUnexp{new ptCldColor};
	stVec.setGridDim(::HAFstVecGridSize);
	stVec.setMaintainScale(true);
	*tempPtCldObj = *env->ptrPtCldObject;
	*tempPtCldUnexp = *env->ptrPtCldUnexp;
	stVec.setInput(tempPtCldObj, tempPtCldUnexp, cur->path.back());
	stVec.calculate();
	return stVec.getStateVec();
}

std::vector<float> getStateVec()
{
	AVLOG("getSTateVec",logging,1);
	active_vision::getStatePCASRV stateSrv;
	pcl::toROSMsg(*env->ptrPtCldObject, stateSrv.request.pcd_obj);
	pcl::toROSMsg(*env->ptrPtCldUnexp, stateSrv.request.pcd_unexpobj);
	stateSrv.request.feature_type = feature_type;
	stateSrv.request.c0 = cur->path.back()[0];
	stateSrv.request.c1 = cur->path.back()[1];
	stateSrv.request.c2 = cur->path.back()[2];

	stateGetservice.call(stateSrv);
	return stateSrv.response.stateVec.data;
}

bool checkDone()
{
	return cur->success;
}

void print(std::vector<double> const &a)
{
	for (int i = 0; i < a.size(); i++)
	{
		std::cout << a.at(i) << " ";
	}
	std::cout << std::endl;
}

std::vector<double> discreteMove(int direction, float step)
{
	std::vector<int> dirPref = nearbyDirections(direction);
	int prfID = 0;
	
	std::vector<double> nextPose = calcExplorationPose(cur->path.back(), dirPref[prfID], ::mode, step);
	// printf("Calculated next pose.\n");
	print(nextPose);
	while (checkValidPose(nextPose) == false || checkIfNewPose(cur->path, nextPose, step) == false)
	{
		prfID++;
		nextPose = calcExplorationPose(cur->path.back(), dirPref[prfID], step);
		print(nextPose);
	}
	// printf("Checked next pose.\n");
	if (prfID != 0)
	{
		// printf("Direction modified from %d -> %d.\n", dirPref[0], dirPref[prfID]);
	}
	// std::cout << dirLookup[dirPref[prfID]] << ",";
	singlePass(*env, nextPose, false, true, 2, logging);
	// printf("Finished single pass.\n");
	updateRouteData(*env, *cur, false, "dummy");
	
	cur->path.push_back(nextPose);
	cur->nSteps++;
	if (cur->nSteps == 1)
	{
		// printf("Updating home direction.\n");
		home->direction = direction;
	}
	// printf("Done with discreteMove.\n");
	return nextPose;
}

std::vector<double> discreteMove(int direction)
{
	return discreteMove(direction, ::mode);
}

std::vector<double> continuousMove(int direction)
{
	int angleTried = 1;
	int sign = 1;
	int increment = 5;
	int nextAngle = 0;
	// string currentpath_string = to_string(cur->path);
	// print(cur->path.back());
	std::vector<double> nextPose = calcExplorationPose(cur->path.back(), direction, ::mode);
	// printf("Calculated next pose for direction %d.\n", direction);
	// print(nextPose);
	while (checkValidPose(nextPose) == false || checkIfNewPose(cur->path, nextPose, ::mode) == false)
	{
		nextAngle = direction + (sign * angleTried);
		nextPose = calcExplorationPose(cur->path.back(), nextAngle, ::mode);
		//Flip the next direction and increment the angle if we've tried
		// both positive and negative
		sign *= -1;
		if (0 < sign)
		{
			angleTried += increment;
		}
		// print(nextPose);
	}
	
	AVLOG("Checked next pose. ", logging, 1);
	if (0 != nextAngle)
	{
		// printf("Direction modified from %d -> %d.\n", direction, nextAngle);
	}
	// printf("%.2f, %.2f, %.2f\n", nextPose[0], nextPose[1], nextPose[2]);
	singlePass(*env, nextPose, false, true, 2, logging);
	// printf("Finished single pass.\n");
	updateRouteData(*env, *cur, false, "dummy");
	cur->path.push_back(nextPose);
	cur->nSteps++;
	if (cur->nSteps == 1)
	{
		
		AVLOG("Updating home direction. ", logging, 1);
		home->direction = direction;
	}
	
	AVLOG("Done with continuous move. ", logging, 1);
	return nextPose;
}

//Only reacts to q, which should close it not advance the sim. 
void redrawViewer(){
	keyPress.called = false;
	viewer->resetStoppedFlag();
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	addRGB(viewer, env->ptrPtCldObject, "Obj", vp[0]);
	addRGB(viewer, env->ptrPtCldUnexp, "Unexp", vp[0]);
	ptCldColor::Ptr tempPtCld{new ptCldColor};
	// *tempPtCld = cur->env;
	// addRGB(viewer, tempPtCld, "Env", vp[1]);
	pcl::PointXYZ table, a1, a2;
	table.x = env->tableCentre[0];
	table.y = env->tableCentre[1];
	table.z = env->tableCentre[2];

	addViewsphere(viewer, vp.back(), table, cur->path[0][0], false);
	a1 = sphericalToCartesian(cur->path[0], table);
	viewer->addSphere(a1, 0.04, 1, 0, 0, "Start", vp[1]);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Start");

	for (int i = 0; i < cur->path.size() - 1; i++)
	{
		a1 = sphericalToCartesian(cur->path[i], table);
		a2 = sphericalToCartesian(cur->path[i + 1], table);
		viewer->addArrow(a2, a1, 0, 1, 0, false, std::to_string(i), vp[1]);
	}

	while(!viewer->wasStopped() && keyPress.called == false)
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	// setCamView(viewer, cur->path.back(), table);
}

void recordResults(){
	std::fstream fout;
	AVLOG(dir+" "+saveLocation, logging, 1);
	fout.open(dir + saveLocation, std::ios::out | std::ios::app);
	AVLOG("Starting save", logging, 1);
	saveData(*cur, fout, dir, false);
	AVLOG("Finished save", logging, 1);
}

bool startObj(active_vision::restartObjSRV::Request &req,
			  active_vision::restartObjSRV::Response &res)
{
	AVLOG("Starting reset", logging, 1);
	env->deleteObject(cObj, 3);
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	AVLOG("Deleted object " + to_string(cObj), logging, 1);

	//kludge- reset file to save to whenever we get a new object
	std::string tempPolicy;
	ros::NodeHandle nh;
	nh.getParam("/active_vision/policyTester/policy", tempPolicy);
	if(cObj != req.object || cPolicy != tempPolicy){
		cPolicy = tempPolicy;
		saveLocation = cPolicy+":"+getCurTime()+"_dataRec.csv";
	}
	feature_type = req.feature_type;
	cObj = req.object;
	resetEnv();
	AVLOG("Reset object", logging, 1);

	env->moveObject(cObj, req.objPoseCode, req.objYaw * (M_PI / 180.0));
	env->moveCameraViewsphere(startPose);

	int counter = 2;
	while(0 < counter--){
		env->readCamera();
		env->fuseLastData();
		env->dataExtract();
		env->genUnexploredPtCld();
		env->updateUnexploredPtCld();
	}
	

	// printf("%.2f, %.2f, %.2f\n", startPose[0], startPose[1], startPose[2]);
	
	// PLOG;
	home->objType = env->objectDict[cObj].description;
	home->objPose = {env->objectDict[cObj].poses[req.objPoseCode][1],
					 env->objectDict[cObj].poses[req.objPoseCode][2],
					 req.objYaw * (M_PI / 180.0)};
	*cur = *home;
	AVLOG("Reset home", logging, 1);

	singlePass(*env, startPose, true, true, 2, logging);
	AVLOG("Finished pass", logging, 1);

	updateRouteData(*env, *home, false, "Home");
	
	res.stateVec.data = getStateVec();
	
	g->setNormals(env->ptrObjNormal);
	g->preprocessing(env->ptrPtCldObject, env->ptrPtCldUnexp);
	g->calcGraspPairs();
	vector<double> viz;
	g->getCurrentVisibility(viz);
	res.viz.data = viz;
	if(0 < visualizationLevel)
	{
		redrawViewer();	
	}
	

	AVLOG("Done with reset", logging, 1);
	return true;
}

bool takeStep(active_vision::controlSRV::Request &req,
			  active_vision::controlSRV::Response &res)
{
	
	string iteration_string = "itertaion_"+to_string(iteration);
	iteration++;
	AVLOG(iteration_string,logging,1);
	AVLOG("Starting step", logging, 1);
	if (mode == 3)
	{
		continuousMove(req.direction);
	}
	else
	{
		discreteMove(req.direction, req.step);
	}
	AVLOG("finished move", logging, 2);
	
	active_vision::heuristicPolicySRV policy_srv;
	pcl::toROSMsg(*env->ptrPtCldObject, policy_srv.request.object);
	pcl::toROSMsg(*env->ptrPtCldUnexp, policy_srv.request.unexplored);
	policy_srv.request.path.data = cur->path.back();
	policy_srv.request.mode = ::mode;
	policy_srv.request.minPtsVis = 20;
	AVLOG("Calling heuristic", logging, 2);
	heuristicPolicyClient.call(policy_srv);
	AVLOG("Checking state vector", logging, 2);
	res.stateVec.data = getStateVec();
	AVLOG("Checking GrspSynth", logging, 2);
	g->setNormals(env->ptrObjNormal);
	AVLOG(".", logging, 1);
	// cout<<"Env :"<<env<<endl;
	g->preprocessing(env->ptrPtCldObject, env->ptrPtCldUnexp);
	AVLOG("..", logging, 1);
	g->calcGraspPairs();
	AVLOG("...", logging, 1);
	vector<double> viz;
	g->getCurrentVisibility(viz);
	res.viz.data = viz;
	res.done = checkDone();
	AVLOG("....", logging, 1);
	res.steps = cur->nSteps;
	AVLOG("", logging, 1);
	// res.visibilityReward = policy_srv.response.vecVisiblePointsAllDirection.data[req.direction-1]/100.0;
	
	AVLOG("takeStep before redrawViewer :", logging, 1);
	if(0 < visualizationLevel)
	{
		AVLOG("", logging, 3);
		redrawViewer();	
		AVLOG("", logging, 3);
	}
	if(0 < recordCode && (res.done || maxSteps < res.steps)){
		AVLOG("", logging, 3);
		recordResults();
		AVLOG("", logging, 3);
	}
	AVLOG("Done with step", logging, 1);
	return true;
}

void testGrasp(const std_msgs::String::ConstPtr& msg)
{
	AVLOG("Grasp found " + to_string(env->graspID), logging, 1);
	string display_string = to_string(env->graspData.pose[0]) + ", " +
	to_string(env->graspData.pose[1]) + ", " +
	to_string(env->graspData.pose[2]) + ", " +
	to_string(env->graspData.pose[3]) + ", " +
	to_string(env->graspData.pose[4]) + ", " +
	to_string(env->graspData.pose[5]) + ". " ;
	
	AVLOG(display_string, logging, 1);
	
	env->graspObject(env->graspData);
}

void debugCallback(const std_msgs::String::ConstPtr& msg)
{
  pair<string, int> details = fromDebugMessage(msg->data.c_str());
  if("Camera_Service" != details.first){
    return;
  }
  logging = details.second;
}

int main(int argc, char **argv)
{
	chdir(getenv("HOME"));
	ros::init(argc, argv, "Camera_Service");
	ros::NodeHandle nh;
	ros::ServiceClient policy;
	env = new environment(&nh);
	signal(SIGINT, dieWithGrace);
	sleep(1);

	int objID;
	nh.getParam("/active_vision/policyTester/objID", objID);
	nh.getParam("/active_vision/policyTester/recordCode", recordCode);
	nh.getParam("/active_vision/policyTester/maxSteps", maxSteps);
	nh.getParam("/active_vision/policyTester/directory", dir);
	nh.getParam("/active_vision/policyTester/policy", cPolicy);
	dir = ros::package::getPath("active_vision") + dir;
	saveLocation = cPolicy+":"+getCurTime()+"_dataRec.csv";
	if (env->objectDict.count(objID) == 0)
	{
		std::cout << "ERROR. Incorrect Object ID." << std::endl;
		return (-1);
	}
	cObj = objID;

	nh.getParam("/active_vision/cameraMode", ::mode);

	//Float mode:
	::mode = 3;

	nh.getParam("/active_vision/policyTester/HAFstVecGridSize", ::HAFstVecGridSize);
	nh.getParam("/active_vision/policyTester/PolicyVis", ::visualizationLevel);
	nh.getParam("/active_vision/policyTester/feature_type", ::feature_type);
	g = new graspSynthesis();

	if(0 < ::visualizationLevel)
	{
		ptCldVis::Ptr tempViewer(new ptCldVis("PCL Viewer"));
		swap(viewer, tempViewer);
		setupViewer(viewer, 2, vp);
		keyPress = keyboardEvent(viewer, 2);
		viewer->removeCoordinateSystem();
	}

	ros::ServiceServer service_reset = nh.advertiseService("/active_vision/restartEnv", startObj);
	ros::ServiceServer service_step = nh.advertiseService("/active_vision/moveKinect", takeStep);
	ros::Subscriber dSub = nh.subscribe("/debugDelta", 1, debugCallback);
	ros::Subscriber gSub = nh.subscribe("/testGrasp", 1, testGrasp);
	stateGetservice = nh.serviceClient<active_vision::getStatePCASRV>("/active_vision/getStateVectorPCA");
	heuristicPolicyClient = nh.serviceClient<active_vision::heuristicPolicySRV>("/active_vision/heuristic_policy");
	ROS_INFO("Camera control service ready.");

	while (ros::ok())
	{
		ros::spin();
	}
	free(g);
	return (0);
}
