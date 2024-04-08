#include <active_vision/environment.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/toolVisualization.h>
#include "../include/dataset/dataset.h"

int mode;
int nRdmSearches;
int maxRndSteps;
int maxOptStep = 2;

class processTimer
{
private:
	std::chrono::high_resolution_clock::time_point start, end;
	char *process_name;
	int level;

public:
	processTimer(char *name, int level);
	double getTime();
};

processTimer::processTimer(char *name, int level)
{
	this->process_name = name;
	this->start = std::chrono::high_resolution_clock::now();
	this->level = level;
}

double processTimer::getTime()
{
	this->end = std::chrono::high_resolution_clock::now();
	double elapsed_milis = (std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count();
	std::cout << std::string(this->level, '\t');
	// printf("Process %s took %.2f ms\n", this->process_name, elapsed_milis);
	return elapsed_milis;
}

std::vector<int> randomDirection()
{
	std::vector<int> dirs = {1, 2, 3, 4, 5, 6, 7, 8};
	std::vector<int> ret;
	int randIndex;
	while (dirs.size() > 0)
	{
		randIndex = rand() % (dirs.size());
		ret.push_back(dirs[randIndex]);
		dirs.erase(dirs.begin() + randIndex);
	}
	return ret;
}

// Explore one step in all deirection from the home/parent position
// Configurations are saved
bool exploreOneStepAllDir(environment &env, RouteData &home, std::vector<RouteData> &opOneStep)
{
	processTimer t = processTimer((char *)"oneStepAllDir", 3);
	std::vector<double> curPose, timing;
	RouteData temp; // Used to record the iteration details

	// Sanity check
	if (home.childID == -1 || home.success == true)
	{
		printf("ERROR exploreOneStepAllDir: Invalid child ID / Grasp already found.\n");
		return (false);
	}

	temp.parentID = home.childID;
	temp.success = false;
	// Going to each direction for a step and saving their configurations
	std::vector<int> dirs = randomDirection();
	for (int i = 1; i <= 8 && temp.success == false; i++)
	{
		// Rollback to the parent/home configuration
		env.rollbackConfiguration(temp.parentID);
		curPose = calcExplorationPose(env.lastCameraPoseViewsphere, dirs[i - 1], 1);
		// std::cout << "." << std::flush;
		// If camera pose if above the table, then proceed in that direction
		if (checkValidPose(curPose) == true && checkIfNewPose(home.path, curPose, ::mode) == true)
		{
			// Saving the direction and path taken
			temp.direction = dirs[i - 1];
			temp.path = home.path;
			temp.path.push_back(curPose);

			// Doing a pass and saving the child
			timing = singlePass(env, curPose, false, true);
			temp.success = (env.graspID != -1);
			temp.childID = env.saveConfiguration(env.configurations[temp.parentID].description + "_C" + std::to_string(dirs[i - 1]));
			// printf("Timing details: ");
			// for(int j=0; j<timing.size(); j++){
			// 	std::cout << timing[j] << " " << std::flush;
			// }
			// printf("\n");
			// Exit the loop and updating variables used for visualization
			if (temp.success == true)
			{
				temp.graspQuality = env.graspData.quality;
				env.updateGripper(env.graspID, 0);
				temp.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
			}
			else
			{
				temp.graspQuality = -1;
				temp.env = *env.ptrPtCldEnv;
			}
			temp.env += *env.ptrPtCldUnexp;

			opOneStep.push_back(temp);
		}
	}
	t.getTime();
	return (temp.success);
}

// Explore a max of "nSteps" random steps from the home/parent position
// Configurations not saved
bool exploreRdmStep(environment &env, RouteData &home, int nSteps, RouteData &opData)
{

	std::vector<double> curPose;

	// Sanity check
	if (home.childID == -1 || home.success == true)
	{
		printf("ERROR exploreRdmStep: Invalid child ID / Grasp already found.\n");
		return (false);
	}

	opData.parentID = home.childID;
	opData.direction = home.direction;
	opData.path = home.path;

	// Rollback to the parent/home configuration
	env.rollbackConfiguration(opData.parentID);

	opData.success = false;
	// Going taking a max of nSteps from the parent position
	for (int i = 1; i <= nSteps && opData.success == false; i++)
	{
		// Do until the pose is valid
		std::vector<int> dirs = randomDirection();
		bool foundNextPose = false;
		for (int j = 0; j < 7; j++)
		{
			curPose = calcExplorationPose(env.lastCameraPoseViewsphere, dirs[j], 1);
			if (checkValidPose(curPose) == true && checkIfNewPose(opData.path, curPose, ::mode) == true)
			{
				foundNextPose = true;
				break;
			}
		}

		if (foundNextPose == true)
		{
			// Saving path taken
			opData.path.push_back(curPose);

			// Doing a pass
			singlePass(env, curPose, false, true);
			opData.success = (env.graspID != -1);

			// Updating variables used for visualization
			if (opData.success == true)
			{
				opData.graspQuality = env.graspData.quality;
				env.updateGripper(env.graspID, 0);
				opData.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
				opData.env += *env.ptrPtCldUnexp;
			}
		}
		else
		{
			break;
		}
	}
	return (opData.success);
}

void updateTemp(environment &env, RouteData &data, std::string &name)
{
	data.success = (env.graspID != -1);
	data.obj = *env.ptrPtCldObject;
	data.unexp = *env.ptrPtCldUnexp;
	data.childID = env.saveConfiguration(name);
	if (data.success == true)
	{
		data.goodInitialGrasp = true;
		data.graspQuality = env.graspData.quality;
		env.updateGripper(env.graspID, 0);
		data.env = *env.ptrPtCldEnv + *env.ptrPtCldGripper;
	}
	else
	{
		data.goodInitialGrasp = false;
		data.env = *env.ptrPtCldEnv;
	}
	data.env += *env.ptrPtCldUnexp;
}

// Generating the home poses from which we need to search
void genHomePoses(environment &env, std::vector<double> &stPose, std::string &csvName, std::vector<RouteData> &opHomePoses)
{
	processTimer t = processTimer((char *)"genHomePoses", 2);
	std::vector<std::vector<std::string>> data = readCSV(csvName);
	std::vector<nodeData> tree = convertToTree(data);
	std::vector<double> nextPose;

	std::string name;
	int dir = 0;
	RouteData temp;
	temp.reset();

	if (tree[0].parentID != -1)
		return;

	for (int i = 0; i < tree.size(); i++)
	{
		std::cout << "." << std::flush;
		if (tree[i].checked != 0)
			continue;
		temp.type = tree[i].dirs.length() + 1;
		temp.path = {stPose};
		if (i == 0)
		{
			singlePass(env, stPose, true, true);
			tree[0].checked = 1;
		}
		else
		{
			env.rollbackConfiguration(tree[tree[i].parentID].configID);
			dir = tree[i].dirs.back() - '0';
			nextPose = calcExplorationPose(env.lastCameraPoseViewsphere, dir, 1);
			singlePass(env, nextPose, false, true);
			tree[i].checked = 1;
		}

		// Updating the path
		for (int j = 0; j < tree[i].dirs.length(); j++)
		{
			dir = tree[i].dirs[j] - '0';
			temp.path.push_back(calcExplorationPose(temp.path.back(), dir, 1));
		}
		name = "H" + std::to_string(tree[i].nodeID);
		updateTemp(env, temp, name);
		// No need to add the children as home poses if current pose is successful
		if (temp.success == true)
		{
			printf("found grasp at %d\n", i);
			tree[i].checked = 2;
			env.configurations.pop_back();
			for (int j = i; j < tree.size(); j++)
			{
				if (tree[j].checked != 0)
				{
					for (int k = 0; k < tree[j].childIDs.size(); k++)
					{
						tree[tree[j].childIDs[k]].checked = 2;
					}
				}
			}
		}
		else
		{
			tree[i].configID = temp.childID;
			temp.nodeInfo = tree[i];
			opHomePoses.push_back(temp);
		}
	}
	// for(int i = 0; i < tree.size(); i++){
	//   tree[i].print();
	// }
}

// Function to save optimal paths for the given position.
int findOptimalPaths(environment &av, int object, std::string homePosesTree, std::string dir, std::string saveLocation, int currentPoseCode, int currentYaw, Dataset &d)
{
	processTimer t = processTimer((char *)"FindOptimalPaths", 1);
	std::fstream fout;

	fout.open(dir + saveLocation, std::ios::out | std::ios::app);

	std::cout << "Spawning object" << std::endl;
	av.spawnObject(object, 0, 0);

	std::vector<double> startPose = {av.viewsphereRad, 180 * (M_PI / 180.0), 45 * (M_PI / 180.0)};

	bool graspFound = false;
	RouteData dataFinalTemp, cData;
	std::vector<RouteData> dataOneStep, dataTemp;

	int nHomeConfigs;

	int nSaved = 0;
	int nZeroHomePoses = 0;
	printf("BFS  *** Pose #%d/%d with Yaw %d ***\n", currentPoseCode + 1, int(av.objectDict[object].nPoses), currentYaw);
	// Set up the start pose
	av.moveObject(object, currentPoseCode, currentYaw * (M_PI / 180.0));
	RouteData start;
	start.reset();
	singlePass(av, startPose, true, true);
	std::string name = "H_" + std::to_string(currentPoseCode) + "_" + std::to_string(currentYaw);
	start.childID = av.saveConfiguration(name);
	start.path = {startPose};
	updateTemp(av, start, name);
	// Check the start pose:
	int iteration = 0;
	printf("BFS      ***Testing step %d\n", iteration);
	graspFound = start.success;
	// If no grasp was found, begin the BFS.
	if (not graspFound)
	{
		iteration++;
		printf("BFS      ***Testing step %d\n", iteration);
		graspFound = exploreOneStepAllDir(av, start, dataOneStep);
		iteration++;
	}
	else
	{
		dataOneStep.push_back(start);
	}
	while ((not graspFound) and (iteration < pow(8, ::maxOptStep)))
	{
		printf("BFS      ***Testing step %d\n", iteration);
		cData = dataOneStep[0];
		dataTemp.clear();
		dataOneStep.erase(dataOneStep.begin());
		graspFound = exploreOneStepAllDir(av, cData, dataTemp);
		for (int i = 0; i < dataTemp.size(); ++i)
		{
			dataOneStep.push_back(dataTemp[i]);
		}
		iteration++;
	}

	printf("BFS    %ld\n", dataOneStep.size());
	for (int i = 0; i < dataOneStep.size(); ++i)
	{
		printf("Item %d, passed=%d\n", i, dataOneStep[i].success);
	}
	// Iterate over the last 8 poses to find the successful one
	for (int i = 1; i < 9; ++i)
	{
		dataFinalTemp.reset();
		dataFinalTemp = dataOneStep[dataOneStep.size() - i];
		dataFinalTemp.nodeInfo = start.nodeInfo;
		dataFinalTemp.objType = av.objectDict[object].description;
		dataFinalTemp.objPose = {av.objectDict[object].poses[currentPoseCode][1],
								 av.objectDict[object].poses[currentPoseCode][2],
								 currentYaw * (M_PI / 180.0)};
		dataFinalTemp.type = start.type;
		// Don't count the start position for path length
		dataFinalTemp.nSteps = dataFinalTemp.path.size() - 1;
		dataFinalTemp.obj = start.obj;
		dataFinalTemp.unexp = start.unexp;
		if(8 == i && !dataFinalTemp.success)
		{
			printf("No direction succeeded. Saving failed path\n");
			dataFinalTemp.nSteps = maxOptStep + 2;
			saveData(dataFinalTemp, fout, dir);
		}
		if (dataFinalTemp.success)
		{
			printf("Trying direction %d.... success\n", i);
			dataFinalTemp.filename = getCurTime() + "_" + std::to_string(nSaved);
			av.reset();
			av.deleteObject(object);
			av.moveObject(object, currentPoseCode, currentYaw * (M_PI / 180.0));
			for (int j = 0; j < dataFinalTemp.path.size(); j++)
			{
				std::vector<double> cPose = {dataFinalTemp.path[j][2], dataFinalTemp.path[j][1], dataFinalTemp.path[j][0]};
				singlePass(av, cPose, (0 == j), false);
				ptCldColor::Ptr temp{new ptCldColor};
				*temp = *av.ptrPtCldObject;
				std::string s1 = std::to_string(j) + "_" + dataFinalTemp.filename;
				savePointCloud(temp, dir, s1, 1);
				*temp = *av.ptrPtCldUnexp;
				savePointCloud(temp, dir, s1, 2);
				*temp = *av.ptrPtCldObject + *av.ptrPtCldUnexp;
				savePointCloud(temp, dir, s1, 3);
				Entry e(j, dataFinalTemp.path[j][2], dataFinalTemp.path[j][0], dataFinalTemp.path[j][1], s1 + "_object.pcd", s1 + "_unexp.pcd", s1 + "_result.pcd", std::to_string(object));
				d.addEntry(e);
			}
			saveData(dataFinalTemp, fout, dir);
			break;
		}
		else
		{
			printf("Trying direction %d.... failed\n", i);
		}
	}
	av.deleteObject(object);
	av.reset();
	fout.close();
	t.getTime();
	return (nSaved);
}

// Function the generate the data for training
int generateData(environment &av, int object, std::string homePosesTree, std::string dir, std::string saveLocation, int nDataPoints, Dataset &d)
{
	std::fstream fout;
	fout.open(dir + saveLocation, std::ios::out | std::ios::app);

	av.spawnObject(object, 0, 0);

	std::vector<double> startPose = {av.viewsphereRad, 180 * (M_PI / 180.0), 45 * (M_PI / 180.0)};

	bool graspFound = false;
	RouteData dataFinalTemp;
	std::vector<RouteData> dataOneStep, dataHomePoses, dataFinal;

	int nHomeConfigs;

	int nSaved = 0;
	int currentPoseCode;
	int currentYaw;
	int nZeroHomePoses = 0;

	while (nSaved < nDataPoints)
	{

		currentPoseCode = rand() % (av.objectDict[object].nPoses);
		currentYaw = rand() % (int(av.objectDict[object].poses[currentPoseCode][4]) - int(av.objectDict[object].poses[currentPoseCode][3]) + 1) + int(av.objectDict[object].poses[currentPoseCode][3]);

		printf("  *** Pose #%d/%d with Yaw %d ***\n", currentPoseCode + 1, int(av.objectDict[object].nPoses), currentYaw);
		av.moveObject(object, currentPoseCode, currentYaw * (M_PI / 180.0));

		// Get the homes poses to collect data for based on the step count
		dataHomePoses.clear();
		std::cout << "   * Generating Home Configs" << std::endl;
		std::cout << std::flush;
		genHomePoses(av, startPose, homePosesTree, dataHomePoses);
		dataFinal.resize(dataHomePoses.size());
		nHomeConfigs = av.configurations.size();
		if (nHomeConfigs == 0)
			nZeroHomePoses++;

		std::cout << " " << dataHomePoses.size() << " configs." << std::endl;

		for (int poses = dataHomePoses.size() - 1; poses >= 0; poses--)
		{
			dataFinal[poses].reset();

			printf("      Home #%d/%d : ", poses + 1, int(dataHomePoses.size()));
			std::cout << std::flush;
			graspFound = dataHomePoses[poses].success;
			// If home pose has a successful grasp
			if (graspFound == true)
				std::cout << "How ???" << std::endl;

			// If we do not find a grasp at home, then check for one step in each direction
			printf("8 Dir Search");
			std::cout << std::flush;
			dataOneStep.clear();
			graspFound = exploreOneStepAllDir(av, dataHomePoses[poses], dataOneStep);
			if (graspFound == true)
			{
				dataFinal[poses] = dataOneStep.back();
			}
			else
			{
				// If we dont find a grasp after one step, then check using random searches
				int nSteps = ::maxRndSteps;
				if (dataHomePoses[poses].nodeInfo.possibleSolChildIndex != -1)
				{
					nSteps = dataHomePoses[poses].nodeInfo.allowedSteps - 1;
				}

				for (int rdmSearchCtr = 1; rdmSearchCtr <= ::nRdmSearches && nSteps > 0; rdmSearchCtr++)
				{
					printf(" Random Search #%d.", rdmSearchCtr);
					std::cout << std::flush;

					for (int i = 0; i < dataOneStep.size() && nSteps > 0; i++)
					{
						dataFinalTemp.reset();
						std::cout << nSteps << std::flush;
						bool graspFoundTemp = exploreRdmStep(av, dataOneStep[i], nSteps, dataFinalTemp);
						if (graspFoundTemp == true)
						{
							graspFound = true;
							// If its the first time entering this, then store this in expRes
							if (dataFinal[poses].path.size() == 0)
							{
								dataFinal[poses] = dataFinalTemp;
							}
							// If not, check if current result is better than the previous result and update expRes
							else if ((dataFinalTemp.path.size() < dataFinal[poses].path.size()) ||
									 (dataFinalTemp.path.size() == dataFinal[poses].path.size() &&
									  dataFinalTemp.graspQuality > dataFinal[poses].graspQuality))
							{
								dataFinal[poses] = dataFinalTemp;
							}
							// Update nSteps to speed up the process
							nSteps = std::min(nSteps, int(dataFinal[poses].path.size() - dataOneStep[i].path.size() - 1));
						}
					}
				}
			}

			// If no grasp found, select the child which has a grasp
			if (graspFound == false)
			{
				int index = dataHomePoses[poses].nodeInfo.possibleSolChildIndex;
				if (index != -1)
				{
					graspFound = true;
					dataFinal[poses].path = dataFinal[index].path;
					dataFinal[poses].success = dataFinal[index].success;
					dataFinal[poses].graspQuality = dataFinal[index].graspQuality;
					dataFinal[poses].env = dataFinal[index].env;
					dataFinal[poses].direction = dataFinal[index].nodeInfo.dirs[dataHomePoses[poses].type - 1] - '0';
				}
			}

			// If still no grasp found, good luck
			if (graspFound == false)
			{
				printf(" Not OK.\n");
				dataFinal[poses] = dataHomePoses[poses]; // Storing dummy data
			}

			dataFinal[poses].nodeInfo = dataHomePoses[poses].nodeInfo;
			dataFinal[poses].objType = av.objectDict[object].description;
			dataFinal[poses].objPose = {av.objectDict[object].poses[currentPoseCode][1],
										av.objectDict[object].poses[currentPoseCode][2],
										currentYaw * (M_PI / 180.0)};
			dataFinal[poses].type = dataHomePoses[poses].type;
			dataFinal[poses].nSteps = dataFinal[poses].path.size() - dataFinal[poses].type;
			dataFinal[poses].obj = dataHomePoses[poses].obj;
			dataFinal[poses].unexp = dataHomePoses[poses].unexp;

			if (graspFound == true)
				printf(" OK. %d Steps, Dir : %d.\n", dataFinal[poses].nSteps, dataFinal[poses].direction);

			// Setting max steps for the parent based on the child results
			for (int index = 0; index < dataHomePoses.size(); index++)
			{
				if (dataHomePoses[poses].nodeInfo.parentID == dataHomePoses[index].nodeInfo.nodeID)
				{
					int steps = dataFinal[poses].path.size() - dataHomePoses[index].path.size();
					if (dataHomePoses[index].nodeInfo.allowedSteps > steps)
					{
						dataHomePoses[index].nodeInfo.allowedSteps = steps - 1;
						dataHomePoses[index].nodeInfo.possibleSolChildIndex = poses;
					}
					break;
				}
			}

			// Deleting the new configs saved
			av.configurations.resize(nHomeConfigs);
		}

		for (int poses = 0; poses < dataHomePoses.size(); poses++)
		{
			std::string tempStr = std::to_string(object);
			if (dataFinal[poses].nSteps > 0 && dataFinal[poses].nSteps <= ::maxRndSteps + 1)
			{
				d.addRun(nSaved, tempStr);
				// dataFinal[poses].filename  = getCurTime() + "_" + std::to_string(nSaved);
				dataFinal[poses].filename = std::to_string(nSaved);

				av.reset();
				av.deleteObject(object);
				av.moveObject(object, currentPoseCode, currentYaw * (M_PI / 180.0));
				bool false_run = false;

				for (int j = 0; j < dataFinal[poses].path.size(); j++)
				{
					std::vector<double> cPose = {dataFinal[poses].path[j][2], dataFinal[poses].path[j][1], dataFinal[poses].path[j][0]};

					singlePass(av, cPose, (0 == j), false);

					std::string s1 = std::to_string(j) + "_" + dataFinal[poses].filename;

					// check if cloud empty
					if (av.ptrPtCldObject->empty() || av.ptrPtCldUnexp->empty())
					{
						false_run = true;
						break;
					}

					// save object cloud
					savePointCloud(av.ptrPtCldObject, dir, s1, 1);

					// save unexp cloud
					savePointCloud(av.ptrPtCldUnexp, dir, s1, 2);

					// save result cloud
					ptCldColor::Ptr temp{new ptCldColor};
					*temp = *av.ptrPtCldObject + *av.ptrPtCldUnexp;
					savePointCloud(temp, dir, s1, 3);

					// add entry to the run
					Entry e(j, dataFinal[poses].path[j][2], dataFinal[poses].path[j][0], dataFinal[poses].path[j][1], s1 + "_object.pcd", s1 + "_unexp.pcd", s1 + "_result.pcd", std::to_string(object));

					d.addEntry(e);
				}

				if (!false_run)
				{
					std::cout << "Saving #" << nSaved << "/" << nDataPoints << " run #" << poses << std::endl;
					// saveData(dataFinal[poses], fout, dir);
					d.stopRun(nSaved);
					d.saveData();
					Json::Value handle;
					d.parseJsonFile(d.getFileName(), handle);
					d.updateFileRoot(handle);
					nSaved++;
				}
				else
				{
					std::cout << "not saving run #" << poses << std::endl;
				}
			}
			else
			{
				printf("This shouldn't be called %d\n", poses);
			}
			// dataFinal[poses].filename = getCurTime() + "_" + std::to_string(nSaved);
			// if (dataFinal[poses].nSteps > 0 && dataFinal[poses].nSteps <= ::maxRndSteps + 1)
			// {
			// 	nSaved++;
			// 	saveData(dataFinal[poses], fout, dir);
			// }
		}

		// std::cout << std::endl;
		// for(int poses = dataHomePoses.size()-1; poses >= 0 ; poses--){
		// 	dataHomePoses[poses].nodeInfo.print();
		// }
		av.reset();
	}
	av.deleteObject(object);
	fout.close();
	return (nSaved);
}

void help()
{
	std::cout << "******* Data Collector Node Help *******" << std::endl;
	std::cout << "Arguments : [mode]" << std::endl;
	std::cout << "mode: 0 (default)=data collection, 1=BFS" << std::endl;
	std::cout << "*******" << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Data_Collector_Node");
	srand(time(NULL)); // Randoming the seed used for rand generation
	processTimer t = processTimer((char *)"Data collector", 0);

	ros::NodeHandle nh;

	environment av(&nh);
	sleep(1);

	av.setPtCldNoise(0.0);

	bool relativePath;
	nh.getParam("/active_vision/dataCollector/relative_path", relativePath);

	std::string temp;
	nh.getParam("/active_vision/dataCollector/directory", temp);
	std::string dir;
	if (relativePath == true)
	{
		dir = ros::package::getPath("active_vision") + temp;
	}
	else
	{
		dir = temp;
	}

	std::string time = getCurTime();
	std::string csvName;
	csvName = "BFS:"+time + "_dataRec.csv";
	// nh.getParam("/active_vision/dataCollector/csvName", temp);
	// std::string csvName;
	// if (temp == "default.csv")
	// 	csvName = time + "_dataRec.csv";
	// else
	// 	csvName = temp;

	if (csvName.substr(csvName.size() - 4) != ".csv")
	{
		std::cout << "ERROR. Incorrect file name." << std::endl;
		return (-1);
	}

	int objID;
	nh.getParam("/active_vision/dataCollector/objID", objID);
	if (av.objectDict.count(objID) == 0)
	{
		std::cout << "ERROR. Incorrect Object ID." << std::endl;
		return (-1);
	}

	int nDataPoints;
	nh.getParam("/active_vision/dataCollector/nData", nDataPoints);
    // nDataPoints = 1000;
	
	nh.getParam("/active_vision/dataCollector/homePosesCSV", temp);
	std::string homePosesTreeCSV = ros::package::getPath("active_vision") + "/misc/" + temp;

	nh.getParam("/active_vision/cameraMode", ::mode);
	if (::mode < 1 && ::mode > 2)
		::mode = 2;

	nh.getParam("/active_vision/dataCollector/nRdmSearch", ::nRdmSearches);
	nh.getParam("/active_vision/dataCollector/maxRdmSteps", ::maxRndSteps);

	std::cout << "Start Time : " << getCurTime() << std::endl;
	int ctrData = 0;

	if (argc <= 1 or std::atoi(argv[1]) == 0)
	{
		Dataset d;
		d.setDir(dir);
		d.createNewFile();
		printf("***** Object #%d *****\n", objID);
		ctrData = generateData(av, objID, homePosesTreeCSV, dir, csvName, nDataPoints, d);
	}
	else if (std::atoi(argv[1]) == 1)
	{
		Dataset d;
		d.setDir(dir);
		d.createNewFile();

		// Reading the yawValues csv file
		int objPoseCode, objYaw;
		std::string yawAnglesCSVDir;
		nh.getParam("/active_vision/policyTester/yawAnglesCSVDir", yawAnglesCSVDir);
		yawAnglesCSVDir = ros::package::getPath("active_vision") + yawAnglesCSVDir;
		std::string yawAnglesCSV;
		nh.getParam("/active_vision/policyTester/yawAnglesCSV", yawAnglesCSV);
		std::vector<std::vector<std::string>> yawAngle = readCSV(yawAnglesCSVDir + yawAnglesCSV);
		nh.getParam("/active_vision/dataCollector/objID", objID);
		if (av.objectDict.count(objID) == 0)
		{
			std::cout << "ERROR. Incorrect Object ID." << std::endl;
			return (-1);
		}
		printf("BFS***** Object #%d *****\n", objID);

		// Converting it to a dictionary format (First column is the key)
		std::map<std::string, std::vector<int>> yawAngleDict;
		for (int row = 0; row < yawAngle.size(); row++)
		{
			yawAngleDict.insert({yawAngle[row][0], {}});
			for (int col = 1; col < yawAngle[row].size(); col++)
			{
				yawAngleDict[yawAngle[row][0]].push_back(std::stoi(yawAngle[row][col]));
			}
		}

		int uniformYawStepSize;
		nh.getParam("/active_vision/policyTester/uniformYawStepSize", uniformYawStepSize);
		std::string tempStr = std::to_string(objID);

		for (int objPoseCode = 0; objPoseCode < av.objectDict[objID].nPoses; objPoseCode += 1)
		{
			int tempNDataPoints = nDataPoints;

			// Checking for the uniform steps
			for (int objYaw = av.objectDict[objID].poses[objPoseCode][3]; objYaw < av.objectDict[objID].poses[objPoseCode][4]; objYaw += uniformYawStepSize)
			{

				d.addRun(ctrData, tempStr);
				findOptimalPaths(av, objID, homePosesTreeCSV, dir, csvName, objPoseCode, objYaw, d);
				d.stopRun(ctrData);

				// saving file and reloading it
				d.saveData();
				Json::Value handle;
				d.parseJsonFile(d.getFileName(), handle);
				d.updateFileRoot(handle);

				tempNDataPoints--;
				ctrData++;
			}

			std::string key = std::to_string(int(av.objectDict[objID].poses[objPoseCode][3])) + "-" +
							  std::to_string(int(av.objectDict[objID].poses[objPoseCode][4]));

			if (yawAngleDict.count(key) == 0)
				continue;

			// Checking for the random steps
			for (int ctr = 0; ctr < yawAngleDict[key].size() && tempNDataPoints > 0; ctr++)
			{
				d.addRun(ctrData, tempStr);
				findOptimalPaths(av, objID, homePosesTreeCSV, dir, csvName, objPoseCode, yawAngleDict[key][ctr], d);
				d.stopRun(ctrData);

				// saving file and reloading it
				d.saveData();
				Json::Value handle;
				d.parseJsonFile(d.getFileName(), handle);
				d.updateFileRoot(handle);

				tempNDataPoints--;
				ctrData++;
			}
		}

		d.saveData();
	}
	else
	{
		printf("Error: unknown mode %d\n", mode);
	}

	std::cout << "End Time : " << getCurTime() << std::endl;
	double elapsed = t.getTime() / 1000.0;
	printf("Time Taken (hrs) : %1.2f\n", elapsed / 60 / 60);
	printf("Data Collected : %d\n", ctrData);
	printf("Time per data collected (sec) : %1.2f\n", elapsed / ctrData);
	printf("Data saved to : %s\n", csvName.c_str());
}
