#include <active_vision/environment.h>
#include <active_vision/toolVisualization.h>

#define PRECISION 100 //Controls the decimal to round values to- 100=2 places, 10=1, etc..

//rosrun active_vision visSinglePath /home/rbe07/mer_lab/ros_ws/src/projects/active_vision/dataCollected/Working_Comparisons/072/ RANDOM:2023_02_27_154348_dataRec.csv IL:2023_02_26_191752_dataRec.csv BFS:2023_02_26_201013_dataRec.csv 11
//rosrun active_vision visSinglePath /home/rbe07/mer_lab/ros_ws/src/projects/active_vision/dataCollected/Working_Comparisons/009/ RANDOM:2023_02_27_115616_dataRec.csv IL:2023_02_27_195438_dataRec.csv BFS:2023_02_26_103215_dataRec.csv 11

void passThroughFilterZ(ptCldColor::Ptr ptrPtCld,float minZ, float maxZ){

  ptCldColor::ConstPtr cPtrPtCld{ptrPtCld};
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cPtrPtCld);
  pass.setFilterFieldName("z"); pass.setFilterLimits(minZ,maxZ); pass.filter(*ptrPtCld);
}

void help(){
  std::cout << "******* Recorded Data Visualizer Help *******" << std::endl;
  std::cout << "Arguments : [Directory] [CSV filename]* [Start ID]" << std::endl;
  std::cout << "Directory : Directory where csv and pcd files are there (./DataRecAV/)" << std::endl;
  std::cout << "CSV filename : CSV file name (Data.csv)" << std::endl;
  std::cout << "Start ID : Viewpoint # [0-99]" << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
  if(argc < 4){
    std::cout << "ERROR. Incorrect number of arguments." << std::endl;
    help(); return(-1);
  }
  

  std::string directory(argv[1]);
  int targetID = std::atoi(argv[argc-1]); 
  
  int yawColID = 3;
  int typeColID = 10;
  int pathColID = 11;
  int dirColID = 12;
  int nStepColID = 13;
  int stepColID = 14;
  int nSteps,type;
  double yaw = 0;

  std::vector<double> pose={0,0,0};
  pcl::PointXYZ sphereCentre;
  pcl::PointXYZ table,a1,a2;
  table.x = 0.55; table.y = 0; table.z = 0;

  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};
  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
  setupViewer(viewer, 1, vp);
  viewer->setCameraPosition(-2,0,7,table.x,table.y,table.z,0,0,1);
  viewer->addCube(table.x-0.25,table.x+0.25,
                table.y-0.50,table.y+0.50,
                table.z-0.01,table.z,0.9,0.8,0.4,"Table",vp[0]);

  //Load params from the first csv
  std::string csvFile(argv[2]);
  std::vector<std::vector<std::string>> data;
  data = readCSV(directory+csvFile);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadObj(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string target = AV_PATH + "models/ycbAV/"+std::string(data[targetID][0].c_str())+"/generated/";
  io::loadPCDFile<pcl::PointXYZRGB>(target+"obj.pcd", *loadObj);
  double objYaw = 0.047;
  double pitch = std::atof(data[0][1].c_str());
  double roll = 0.0;
  Eigen::Matrix4d translation;
  translation << 
      (cos(objYaw)*cos(pitch)), (cos(objYaw)*sin(pitch)*sin(roll)-sin(objYaw)*cos(roll)), (cos(objYaw)*sin(pitch)*cos(roll)+sin(objYaw)*sin(roll)), 0.55,
      (sin(objYaw)*cos(pitch)), (sin(objYaw)*sin(pitch)*sin(roll)+cos(objYaw)*cos(roll)), (sin(objYaw)*sin(pitch)*cos(roll)-cos(objYaw)*sin(roll)), 0,
      (-sin(pitch)), (cos(pitch)*sin(roll)), (cos(pitch)*cos(roll)), std::atof(data[0][2].c_str()),
      0, 0, 0, 1;
  pcl::transformPointCloud(*loadObj, *loadObj, translation);
  addRGB(viewer, loadObj, "Object", vp[0]);

  keyboardEvent keyPress(viewer,1); keyPress.help();

  int i = 0;

  for (int i = 2; i < argc-1; ++i)
  {
    csvFile = argv[i];
    data = readCSV(directory+csvFile);

    // for(int j = 0; j < data[i].size(); j++){
    //   std::cout << data[i][j] << " ";
    // }
    std::cout << std::endl;
    type = std::atoi(data[targetID][typeColID].c_str());
    nSteps = std::atoi(data[targetID][nStepColID].c_str());
    yaw = std::atof(data[targetID][yawColID].c_str());
    //Generate a color for the data
    float red = static_cast<double>(std::rand())/RAND_MAX;
    float green = static_cast<double>(std::rand())/RAND_MAX;
    float blue = static_cast<double>(std::rand())/RAND_MAX;
    int offset = i-2;
    viewer->addText(csvFile, 10, 20*offset, red, green, blue, csvFile, vp[0]);
    // int green = std::rand();
    // int blue = std::rand();

    for (int j = stepColID; j+3 < data[targetID].size(); j+=3){
      // std::cout << j << " " << data[targetID].size() << std::endl;
      pose[0] = std::atof(data[targetID][j].c_str());
      pose[1] = std::atof(data[targetID][j+1].c_str());
      pose[2] = std::atof(data[targetID][j+2].c_str());
      a1.x = table.x+pose[0]*sin(pose[2])*cos(pose[1]+yaw);
      a1.y = table.y+pose[0]*sin(pose[2])*sin(pose[1]+yaw);
      a1.z = table.z+pose[0]*cos(pose[2]);
      if(stepColID != j){
        a1.x = floor(a1.x*PRECISION+.5)/PRECISION;
        a1.y = floor(a1.y*PRECISION+.5)/PRECISION;
        a1.z = floor(a1.z*PRECISION+.5)/PRECISION;
      }

      pose[0] = std::atof(data[targetID][j+3].c_str());
      pose[1] = std::atof(data[targetID][j+4].c_str());
      pose[2] = std::atof(data[targetID][j+5].c_str());
      a2.x = table.x+pose[0]*sin(pose[2])*cos(pose[1]+yaw);
      a2.y = table.y+pose[0]*sin(pose[2])*sin(pose[1]+yaw);
      a2.z = table.z+pose[0]*cos(pose[2]);
      a2.x = floor(a2.x*PRECISION+.5)/PRECISION;
      a2.y = floor(a2.y*PRECISION+.5)/PRECISION;
      a2.z = floor(a2.z*PRECISION+.5)/PRECISION;
      float fractionDone = ((j-stepColID+3)/3.0)/nSteps;
      
      viewer->addArrow(a2,a1,red*(1-fractionDone),green*(1-fractionDone),blue*(1-fractionDone),false,std::to_string(i)+"_"+std::to_string(j),vp.back());
      // if(j < stepColID+3){
      //   viewer->addArrow(a2,a1,1,1,1,false,std::to_string(i)+"_"+std::to_string(j),vp.back());
      // }else{
      //   viewer->addArrow(a2,a1,1-fractionDone,1,1-fractionDone,false,std::to_string(i)+"_"+std::to_string(j),vp.back());
      // }
    }

    pose[0] = std::atof(data[targetID][stepColID].c_str());
    pose[1] = std::atof(data[targetID][stepColID+1].c_str());
    pose[2] = std::atof(data[targetID][stepColID+2].c_str());
    a1.x = table.x+pose[0]*sin(pose[2])*cos(pose[1]+yaw);
    a1.y = table.y+pose[0]*sin(pose[2])*sin(pose[1]+yaw);
    a1.z = table.z+pose[0]*cos(pose[2]);

    std::cout << argv[i] << " (" << red << "," << green << "," << blue << ")" << std::endl;

    viewer->addSphere(a1,0.01,1,0,0,"Cam_"+std::to_string(i),vp.back());

  }
  std::cout << pose[0] << std::endl;
  addViewsphere(viewer,vp.back(),table,pose[0],true);
  keyPress.called = false;
  while(!viewer->wasStopped() && keyPress.called==false){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }

  viewer->resetStoppedFlag();
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
}
