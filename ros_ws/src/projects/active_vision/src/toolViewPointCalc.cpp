#include <active_vision/toolViewPointCalc.h>

bool checkValidPose(std::vector<double> pose){
	return (pose[2] <= 85*M_PI/180);
}

// Dont check for this condition. It can go back to the same pose
bool checkIfNewPose(std::vector<std::vector<double>> &oldPoses, std::vector<double> &pose, int type){
  return true;
	// printf("******\n");
  	for(int i = 0; i < oldPoses.size(); i++){
		// for(int j=0; j<pose.size();j++){
		// 	std::cout << "(" << oldPoses[i][j] << " " << pose[j] << ")";
		// }
		// printf("\n");
		if(type == 1){
			if(oldPoses[i] == pose) return false;
		}else{
			// printf("dist=%.2f\n", disBtwSpherical(oldPoses[i],pose));
			if(euclideanDistanceSpherical(oldPoses[i],pose) <= 0.25*(pose[0])*(5*M_PI/180)) return false;
		}
  }
  return true;
}

std::vector<double> calcExplorationPoseA(std::vector<double> &startPose, int dir, double step){
	double azimuthalOffset, polarOffset;
	switch(dir){
		case 1: azimuthalOffset = 0; 		 polarOffset = -step; break;	//N
		case 2: azimuthalOffset = step;  polarOffset = -step; break;	//NE
		case 3: azimuthalOffset = step;  polarOffset = 0; 		break;	//E
		case 4: azimuthalOffset = step;  polarOffset = step;  break;	//SE
		case 5: azimuthalOffset = 0; 		 polarOffset = step;  break;	//S
		case 6: azimuthalOffset = -step; polarOffset = step;  break;	//SW
		case 7: azimuthalOffset = -step; polarOffset = 0; 		break;	//W
		case 8: azimuthalOffset = -step; polarOffset = -step; break;	//NW
		default: printf("ERROR in calcExplorationPose\n");
	}

	std::vector<double> potentialPose = {startPose[0], startPose[1]+azimuthalOffset, startPose[2]+polarOffset};

	// Addressing the NW & NE scenario when polar angle goes from *ve to -ve
	if(potentialPose[2] < 0 && startPose[2] > 0) potentialPose[1] = startPose[1]-azimuthalOffset;

	// Polar angle should be +ve
	if(potentialPose[2] < 0){
		potentialPose[2] = -1*potentialPose[2];
		potentialPose[1] = potentialPose[1] + M_PI;
	}

	// Azhimuthal angle 0 to 360 degree
	potentialPose[1] = fmod(potentialPose[1],2*M_PI);
	if(potentialPose[1] < 0) potentialPose[1] += 2*M_PI;

	// std::cout << dir << " " << startPose[1]*180/M_PI << "," << startPose[2]*180/M_PI << "->" <<
	//                            potentialPose[1]*180/M_PI << "," << potentialPose[2]*180/M_PI << std::endl;

	return(potentialPose);
}

std::vector<double> calcExplorationPoseB(std::vector<double> &startPose, int dir, double step){

  Eigen::Vector3f xAxis,yAxis,zAxis,rotAxis,tempVec;
  Eigen::Vector3f xyPlane(0,0,1);
  Eigen::Matrix3f matA; matA << 1,0,0,0,1,0,0,0,1;
  Eigen::Matrix3f matB, matC, tempMat;
  // tf::Matrix3x3 rotMat;

  pcl::PointXYZ centre(0,0,0);
  pcl::PointXYZ stPoint = sphericalToCartesian(startPose);
  pcl::PointXYZ endPoint;

  zAxis = stPoint.getVector3fMap(); zAxis.normalize();
  xAxis = zAxis.cross(xyPlane);
  if(xAxis.norm() == 0) xAxis << sin(startPose[1]),cos(startPose[1]+M_PI),0;
  xAxis.normalize();
  yAxis = zAxis.cross(xAxis);

	std::vector<double> ratio={0,0};
	switch(dir){
		case 1: ratio[0]=+1; ratio[1]=0;   break;
		case 2: ratio[0]=+1; ratio[1]=-1;  break;
		case 3: ratio[0]=0;  ratio[1]=-1;  break;
		case 4: ratio[0]=-1; ratio[1]=-1;  break;
		case 5: ratio[0]=-1; ratio[1]=0;   break;
		case 6: ratio[0]=-1; ratio[1]=+1;  break;
		case 7: ratio[0]=0;  ratio[1]=+1;  break;
		case 8: ratio[0]=+1; ratio[1]=+1;  break;
	}

  rotAxis = ratio[0]*xAxis + ratio[1]*yAxis; rotAxis.normalize();
  matB << 0,-rotAxis[2],rotAxis[1],rotAxis[2],0,-rotAxis[0],-rotAxis[1],rotAxis[0],0;
  matC = rotAxis*rotAxis.transpose();

  tempMat = cos(step)*matA + sin(step)*matB + (1-cos(step))*matC;
  tempVec = tempMat*stPoint.getVector3fMap();
  endPoint.x = tempVec[0]; endPoint.y = tempVec[1]; endPoint.z = tempVec[2];

  std::vector<double> end = cartesianToSpherical(endPoint);

  // Polar angle 0 to 90 degree
  if(end[2] < 0){
    end[2] = -1*end[2];
    end[1] = end[1] + M_PI;
  }

  // Azhimuthal angle 0 to 360 degree
  end[1] = fmod(end[1],2*M_PI);
  if(end[1] < 0) end[1] += 2*M_PI;

	return end;
}

// std::vector<double> calcExplorationPoseC(std::vector<double> &startPose, int dir, double step){
// 	//Remap dir so that 0 points due South
// 	int offsetDir = (dir+270) % 360;
// 	double azimuthalOffset, polarOffset;
//   	double radians = offsetDir*M_PI/180.0;
//   	azimuthalOffset = step*cos(radians);
//   	polarOffset = -step*sin(radians);
	
// 	std::vector<double> potentialPose = {startPose[0], startPose[1]+azimuthalOffset, startPose[2]+polarOffset};

// 	// Addressing the NW & NE scenario when polar angle goes from *ve to -ve
// 	if(potentialPose[2] < 0 && startPose[2] > 0) potentialPose[1] = startPose[1]-azimuthalOffset;

// 	// Polar angle should be +ve
// 	if(potentialPose[2] < 0){
// 		potentialPose[2] = -1*potentialPose[2];
// 		potentialPose[1] = potentialPose[1] + M_PI;
// 	}

// 	// Azhimuthal angle 0 to 360 degree
// 	potentialPose[1] = fmod(potentialPose[1],2*M_PI);
// 	if(potentialPose[1] < 0) potentialPose[1] += 2*M_PI;

// 	std::cout << dir << " " << startPose[1]*180/M_PI << "," << startPose[2]*180/M_PI << "->" <<
// 	                            potentialPose[1]*180/M_PI << "," << potentialPose[2]*180/M_PI << 
// 								" d=" << 
// 								sqrt(2-2*(sin(startPose[2])*sin(potentialPose[2])*cos(startPose[1]-potentialPose[1])+cos(startPose[2])*cos(potentialPose[2]))) 
// 								<< std::endl;

// 	return(potentialPose);
// }

std::vector<double> calcExplorationPoseC(std::vector<double> &startPose, int dir, double step){

  
  Eigen::Vector3f xAxis,yAxis,zAxis,rotAxis,tempVec;
  Eigen::Vector3f xyPlane(0,0,1);
  Eigen::Matrix3f matA; matA << 1,0,0,0,1,0,0,0,1;
  Eigen::Matrix3f matB, matC, tempMat;
  // tf::Matrix3x3 rotMat;
  int offsetDir = (360-dir) % 360;
  double azimuthalOffset, polarOffset;
  double radians = offsetDir*M_PI/180.0;
  azimuthalOffset = cos(radians);
  polarOffset = -sin(radians);

  // std::cout << offsetDir << " azimuthal offset = " << azimuthalOffset*180/M_PI << " polar offset = " << polarOffset*180/M_PI << std::endl;
  

  pcl::PointXYZ centre(0,0,0);
  pcl::PointXYZ stPoint = sphericalToCartesian(startPose);
  pcl::PointXYZ endPoint;

  zAxis = stPoint.getVector3fMap(); zAxis.normalize();
  xAxis = zAxis.cross(xyPlane);
  if(xAxis.norm() == 0) xAxis << sin(startPose[1]),cos(startPose[1]+M_PI),0;
  xAxis.normalize();
  yAxis = zAxis.cross(xAxis);
  // Eigen::Vector3f rot(cos(step),0,sin(step));
  // Eigen::Vector3f stR(1, 0, 0);
  // Eigen::Matrix3f rotateAboutX;
  // rotateAboutX = Eigen::AngleAxisf(radians,Eigen::Vector3f(1,0,0));
  // rot = rotateAboutX * rot;
  // std::cout << rot[0] << " " << rot[1] << " " << rot[2] << " " << std::endl;
  // Eigen::Matrix3f R;
  // R = Eigen::Quaternionf().setFromTwoVectors(Eigen::Vector3f(1,0,0),stPoint.getVector3fMap());
  // rot = R * rot;
  // stR = R * stR;
  // std::cout << rot[0] << " " << rot[1] << " " << rot[2] << " " << std::endl;
  // std::cout << stR[0] << " " << stR[1] << " " << stR[2] << " " << std::endl;



  std::vector<double> ratio={azimuthalOffset, polarOffset};

  rotAxis = ratio[0]*xAxis + ratio[1]*yAxis; rotAxis.normalize();
  matB << 0,-rotAxis[2],rotAxis[1],rotAxis[2],0,-rotAxis[0],-rotAxis[1],rotAxis[0],0;
  matC = rotAxis*rotAxis.transpose();

  tempMat = cos(step)*matA + sin(step)*matB + (1-cos(step))*matC;
  tempVec = tempMat*stPoint.getVector3fMap();
  endPoint.x = tempVec[0]; endPoint.y = tempVec[1]; endPoint.z = tempVec[2];
  // endPoint.x = rot[0]; endPoint.y = rot[1]; endPoint.z = rot[2];

  std::vector<double> end = cartesianToSpherical(endPoint);

  // Polar angle 0 to 90 degree
  if(end[2] < 0){
    end[2] = -1*end[2];
    end[1] = end[1] + M_PI;
  }

  // Azhimuthal angle 0 to 360 degree
  end[1] = fmod(end[1]+2*M_PI,2*M_PI);

  // std::cout << dir << " " << startPose[1]*180/M_PI << "," << startPose[2]*180/M_PI << "->" <<
	//                             end[1]*180/M_PI << "," << end[2]*180/M_PI << 
	// 							" d=" << 
	// 							sqrt(2-2*(sin(startPose[2])*sin(end[2])*cos(startPose[1]-end[1])+cos(startPose[2])*cos(end[2]))) 
	// 							<< std::endl;

	return end;
}

std::vector<double> calcExplorationPose(std::vector<double> &startPose, int dir, int mode, double step){
  switch(mode){
    case 1: return calcExplorationPoseA(startPose,dir); break;
    case 2: return calcExplorationPoseB(startPose,dir); break;
    case 3: return calcExplorationPoseC(startPose,dir); break;
    default: return calcExplorationPoseA(startPose,dir);
  }
}
