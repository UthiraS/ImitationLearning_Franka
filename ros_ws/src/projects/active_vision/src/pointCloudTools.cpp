#include <active_vision/pointCloudTools.h>

//Why does cpp not already have a library to do this?
// Checks for approximate float equality.

void addRect(PointCloud<PointXYZRGB>::Ptr cloud, double scaleX, double scaleY, double scaleZ, double division)
{
  double x, y, z;
  x = -scaleX;
  y = -scaleY;
  //We need to offset z into the air
  z = -scaleZ;
  //Double comparisons :'(
  while (x <= scaleX*1.001)
  {
    while (y <= scaleY*1.001)
    {
      while (z <= scaleZ*1.001) 
      {
        if (roughlyEqual(scaleX, abs(x)) || roughlyEqual(scaleY, abs(y)) || roughlyEqual(scaleZ, abs(z)))
        {
          PointXYZRGB cPt;
          cPt.x = x;
          cPt.y = y;
          cPt.z = z;
          cPt.b = 125;
          cloud->push_back(cPt);
        }
        z += division;
      }
      z = -scaleZ;
      y += division;
    }
    y = -scaleY;
    x += division;
  }
}

void addCube(PointCloud<PointXYZRGB>::Ptr cloud, double scale, double division)
{
  addRect(cloud, scale, scale, scale, division);
}

// https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
void addSphere(PointCloud<PointXYZRGB>::Ptr cloud, double scale, int nPoints, bool fullSphere)
{
  double r, theta, phi;
  r = scale;
  int cPoint = 0;
  while(cPoint < nPoints){
    double cI = ((double(cPoint)+.5)/double(nPoints));
    phi = acos(1-(2*cI));
    theta = M_PI * (1 + pow(5.0, .5)) * (double(cPoint)+.5);
    PointXYZRGB cPt;
    cPt.x = cos(theta) * sin(phi) * scale;
    cPt.y = sin(theta) * sin(phi) * scale;
    cPt.z = cos(phi) * scale;

    if(fullSphere || ((phi <= 85*M_PI/180) && (phi >= 0*M_PI/180) ))
      cloud->push_back(cPt);
    cPoint++;
  }
}

void addCircle(PointCloud<PointXYZRGB>::Ptr cloud, double scale, int nPoints)
{
  double r, theta, phi;
  r = scale;
  int cPoint = 0;
  while(cPoint < nPoints){
    double cI = ((double(cPoint)+.5)/double(nPoints));
    phi = 45 * (M_PI / 180.0);
    theta = 2* M_PI * (double(cPoint)/double(nPoints));
    PointXYZRGB cPt;
    cPt.x = cos(theta) * sin(phi) * scale;
    cPt.y = sin(theta) * sin(phi) * scale;
    cPt.z = cos(phi) * scale;
    // cPt.g = 255;
    cloud->push_back(cPt);
    cPoint++;
  }
}

bool findNormals(ptCldColor::Ptr ptrPtCldObject, ptCldNormal::Ptr ptrObjNormal, int knn)
{
  NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  // NormalRefinement<Normal> nr;
  search::Search<PointXYZRGB>::Ptr KdTree{new search::KdTree<PointXYZRGB>};
  try
  {
    ne.setInputCloud(ptrPtCldObject);
    ne.setSearchMethod(KdTree);
    ne.setKSearch(knn);
    // ne.setRadiusSearch(r);
    ne.compute(*ptrObjNormal);
    // nr.setInputCloud(ptrObjNormal);
    // nr.filter(*ptrObjNormal);
  }
  catch(const std::exception& e)
  {
    std::cerr << "GBERROR\n";
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}

//https://pointclouds.org/documentation/classpcl_1_1_normal_refinement.html
ptCldNormal::Ptr refineNormals(ptCldColor::Ptr ptrPtCldObject, ptCldNormal::Ptr ptrObjNormal, int k)
{
  ptCldColor cloud(*ptrPtCldObject);
  ptCldNormal normals(*ptrObjNormal);
  ptCldNormal normals_refined;
  
  // Search parameters
  std::vector<Indices > k_indices;
  std::vector<std::vector<float> > k_sqr_distances;
  
  // Run search
  pcl::search::KdTree<pcl::PointXYZRGB> search;
  search.setInputCloud (cloud.makeShared ());
  search.nearestKSearch (cloud, Indices (), k, k_indices, k_sqr_distances);
  
  // Run refinement using search results
  pcl::NormalRefinement<PointNormal> nr (k_indices, k_sqr_distances);
  std::cout << "# of indices searched " << k_indices.size() << std::endl;
  std::cout << "Incoming point cloud size " << normals.points.size() << std::endl;
  nr.setInputCloud (normals.makeShared ());
  nr.filter (normals_refined);
  return normals_refined.makeShared();
}