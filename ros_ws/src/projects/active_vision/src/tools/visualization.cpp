#include <active_vision_tools/visualization.h>

AVVis::~AVVis()
{
  if (!_initialized)
    return;
  free(_vp);
  viewer->close();
}

void AVVis::setup(string name, int numberOfViewpoints)
{
  if(_initialized){
    AVLOG("This viewer has already been initialized!", 0, 0);
    return;
  }
  // string unique_name = name + "_" + to_string((double) time(0));
  viewer = boost::shared_ptr<visualization::PCLVisualizer>(new visualization::PCLVisualizer(name.c_str()));
  _vp = new vector<int>();
  _vp->resize(numberOfViewpoints);
  setupViewer();
  _initialized = true;
}

void AVVis::stepVis()
{
  if (!_initialized)
    return;
  if (_keypress >= SKIP_TO_END_STEP)
    return;
  while (_keypress < CONTINUE_STEP)
  {
    spin();
  }
  if (CONTINUE_STEP == _keypress)
    _keypress = WAIT;
}

void AVVis::spin()
{
  boost::this_thread::sleep(boost::posix_time::microseconds (100000));
  try
  {
    viewer->spinOnce(100);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  // AVLOG(_lastKey, 0, 0);
  if ("KP_0" == _lastKey || "0" == _lastKey || "q" == _lastKey)
  {
    _keypress = max(CONTINUE_STEP, _keypress);
  }
  if ("KP_1" == _lastKey || "1" == _lastKey)
  {
    _keypress = max(SKIP_TO_END_STEP, _keypress);
  }
  if ("KP_2" == _lastKey || "2" == _lastKey)
  {
    _keypress = max(DISABLE_WAIT, _keypress);
  }
  _lastKey = "";
}

void AVVis::setRGB(ptCldColor::Ptr cloud, string name, int pointSize, int vp)
{
  if(!_initialized) return;
  if(vp >= _vp->size()){
    vp = 0;
    AVLOG("ERROR: viewpoint requested not present, defaulting to 1st viewpoint", 0, 0);
  }
  if (viewer->contains(name))
  {
    viewer->updatePointCloud(cloud, name);
    return;
  }
  visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<PointXYZRGB>(cloud, rgb, name, _vp->at(vp));
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, name, _vp->at(vp));
}

void AVVis::setCamView(vector<double> pose, PointXYZ center, int vp)
{
  if(!_initialized) return;
  if(vp >= _vp->size()){
    vp = 0;
    AVLOG("ERROR: viewpoint requested not present, defaulting to 1st viewpoint", 0, 0);
  }
  pose[0] *= 1.5;
  PointXYZ temp;
  temp.x = center.x + pose[0] * sin(pose[2]) * cos(pose[1]);
  temp.y = center.y + pose[0] * sin(pose[2]) * sin(pose[1]);
  temp.z = center.z + pose[0] * cos(pose[2]);
  if (pose[2] == 0)
  {
    viewer->setCameraPosition(temp.x, temp.y, temp.z, center.x, center.y, center.z, cos(pose[1] + M_PI), sin(pose[1]), 0, _vp->at(vp));
  }
  else
  {
    viewer->setCameraPosition(temp.x, temp.y, temp.z, center.x, center.y, center.z, 0, 0, 1, _vp->at(vp));
  }
}

void AVVis::addViewsphere(PointXYZ center, int vp, double rad)
{
  if(!_initialized) return;
  if(vp >= _vp->size()){
    vp = 0;
    AVLOG("ERROR: viewpoint requested not present, defaulting to 1st viewpoint", 0, 0);
  }
  viewer->addSphere(center, rad, 0, 0, 1, "Viewsphere" + std::to_string(vp), _vp->at(vp));
  viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Viewsphere" + std::to_string(vp));
}

void AVVis::endStep()
{
  if(!_initialized) return;
  if (_keypress > SKIP_TO_END_STEP)
    return;
  _keypress = WAIT;
}

void AVVis::setupViewer()
{
  viewer->initCameraParameters();
  createViewports();
  for (int i = 0; i < _vp->size(); i++)
  {
    viewer->setBackgroundColor(0.5, 0.5, 0.5, _vp->at(i));
  }
  viewer->removeAllShapes();
  viewer->removeAllPointClouds();
  viewer->resetStoppedFlag();
  viewer->registerKeyboardCallback(&AVVis::processKeyboardEvent, *this);
  viewer->registerPointPickingCallback(&AVVis::processPointPickingEvent, *this);
  // viewer->addCoordinateSystem(1.0, "reference", 0);
}

void AVVis::createViewports()
{
  switch (_vp->size())
  {
  case 2:
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, _vp->at(0));
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, _vp->at(1));
    break;
  case 3:
    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, _vp->at(0));
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, _vp->at(1));
    viewer->createViewPort(0.25, 0.0, 0.75, 0.5, _vp->at(2));
    break;
  case 4:
    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, _vp->at(0));
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, _vp->at(1));
    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, _vp->at(2));
    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, _vp->at(3));
    break;
  case 8:
    viewer->createViewPort(0.00, 0.66, 0.33, 1.00, _vp->at(0));
    viewer->createViewPortCamera(_vp->at(0));
    viewer->createViewPort(0.33, 0.66, 0.66, 1.00, _vp->at(1));
    viewer->createViewPortCamera(_vp->at(1));
    viewer->createViewPort(0.66, 0.66, 1.00, 1.00, _vp->at(2));
    viewer->createViewPortCamera(_vp->at(2));

    viewer->createViewPort(0.00, 0.33, 0.33, 0.66, _vp->at(3));
    viewer->createViewPortCamera(_vp->at(3));
    viewer->createViewPort(0.33, 0.33, 0.66, 0.66, _vp->at(4));
    viewer->createViewPortCamera(_vp->at(4));
    viewer->createViewPort(0.66, 0.33, 1.00, 0.66, _vp->at(5));
    viewer->createViewPortCamera(_vp->at(5));

    viewer->createViewPort(0.00, 0.00, 0.50, 0.33, _vp->at(6));
    viewer->createViewPortCamera(_vp->at(6));
    viewer->createViewPort(0.50, 0.00, 1.00, 0.33, _vp->at(7));
    viewer->createViewPortCamera(_vp->at(7));
    break;
  case 9:
    viewer->createViewPort(0.00, 0.00, 0.33, 0.33, _vp->at(6));
    viewer->createViewPortCamera(_vp->at(6));
    viewer->createViewPort(0.33, 0.00, 0.66, 0.33, _vp->at(5));
    viewer->createViewPortCamera(_vp->at(5));
    viewer->createViewPort(0.66, 0.00, 1.00, 0.33, _vp->at(4));
    viewer->createViewPortCamera(_vp->at(4));

    viewer->createViewPort(0.00, 0.33, 0.33, 0.66, _vp->at(7));
    viewer->createViewPortCamera(_vp->at(7));
    viewer->createViewPort(0.33, 0.33, 0.66, 0.66, _vp->at(0));
    viewer->createViewPortCamera(_vp->at(0));
    viewer->createViewPort(0.66, 0.33, 1.00, 0.66, _vp->at(3));
    viewer->createViewPortCamera(_vp->at(3));

    viewer->createViewPort(0.00, 0.66, 0.33, 1.00, _vp->at(8));
    viewer->createViewPortCamera(_vp->at(8));
    viewer->createViewPort(0.33, 0.66, 0.66, 1.00, _vp->at(1));
    viewer->createViewPortCamera(_vp->at(1));
    viewer->createViewPort(0.66, 0.66, 1.00, 1.00, _vp->at(2));
    viewer->createViewPortCamera(_vp->at(2));
    break;
  default:
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, _vp->at(0));
  }
}
void AVVis::processKeyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
{
  if(!_initialized) return;
  if (event.keyDown())
    _lastKey = event.getKeySym();
  if ("Escape" == _lastKey)
    _keypress = DISABLE_WAIT;
}

void AVVis::processPointPickingEvent(const pcl::visualization::PointPickingEvent &event, void *)
{
  _selectedIndex = event.getPointIndex();
}