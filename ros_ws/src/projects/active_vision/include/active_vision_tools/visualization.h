#pragma once


#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <active_vision_tools/mathConvenience.h>
#include <active_vision_tools/commonAV.h>
#include <thread>

enum keypressLevels{
  WAIT,
  CONTINUE_STEP,
  SKIP_TO_END_STEP,
  DISABLE_WAIT
};

class AVVis{
public:
  visualization::PCLVisualizer::Ptr viewer;
  AVVis() {};
  ~AVVis();

  /// @brief Initializes the viewer
  /// @param name Viewer's name
  /// @param numberOfViewpoints Number of sub-windows to open for the viewer
  virtual void setup(string name="viewer", int numberOfViewpoints=1);

  /// @brief Takes a single step in the visualization
  /// @details Skipped if keypress >= SKIP_TO_END_STEP. Resets keypress to WAIT after completion.
  virtual void stepVis();


  /// @brief Sets or updates a color point cloud
  /// @param cloud The cloud to visualize
  /// @param name The ID used to track the cloud internally.
  /// @param pointSize The point size to display. [0,inf)
  /// @param vp The viewport to display the cloud in
  void setRGB(ptCldColor::Ptr cloud, string name="color cloud", int pointSize=3, int vp=0);


  /// @brief Sets the camera to a desired location
  /// @param pose The desired orientation of the camera in (r, phi, theta) notation
  /// @param center The point the camera is moving about
  /// @param vp The viewport to move the camera in
  void setCamView(vector<double> pose, PointXYZ center, int vp=0);


  /// @brief Draws a wireframe viewsphere
  /// @param center The point the viewsphere is centered on
  /// @param vp The viewport to draw the viewsphere in
  /// @param rad The radius of the viewsphere
  void addViewsphere(PointXYZ center, int vp=0, double rad=0.5);


  /// @brief Signals that a single visualization step has ended and resets keypress
  void endStep();
  
protected:
  bool _initialized = false;
  int _selectedIndex = -1;
  vector<int>* _vp;
  keypressLevels _keypress = WAIT;
  string _lastKey = "";

  /// @brief Takes whatever internal action is needed to allow the user to interact with the viewer
  virtual void spin();


  /// @brief Helper, handles internal viewer setup
  void setupViewer();


  /// @brief Helper, handles viewport creation
  void createViewports();


  /// @brief Allows the viewer to receive key input
  /// @param event The keypress event
  /// @param  
  virtual void processKeyboardEvent(const pcl::visualization::KeyboardEvent &event, void*);


  /// @brief Allows the viewer to detect when a point is selected
  /// @param event The point selection event
  /// @param  
  virtual void processPointPickingEvent(const pcl::visualization::PointPickingEvent& event, void*);
};