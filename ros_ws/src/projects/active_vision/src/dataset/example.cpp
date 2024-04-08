#include "../include/dataset/dataset.h"
#include <time.h>

int main()
{
     Dataset d;
     Entry e1(1, 1.0, 2.0, 3.0, "a", "b", "c", "bottle");
     Entry e2(2, 1.0, 2.0, 3.0, "a", "b", "c", "bottle");

     Entry e3(1, 1.0, 2.0, 3.0, "a", "b", "c", "cat");
     Entry e4(2, 1.0, 2.0, 3.0, "a", "b", "c", "cat");

     std::string obj1 = "bottle";
     std::string obj2 = "cat";

     std::string dir = "/home/avnish/mer_lab/ros_ws/src/projects/active_vision/src/dataset/";
     d.setDir(dir);
     d.createNewFile();

     d.addRun(1, obj1);
     d.addEntry(e1);
     d.addEntry(e2);
     d.stopRun(1);
    
     d.saveData();

     Json::Value handle;
     d.parseJsonFile(d.getFileName(), handle);
     d.updateFileRoot(handle);
     
     d.addRun(2, obj2);
     d.addEntry(e3);
     d.addEntry(e4);
     d.stopRun(2);
     
     d.saveData();
     d.parseJsonFile(d.getFileName(), handle);
     
     return 0;
}
