#include "../include/dataset/dataset.h"
#include <time.h>


Dataset::Dataset()
{
  static char name[40];
  time_t now = time(0);
  strftime(name, sizeof(name), "log_%Y%m_%d_%H_%M_%s.txt", localtime(&now));
  filename = std::string(name, sizeof(name));
  std::cout << filename << std::endl;
}

void Dataset::setDir(std::string &dir)
{
  filename = dir + filename;
}

Dataset::~Dataset(){}

void Dataset::createNewFile()
{
  eventRoot["data"] = Json::arrayValue;
  initialized = true;
}

bool Dataset::addRun(int run_id, std::string &obj_name)
{
  if(!initialized){
    createNewFile();
  }
  eventRun = new Json::Value();
  (*eventRun)["run_id"] = run_id;
  (*eventRun)["obj_name"] = obj_name;
  eventEntries = Json::arrayValue;
  return true;
}

bool Dataset::addEntry(Entry &e)
{
  if ((*eventRun)["obj_name"] != e.objname)
  {
    std::cout << "[WARNING] Run id does not match, not adding entry" << std::endl;
    eventEntries.clear();
    return false;
  }

  Json::Value entry;
  entry["pcdobj"] = e.pcdobj;
  entry["pcdunexp"] = e.pcdunexp;
  entry["pcdfused"] = e.pcdfused;
  entry["r"] = e.r;
  entry["phi"] = e.phi;
  entry["theta"] = e.theta;
  entry["step"] = e.step;
  eventEntries.append(entry);

  return true;
}

bool Dataset::stopRun(int run_id)
{
  if ((*eventRun)["run_id"] != run_id)
  {
    std::cout << "[WARNING] Run id does not match, not adding the run to file" << std::endl;
    return false;
  }

  (*eventRun)["steps"] = eventEntries;
  eventRoot["data"].append((*eventRun));
  return true;
}

bool Dataset::saveData()
{
  fileId.open(filename);
  fileId << styledWriter.write(eventRoot);
  fileId.close();
  return true;
}

void Dataset::updateFileRoot(Json::Value handle)
{
  eventRoot["data"] = handle["data"];
}

void Dataset::parseJsonFile(std::string filename, Json::Value &obj)
{
  std::ifstream blocks_stream(filename);
  Json::Reader reader;
  reader.parse(blocks_stream, obj);
  std::cout << obj["data"].size() << std::endl;
}

std::string Dataset::getFileName()
{
  return filename;
}
