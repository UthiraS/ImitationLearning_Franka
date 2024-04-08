#ifndef DATASET_H
#define DATASET_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>

struct Entry{
    int step;
    float r, theta, phi;
    std::string pcdobj, pcdunexp, pcdfused;
    std::string objname;

    Entry(int _step, float _r, float _theta, float _phi, std::string _pcdobj, std::string _pcdunexp, std::string _pcdfused, std::string _objname): step(_step), r(_r), theta(_theta), phi(_phi), pcdobj(_pcdobj), pcdunexp(_pcdunexp), pcdfused(_pcdfused), objname(_objname)
    {};
};

class Dataset 
{
public:
    Dataset();

    ~Dataset();

    void createNewFile();

    void setDir(std::string &dir);
    
    bool addRun(int run_id, std::string &obj_name);
    
    bool addEntry(Entry &en);
    
    bool stopRun(int run_id);

    bool saveData();
    
    std::string getFileName();

    void parseJsonFile(std::string filename, Json::Value &obj);
    
    void updateFileRoot(Json::Value handle);

private:
    std::string filename;
    std::ofstream fileId;
    Json::Value *eventRun;
    Json::Value eventEntries;
    Json::Value eventRoot;
    Json::StyledWriter styledWriter;
    bool initialized = false;

};

#endif
