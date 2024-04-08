#include <active_vision_tools/commonAV.h>

void dieWithGrace(sig_atomic_t s)
{
  exit(1);
}

string toDebugMessage(string node, int debugLevel){
  return to_string(debugLevel)+ " " + node;
}

pair<string, int> fromDebugMessage(string message){
  pair<string, int> ret;
  int space_index = message.find(' ');
  ret.second = atoi(message.substr(0,space_index).c_str());
  ret.first = message.substr(space_index+1);
  return ret;
}

void log(string message, string file, string func, int line, int logLevel, int targetSeverity){
  if(targetSeverity > logLevel) return;
  string disp = file.substr(0, file.size()-4) + ":" + func + ":" + to_string(line) + ": " + message;
  cout << disp << endl;
}