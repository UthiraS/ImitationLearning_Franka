from collections import Counter
import re
import matplotlib.pyplot as plt
from math import sqrt

def readVarLog():
  with open("./featur.txt", "r") as file:
    data = file.read()
    data = data.split("---------------")
    lastRuns = {}
    pClass = ""
    for item in data:
      res = Counter(item)
      if(0 == res['\n']):
        pClass = item
        continue
      lastRuns[pClass] = item.split('\n')
    return lastRuns


def processPerformance(line: str):
  #https://stackoverflow.com/questions/11339210/how-to-get-integer-values-from-a-string-in-python
  split = re.findall('\d+', line)
  iteration = int(split[0])
  naive_mean = int(split[1]) + int(split[2])/100.0
  naive_std = int(split[3]) + int(split[4])/100.0
  expert_mean = int(split[5]) + int(split[6])/100.0
  expert_std = int(split[7]) + int(split[8])/100.0
  copy_mean = int(split[9]) + int(split[10])/100.0
  copy_std = int(split[11]) + int(split[12])/100.0
  return (iteration, naive_mean, sqrt(naive_std), expert_mean, sqrt(expert_std), copy_mean, sqrt(copy_std))

def processTime(line: str):
  split = line.split("Last step to this one took ")[1]
  split = split.split(", this evaluation took ")[:2]
  #Drop trailing period
  split[1] = split[1][:-1]
  return split

class runData():
  def __init__(self, name: str):
    self.name = name
    self.y = []
    self.n_mean = []
    self.n_std = []
    self.e_mean = []
    self.e_std = []
    self.c_mean = []
    self.c_std = []
    self.train_time = []
    self.eval_time = []

  def update1(self, iteration, naive_mean, naive_std, expert_mean, expert_std, copy_mean, copy_std):
    self.y.append(iteration)
    self.n_mean.append(naive_mean)
    self.n_std.append(naive_std)
    self.e_mean.append(expert_mean)
    self.e_std.append(expert_std)
    self.c_mean.append(copy_mean)
    self.c_std.append(copy_std)

  def update2(self, t1, t2):
    self.train_time.append(t1)
    self.eval_time.append(t2)

  def graph(self):
    plt.figure(self.name)
    plt.errorbar(self.y, self.n_mean, yerr=self.n_std, capsize=5.0, alpha=.7)
    plt.errorbar(self.y, self.e_mean, yerr=self.e_std, capsize=5.0, alpha=.7)
    plt.errorbar(self.y, self.c_mean, yerr=self.c_std, capsize=5.0, alpha=.7)
    plt.show()

  def graph2(self):
    fig, (ax1, ax2) = plt.subplots(1, 2)
    fig.suptitle(self.name)
    ax1.set_title("Time")
    ax1.plot(self.y, self.train_time)
    ax1.plot(self.y, self.eval_time)
    ax2.set_title("Performance")
    ax2.errorbar(self.y, self.n_mean, yerr=self.n_std, capsize=5.0, alpha=.7)
    ax2.errorbar(self.y, self.e_mean, yerr=self.e_std, capsize=5.0, alpha=.7)
    ax2.errorbar(self.y, self.c_mean, yerr=self.c_std, capsize=5.0, alpha=.7)
    plt.show()

def postProcessLog(lastRuns):
  allData = []  
  for key in lastRuns.keys():
    cData = runData(key)
    #Remove the trailing 1st & last spaces
    lastRuns[key] = lastRuns[key][1:-1]
    for i in range(len(lastRuns[key])):
      if(0 == i%2):
        r = processPerformance(lastRuns[key][i])
        cData.update1(r[0], r[1], r[2], r[3], r[4], r[5], r[6])
      else:
        r = processTime(lastRuns[key][i])
        cData.update2(r[0], r[1])
    allData.append(cData)
    cData.graph()
  return allData

if __name__ == "__main__":
  r = postProcessLog(readVarLog())
