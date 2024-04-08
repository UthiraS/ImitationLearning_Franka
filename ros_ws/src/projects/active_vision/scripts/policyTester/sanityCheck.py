#!/usr/bin/env python3

import sys, csv,os, copy, io
import numpy as np
import pandas as pd
from summarizerDataCollected import readInput

#https://stackoverflow.com/questions/11707586/how-do-i-expand-the-output-display-to-see-more-columns-of-a-pandas-dataframe
pd.set_option('display.max_rows', 500)
pd.set_option('display.max_columns', 500)
HEADER = ["Object", "Pose #", "Roll?", "Pitch?", "-", "Direction?", "--", "time", "Dummy", "---", "----", "time2", "Start direction", "Steps"]

#Test to ensure that values are roughly what is expected for each method

class Policy(object):
    def __init__(self) -> None:
        self.multiple = 1
        self.initialized = False
    
    def __init__(self, name, multiple=1) -> None:
        self.name = name
        self.multiple = multiple
        self.initialized = False

    def accumulate(self, directoryPath:str, filename:str):
        if(not filename.endswith(".csv")):
           return
        if(not filename.startswith(self.name+":")):
            return
        if(not self.initialized):
            self.data = pd.read_csv(directoryPath+"/"+file, header=None, names=HEADER, usecols=range(14))
            self.data["Steps"] *= self.multiple
            self.initialized = True
            return
        newDF = pd.read_csv(directoryPath+"/"+file, header=None, names=HEADER, usecols=range(14))
        newDF["Steps"] *= self.multiple
        self.data = pd.concat([self.data, newDF])

            
    def compareGeq(self, other):
        totalCases = self.data.shape[0]
        matchingCases = self.data[self.data["Steps"]>=other.data["Steps"]].shape[0]
        print("%s expected to be >= to %s, and it is in %d/%d cases" % (self.name, other.name, matchingCases, totalCases))
        print("Exceptions are: ")
        print(self.data[self.data["Steps"]<other.data["Steps"]])

    def compareLeq(self, other):
        totalCases = self.data.shape[0]
        matchingIndices = self.data["Steps"]<=other.data["Steps"]
        matchingCases = self.data[matchingIndices].shape[0]
        print("%s expected to be <= to %s, and it is in %d/%d cases" % (self.name, other.name, matchingCases, totalCases))
        print("Matches are: ")
        diff = pd.DataFrame(other.data["Steps"] - self.data["Steps"])
        diff.columns = ["Delta"]

        ownSteps = self.data[matchingIndices][["Object", "Roll?", "Pitch?", "Steps"]]
        ownSteps["Pitch?"] *= (180/3.1415)
        ownSteps["Pitch?"] = round(ownSteps["Pitch?"])
        otherSteps = other.data[matchingIndices][["Roll?", "Pitch?", "Steps"]]
        otherSteps["Pitch?"] *= (180/3.1415)
        otherSteps["Pitch?"] = round(otherSteps["Pitch?"])
        # https://stackoverflow.com/questions/44156051/add-a-series-to-existing-dataframe
        df = pd.concat([ownSteps, otherSteps, diff[matchingIndices]], axis=1)
        print(df)
        print("Exceptions are: ")
        
        ownSteps = self.data[~matchingIndices][["Object", "Roll?", "Pitch?", "Steps"]]
        ownSteps["Pitch?"] *= (180/3.1415)
        ownSteps["Pitch?"] = round(ownSteps["Pitch?"])
        # otherSteps = other.data[~matchingIndices][["Object", "Roll?", "Pitch?", "Steps"]]
        otherSteps = other.data[~matchingIndices][["Roll?", "Pitch?", "Steps"]]
        otherSteps["Pitch?"] *= (180/3.1415)
        otherSteps["Pitch?"] = round(otherSteps["Pitch?"])
        df = pd.concat([ownSteps, otherSteps, diff[~matchingIndices]], axis=1)
        print(df)
        
        print("Other - Self (should be positive) = %d" % diff.sum())
        print("Median correct value = %d, median wrong value = %d" % (diff[matchingIndices].median(), diff[~matchingIndices].median()))
        print("Avg correct value = %d (std %.1f), avg wrong value = %d (std %.1f)" % (diff[matchingIndices].mean(), diff[matchingIndices].std(), diff[~matchingIndices].mean(), diff[~matchingIndices].std()))
        return diff.sum()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(("Incorrect number of arguments ", len(sys.argv)))
        sys.exit()

    path = sys.argv[1]

    BFS = Policy("BFS")
    Brick = Policy("BRICK")
    Imitation = Policy("IL")
    E_opt = Policy("C_OPT")
    policies = [Brick, E_opt]
    # policies = [BFS, Brick, Imitation, E_opt]
    for root,dirs,files in os.walk(path):
        for file in files:
            for policy in policies:
                policy.accumulate(root, file)
    
    E_opt.compareLeq(Brick)