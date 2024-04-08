import os, shutil
from os.path import expanduser
import fnmatch
from datetime import datetime

folder1 = "2023-02-08"

file_names_jpg = []
file_names_json = []
for filename in os.listdir(folder1):
	if filename.endswith(".json"):
		file_names_json.append(filename)

	if filename.endswith(".jpg"):
		file_names_jpg.append(filename)

file_names_jpg.sort()
file_names_json.sort()


for inx in range(len(file_names_jpg)):
	src = f"{folder1}/{file_names_jpg[inx]}"
	dst = f"{folder1}/{str(inx).zfill(6)}.rgb.jpg"

	print(inx,src,dst)
	os.rename(src, dst)
	inx += 1
	

for inx in range(len(file_names_json)):
	src = f"{folder1}/{file_names_json[inx]}"
	dst = f"{folder1}/{str(inx).zfill(6)}.json"

	print(inx,src,dst)
	os.rename(src, dst)
	inx += 1

print(os.listdir(folder1))

