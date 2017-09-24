"""
Ad hoc data cleaning pipeline:
  Clean the raw data with bounding boxes labels collected from the simulator.

"""

import os, glob
import datetime
from shutil import move
import csv
import pandas as pd


PATH_TO_DIR = 'raw_data/green'
OUTPUT_CSV = 'sim_labels.csv'

WIDTH = 800
HEIGHT = 600
IMAGE_FORMAT = 'png'

IMAGE_SRC_DIR = PATH_TO_DIR
IMAGE_DST_DIR = 'images'

CSV_SRC_DIR = PATH_TO_DIR
CSV_DST_DIR = 'csvs'

#======================================================
# Rename files with date and time appended in order to
# avoid name conflict
#======================================================
os.chdir(PATH_TO_DIR)

num_examples =  int(len(os.listdir())/3)
for i in range(1, num_examples+1):
    # Get the time string
    dt_now = str(datetime.datetime.now())
    dt_in = datetime.datetime.strptime(dt_now, '%Y-%m-%d %H:%M:%S.%f')
    dt_str = dt_in.strftime('%Y_%m_%d_%H_%M_%S_%f')

    for file_name in glob.glob('image_{0}.*'.format(i)):

        if file_name.endswith("processed.png"):
            new_name = file_name[:6] + dt_str + file_name[-14:]
        elif file_name.endswith("png"):
            new_name = file_name[:6] + dt_str + file_name[-4:]
        elif file_name.endswith("csv"):
            new_name = file_name[:6] + dt_str + '.csv'
        else:
            pass

        os.rename(file_name, new_name)

os.chdir("..")
os.chdir("..")


#======================================================
# Move image files to directory /dataset/images
#======================================================
for file_name in os.listdir(IMAGE_SRC_DIR):
    if file_name.endswith('png') and not file_name.endswith('processed.png'):
        src = os.path.join(IMAGE_SRC_DIR, file_name)
        dst = os.path.join(IMAGE_DST_DIR, file_name)
        move(src, dst)


#======================================================
# Move csv files to directory /dataset/csvs
#======================================================
for file_name in os.listdir(CSV_SRC_DIR):
    if file_name.endswith('csv'):
        src = os.path.join(CSV_SRC_DIR, file_name)
        dst = os.path.join(CSV_DST_DIR, file_name)
        move(src, dst)


#======================================================
# Read and combine csv files into one csv
#======================================================
os.chdir(CSV_DST_DIR)

lines = []
for file in os.listdir():
    with open(file, "rt") as csvfile:
        reader = csv.reader(csvfile, delimiter=' ')
        for row in reader:
            ln = []
            ln.append(file[:-3]+IMAGE_FORMAT)    # image file
            ln.append(WIDTH)                     # image width
            ln.append(HEIGHT)                    # image height
            for i in range(1, 6):                # class, ymin, xmin, ymax, xmax
                ln.append(row[i]) 
            lines.append(ln)

column_name = ['image', 'width', 'height', 'class', 'ymin', 'xmin', 'ymax', 'xmax']
df = pd.DataFrame(lines, columns=column_name)
os.chdir('..')
df.to_csv(OUTPUT_CSV, index=None)
print('Full labels csv done')

