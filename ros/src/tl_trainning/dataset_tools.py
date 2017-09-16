import os
import glob
import numpy as np

from sklearn.model_selection import train_test_split


def get_file_list(folder):
    files = []

    # Check if the folder exist
    if not os.path.exists(folder):
        return files

    # Load Files in the Folder
    folder_wildcard = os.path.join(folder, '*')
    files.extend([file for file in glob.glob(folder_wildcard) if  os.path.isfile(file)])

    return files


def load_images(files):
    images = []
    for file in files:
        image = mpimg.imread(file)
        images.append(image)
    return images


def generate_labels(files):
    labels = []
    for filename in files:
        # Unknown = 4 Green = 2 Yellow = 1 Red = 0
        if '.0.' in filename:
            labels.append(0)
        elif '.1.' in filename:
            labels.append(1)
        elif '.2.' in filename:
            labels.append(2)
        else:
            labels.append(4)
    return labels


def prepare_dataset(dataset_folder, test_size=0.2):
    # Get the list of files
    dataset_files = get_file_list(dataset_folder)

    # Load the images
    dataset_images = load_images(dataset_files)

    # Generate the labels
    dataset_labels = generate_labels(dataset_files)

    # Prepare the Traning set and Test Set
    rand_state = np.random.randint(0, 100)
    features_train, features__test, labels__train, labels__test = train_test_split(dataset_images, dataset_labels, test_size=test_size, random_state=rand_state)

    # Generate the dataset
    dataset = {}
    dataset['features_train'] = features_train
    dataset['features_test']  = features__test
    dataset['labels_train']   = labels__train
    dataset['labels_test']    = labels__test

    return dataset


# ##### Test of the functionality
# dataset_folder = './dataset/'

# dataset = prepare_dataset(dataset_folder)

# print("Tranining Set Size: ", len(dataset['features_train']))
# print('Test Set Size: ',      len(dataset['features_test']))