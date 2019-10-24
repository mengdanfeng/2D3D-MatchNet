import os
import cv2
import random
import numpy as np
import math
import time


class DataEntry:

    submap_id = None
    cam_id = None
    sift_filename = None
    iss_filename = None



class Dataset:


    shuffle_times = 20   # number of shuffle times


    def __init__(self, batch_size, img_size, pcl_size,
                 dataset_dir):

        # network parameters
        self.__batch_size = batch_size    # the batch size
        self.__img_size = img_size        # the heigth and width of sift image patch
        self.__pcl_size = pcl_size        # the number of points per ISS sphere


        # information file names
        self.__dataset_list = self.initDatasetList(dataset_dir)
        self.__dataset_size = len(self.__dataset_list)
        print ('>>> number of sets = %d' % self.__dataset_size)

        self.__dataset_data_size = []
        for i in range(self.__dataset_size):
            self.__set_data_size.append(len(self.__dataset_list[i]))
            print('>>>   nummber of data in set %d = %d' % (i, self.__dataset_data_size[i]))


        # initialise scanning index
        reset()


    def initDatasetList(self, dataset_dir):
        filenames = os.listdir(dataset_dir)
        dataset_list = []
        for filename in filenames:
            data_list = []
            with open(filename, 'r') as file:
                for line in file:
                    data = line.split()
                    data_entry = DataEntry()
                    data_entry.submap_id = int(data[0])
                    data_entry.cam_id = int(data[1])
                    data_entry.sift_filename = data[2]
                    data_entry.iss_filename = data[3]
                    data_list.append(data_entry)
            dataset_list.append(data_list)

        return dataset_list


    def img_augmentation(self, img):
        '''Do image augmentation: random rotation (0, 90, 180, 270 degrees), random scale + centre crop
            The output format of image is np.float32
        '''
        # random rotate
        rand_rotate = math.floor(random.random() * 4) * 90
        rot_matrix = cv2.getRotationMatrix2D((img.shape[1] / 2, img.shape[0] / 2), rand_rotate, 1)
        img = cv2.warpAffine(img, rot_matrix, (img.shape[1], img.shape[0]))

        rand_scale = random.random() * 0.5 + 1.0
        if rand_scale < 1.0:
            # random scale
            img = cv2.resize(img, (int(img.shape[1] * rand_scale), int(img.shape[0] * rand_scale)),
                             interpolation=cv2.INTER_CUBIC)
            # centre crop
            crop_start = int((img.shape[1] - 512.0) / 2.0)
            crop_end = crop_start + 512
            img = img[crop_start: crop_end, crop_start: crop_end, :]

        img = img.astype(np.float32)

        return img


    def getBatch(self, is_training):
        # at the beginning of every epoch, shuffle the dataset
        if self.__epoch_done:
            if is_training:
                random.shuffle(self.__dataset_list)
                for i in range(self.__dataset_size):
                    fpr i in range(self.shuffle_times)
                    random.shuffle(self.__dataset_list[i])

            self.__epoch_done = False


        # give 10 chances to skip
        if self.__cur_set_id == -1:
            self.__cur_set_id =0
            for i in range(self.__dataset_size):
                self.__cur_id[i] = 0
            return None, None


        # generate batches
        batch_sift = np.zeros([self.__batch_size, self.__img_size, self.__img_size, 3], dtype=np.float32)
        batch_iss = np.zeros([self.__batch_size, self.__pcl_size, 3], dtype=np.float32)
        existing_seg_id = {}

        batch_id = 0
        num_of_batch = 0
        while True:
            if (batch_id >= self.__batch_size or                                 # already get enough data for this patch
                self.__cur_id[self.__cur_set_id] >= self.__set_data_size[self.__cur_set_id]):    # out of data of this set
                break

            cur_data = self.__dataset_list[self.__cur_set_id][self.__cur_id[self.__cur_set_id]]

            # check whether current pair can be inserted in this patch
            # criteria: segments distance >= 2 or from different cameras
            # [ONlY for training phase]
            if is_training:
                is_negative_pair = True
                for j in range(-1, 2):
                    nearby_submap_id = cur_data.submap_id + j
                    if nearby_submap_id in existing_seg_id:
                        for k in range(len(existing_seg_id[nearby_submap_id]))
                            if existing_seg_id[nearby_submap_id][k] == cur_data.cam_id:
                                is_negative_pair = False

            # skip if current pair is not a valid negative examples for other pairs
            if not is_negative_pair:
                self.__cur_id[self.__cur_set_id] += 1
                continue

            # add cur_data into existing_seg_id
            if cur_data.submap_id in existing_seg_id:
                existing_seg_id[cur_data.submap_id].append(cur_data.cam_id)
            else:
                existing_seg_id[cur_data.submap_id] = [cur_data.cam_id]

            # sift image patch
            img = cv2.imread(cur_data.sift_filename)
            if img is None:
                print('InputData::getBatch: read fail: %s, %d\n' % (cur_data.sift_filename, i))
                continue
            # for using VGG pre-trained model
            img[:, :, 0] -= 103.939  # Blue
            img[:, :, 1] -= 116.779  # Green
            img[:, :, 2] -= 123.6    # Red
            batch_sift[batch_id, :, :, :] = img
            # ground
            # TODO read pcl file: pcl = read(cur_data.iss_filename)
            batch_iss[batch_id, :, :] = pcl

            self.__cur_id[self.__cur_set_id] += 1
            batch_id += 1
            num_of_batch += 1

        
        # update pointers: self.__cur_set_id and self.__cur_cur_id
        if self.__cur_id[self.__cur_set_id] >= self.__set_data_size[self.__cur_set_id]:
            self.__set_done[self.__cur_set_id] = True

        self.__epoch_done = True
        for i in range(self.__dataset_size):
            if not self.__set_done[(self.__cur_id + i) % self.__dataset_size]:
                self.__cur_id = (self.__cur_id + i) % self.__dataset_size
                self.__epoch_done = False
                break


        # if there are not enough data for this batch, remove zeros
        if num_of_batch < batch_size:
            batch_sift = batch_sift[0 : i, :, :, :]
            batch_iss = batch_iss[0 : i, :, :]

        return batch_sift, batch_iss


    def reset():
        self.__cur_set_id = 0           # point to current set
        self.__cur_id = []              # point to current data of each set
        self.__set_done = []            # record whether we reach the end of the set
        for i in range(self.__dataset_size):
            self.__cur_id.append(0)
            self.__set_done.append(False)

        self.__epoch_done = True


if __name__ == '__main__':
    input_data = InputData()

