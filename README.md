# 2D3D-MatchNet: Learning to Match Keypoints Across 2D Image and 3D Point Cloud

## Abstract

Large-scale point cloud generated from 3D sensors is more accurate than its image-based counterpart. However, it is seldom used in visual pose estimation due to the difficulty in obtaining 2D-3D image to point cloud correspondences. In this paper, we propose the 2D3D-MatchNet - an end-to-end deep network architecture to jointly learn the descriptors for 2D and 3D keypoint from image and point cloud, respectively. As a result, we are able to directly match and establish 2D3D correspondences from the query image and 3D point cloud reference map for visual pose estimation. We create our Oxford 2D-3D Patches dataset from the Oxford Robotcar dataset with the ground truth camera poses and 2D-3D image to point cloud correspondences for training and testing the deep network. Experimental results verify the feasibility of our approach


## Oxford 2D-3D Patches Dataset

Please download the **Oxford 2D-3D Patches Dataset** from the following links:

SIFT patches:



ISS volumes:


After the download, you need to put the all folders of SIFT patches (from 2 download) into one directory **sift_patch/**.

**sift_patch/** and **iss_volume** have same directory architectures (See following descrition). The corresponding SIFT patch and ISS volume have the same file name.


## Reference

If you are interested in our work, please read our [paper](https://arxiv.org/pdf/1904.09742.pdf).

@InProceedings{Feng2019ICRA,
  author = {Feng, Mengdan and Hu, Sixing and Ang, Marcelo and Lee, Gim Hee},
  title = {2D3D-MatchNet: Learning to Match Keypoints Across 2D Image and 3D Point Cloud},
  booktitle = {The IEEE International Conference on Robotics and Automation (ICRA)},
  month = {May},
  year = {2019}
}

