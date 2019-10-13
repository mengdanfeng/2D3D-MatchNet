# 2D3D-MatchNet: Learning to Match Keypoints Across 2D Image and 3D Point Cloud

## Abstract

Large-scale point cloud generated from 3D sensors is more accurate than its image-based counterpart. However, it is seldom used in visual pose estimation due to the difficulty in obtaining 2D-3D image to point cloud correspondences. In this paper, we propose the 2D3D-MatchNet - an end-to-end deep network architecture to jointly learn the descriptors for 2D and 3D keypoint from image and point cloud, respectively. As a result, we are able to directly match and establish 2D3D correspondences from the query image and 3D point cloud reference map for visual pose estimation. We create our Oxford 2D-3D Patches dataset from the Oxford Robotcar dataset with the ground truth camera poses and 2D-3D image to point cloud correspondences for training and testing the deep network. Experimental results verify the feasibility of our approach


## Oxford 2D-3D Patches Dataset

Please download the **Oxford 2D-3D Patches Dataset** from the following links:

SIFT patches:
http://www.mediafire.com/file/d39h2mi5qr3db9c/sift_patch_1.zip
http://www.mediafire.com/file/8xzwccx5qb5q8i2/sift_patch_2.zip

ISS volumes:
http://www.mediafire.com/file/lojp7u8b69e6u78/iss_volume.zip

After the download, you need to put the all folders of SIFT patches (from 2 download) into one directory **sift_patch/**.

**sift_patch/** and **iss_volume/** have same directory architectures (See following descrition). The corresponding SIFT patch and ISS volume have the same file name.

### Dataset file architecture:
```
Oxford_2D-3D_Patches_Dataset/
  iss_volume/
    2014-06-26-09-53-12/
    2014-06-26-09-53-12_1/
    2014-07-14-15-16-36/
    2014-07-14-15-42-55/
    2014-11-14-16-34-33/
    2014-11-18-13-20-12/
    2014-11-25-09-18-32/
    2014-11-28-12-07-13/
    2014-12-02-15-30-08/
    2014-12-05-11-09-10/
    2014-12-05-15-42-07/
    2014-12-09-13-21-02/
    2014-12-10-18-10-50/
    2014-12-12-10-45-15/
    2014-12-16-09-14-09/
    2015-02-03-08-45-10/
    2015-02-10-11-58-05/
    2015-02-13-09-16-26/
    2015-02-17-14-42-12/
    2015-02-20-16-34-06/
    2015-02-24-12-32-19/
    2015-03-03-11-31-36/
    2015-03-10-14-18-10/
    2015-03-17-11-08-44/
    2015-03-24-13-47-33/
    2015-04-24-08-15-07/
    2015-05-19-14-06-38/
    2015-05-22-11-14-30/
    2015-06-09-15-06-29/
    2015-06-12-08-52-55/
    2015-06-26-08-09-43/
    2015-07-03-15-23-28/
    2015-07-08-13-37-17/
    2015-07-10-10-01-59/
    2015-07-14-15-16-39/
    2015-07-14-16-17-39/
    2015-08-13-16-02-58/
    2015-04-17-09-06-25/
  sift_patch/
    2014-06-26-09-53-12/
    2014-06-26-09-53-12_1/
    2014-07-14-15-16-36/
    2014-07-14-15-42-55/
    2014-11-14-16-34-33/
    2014-11-18-13-20-12/
    2014-11-25-09-18-32/
    2014-11-28-12-07-13/
    2014-12-02-15-30-08/
    2014-12-05-11-09-10/
    2014-12-05-15-42-07/
    2014-12-09-13-21-02/
    2014-12-10-18-10-50/
    2014-12-12-10-45-15/
    2014-12-16-09-14-09/
    2015-02-03-08-45-10/
    2015-02-10-11-58-05/
    2015-02-13-09-16-26/
    2015-02-17-14-42-12/
    2015-02-20-16-34-06/
    2015-02-24-12-32-19/
    2015-03-03-11-31-36/
    2015-03-10-14-18-10/
    2015-03-17-11-08-44/
    2015-03-24-13-47-33/
    2015-04-24-08-15-07/
    2015-05-19-14-06-38/
    2015-05-22-11-14-30/
    2015-06-09-15-06-29/
    2015-06-12-08-52-55/
    2015-06-26-08-09-43/
    2015-07-03-15-23-28/
    2015-07-08-13-37-17/
    2015-07-10-10-01-59/
    2015-07-14-15-16-39/
    2015-07-14-16-17-39/
    2015-08-13-16-02-58/
    2015-04-17-09-06-25/
```


## Reference

If you are interested in our work, please read our [paper](https://arxiv.org/pdf/1904.09742.pdf).

```
@InProceedings{Feng2019ICRA,
  author = {Feng, Mengdan and Hu, Sixing and Ang, Marcelo and Lee, Gim Hee},
  title = {2D3D-MatchNet: Learning to Match Keypoints Across 2D Image and 3D Point Cloud},
  booktitle = {The IEEE International Conference on Robotics and Automation (ICRA)},
  month = {May},
  year = {2019}
}
```
