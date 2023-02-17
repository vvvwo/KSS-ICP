# *KSS-ICP: A Point Cloud Registration Method based on Kendall Shape Space*

Created by C. Lv, W. Lin, and Q. Bao from Nanyang Technological University. The paper is accepted by TIP.

![image](https://user-images.githubusercontent.com/65271555/219576868-883b64f1-0873-404b-9814-a925c429b447.png)

## Introduction

Point cloud registration is a popular topic {that} has been widely used in 3D model reconstruction, location, and retrieval. In this paper, we propose a new registration method, KSS-ICP, to address the rigid registration task in Kendall shape space (KSS) with Iterative Closest Point (ICP). The KSS is a quotient space that removes influences of translations, scales, and rotations for shape feature-based analysis. Such influences can be concluded as the similarity transformations that do not change the shape feature. The point cloud representation in KSS is invariant to similarity transformations. We utilize such property to design the KSS-ICP for point cloud registration. To tackle the difficulty to achieve the KSS representation in general, the proposed KSS-ICP formulates a practical solution that does not require complex feature analysis, data training, and optimization. With a simple implementation, KSS-ICP achieves more accurate registration from point clouds. It is robust to similarity transformation, non-uniform density, noise, and defective parts. Experiments show that KSS-ICP has better performance than the state-of-the-art. 

## Citation
If you find our work useful in your research, please consider citing:

     @article{lv2022kss,
         title={KSS-ICP: Point Cloud Registration based on Kendall Shape Space},
         author={Lv, Chenlei and Lin, Weisi and Zhao, Baoquan},
         journal={arXiv preprint arXiv:2211.02807},
         year={2022}
     }

## Implementation
This project is implemented by VS2019, C++.

The addtional libiary should be included:

PCL library 1.8.1

CGAL libiary

OpenGL: freeglut64

## Update

2023/2/16 The paper of KSS-ICP is accepted by IEEE TIP. 

2021/3/16 The second version of KSS-ICP, additional process is updated. 

2021/3/6 The first version of KSS-ICP, including construction of point cloud representation and alignment. 


