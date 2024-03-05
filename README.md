# DORES(Dynamic_object_Removal_for_Effective_SLAM)

## 1. Why we need dynamic_object_removal?
While performing SLAM or Localization, robots may encounter various situations. 
For instance, there could be instances where dynamic objects are detected by sensors, or adverse weather conditions such as rain or snow causing interference with optical sensors like cameras or lidars. 
These situations involve objects other than map data being acquired by sensor data, which consequently leads to a decrease in accuracy in localization. 
Therefore, removing dynamic objects can enhance the accuracy of SLAM or Localization, and also improve the robot's perception capabilities in extreme conditions.

## 2. Our Method
![Screenshot from 2024-03-05 12-44-23](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/fbf50c05-8a92-4098-9691-08d6afd8a524)

The most basic idea is based on the physical characteristics of the point clouds constituting dynamic and static objects. 
Dynamic objects result in lower density point clouds, while static objects, lead to higher density point clouds. 
However, a problem arises due to the physical properties of lidar sensors. Lidars emit light radially to detect surrounding objects, causing point clouds of distant objects to appear sparse and those of closer objects to appear dense. 
Thus, even dynamic objects, when closer, may exhibit higher density compared to distant static objects. 
Therefore, the distance between lidar paths is calculated within each clustering, reflecting it in the density characteristic.
There are two methods for calculating the density characteristic. 
Method A randomly selects points and measures the number of points within a certain distance around them. 
Method B randomly selects points and computes the average of the distances to the closest n points. 
The values obtained from Method A and Method B are combined, and lower values are considered as dynamic objects, and those points are removed.

## 3. Experiments & Results

![Screenshot from 2024-03-05 12-55-30](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/2e51c4f7-d62b-4419-afef-0ddda51ed147)
