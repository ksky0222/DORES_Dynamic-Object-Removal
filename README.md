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
### Before_remove
![Screenshot from 2024-03-05 12-59-19](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/a8637f6c-1869-40a9-aae9-791094e6dd6b)

### After_remove
![Screenshot from 2024-03-05 12-59-29](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/50aa43eb-8724-42e3-b955-acf1f9a4ad08)


### Remove in Kitti_semantic_dataset
#### Comparison Only A, Only B, A+B
![Screenshot from 2024-03-05 12-55-30](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/2e51c4f7-d62b-4419-afef-0ddda51ed147)

#### Comparison with other Algorithms
##### Sequence 0
![Screenshot from 2024-03-05 13-03-26](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/23af71ed-75db-4eb3-8728-b5ddeb12da5e)
##### Sequence 1
![Screenshot from 2024-03-05 13-03-34](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/9e4be6c9-ec0c-43a5-be30-fc5c568d7cfc)
##### Sequence 2
![Screenshot from 2024-03-05 13-03-41](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/5c8e0a3e-50bf-450a-8618-1966aceb6103)
##### Sequence 5
![Screenshot from 2024-03-05 13-03-50](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/0b2efa90-baf6-4f50-8e98-2b828c1670bf)
##### Sequence 7
![Screenshot from 2024-03-05 13-03-56](https://github.com/ksky0222/DORES_Dynamic-Object-Removal/assets/109937431/70eba2fc-8245-434f-9c1d-58b746d52f7f)
