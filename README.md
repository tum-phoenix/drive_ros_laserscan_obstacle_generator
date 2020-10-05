# Laserscan Obstacle Generator
Generates obstacles based on LIDAR data:
1. transforms the laserscan message into a point cloud
2. generates a KD-tree from the point cloud to speed up computation
3. uses euclidean cluster extraction to segment point cloud into obstacles
4. checks if obstacles dimensions are below an upper and are bigger than a lower threshold
5. assigns a static trust value to each object
6. assigns static covariances to each object
