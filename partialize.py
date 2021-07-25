import numpy as np
import open3d as o3d
import os
import random
import glob


npy_files = glob.glob('*.npy')

for fname in npy_files:
	cloud = np.load(fname)
	#fname="a96226408895c2685e3c4d3bc6ac3be0.xyz.npy"
	print(fname)
	cnt = 0
	#for 10 random initial points
	for i in range(2):
		initial_point = cloud[np.random.choice(cloud.shape[0]-1, size=1)]
		#print(cnt)
		cnt += 1

		#print(initial_point)

		
		dist_dict = {}
		#get closest to farest points into a dict
		for index, coord in enumerate(cloud):
			#calc istance from initial point
			dist = np.linalg.norm(initial_point-coord)
			dist_dict[index] = dist
		
		'''
		#Adding noise around the partial part
		sample_x = np.random.normal(initial_point[0][0], 0.1, 1000)
		sample_y = np.random.normal(initial_point[0][1], 0.1, 1000)
		sample_z = np.random.normal(initial_point[0][2], 0.1, 1000)

		x = np.array(list(zip(sample_x,sample_y,sample_z)))
		'''

		dist_dict = sorted(dist_dict.items(), key=lambda x: x[1])
		closest_n_pts = []

		#get closest N points
		
		counter = 0
		for i in dist_dict:
			closest_n_pts.append(i[0])
			counter +=1
			if counter == 2000:
				break
		
		
		#x = x.astype(np.float32)

		#delete N closest points
		cloud = np.delete(cloud, closest_n_pts, axis=0)
		

		#add noise
		#cloud = np.concatenate((cloud,x),axis=0) 


	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(cloud)

	file_name = "{0}ply".format(fname[:-3])

	o3d.io.write_point_cloud(os.path.join('/Users/berkbilir/Desktop/noiseify_and_partial/partial_meshes', file_name), pcd)


	np.save("/Users/berkbilir/Desktop/noiseify_and_partial/04_pts_partial"+"/"+fname, cloud)

