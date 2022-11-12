# Author: Muhammad Hamas Khan
# Please add this reference if you want to use part of this code:  https://github.com/hamaskhan
# Email: m.hamaskhan@gmail.com
# ---------------------------------------------------------------------------------------------

# Libraries and dependencies
# ---------------------------------------
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import math as mth
import open3d as o3d
import time

# Useful Resources, References, and Links------------------------------------------------------
# PCL Libraries
# https://pointclouds.org/documentation/tutorials/
# https://pointclouds.org/

# Open3D libraries
# http://www.open3d.org/
# ---------------------------------------------------------------------------------------------
'''
# The format of data in the text file is as below
//  X  ;   Y   ;   Z  ;Value;    Nx  ;     Ny  ;    Nz
385.888;145.156;44.359;1.504;0.272890;-0.422294;0.864407
409.526;176.741;22.387;0.020;0.956552;-0.021037;0.290802
388.059;145.798;42.530;0.953;0.426310;-0.423093;0.799533
398.934;167.566;38.444;1.196;0.734180;-0.292712;0.612617
'''
# Suppresing scientific notation for easy readability
np.set_printoptions(suppress=True)

# 3D plotting of original, sampled and nearest points
# --------------------------------------------------------------------
def plot_3D():
    fig=plt.figure()
    ax = plt.axes(projection='3d')

    #print("X axis of sample test points are ",test_sample[:,0])
    #print("Y axis of sample test points are ",test_sample[:,1])
    #print("Z axis of sample test points are ",test_sample[:,2])

    ax.scatter3D(originalxyz[:,0], originalxyz[:,1], originalxyz[:,2], label="Original Points", marker=".")
    ax.scatter3D(sampledxyz[:,0], sampledxyz[:,1], sampledxyz[:,2], label="Sample Points", marker=".")

    # Select for which points you want to show neighbors
    # ---------------------------------------------------------------------
    ax.scatter3D(nearest[0][:,0], nearest[0][:,1], nearest[0][:,2], label="Nearest 2 Points for original point at index[0,0,0]", marker="x")
    #ax.scatter3D(nearest[:, :, 0], nearest[:, :, 1], nearest[:, :, 2], label="Nearest 2 Points for all original points", marker="x")

    plt.title("3D scatter plot for Nearest Neighbor")
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.legend(loc="upper right")
    plt.show()

'''
def magnitude(vector):
    mag=[]
    for element in vector:
        #print("Elements of vector ", element)
        mag.append(mth.sqrt(np.linalg.norm(element)))
    return mag
'''

def find_angles():

    # Finding angles between normals of original and sampled mesh data
    original_norms=original2d[:, [4, 5,6]]
    sampled_norms=sampled2d[:, [3, 4,5]]
    nearest_points=sampled2d[points]

    # Note here that the structure of the 2 nearest points data are (no. of rows of orignal data, 2, 7 ).
    # Here 2 represents nearest 2 neighbors for each row of original data
    # If KD tree of 3 nearest neighbors is called, then it becomes (no. of rows of orignal data, 3, 7 )
    # Here 3 represents nearest 3 neighbors for each row of original data
    # For now, I am extracting the normals for only the first nearest neighbor. For the second neighbor, use [:,1, [4, 5,6]]
    # To take average of normals of all nearest, call [:,:, [4, 5,6]] and take average
    nearest_norms= nearest_points[:,0, [3, 4,5]]

    print("Normals of the original data are ", original_norms)
    #print("Normals of the sampled data are ", sampled_norms)
    #print("Nearest 2 points with normals are ", nearest_points)
    print("Normals of the nearest data (taking only first neighbor) are ", nearest_norms)

    dot_product=np.sum(original_norms*nearest_norms, axis=1)
    
    print('Results of dot product are ', dot_product)

    # --------------------------------------------------------------------------------
    # Magnitudes of original_norms and nearest_norms are not needed because sqrt of sum of squared normals is 1 for all.
    
    #orig_mag=magnitude(original_norms)
    #near_mag=magnitude(nearest_norms)
    #print("Magnitudes of original norms ", orig_mag)
    #print("Magnitudes of near norms ", near_mag)
    # --------------------------------------------------------------------------------

    theta_r=np.arccos(dot_product)
    print("The angles in radians are ", theta_r)
    theta_d=(theta_r*180)/3.141592
    print("The angles in degrees are ", theta_d)
    
    return theta_d


def Octree(originalxyz,sampledxyz):

# Create Octree as in https://github.com/isl-org/Open3D/blob/master/examples/python/geometry/octree_point_cloud.py    
    octree_o = o3d.geometry.Octree(max_depth=4)     # Octree for Original Data
    octree_o.convert_from_point_cloud(pcd_o, size_expand=0.01)
#    print("The Original Octree results are ", octree_o)

    print('\nDisplaying Octree of the Original cloud')
    o3d.visualization.draw_geometries([octree_o])
    #o3d.visualization.draw([octree_o])    ---- This requires OpenGL 4.1 version minimum

    octree_s = o3d.geometry.Octree(max_depth=4)     # Octree for Sampled Data
    octree_s.convert_from_point_cloud(pcd_s, size_expand=0.01)
#    print("The Sampled Octree results are ", octree_s)

    print('\nDisplaying Octree of the Sampled cloud')
    o3d.visualization.draw_geometries([octree_s])
    #o3d.visualization.draw([octree_s])    ---- This requires OpenGL 4.1 version minimum
    
    print("\nTraversing octrees. Node threshold set to 250000 points\n")
    octree_o.traverse(octree_traverse)


def octree_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0

            for child in node.children:
                if child is not None:
                    n += 1
            print("{} Processing Node {} at depth {} with {} points."
                .format('    ' * node_info.depth,
                        node_info.child_index, node_info.depth,
                        len(node.indices)))
                        #len(node.indices), node_info.origin))

            # we only want to process nodes / spatial regions with enough points
            early_stop = len(node.indices) < 250000

            if early_stop == True:

                #print("Processing Node indices ", node.indices)
                dist,points=kdtree.query(originalxyz[node.indices],2)   # 2 means find nearest 2 neighbors
                #print("The nearest 2 neighbor point indices in this Octree are ", points)

                # Use below if Normals already present in file
                #Oct_points_o=original2d[node.indices]
                #print("Displaying points: ", Oct_points_o)
                #nearest = sampled2d[points]
                #print("The nearest 2 neighbor points in this Octree are ", nearest)
                #original_norms=Oct_points_o[:, [4, 5,6]]
                #nearest_norms= nearest[:,0, [3, 4,5]]
                # ------------------------------------------------------------------------------

                # Retrieving Normals from in-built function results-----------------------------
                original_norms=normals_orig[node.indices]
                nearest_norms= sample_norms[points[:,0]]    # 0 means considering first neightbor normal. Replace with 1 for second neighbor

                #print("Normals of the original data are ", original_norms)
                #print("Normals of the nearest data (taking only first neighbor) are ", nearest_norms)

                dot_product=np.sum(original_norms*nearest_norms, axis=1)         
                #print('Results of dot product are ', dot_product)
                
                # --------------------------------------------------------------------------------
                # Magnitudes of original_norms and nearest_norms are not needed because sqrt of sum of squared normals is 1 for all.        
                #orig_mag=magnitude(original_norms)
                #near_mag=magnitude(nearest_norms)

                # --------------------------------------------------------------------------------
                theta_r=np.arccos(dot_product)
                #print("The angles in radians are ", theta_r)
                theta_d=(theta_r*180)/3.141592
                #print("The angles in degrees are ", theta_d)
                nodes_i=node.indices
                    
                print("Removing points with Normals greater than 10 degrees. Saving Outliers")
                for t in range(0,theta_d.size):

                    if theta_d[t]<10:
                        final.append(original2d[nodes_i[t]])
                        #print("Saving point ", original2d[nodes_i[t]], " at index ", nodes_i[t])
                    else:
                        #print("Angle threshold exceeded. Saving in Outliers")
                        outliers.append(original2d[nodes_i[t]])

    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop


def Normals():

    print("\nComputing Normals of the Original cloud")
    pcd_o.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    print("\nDisplaying Normals of the Original cloud")
    o3d.visualization.draw_geometries([pcd_o], point_show_normal=True)
    #o3d.visualization.draw([[pcd_o], point_show_normal=True])    ---- This requires OpenGL 4.1 version minimum
    
    #print(np.asarray(pcd_o.normals)[:10, :])

    print("\nSaving Normals of the Original cloud")
    #o3d.io.write_point_cloud("original_normals.pcd", pcd_o, write_ascii=True, compressed=False, print_progress=True)             

    normals_orig=np.asarray(pcd_o.normals)

    print("\nComputing Normals of the Sampled cloud")
    pcd_s.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    print("\nDisplaying Normals of the Sampled cloud")
    o3d.visualization.draw_geometries([pcd_s], point_show_normal=True)
    #o3d.visualization.draw([[pcd_s], point_show_normal=True])    ---- This requires OpenGL 4.1 version minimum

    #print(np.asarray(pcd_s.normals)[:10, :])

    print("\nSaving Normals of the Sampled cloud")
    #o3d.io.write_point_cloud("sample_normals.pcd", pcd_s, write_ascii=True, compressed=False, print_progress=True)
    sample_norms=np.asarray(pcd_s.normals)

    return normals_orig, sample_norms

if __name__ == "__main__":

    # Data loading and pre-processing
    # The primary key for both (original & post processing) list/vector/array must be maintained
    # To later retrieve the data from original files
    # Data is loaded in to an NxM matrix using the delimiter arguments
    # ------------------------------------------------------------
    print("\nLoading Original and Sampled Clouds")
    original = np.genfromtxt("original.txt", skip_header=1, dtype=str, delimiter=';') #Open3D File IO can also be used
    sampled = np.genfromtxt("sampled.txt", skip_header=1, dtype=str, delimiter=';') #Open3D File IO can also be used
    print("Original Cloud no. of points are ", original.shape)
    print("Sampled Cloud no. of points are ", sampled.shape)

    # Convert string matrix to int
    original2d=original.astype(np.float32)
    sampled2d=sampled.astype(np.float32)

    # Select first three columns for all rows to get only x, y,z data
    originalxyz=original2d[:, [0, 1,2]]
    sampledxyz=sampled2d[:, [0, 1,2]]


# Create Point Cloud as in https://github.com/isl-org/Open3D/blob/master/examples/python/geometry/point_cloud_with_numpy.py
    pcd_o = o3d.geometry.PointCloud()      # Open3d PointCloud object of originalxyz data
    pcd_o.points = o3d.utility.Vector3dVector(originalxyz)
    pcd_o.paint_uniform_color([0, 0, 1])

    pcd_s = o3d.geometry.PointCloud()      # Open3d PointCloud object of sampledxyz data
    pcd_s.points = o3d.utility.Vector3dVector(sampledxyz)
    pcd_s.paint_uniform_color([1, 0.706, 0])
    print("\nData Loaded")

    # Computing Point Normals
    normals_orig, sample_norms=Normals()

    #Normals of the test data
    #normals_orig=np.array([[0.272890,-0.422294,0.864407],[0.956552,-0.021037,0.290802],[0.426310,-0.423093,0.799533],[0.734180,-0.292712,0.612617]])
    #sample_norms=np.array([[-0.836939,0.362726,-0.409833],[-0.789443,-0.154881,-0.593962],[-0.635588,-0.304117,-0.709606],[-0.577758,-0.374806,-0.725062]])

    # Initializing KD Tree and nearest neighbor return
    # ----------------------------------------------------------------------------
    print("\nInitializing KD Tree with Sample cloud as reference")
    kdtree=KDTree(sampledxyz)

    '''
# --------------------------------------------------------------------------------
# Below processing is without Octree Implementation-------------------------------
    print("Starting Normal Angles Estimation And Outlier Removal WithOut Octree Data Structure")
    start_time = time.time()

    dist,points=kdtree.query(originalxyz,2)
#    print("The distance for nearest 2 neighbors are ", dist)
    print("The nearest 2 neighbor point indices are ", points)
    nearest = sampledxyz[points]
    print("The nearest 2 neighbor points are ", nearest)
    theta=find_angles()
#    print("Size of theta is", theta.size)
    final_no_oct=[]

    for t in range(0,theta.size):
        #print(theta[t])
        if theta[t]<150:
            final_no_oct.append(original2d[t])

    final_no_oct=np.array(final_no_oct)
    print("\nSaving results to file named final_no_oct.txt")
    np.savetxt('final_no_oct.txt', np.around(final_no_oct,decimals=2), header='//X;Y;Z;Scalar Field;Nx;Ny;Nz', delimiter=';', fmt='%f', comments='')
    print("Final no. of points of final_no_oct: ", final_no_oct.shape)
    print("Execution time without Octrees: %s seconds" % (time.time() - start_time))

    '''

# ---------------------------------------------------------------------------------------------
# Below processing is with Octree Implementation-----------------------------------------------
    print("\nStarting Normal Angles Estimation And Outlier Removal With Octree Data Structure")
    start_time = time.time()
    final=[]
    outliers=[]
    Octree(originalxyz,sampledxyz)

    final=np.array(final)
    outliers=np.array(outliers)
    #print("Final original vector WITH Octree is ", final)
    print("\nSaving results to files named final.txt and outliers.txt")
    np.savetxt('final.txt', np.around(final,decimals=2), header='//X;Y;Z;Scalar Field;Nx;Ny;Nz', delimiter=';', fmt='%f', comments='')
    print("Final no. of Points after Outlier Removal Based on Normal Angle threshold: ", final.shape)
    np.savetxt('outliers.txt', np.around(outliers,decimals=2), header='//X;Y;Z;Scalar Field;Nx;Ny;Nz', delimiter=';', fmt='%f', comments='')
    print("Final no. of Outliers after Outlier Removal Based on Normal Angle threshold: ", outliers.shape)
    print("Execution time with Octrees: %s seconds" % (time.time() - start_time))

    #plot_3D()