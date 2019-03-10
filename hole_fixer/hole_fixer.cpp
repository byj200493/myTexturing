// hole_fixer.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include <iostream>
#include "hole_fixer_api.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

void createHolePatch(std::vector<std::vector<float>> &holeEdges,
					 std::vector<std::vector<float>> &samplePoints,
					 std::vector<std::vector<float>> &patchVertices,
					 std::vector<std::vector<float>> &patchNormals,
					 std::vector<std::vector<int>> &patchIndices)
{
	/*if (holeEdges.size() < 5)
		return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < samplePoints.size(); ++i)
	{
		pcl::PointXYZ p;
		p.x = samplePoints[i][0];
		p.y = samplePoints[i][1];
		p.z = samplePoints[i][2];
		cloud->push_back(p);
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(5);//20
	n.compute(*normals);
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	PCL_INFO("pass concatenate\n");
	
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);//0.025
							   // Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	pcl::PointCloud<pcl::PointNormal> cloud1;
	pcl::fromPCLPointCloud2(triangles.cloud, cloud1);
	for (int i = 0; i < cloud1.size(); ++i)
	{
		std::vector<float> vt, normal;
		vt.resize(3);
		normal.resize(3);
		vt[0] = cloud1[i].x;
		vt[1] = cloud1[i].y;
		vt[2] = cloud1[i].z;
		normal[0] = cloud1[i].normal[0];
		normal[1] = cloud1[i].normal[1];
		normal[2] = cloud1[i].normal[2];
		patchVertices.push_back(vt);
		patchNormals.push_back(normal);
	}
	for (int i = 0; i < triangles.polygons.size(); ++i)
	{
		std::vector<int> indices;
		indices.resize(3);
		indices[0] = triangles.polygons[i].vertices[0];
		indices[1] = triangles.polygons[i].vertices[1];
		indices[2] = triangles.polygons[i].vertices[2];
		patchIndices.push_back(indices);
	}*/
}