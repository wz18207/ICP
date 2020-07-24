// ICP�㷨������׼.cpp : �������̨Ӧ�ó������ڵ㡣
//

/*
    �ο����ӣ�
	https://blog.csdn.net/wokaowokaowokao12345/article/details/73741957
	http://pointclouds.org/documentation/tutorials/interactive_icp.php#interactive-icp
*/

#include "stdafx.h"
#include<iostream>
#include <string>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>//���ӻ�ͷ�ļ�
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/parse.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d & matrix)    //��ӡ��ת�����ƽ�ƾ���
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{  
	//ʹ�ÿո�������ӵ�����������������ʾ
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}


int main(int argc, char** argv)
{
	// ��������ָ��
	PointCloudT::Ptr cloud_in(new PointCloudT);  // ԭʼ����
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // ת����ĵ���
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP �������

	//��ȡpcd�ļ�
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *cloud_in) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud_in->size() << " data points from file1" << std::endl;

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("2.pcd", *cloud_icp) == -1)
	{
		PCL_ERROR("Couldn't read file2 \n");
		return (-1);
	}
	std::cout << "Loaded " << cloud_icp->size() << " data points from file2" << std::endl;


	int iterations = 1;  // Ĭ�ϵ�ICP��������


	// ������ת�����ƽ������Matrix4d��Ϊ4*4�ľ���
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// ��ʼ��(�� https://en.wikipedia.org/wiki/Rotation_matrix)
	double theta = M_PI / 8;  // ��ת�ĽǶ��û��ȵı�ʾ����
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);
	// Z���ƽ������ (0.4 meters)
	transformation_matrix(2, 3) = 0.4;
	// ��ӡת������
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);
	// ʹ�ø��Ա任��ԭʼ���ƽ��б任
	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);  // ����cloud_in����ԭʼ���ƣ�cloud_tr��cloud_icp����ת��/��ת����
	*cloud_tr = *cloud_icp;  // ����cloud_icp��ֵ��cloud_tr���Ա�������ʾ�ı��ݣ��̵��ƣ�


	// ����ICP���󣬲��趨ICP�㷨����
	// icp��׼(���������㷨)
	pcl::console::TicToc time;
	time.tic();
	// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //����ICP��������ICP��׼
	pcl::IterativeClosestPoint<PointT, PointT> icp; //����ICP��������ICP��׼
	icp.setMaximumIterations(iterations);    //��������������iterations=true
	//icp.setInputCloud(cloud_icp); //�����������
	icp.setInputSource(cloud_icp); //�����������
	icp.setInputTarget(cloud_in); //����Ŀ����ƣ�������ƽ��з���任���õ�Ŀ����ƣ�
	icp.align(*cloud_icp);          //ƥ���Դ����
	icp.setMaximumIterations(1);  // ����Ϊ1�Ա��´ε���
	std::cout << "Applied " << iterations << " ICP iteration(s)" << std::endl;
	// �ж�ICP�㷨�ľۺϣ����δ�ۺϣ����˳��������ɹ��ۺϣ�����4*4����ĸ�ʽ����ת������Ȼ���ӡ�����Ծ���任
	if (icp.hasConverged())  // icp.hasConverged ()=1��true������任������ʺ�������
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (-1);
	}


	//���ӻ�
	pcl::visualization::PCLVisualizer viewer("ICP demo");  // ���ӻ���������Ϊ��ICP demo��
	// ����������ֱ�ֿ����ӵ�
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	// �����������ڵ��ƺڰ�ɫ�л��ı���
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;
	// ��ԭʼ��������������ӵ㴰�ڣ����԰�ɫ��ʾ����ʹ����ת������ת��֮��ĵ�������ɫ��ʾ���󴰿ڣ�����ICP����ĵ����Ժ�ɫ��ʾ���Ҵ���
	// ԭʼ�ĵ�������Ϊ��ɫ��
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);  // ����ԭʼ�ĵ��ƶ�����ʾΪ��ɫ
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
	// ת����ĵ�����ʾΪ��ɫ����ʾ���󴰿�
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
	// ICP��׼��ĵ���Ϊ��ɫ����ʾ���Ҵ���
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// �ڸ��Ե��ӿڽ�����ӵ��Ƶ��ı��������Ա���ʹ�����˽�˭��Ӧ˭
	// ��ָ���ӿ�viewport=v1����ַ�����white ������������"icp_info_1"������ַ�����ID��־����10��15��Ϊ����16Ϊ�ַ���С ����ֱ���RGBֵ
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
	// �ַ�����ss���ڽ���������ת��Ϊ�ַ���
	std::stringstream ss;
	ss << iterations;            //����ĵ����Ĵ���
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// �������ñ���bckgr_gray_level�����˱������ڵ���ɫ���ڲ鿴���а��¡�C���Ի�ȡ���������Ȼ�󽫲������Ƶ���������������������λ��/����/���㡣
	// ���ñ�����ɫ
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	// ���������λ�úͷ���
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // ���ӻ����ڵĴ�С
	// ע�ᰴ���ص�����
	viewer.registerKeyboardCallback(&KeyboardEventOccurred, (void*)NULL);

	// ��ʾ
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();  // ����һ�����涯������������δ���°�������༭���ȴ��˳�

		//���¿ո���ĺ���,���ݻص�����KeyboardEventOccurred���������а��¼����ϡ�space�������������next_iteration��ֵΪtrue
		if (next_iteration)
		{
			// ���������㷨
			icp.align(*cloud_icp);
			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate!
				print4x4Matrix(transformation_matrix);  // ��ӡ����任

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}
	return 0;
}





//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//bool next_iteration = false;
//
//void print4x4Matrix(const Eigen::Matrix4d & matrix)
//{
//	printf("Rotation matrix :\n");
//	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
//	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
//	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
//	printf("Translation vector :\n");
//	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
//}
//
//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
//{
//	if (event.getKeySym() == "space" && event.keyDown())
//		next_iteration = true;
//}
//
//int main()
//{
//	// The point clouds we will be using
//	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
//	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
//	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud
//
//
//	int iterations = 1;  // Default number of ICP iterations
//
//
//	pcl::console::TicToc time;
//	time.tic();
//	if (pcl::io::loadPLYFile("fish-2.ply", *cloud_in) < 0)
//	{
//		PCL_ERROR("Error loading cloud %s.\n");
//		return (-1);
//	}
//	std::cout << "\nLoaded file " << "fish-2.ply" << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;
//
//	// Defining a rotation matrix and translation vector
//	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//
//	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//	double theta = M_PI / 8;  // The angle of rotation in radians
//	transformation_matrix(0, 0) = cos(theta);
//	transformation_matrix(0, 1) = -sin(theta);
//	transformation_matrix(1, 0) = sin(theta);
//	transformation_matrix(1, 1) = cos(theta);
//
//	// A translation on Z axis (0.4 meters)
//	transformation_matrix(2, 3) = 0.4;
//
//	// Display in terminal the transformation matrix
//	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
//	print4x4Matrix(transformation_matrix);
//
//	// Executing the transformation
//	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
//	*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
//
//	// The Iterative Closest Point algorithm
//	time.tic();
//	pcl::IterativeClosestPoint<PointT, PointT> icp;
//	icp.setMaximumIterations(iterations);
//	icp.setInputSource(cloud_icp);
//	icp.setInputTarget(cloud_in);
//	icp.align(*cloud_icp);
//	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
//	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;
//
//	if (icp.hasConverged())
//	{
//		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
//		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//		transformation_matrix = icp.getFinalTransformation().cast<double>();
//		print4x4Matrix(transformation_matrix);
//	}
//	else
//	{
//		PCL_ERROR("\nICP has not converged.\n");
//		return (-1);
//	}
//
//	// Visualization
//	pcl::visualization::PCLVisualizer viewer("ICP demo");
//	// Create two vertically separated viewports
//	int v1(0);
//	int v2(1);
//	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//
//	// The color we will be using
//	float bckgr_gray_level = 0.0;  // Black
//	float txt_gray_lvl = 1.0 - bckgr_gray_level;
//
//	// Original point cloud is white
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
//		(int)255 * txt_gray_lvl);
//	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
//	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
//
//	// Transformed point cloud is green
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
//	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
//
//	// ICP aligned point cloud is red
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
//	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
//
//	// Adding text descriptions in each viewport
//	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
//	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
//
//	std::stringstream ss;
//	ss << iterations;
//	std::string iterations_cnt = "ICP iterations = " + ss.str();
//	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
//
//	// Set background color
//	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
//	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
//
//	// Set camera position and orientation
//	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//	viewer.setSize(1280, 1024);  // Visualiser window size
//
//	// Register keyboard callback :
//	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
//
//	// Display the visualiser
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//
//		// The user pressed "space" :
//		if (next_iteration)
//		{
//			// The Iterative Closest Point algorithm
//			time.tic();
//			icp.align(*cloud_icp);
//			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;
//
//			if (icp.hasConverged())
//			{
//				printf("\033[11A");  // Go up 11 lines in terminal output.
//				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
//				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
//				print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose
//
//				ss.str("");
//				ss << iterations;
//				std::string iterations_cnt = "ICP iterations = " + ss.str();
//				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
//				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
//			}
//			else
//			{
//				PCL_ERROR("\nICP has not converged.\n");
//				return (-1);
//			}
//		}
//		next_iteration = false;
//	}
//	return (0);
//}

