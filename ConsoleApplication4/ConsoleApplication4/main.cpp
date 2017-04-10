/*
��ɵĹ����У���ά���ƻ�ȡ�����������²�����Ϊ����������٣���ƽ����ͻ�ȡ��3D��ʾ ��һ������ţ�˵ĳ����ϸĵģ�ԭ��ֻ֧�ֵ��ƻ�ȡ����ʾ�����������ƴ���
���⣬ԭ���и�bug���ᵼ�´�����CPUʱ��������ʾ��Ч�ĵ������ݣ��ѱ������������
�������и��꿪��CLOUD_DISPLAY��������������������ά��ʾ������������������ر���ά��ʾ���ܡ��ر���ά��ʾ��Ŀ����Ϊ�˼�����ά�������ݵĴ���

*/
// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// RANSAC
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>

#define CLOUD_DISPLAY 

typedef pcl::PointXYZRGBA PointType;
//typedef pcl::PointXYZ PointType;

int produced_frame_count_last = 0, produced_frame_count_current = 0;

int main(int argc, char* argv[])
{
#ifdef CLOUD_DISPLAY
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
#endif
	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	//RANSAC
	pcl::PointCloud<PointType>::Ptr final(new pcl::PointCloud<PointType>);


	bool is_plane = true;

	if (pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		is_plane = true;
	}
	else if (pcl::console::find_argument(argc, argv, "-sf") >= 0)
	{
		is_plane = false;
	}

	std::vector<int> inliers;




	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex, &is_plane, &final](const pcl::PointCloud<PointType>::ConstPtr& ptr)
	{
		boost::mutex::scoped_lock lock(mutex);
		// ��������
		cloud = ptr;
		produced_frame_count_current++;

	};

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	//��ʼ Start Grabber
	grabber->start();

	DWORD tick_start, tick_end;
	int handled_frame_count_last = 0, handled_frame_count_current = 0, frame_count_before = 0;
	tick_start = GetTickCount();



#ifdef CLOUD_DISPLAY
	while (!viewer->wasStopped())
	{
		// Update Viewer
		viewer->spinOnce();
#else
	Sleep(100);
	while (1) {
#endif
		if (produced_frame_count_current == produced_frame_count_last) {
			Sleep(1);
			continue;
		}
		produced_frame_count_last = produced_frame_count_current;

		// ���߳�
		boost::mutex::scoped_try_lock lock(mutex);
		if (cloud && lock.owns_lock()){
			// handle cloud here


			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
			pcl::VoxelGrid<PointType> sor;
			// ��ʼ��������
			sor.setInputCloud(cloud);
			sor.setLeafSize(0.01f, 0.01f, 0.01f);
			sor.filter(*cloud_filtered);
			cloud = cloud_filtered;

			/*
			// created RandomSampleConsensus object and compute the appropriated model
			pcl::SampleConsensusModelSphere<PointType>::Ptr model_s(new pcl::SampleConsensusModelSphere<PointType>(cloud_filtered));
			pcl::SampleConsensusModelPlane<PointType>::Ptr  model_p(new pcl::SampleConsensusModelPlane<PointType>(cloud_filtered));

			if (is_plane)
			{
			pcl::RandomSampleConsensus<PointType> ransac(model_p);
			ransac.setDistanceThreshold(0.01);
			ransac.computeModel();
			ransac.getInliers(inliers);
			}
			else
			{
			pcl::RandomSampleConsensus<PointType> ransac(model_s);
			ransac.setDistanceThreshold(1);
			ransac.computeModel();
			ransac.getInliers(inliers);
			}

			// copies all inliers of the model computed to another PointCloud
			pcl::copyPointCloud<PointType>(*cloud, inliers, *final);




			handled_frame_count_current++;
			tick_end = GetTickCount();

			if (tick_end - tick_start > 1000) {
			float fps = (handled_frame_count_current - handled_frame_count_last) / ((tick_end - tick_start) / 1000.);

			//frame_count_current++;
			printf("cloud = %d, inliers = %d, fps = %4.2f, frame_produced: %d, frame_handled:%d\n", cloud->size(), inliers.size(), fps,
			produced_frame_count_current, handled_frame_count_current);
			tick_start = tick_end;
			handled_frame_count_last = handled_frame_count_current;

			}

			cloud = final;
			*/

			if (cloud->size() != 0)
			{
				/* Processing to Point Cloud */
// #ifdef CLOUD_DISPLAY
				// Update Point Cloud
				if (!viewer->updatePointCloud(cloud, "cloud")){
					viewer->addPointCloud(cloud, "cloud");
				}
// #endif
			}
			else
			{
				std::cout << "cloud Ϊ 0" << std::endl;
			}
		}
	}

	//ֹͣ Stop Grabber
	grabber->stop();

	return 0;
	}