//#include "stdio.h"
#include "iostream"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#pragma comment (lib,"opencv_world450.lib")
using namespace std;
#include "utils.hpp"
#include "dotMatrix.hpp"


int main()
{
	//# 状态初始化
	float initial_target_box[] = { 729, 238, 764, 339 };  // 目标初始bouding box
	//float	initial_target_box[] = { 160, 414, 223, 566 };//  # 目标初始bouding box hk
	//float	initial_target_box[] = { 193, 342, 250, 474 };
	//float	initial_target_box[] = { 193, 342, 250, 474 };
	float	initial_box_state[] = { 0, 0, 0, 0 };
	int a=xyxy_to_xywh(initial_target_box, initial_box_state);
	//for (int i = 0; i < 4; ++i)
	//{
	//	cout << "initial_box_state"<<initial_box_state[i] << endl;
	//}
	float	initial_state[6][1] = { initial_box_state[0],initial_box_state[1],initial_box_state[2],initial_box_state[3],0,0 };//initial_box_state[1-4][0][0]
	//for (int i = 0; i < 6; ++i)
	//{
	//	cout << "initial_state" << initial_state[i][0] << endl;
	//}
	//int	initial_state = np.array([[initial_box_state[0], initial_box_state[1], initial_box_state[2], initial_box_state[3],
	//	0, 0]]).T; //[中心x, 中心y, 宽w, 高h, dx, dy]
	float	IOU_Threshold = 0.3;// 匹配时的阈值  ori : 0.3

	//	//# 状态转移矩阵，上一时刻的状态转移到当前时刻
	//	A = np.array([[1, 0, 0, 0, 1, 0],
	//		[0, 1, 0, 0, 0, 1],
	//		[0, 0, 1, 0, 0, 0],
	//		[0, 0, 0, 1, 0, 0],
	//		[0, 0, 0, 0, 1, 0],
	//		[0, 0, 0, 0, 0, 1]])
	#define SIZE 6
	//int A[SIZE][SIZE] = { 0 };
	float A[SIZE][SIZE] = { { 1, 0, 0, 0, 1, 0 },
							{ 0, 1, 0, 0, 0, 1 },
							{ 0, 0, 1, 0, 0, 0 },
							{ 0, 0, 0, 1, 0, 0 },
							{ 0, 0, 0, 0, 1, 0 },
							{ 0, 0, 0, 0, 0, 1 } };
	float A_T[SIZE][SIZE] = { { 1, 0, 0, 0, 0, 0 },
	{ 0, 1, 0, 0, 0, 0 },
	{ 0, 0, 1, 0, 0, 0},
	{ 0, 0, 0, 1, 0, 0},
	{ 1, 0, 0, 0, 1, 0},
	{ 0, 1, 0, 0, 0, 1}};
	//for (int i = 0; i < SIZE; ++i)
	//{
	//	A[i][i] = 1;
	//	//cout << A[i][i] << endl;
	//}
	float H[SIZE][SIZE] = { 0 };
	for (int i = 0; i < SIZE; ++i)
	{
		H[i][i] = 1;
	}
	float H_T[SIZE][SIZE] = { 0 };
	for (int i = 0; i < SIZE; ++i)
	{
		H_T[i][i] = 1;
	}

	float Q[SIZE][SIZE] = { 0 };
	for (int i = 0; i < SIZE; ++i)
	{
		Q[i][i] = 0.1; //0.1
	}
	float R[SIZE][SIZE] = { 0 };
	for (int i = 0; i < SIZE; ++i)
	{
		R[i][i] = 1;
	}
	float P[SIZE][SIZE] = { 0 };
	for (int i = 0; i < SIZE; ++i)
	{
		P[i][i] = 1;
	}

	//for (int i = 0; i < SIZE; ++i)
	//	for (int j = 0; j < SIZE; ++j)
	//{
	//	//A[i][i] = 1;
	//	cout << H[i][j] << endl;
	//}

		// 状态观测矩阵
	//H = np.eye(6);

	string video_path = "../testvideo1.mp4";
	string 	label_path = "../labels/";
	string 	file_name = "testvideo1";
	cv::VideoCapture	cap = cv::VideoCapture(video_path);
	//# ---------状态初始化----------------------------------------
	int frame_counter = 1;
	//float	X_posterior[6] ={ initial_box_state[0],initial_box_state[1],initial_box_state[2],initial_box_state[3],0,0 };//initial_box_state[1-4][0][0]
	float X_posterior[SIZE][1] = { { initial_box_state[0] },  // X_posterior 需要不断更新 // X_posterior 在物体被遮挡时候不再更新了！！
	{ initial_box_state[1] },
	{ initial_box_state[2] },
	{ initial_box_state[3] },
	{ 0. },
	{ 0. } };
	float P_posterior[SIZE][SIZE] = { 0 };
	for (int i = 0; i < SIZE; ++i)
	{
		P_posterior[i][i] = 1;
	}
	//int	P_posterior = np.array(P);
	float	Z[6][1] = { initial_box_state[0],initial_box_state[1],initial_box_state[2],initial_box_state[3],0,0 };
	vector<float>	trace_list; // # 用于保存目标box的轨迹

	while (true) 
	{
		// Capture frame - by - frame
		cv::Mat frame;
	    cap>> frame;
		if (frame.empty())
			break;
		//matrixShow("X_posterior2", *X_posterior, 6, 1);
		float tem[]= {  X_posterior[0][0] , X_posterior[1][0] , X_posterior[2][0] , X_posterior[3][0] };
		float last_box_posterior[] = {0,0,0,0};
		xywh_to_xyxy(tem, last_box_posterior);
		//for (int i = 0; i < 4; ++i)
		//{
			//cout << "d" << last_box_posterior[i]<< endl;
		//}
		cv::Scalar color = (255, 255, 255);
		//plot_one_box(last_box_posterior, frame, color, false);


		// 1. read detection file
		ifstream detectionFile;
		char detpath[150];
		sprintf(detpath, "%s%s_%d.txt", label_path.c_str(),file_name.c_str(), (frame_counter));
		//sprintf_s(detpath, "%s%s_%s.txt", "sdfsf", file_name, to_string(frame_counter));

		//cout << " find file " << detpath << endl;
		detectionFile.open(detpath);

		if (!detectionFile.is_open())
		{
			cerr << "Error: can not find file " << detpath << endl;
			return 1;
		}

		string detLine;
		//istringstream ss;
		//vector<TrackingBox> detData;
		 
		float max_iou = IOU_Threshold;
		bool	max_iou_matched = false;
		float  target_box[] = { 0,0,0,0 };
		while (getline(detectionFile, detLine))
		{
			//TrackingBox tb;

			//ss.str(detLine);
			//cout << detLine << endl;

			float  xyxy[] = { 0,0,0,0 };
			
			//string imageName;
			float res = getImageNameFromPath(detLine.c_str(), 1, 2);
			//cout << "imageName:" << res << endl;
			xyxy[0] = res;
			res = getImageNameFromPath(detLine.c_str(), 2, 3);
			//cout << "imageName:" << res << endl;
			xyxy[1] = res;
			res = getImageNameFromPath(detLine.c_str(), 3, 4);
			//cout << "imageName:" << res << endl;
			xyxy[2] = res;
			res = getImageNameFromPath(detLine.c_str(), 4, 5);
			//cout << "imageName:" << res << endl;
			xyxy[3] = res;
			//float  xyxy[] = { float(p[1]),float(p[2]), float(p[3]), float(p[4]) };
			//for (int i = 0; i < 4; ++i)
			//{
			//	cout << "d" << xyxy[i]<< endl;
			//}
			
			plot_one_box(xyxy, frame, cv::Scalar(0, 0, 0),false); //163
			float iou = cal_iou(xyxy, last_box_posterior); //每一个检测框和上一帧的预测框（last_box_posterior来自于X_posterior）的交并比！
			//cout << "xyxy " << endl;
			//for (int i = 0; i < 4; ++i)
			//{
			//	cout << xyxy[i]<< " ";
			//	//cout << "last_box_posterior " << last_box_posterior[i] << endl;
			//}
			//cout << "last_box_posterior " << endl;
			//cout << "iou "<< iou << endl;
			if (iou > max_iou)
			{
				for (int i = 0; i < 4;i++) {
					target_box[i] = xyxy[i];
					//cout << "box" << target_box[i];
			        }
				//cout << iou << endl;
				max_iou = iou;
				max_iou_matched = true;
			}

		}
		if (max_iou_matched == true)
		{
			// 如果找到了最大IOU BOX, 则认为该框为观测值
			//plot_one_box(target_box, frame, cv::Scalar(0, 0, 255), true);
			float	xywh[] = { 0, 0, 0, 0 };
			xyxy_to_xywh(target_box, xywh);
			float	box_center[] = { 0, 0 };
			box_center[0] = int((target_box[0] + target_box[2]) / 2);
			box_center[1] = int((target_box[1] + target_box[3]) / 2);
			trace_list = updata_trace_list(box_center, trace_list, 200);
			
			cv::putText(frame, "Tracking", cv::Point(int(target_box[0]), int(target_box[1] - 5)), cv::FONT_HERSHEY_SIMPLEX,
				0.7,cv::Scalar(255, 0, 0), 2);
			//		// 计算dx, dy
			float dx = xywh[0] - X_posterior[0][0];
			float dy = xywh[1] - X_posterior[1][0];

			//Z[0:4] = np.array([xywh]).T;
			for (int i = 0; i < 4;i++)
			{
				Z[i][0] = xywh[i];
			}
			//Z[4::] = np.array([dx, dy]);
			Z[4][0] = dx;
			Z[5][0] = dy;
			//matrixShow("Z",*Z, 6, 1);
		}
		float box_posterior[] = { 0,0,0,0 };
		if (max_iou_matched)
		{
			//-----进行先验估计----------

			//float	**X_prior,**P_prior_1, **P_prior;// = { 0 };			
			float X_prior[6][1] = { 0 };
			matrixShow("X_posterior1", *X_posterior, 6, 1);
			matrixMultiply(*A, *X_posterior, *X_prior, 6, 6, 1);
			matrixShow("X_prior",*X_prior,6,1);
			//# -----计算状态估计协方差矩阵P--------
			float P_prior_1[6][6] = { 0 };
			//P_prior_1 = MatrixMultiply(*A, *P_posterior,6,6,6);
			matrixMultiply(*A, *P_posterior, *P_prior_1, 6, 6, 6);
			//matrixShow("P_prior_1",*P_prior_1, 6, 6);
			float P_prior[6][6] = { 0 };
			matrixMultiply(*P_prior_1, *A_T, *P_prior, 6, 6, 6);
			//matrixShow("P_prior", *P_prior, 6, 6);
			matrixAdd(*P_prior, *Q, 6, 6);
			//matrixShow("P_prior", *P_prior, 6, 6);
			//## ------计算卡尔曼增益---------------------
			float k1[6][6] = {0},  k2_tem[6][6] = { 0 }, k2[6][6] = { 0 }, k[6][6] = { 0 };
			matrixMultiply(*P_prior, *H_T,*k1,6,6,6);
			//matrixShow("k1", *k1, 6, 6);
			matrixMultiply(*H, *P_prior, *k2_tem, 6, 6, 6);
			matrixMultiply(*k2_tem, *H_T, *k2, 6, 6, 6);
			matrixAdd(*k2, *R, 6, 6);
			//matrixShow("k2", *k2, 6, 6);
			float a_inverse[6][6] = { 0. };
			matrixInverse(*k2, *a_inverse, 6, 6);
			//matrixShow("a_inverse", *a_inverse, 6, 6);
			matrixMultiply(*k1, *a_inverse, *k, 6, 6, 6);
			//matrixShow("k", *k, 6, 6);
			//# --------------后验估计------------
			float X_posterior_1[6][1], P_posterior_1[6][6] = { 0 };// // X_posterior 需在这里重新定义了不然干扰乘积！！， X_posterior[6][1] = { 0 } 也不能重新定义，不然局部变量不能上传，需要置零,最后发现可以在matrixMultiply里面置零

			matrixMultiply(*H, *X_prior, *X_posterior_1, 6, 6, 1);
			matrixSub(*X_posterior_1, *Z, 6, 1);
			matrixMultiply(*k, *X_posterior_1, *X_posterior, 6, 6, 1);
			matrixShow("X_prior", *X_prior, 6, 1);
			matrixAdd(*X_posterior, *X_prior, 6, 1);
			matrixShow("X_posterior", *X_posterior, 6, 1);
			float tem2[] = { X_posterior[0][0] , X_posterior[1][0] , X_posterior[2][0] , X_posterior[3][0] };
			
			xywh_to_xyxy(tem2, box_posterior);
			plot_one_box(box_posterior, frame, color, false);
			//# ---------更新状态估计协方差矩阵P-----
			matrixMultiply(*k, *H, *P_posterior_1, 6, 6, 6);
			matrixSub(*P_posterior_1, *H, 6, 6);
			//matrixShow("P_posterior_1", *P_posterior_1, 6, 6);
			matrixMultiply(*P_posterior_1, *P_prior, *P_posterior, 6, 6, 6);
			//matrixShow("P_posterior", *P_posterior, 6, 6);
			//P_posterior_1 = np.eye(6) - np.dot(K, H)
			//	P_posterior = np.dot(P_posterior_1, P_prior)

		}
		else
		{
			//# 如果IOU匹配失败，此时失去观测值，那么直接使用上一次的最优估计作为先验估计
			//	# 此时直接迭代，不使用卡尔曼滤波
			float temp3[SIZE][1] = { 0 };
			matrixMultiply(*A, *X_posterior, *temp3, 6, 6, 1);
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 1; j++)
				{
					X_posterior[i][j] = temp3[i][j];  //这个与 float P_posterior[6][6] = { 0 } 截然不同！
				}
			}

			//X_posterior = temp3;
			//X_posterior = np.dot(A, X_posterior)
			//# X_posterior = np.dot(A_, X_posterior)
			float tem3[] = { X_posterior[0][0] , X_posterior[1][0] , X_posterior[2][0] , X_posterior[3][0] };
			xywh_to_xyxy(tem3 , box_posterior);
			plot_one_box(box_posterior, frame, cv::Scalar(0, 0, 255), false);
			float box_center[2] = { (int(box_posterior[0] + box_posterior[2]) / 2), int((box_posterior[1] + box_posterior[3]) / 2) };
			//trace_list = updata_trace_list(box_center, trace_list, 20)
			trace_list = updata_trace_list(box_center, trace_list, 20);
			putText(frame, "Lost", cv::Point(box_center[0], box_center[1] - 5), cv::FONT_HERSHEY_SIMPLEX, 0.7,
				(255, 0, 0), 2);

		}
		draw_trace(frame, trace_list);
		detectionFile.close();

		cv::imshow("track", frame);
		//cv::waitKey();
		frame_counter = frame_counter + 1;


		if (cv::waitKey(100) == 27)
			break;
	}

        cap.release();
	return 0;
}
