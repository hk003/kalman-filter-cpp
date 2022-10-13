//#include "demo.hpp" 
#include "iostream"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#pragma comment (lib,"opencv_world450.lib")
using namespace std;
int xyxy_to_xywh(float(&xyxy)[4], float(&xywh)[4])
{
	float center_x = ((xyxy[0] + xyxy[2]) / 2.0);
	float intcenter_y = ((xyxy[1] + xyxy[3]) / 2.0);
	float w = xyxy[2] - xyxy[0];
	float h = xyxy[3] - xyxy[1];
	xywh[0] = center_x;
	xywh[1] = intcenter_y;
	xywh[2] = w;
	xywh[3] = h;
	//for (int i = 0; i < 4; ++i)
	//{
	//	cout << "xywh" << xywh[i] << endl;
	//}
	return 1;//(center_x, center_y, w, h)
}

int xywh_to_xyxy(float(&xywh)[4], float(&last_box_posterior)[4])
{
	float x1 = xywh[0] - floor(xywh[2] / 2.0);
	float	y1 = xywh[1] - floor(xywh[3] / 2.0);
	float	x2 = xywh[0] + floor(xywh[2] / 2.0);
	float y2 = xywh[1] + floor(xywh[3] / 2.0);
	//cout <<x1 << endl;
	last_box_posterior[0] = x1;
	last_box_posterior[1] = y1;
	last_box_posterior[2] = x2;
	last_box_posterior[3] = y2;
	return 1;
}

int plot_one_box(float xyxy[], cv::Mat img, cv::Scalar color = (0, 200, 0), bool target = false) {
	int xy1[] = { int(xyxy[0]), int(xyxy[1]) };
	int xy2[] = { int(xyxy[2]), int(xyxy[3]) };
	if (target == true)
	{
		color = cv::Scalar(0, 0, 255);
		//cv::rectangle(img, xy1, xy2, color, 1, cv::LINE_AA);// # filled
	}
	cv::rectangle(img, cv::Rect(xy1[0], xy1[1], xy2[0] - xy1[0], xy2[1] - xy1[1]), color, 1, cv::LINE_AA);
	return 0;
}

float getImageNameFromPath(string ImagePath, int start, int end) {     //string& 此從是c++的引用。
	string imageName;
	int length = 0;
	int length2 = 0;
	if (start >= end) {
		printf("length can not bigger than length2!!!");
		return -1;
	}
	int ready = 0;
	int num_ = 0;
	int sizeOfPath = ImagePath.length();
	for (int i = 0; i < sizeOfPath; i++)
	{
		if (ImagePath[i] == 32) // 92 //
		{
			num_++;
		}
		if (num_ == start && ready == 0)
		{
			length = i;
			ready = 1;
		}
		if (num_ == end)
		{
			length2 = i;
			break;
		}
	}

	imageName = ImagePath.substr(length + 1, length2 - 2); // ImagePath.length() - length 截取到結尾，最後沒有\\ ,所以不減1  
														   //ImageName = ImagePath.substr(length + 1, length2 - length - 4);    //最後有\\要減少1
	return atof(imageName.c_str());
	//return 0;
}
float cal_iou(float box1[], float box2[]) {
	/*
	: param box1 : xyxy 左上右下
	: param box2 : xyxy
	: return :

	*/
	float x1min, y1min, x1max, y1max;
	x1min = box1[0]; y1min = box1[1]; x1max = box1[2]; y1max = box1[3];
	float	x2min, y2min, x2max, y2max;
	x2min = box2[0], y2min = box2[1], x2max = box2[2], y2max = box2[3];
	//# 计算两个框的面积
	float	s1 = (y1max - y1min + 1.) * (x1max - x1min + 1.);
	float	s2 = (y2max - y2min + 1.) * (x2max - x2min + 1.);

	//# 计算相交部分的坐标
	float	xmin = max(x1min, x2min);
	float	ymin = max(y1min, y2min);
	float	xmax = min(x1max, x2max);
	float	ymax = min(y1max, y2max);
	//cout << xmin << " " << ymin << " " << xmax << " " << ymax << " " << endl;
	//float	inter_h = (std::max)(ymax - ymin + 1, 0);
	//float	inter_w = (std::max)(xmax - xmin + 1, 0);
	float a = 0;
	float	inter_h = (std::max)(ymax - ymin + 1, a);
	float	inter_w = (std::max)(xmax - xmin + 1, a);
	float	intersection = inter_h * inter_w;
	//cout << "inter_h " << inter_h << endl;
	//cout << "inter_w " << inter_w << endl;
	float   unionarea = s1 + s2 - intersection;

	//# 计算iou
	float iou = intersection / unionarea;
	return iou;
}


vector<float> updata_trace_list(float box_center[], vector<float> trace_list, int max_list_len = 50)
{
	if (trace_list.size() <= max_list_len)
	{
		trace_list.push_back(box_center[0]);
		trace_list.push_back(box_center[1]);

	}
	else
	{
		//trace_list.pop_back();
		//trace_list.pop_back();
		//trace_list.erase(trace_list.begin());
		trace_list.erase(trace_list.begin(), trace_list.begin()+2);
		trace_list.push_back(box_center[0]);
		trace_list.push_back(box_center[1]);

	}
	//for (int i = 0; i <trace_list.size(); i++)
	//{
	//	cout << "trace_list " << trace_list[i] << endl;

	//}
	return trace_list;
}

void draw_trace(cv::Mat img, vector<float> trace_list)
{
	/*
	更新trace_list, 绘制trace
	: param trace_list :
	: param max_list_len :
	: return :
	*/
	//for i, item in enumerate(trace_list) :
	for (int i = 0; i < trace_list.size() / 2 - 1; i++)
	{
		if (i < 2)
		{
			continue;
		}

		cv::line(img,
			cv::Point(trace_list[i * 2 + 2], trace_list[i * 2 + 3]), cv::Point(trace_list[i * 2 + 0], trace_list[i * 2 + 1]),
			cv::Scalar(255, 255, 0), 3);
	}
}


//矩阵相乘函数
void matrixMultiply(float a[], float b[], float c[], int n, int k, int m) {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			//c[i][j] = 0;
			c[i*m + j] = 0;
			for (int v = 0; v < k; v++) {
				//cout << "a : "<<a[i*k + v] << endl;
				c[i*m + j] += a[i*k + v] * b[v*m + j]; //+
			}
		}
	}
}
/*
矩阵的转置，created by hk
*/
void matrixTranspose(float a[], float b[], int out_row, int out_col) {
	for (int i = 0; i < out_row; i++) {
		for (int j = 0; j < out_col; j++) {
			b[i*out_col + j] = a[j*out_row + i];
		}
	}
}
//打印输出函数
void matrixShow(string matname, float c[], int n, int m) {
	cout << "输出矩阵" << matname << endl;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			cout << c[i*m + j] << ' ';
		}
		cout << endl;
	}
}

////矩阵加 返回和矩阵a
void matrixAdd(float a[], float b[], int n, int m) {
	//cout << "输出矩阵" << matname << endl;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			//cout << c[i][j] << ' ';
			a[i*m + j] = a[i*m + j] + b[i*m + j];
		}
		//cout << endl;
	}
}
////矩阵b-a, 返回和矩阵a
void matrixSub(float a[], float b[], int n, int m) {
	//cout << "输出矩阵" << matname << endl;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			//cout << c[i][j] << ' ';
			//a[i*m + j] = a[i*m + j] - b[i*m + j];
			a[i*m + j] = b[i*m + j] - a[i*m + j];
		}
		//cout << endl;
	}
}

