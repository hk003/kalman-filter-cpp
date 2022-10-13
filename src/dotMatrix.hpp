//��������������������������������
//��Ȩ����������ΪCSDN�������Ӿ�С��ѽ����ԭ�����£���ѭCC 4.0 BY - SA��ȨЭ�飬ת���븽��ԭ�ĳ������Ӽ���������
//ԭ�����ӣ�https ://blog.csdn.net/qq_32146861/article/details/125777849

#include<iostream>
#include <cmath>
#define N 10
using namespace std;
class Mat
{
public:
	int m = 1, n = 1; //����������
	double mat[N][N] = { 0 };  //����ʼ��Ԫ��

	Mat() {}
	Mat(int mm, int nn)
	{
		m = mm; n = nn;
	}
	void create();//��������
	void Print();//��ӡ����
	void eye();//����������±���ͬʱֵΪ 1����ͬʱΪ 0
	void matrixInverse(float a[], float b[], int n, int m);
	bool inv(Mat a);//��������
};

void Mat::create()
{
	for (int i = 1; i <= m; i++)
	{
		for (int j = 1; j <= n; j++)
		{
			cin >> mat[i][j];
		}
	}
}
void Mat::Print()
{
	for (int i = 1; i <= m; i++)
	{
		for (int j = 1; j <= n; j++)
		{
			cout << mat[i][j] << "\t";
		}
		cout << endl;
	}
}
void Mat::eye()
{
	for (int i = 1; i <= m; i++)
	{
		for (int j = 1; j <= n; j++)
		{
			if (i == j)
				mat[i][j] = 1;
			else
				mat[i][j] = 0;
		}
	}
}
bool Mat::inv(Mat a)
{
	if (a.n != a.m)
	{
		cout <<"hk"<< "�����棡" << endl;
		return false;
	}
	m = a.m; n = a.n;
	eye(); //������λ����

		   //�����������϶��µĳ����б任��ʹ�þ��� a.mat ��ɵ�λ�����Ǿ���
	for (int i = 1; i <= m; i++) //ע������Ҫ i<=m����֮ǰ�������Ǿ����в�ͬ
	{                         //��ΪҪ�ж����һ�л�Ϊ�����Ǿ�������һ�����һ��Ԫ���Ƿ�Ϊ 0
							  //Ѱ�ҵ� i �в�Ϊ���Ԫ��
		int k;
		for (k = i; k <= m; k++)
		{
			if (fabs(a.mat[k][i]) > 1e-10) //�����������ʱ����Ϊ���Ԫ�ز�Ϊ0
				break;
		}
		if (k <= m)//˵���� i ���в�Ϊ0��Ԫ��
		{
			if (k != i)//˵���� i �� �� i ��Ԫ��Ϊ�㣬��Ҫ�������н���
			{
				//������ i �к͵� k ������Ԫ��
				for (int j = 1; j <= n; j++)//��ӵ�һ��Ԫ�ؽ�����ע����֮ǰ�������Ǿ���ͬ
				{//ʹ��mat[0][j]��Ϊ�м��������Ԫ��,��������Ҫ����
					a.mat[0][j] = a.mat[i][j]; a.mat[i][j] = a.mat[k][j]; a.mat[k][j] = a.mat[0][j];
					mat[0][j] = mat[i][j]; mat[i][j] = mat[k][j]; mat[k][j] = mat[0][j];
				}
			}
			double b = a.mat[i][i];//����
								   //������ a.mat �����Խ���Ԫ�ػ�Ϊ 1
			for (int j = 1; j <= n; j++)//�ӵ�һ��Ԫ�ؿ�ʼ
			{
				a.mat[i][j] /= b;
				mat[i][j] /= b;
			}
			for (int j = i + 1; j <= m; j++)
			{
				//ע�Ȿ��Ϊ -a.mat[j][i]/a.mat[i][i],��Ϊa.mat[i][i]���� 1������Ҫ����
				b = -a.mat[j][i];
				for (k = 1; k <= n; k++)
				{
					a.mat[j][k] += b * a.mat[i][k];//�� i �� b ���ӵ��� j ��
					mat[j][k] += b * mat[i][k];
				}
			}
		}
		else
		{
			cout << "last" << "�����棡" << endl;
			return false;
		}
	}

	//����������¶��ϵ��б任���� a.mat ����Ϊ��λ����
	for (int i = m; i > 1; i--)
	{
		for (int j = i - 1; j >= 1; j--)
		{
			double b = -a.mat[j][i];
			a.mat[j][i] = 0; //ʵ������ͨ�������б任�����Ԫ�ػ�Ϊ 0,
			for (int k = 1; k <= n; k++)
			{//ͨ����ͬ�ĳ����б任���任�ұ߾���
				mat[j][k] += b * mat[i][k];
			}
		}
	}
	return true;
}
void matrixInverse(float x[], float a_inverse[], int n,  int m) {
	int size = 6;
	Mat a(size, size);

	for (int i = 1; i <= size; i++)
	{
		for (int j = 1; j <= size; j++) {
			a.mat[i][j] = x[(i - 1)*6+j - 1];
			//cout << (a.mat[i][j]) << endl;
		}

	}

	Mat b;
	//if (b.inv(a))
	//	b.Print();
	b.inv(a);
	for (int i = 1; i <= 6; i++)
	{
		for (int j = 1; j <= 6; j++) {
			//a.mat[i][j] = tem[i][j];
			a_inverse[(i - 1) * 6 + j - 1] = b.mat[i][j];
			//cout << a_inverse[(i - 1) * 6 + j - 1] << " ";
		}
		cout << endl;
	}
}