#include "Spline.h"
#include <cstring>
#include <algorithm>

using namespace std;

namespace SplineSpace
{

	//���캯��
	Spline::Spline(const double *x0, const double *y0, const int &num,
				   BoundaryCondition bc, const double &leftBoundary, const double &rightBoundary)
		: GivenX(x0), GivenY(y0), GivenNum(num), Bc(bc), LeftB(leftBoundary), RightB(rightBoundary)
	{
		if ((x0 == NULL) | (y0 == NULL) | (num < 3))
		{
			throw SplineFailure("参数错误");
		}
		PartialDerivative = new double[GivenNum]; //��ƫ������ռ�
		MaxX = *max_element(GivenX, GivenX + GivenNum);
		MinX = *min_element(GivenX, GivenX + GivenNum);
		if (Bc == GivenFirstOrder) //I�ͱ߽�����
			PartialDerivative1();
		else if (Bc == GivenSecondOrder) //II�ͱ߽�����
			PartialDerivative2();
		else
		{
			delete[] PartialDerivative;
			throw SplineFailure("参数错误");
		}
	}

	//I�ͱ߽�������ƫ��
	void Spline::PartialDerivative1(void)
	{
		//  ׷�Ϸ��ⷽ�������ƫ����
		double *a = new double[GivenNum]; //  a:ϡ��������±�һ����	
		double *b = new double[GivenNum]; //  b:ϡ��������м�һ����
		double *c = new double[GivenNum]; //  c:ϡ��������ϱ�һ����
		double *d = new double[GivenNum];

		double *f = new double[GivenNum];

		double *bt = new double[GivenNum];
		double *gm = new double[GivenNum];

		double *h = new double[GivenNum];

		for (int i = 0; i < GivenNum; i++)  //�洢�Խ�������
			b[i] = 2; //  �м�һ����Ϊ2
		for (int i = 0; i < GivenNum - 1; i++)	//�洢h
			h[i] = GivenX[i + 1] - GivenX[i]; // ���β���
		for (int i = 1; i < GivenNum - 1; i++)	//�洢 ��
			a[i] = h[i - 1] / (h[i - 1] + h[i]);
		a[GivenNum - 1] = 1;

		// ���㰴���ϵĹ�ʽ��
		c[0] = 1;
		for (int i = 1; i < GivenNum - 1; i++)
			c[i] = h[i] / (h[i - 1] + h[i]);

		for (int i = 0; i < GivenNum - 1; i++)
			f[i] = (GivenY[i + 1] - GivenY[i]) / (GivenX[i + 1] - GivenX[i]);

		d[0] = 6 * (f[0] - LeftB) / h[0];
		d[GivenNum - 1] = 6 * (RightB - f[GivenNum - 2]) / h[GivenNum - 2];

		for (int i = 1; i < GivenNum - 1; i++)
			d[i] = 6 * (f[i] - f[i - 1]) / (h[i - 1] + h[i]);

		bt[0] = c[0] / b[0]; //  ׷�Ϸ���ⷽ��
		for (int i = 1; i < GivenNum - 1; i++)
			bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

		gm[0] = d[0] / b[0];
		for (int i = 1; i <= GivenNum - 1; i++)
			gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

		PartialDerivative[GivenNum - 1] = gm[GivenNum - 1];
		for (int i = GivenNum - 2; i >= 0; i--)
			PartialDerivative[i] = gm[i] - bt[i] * PartialDerivative[i + 1];

		delete[] a;
		delete[] b;
		delete[] c;
		delete[] d;
		delete[] gm;
		delete[] bt;
		delete[] f;
		delete[] h;
	}

	//II�ͱ߽�������ƫ��
	void Spline::PartialDerivative2(void)
	{
		//  ׷�Ϸ��ⷽ�������ƫ����

		double *a = new double[GivenNum]; //  a:ϡ��������±�һ����
		double *b = new double[GivenNum]; //  b:ϡ��������м�һ����
		double *c = new double[GivenNum]; //  c:ϡ��������ϱ�һ����
		double *d = new double[GivenNum];

		double *f = new double[GivenNum];

		double *bt = new double[GivenNum];
		double *gm = new double[GivenNum];

		double *h = new double[GivenNum];

		for (int i = 0; i < GivenNum; i++)
			b[i] = 2;
		for (int i = 0; i < GivenNum - 1; i++)
			h[i] = GivenX[i + 1] - GivenX[i];
		for (int i = 1; i < GivenNum - 1; i++)
			a[i] = h[i - 1] / (h[i - 1] + h[i]);
		a[GivenNum - 1] = 1;

		c[0] = 1;
		for (int i = 1; i < GivenNum - 1; i++)
			c[i] = h[i] / (h[i - 1] + h[i]);

		for (int i = 0; i < GivenNum - 1; i++)
			f[i] = (GivenY[i + 1] - GivenY[i]) / (GivenX[i + 1] - GivenX[i]);

		for (int i = 1; i < GivenNum - 1; i++)
			d[i] = 6 * (f[i] - f[i - 1]) / (h[i - 1] + h[i]);

		d[1] = d[1] - a[1] * LeftB;
		d[GivenNum - 2] = d[GivenNum - 2] - c[GivenNum - 2] * RightB;

		bt[1] = c[1] / b[1];
		for (int i = 2; i < GivenNum - 2; i++)
			bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);                                                            
		gm[1] = d[1] / b[1];
		for (int i = 2; i <= GivenNum - 2; i++)
			gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

		PartialDerivative[GivenNum - 2] = gm[GivenNum - 2]; //
		for (int i = GivenNum - 3; i >= 1; i--)
			PartialDerivative[i] = gm[i] - bt[i] * PartialDerivative[i + 1];

		PartialDerivative[0] = LeftB;
		PartialDerivative[GivenNum - 1] = RightB;

		delete[] a;
		delete[] b;
		delete[] c;
		delete[] d;
		delete[] gm;
		delete[] bt;
		delete[] f;
		delete[] h;
	}

	//������ֵ��ʵ��
	bool Spline::SinglePointInterp(const double &x, double &y) throw(SplineFailure)
	{
		if ((x < MinX) | (x > MaxX))
			throw SplineFailure("��֧�����ֵ");
		int klo, khi, k;
		klo = 0;
		khi = GivenNum - 1;
		double hh, bb, aa;

		while (khi - klo > 1) //  ���ַ�����x���������  �ǵݹ��㷨
		{
			k = (khi + klo) >> 1;
			if (GivenX[k] > x)
				khi = k;
			else
				klo = k;
		}
		hh = GivenX[khi] - GivenX[klo];

		aa = (GivenX[khi] - x) / hh;
		bb = (x - GivenX[klo]) / hh;
		// ����s��x��
		y = aa * GivenY[klo] + bb * GivenY[khi] + ((aa * aa * aa - aa) * PartialDerivative[klo] + (bb * bb * bb - bb) * PartialDerivative[khi]) * hh * hh / 6.0;
		return true;
	}

	//�����ֵ��ʵ��
	bool Spline::MultiPointInterp(const double *x, const int &num, double *y) throw(SplineFailure)
	{
		for (int i = 0; i < num; i++)
		{
			SinglePointInterp(x[i], y[i]);
		}
		return true;
	}

	//�Զ������ֵ��ʵ��
	bool Spline::AutoInterp(const int &num, double *x, double *y) throw(SplineFailure)
	{
		if (num < 2)
			throw SplineFailure("����Ҫ���������");
		double perStep = (MaxX - MinX) / (num - 1);

		for (int i = 0; i < num; i++)
		{
			x[i] = MinX + i * perStep;
			SinglePointInterp(x[i], y[i]);
		}
		return true;
	}

	Spline::~Spline()
	{
		delete[] PartialDerivative;
	}

	//�쳣��ʵ��
	SplineFailure::SplineFailure(const char *msg) : Message(msg){};
	const char *SplineFailure::GetMessage() { return Message; }

}