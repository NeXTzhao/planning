#include <iostream>
#include <cstdio>
#include <graphics.h>
#include <cmath>
using namespace std;
class Point//简单写的一个Point的数据结构
{
public:
	int x, y;
	Point(int x1, int y1)
	{
		x = x1;
		y = y1;
	}
	Point()
	{
		x = 0;
		y = 0;
	}
	void Set(int x1, int y1)
	{
		x = x1;
		y = y1;
	}
};
int main()
{
	initgraph(1000, 1000);
	//简单画出一个六边形
	line(250, 50, 750, 50);
	line(750, 50, 1000, 300);
	line(1000, 300, 750, 550);
	line(750, 550, 250, 550);
	line(250, 550, 0, 300);
	line(0, 300, 250, 50);
	
	//把相应点初始化
	Point P[7];
	P[0].Set(250, 50);
	P[1].Set(750, 50);
	P[2].Set(1000, 300);
	P[3].Set(750, 550);
	P[4].Set(250, 550);
	P[5].Set(0,300);
	P[6].Set(250, 50);
	int n = 4;//P[0]-P[n-1]参与绘制Bezier曲线
	Point PO;
	double t;
	for (t = 0; t < 1; t += 0.000001)
	{
		//每一次运算都要重新赋一次初始值
		P[0].Set(250, 50);
		P[1].Set(750, 50);
		P[2].Set(1000, 300);
		P[3].Set(750, 550);
		//参照n的值，一直赋初始值到P[n-1]
		
		int x = n;
		while (1)
		{
			if (x == 1)
				break;
			for (int i = 0; i < x - 1; i++)
			{
				PO.x = ((1 - t) * P[i].x + t * P[i + 1].x);
				PO.y = ((1 - t) * P[i].y + t * P[i + 1].y);
				P[i] = PO;
			}
			x--;
		}
		putpixel(P[0].x, P[0].y, RED);
		//计算出比例系数为t时对应的Bezier曲线上的点，画出该点。
	}
	getchar();
}
