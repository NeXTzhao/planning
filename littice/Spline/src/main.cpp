/*
 * @Author: your name
 * @Date: 2021-09-23 14:38:13
 * @LastEditTime: 2021-09-25 15:03:29
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Spline/main.cpp
 */
#include <cstdio>
#include <iostream>
#include "Spline.h"

using namespace std;
using namespace SplineSpace;

int main()
{
	//?????????

	//double x0[2]={1,2};		//??????????
	//double y0[2]={3,4};
	//try
	//{
	//	SplineInterface* sp = new Spline(x0,y0,2);
	//}
	//catch(SplineFailure sf)
	//{
	//	cout<<sf.GetMessage()<<endl;
	//}
	//getchar();		//???????

	// ??????????

	double x0[5] = {1, 2, 4, 5, 6}; //??????????
	double y0[5] = {1, 3, 4, 2, 5};
	try
	{
		//Spline sp(x0,y0,5,GivenSecondOrder,0,0);
		SplineInterface *sp = new Spline(x0, y0, 5); //????????????????????
		double x = 4.5;
		double y;
		sp->SinglePointInterp(x, y); //??x???????y
		cout << "x=" << x << "value:" << y << endl;
	}
	catch (SplineFailure sf)
	{
		cout << sf.GetMessage() << endl;
	}
	getchar(); //???????

	//?????????

	// double x0[5]={1,2,4,5,6};		//??????????
	// double y0[5]={1,3,4,2,5};

	// double x[4] = {1.5,2.5,3.5,4.5};	//?????
	// double y[4];
	// double leftBound=0,RightBound=0;	//??��??

	// try
	// {
	// 	Spline sp(x0,y0,5,GivenSecondOrder,leftBound,RightBound);
	// 	sp.MultiPointInterp(x,4,y);			//??x???????y
	// 	for(int i = 0;i < 4;i++)
	// 	{
	// 		cout<<"x="<<x[i]<<"?????????:"<<y[i]<<endl;
	// 	}
	// }
	// catch(SplineFailure sf)
	// {
	// 	cout<<sf.GetMessage()<<endl;
	// }
	// getchar();	//???????

	// ??????????

	// double x0[5]={1,2,4,5,6};		//??????????
	// double y0[5]={1,3,4,2,5};

	// double x[10];	//?????
	// double y[10];

	// try
	// {
	// SplineInterface* sp = new Spline(x0,y0,5);	//????????????????????
	// sp->AutoInterp(10,x,y);			//??x???????y

	// for(int i = 0;i < 10;i++)
	// cout<<x[i]<<",";
	// cout<<endl;
	// for(int i = 0;i < 10;i++)
	// cout<<y[i]<<",";
	// }
	// catch(SplineFailure sf)
	// {
	// cout<<sf.GetMessage()<<endl;
	// }
	// getchar();	//???????
}