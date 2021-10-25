1. eigen安装见csdn，安装完成之后要使用，需添加头文件#include <Eigen/Eigen>

2. 
    Eigen通过tyepdef定义了许多内置类型，不过底层仍然是Eigen::Matrix,如下所示：
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero():     //初始化为0       //Matrix3d实质上是Eigen::Matrix<double, 3, 3> 
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;     //如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::MatrixXd matrix_xd;                                                //与上类似，表示任意大小的元素类型为double的矩阵变量
    Eigen::MatrixXf matrix_xf;                                                //表示任意大小的元素类型为float的矩阵变量
    Eigen::Vector3d v_3d;                                                     //Vector3d实质上是Eigen::Matrix<double, 3, 1>   三行一列

3. 
    3.1 加减乘除跟数字操作一样
    3.2 赋值操作:
        matrix_33<<1,2,3,4,5,6,7,8,9;
        或 matrix_33(0,0)=1;
    3.3
    cout << matrix_33.transpose() << endl;    //转置
	cout << matrix_33.sum() << endl;          //各元素和
	cout << matrix_33.trace() << endl;        //迹
	cout << matrix_33 * 10 << endl;           //数乘
	cout << matrix_33.inverse() << endl;      //逆
	cout << matrix_33.determinant() << endl;  //行列式
    cout << matrix_33.adjoint() << endl;      //伴随
    cout << matrix_33.cwiseAbs() << endl;     //绝对值
    cout << matrix_33.maxCoeff() << endl;     //求最大系数

    3.4
    Eigen::Vector2d C = A.colPivHouseholderQr().solve(b); // 求解Ax=b ; Vector2d => Matrix<double,2, 1>
    cwiseAbs()求绝对值、maxcoff()求最大系数

    3.5 设置单位矩阵
        Eigen::MatrixXd::Identity(4, 4)  
        或
        Eigen::Matrix<double ,4,4> A;
        A.setIdentity(4, 4);