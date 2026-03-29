#include<iostream>
void kalman(double& Zk,int Em)//Em是测量误差
{
	static double Xkl = 40.0;//估计值
	static double Eel = 5.0;//上一次估计误差//这里是初始
	
	double Kk = Eel / (Eel + Em);
	double Xk = Xkl + Kk * (Zk - Xkl);
	double Ee = (1 - Kk) * Eel;//更新的估计误差
	Zk = Xk;
	Xkl = Xk;
    Eel = Ee;
    double Kkl = Kk;
	
}

int main(int argc, char* argv[])
{
	double arr[16] = { 52.0,48.0,47.0,52.0,51.0,48.0,49.0,53.0,48.0,49.0,52.0,53.0,51.0,52.0,49.0,50.0 };
	double filtered[16];
	double x = 0.0;
	for (int i = 0; i < 16; ++i)
	{
		double Mes = arr[i];
		kalman(Mes, 3);
		filtered[i] = Mes;
	}
	for (int i = 0; i < 16; ++i)
	{
		std::cout << filtered[i] << " ";
	}

	return 0;
}