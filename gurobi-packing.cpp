#include "gurobi_c++.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
//輸入擺放的物品數量 太多會算不完
#define N 30
//最多棧板數量 太少會infeasible
#define K 10
//輸入物品有多少種
#define S 4
using namespace std;
string itos(int i)
{
	stringstream s;
	s << i;
	return s.str();
}
int main(int   argc,char *argv[])
{
	string s;
	//輸入名稱
	string item[S]={"SHELF","BRACKET","PLATE","料件"};
	//輸入物品大小
	int cl[S]={113,113,112,96};
	int cw[S]={113,113,111,90};
	int ch[S]={64,40,70,54};
	//輸入空間大小
	int length=114;
	int width=114;
	int height=210;
	int cd[S];
	int p[N];
	int q[N];
	int r[N];

	cout<<"輸入總量請勿超過"<<N<<"\n\n";
	int count=0;
	for(int i=0;i<S;i++){

		cout<<"輸入"<<item[i]<<"的數量"<<endl;
		cin>>cd[i];
		if(count+cd[i]>N){cout<<"數量太多了"<<endl;  system("pause"); return 0;}
		for(int j=0;j<cd[i];j++){
			p[count]=cl[i];
			q[count]=cw[i];
			r[count]=ch[i];
			count++;
		}
	}
	//剩餘的物品長寬高皆為0
	for(int i=count;i<N;i++){
			p[i]=0;
			q[i]=0;
			r[i]=0;
	}
	cout<<"正在開始計算"<<endl;

	int M=9999999;
  try {
    GRBEnv env = GRBEnv();			//創建一個GUROBI環境叫做 env

    GRBModel model = GRBModel(env);	//創建一個GUROBI模型叫做model

												// Create variables
	GRBVar x[N];
	GRBVar y[N];
	GRBVar z[N];
	GRBVar lx[N];
	GRBVar ly[N];
	GRBVar lz[N];
	GRBVar wx[N];
	GRBVar wy[N];
	GRBVar wz[N];
	GRBVar hx[N];
	GRBVar hy[N];
	GRBVar hz[N];
	GRBVar a[N][N];
	GRBVar b[N][N];
	GRBVar d[N][N];
	GRBVar e[N];
	GRBVar g[N][K];
	GRBVar pallet[K];


	for(int i=0;i<N;i++){
			s="x"+itos(i);
			x[i]=model.addVar(0, length, 0, GRB_INTEGER, s);
			s="y"+itos(i);
			y[i]=model.addVar(0, width, 0, GRB_INTEGER, s);
			s="z"+itos(i);
			z[i]=model.addVar(0, height, 0, GRB_INTEGER, s);


			s="lx"+itos(i);
			lx[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="ly"+itos(i);
			ly[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="lz"+itos(i);
			lz[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="wx"+itos(i);
			wx[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="wy"+itos(i);
			wy[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="wz"+itos(i);
			wz[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="hx"+itos(i);
			hx[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="hy"+itos(i);
			hy[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="hz"+itos(i);
			hz[i]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="e"+itos(i);
			e[i]=model.addVar(0,1,0.0,GRB_BINARY,s);

		for(int j=0;j<N;j++){
			s="a"+itos(i)+itos(j);
			a[i][j]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="b"+itos(i)+itos(j);
			b[i][j]=model.addVar(0,1,0.0,GRB_BINARY,s);
			s="d"+itos(i)+itos(j);
			d[i][j]=model.addVar(0,1,0.0,GRB_BINARY,s);
		}
	}
	for(int k=0;k<K;k++){
		for(int i=0;i<N;i++){
			s="g"+itos(i)+itos(k);
			g[i][k]=model.addVar(0,1,0.0,GRB_BINARY,s);
		}
			s="pallet"+itos(k);
			pallet[k]=model.addVar(0,1,0,GRB_BINARY, s);
	}

    model.update();
	GRBLinExpr sum=0;
	for(int k=0;k<K;k++)
	{sum+=(k+1)*pallet[k];}
																	 // 設定目標式
    model.setObjective(sum, GRB_MINIMIZE);


															// Add constraint
	for(int i=0;i<N;i++){
		for(int j=0;j<N;j++){
			if(i<j){
				model.addConstr(x[i]+p[i]*lx[i]+q[i]*wx[i]+r[i]*hx[i]<=x[j]+(1-a[i][j])*M,"c1");
				model.addConstr(x[j]+p[j]*lx[j]+q[j]*wx[j]+r[j]*hx[j]<=x[i]+(1-a[j][i])*M,"c2");
				model.addConstr(y[i]+p[i]*ly[i]+q[i]*wy[i]+r[i]*hy[i]<=y[j]+(1-b[i][j])*M,"c3");
				model.addConstr(y[j]+p[j]*ly[j]+q[j]*wy[j]+r[j]*hy[j]<=y[i]+(1-b[j][i])*M,"c4");

				model.addConstr(z[i]+p[i]*lz[i]+q[i]*wz[i]+r[i]*hz[i]<=z[j]+(1-d[i][j])*M,"c5");
				model.addConstr(z[i]+p[i]*lz[i]+q[i]*wz[i]+r[i]*hz[i]>=z[j]-(1-d[i][j])*M,"c5");
				model.addConstr(z[j]+p[j]*lz[j]+q[j]*wz[j]+r[j]*hz[j]<=z[i]+(1-d[j][i])*M,"c6");
				model.addConstr(z[j]+p[j]*lz[j]+q[j]*wz[j]+r[j]*hz[j]>=z[i]-(1-d[j][i])*M,"c6");
				model.addConstr(p[j]*q[j]*hz[j]+q[j]*r[j]*lz[j]+p[j]*r[j]*wz[j]<=p[i]*q[i]*hz[i]+q[i]*r[i]*lz[i]+p[i]*r[i]*wz[i]+(1-d[i][j])*M,"c22");
				model.addConstr(p[i]*q[i]*hz[i]+q[i]*r[i]*lz[i]+p[i]*r[i]*wz[i]<=p[j]*q[j]*hz[j]+q[j]*r[j]*lz[j]+p[j]*r[j]*wz[j]+(1-d[j][i])*M,"c23");

			}
			for(int k=0;k<K;k++){
				if(i<j){
					model.addConstr(a[i][j]+a[j][i]+b[i][j]+b[j][i]+d[i][j]+d[j][i]+1-g[i][k]+1-g[j][k]>=1,"c7");
				}
			}
		}
		model.addConstr(x[i]+p[i]*lx[i]+q[i]*wx[i]+r[i]*hx[i]<=length,"c8");
		model.addConstr(y[i]+p[i]*ly[i]+q[i]*wy[i]+r[i]*hy[i]<=width,"c9");
		model.addConstr(z[i]+p[i]*lz[i]+q[i]*wz[i]+r[i]*hz[i]<=height,"c10");
		model.addConstr(x[i]>=0,"c11");
		model.addConstr(y[i]>=0,"c12");
		model.addConstr(z[i]>=0,"c13");
		model.addConstr(lx[i]+ly[i]+lz[i]==1,"c14");
		model.addConstr(wx[i]+wy[i]+wz[i]==1,"c15");
		model.addConstr(hx[i]+hy[i]+hz[i]==1,"c16");
		model.addConstr(lx[i]+wx[i]+hx[i]==1,"c17");
		model.addConstr(ly[i]+wy[i]+hy[i]==1,"c18");
		model.addConstr(lz[i]+wz[i]+hz[i]==1,"c19");
		for(int k=0;k<K;k++){model.addConstr(g[i][k]<=pallet[k],"c21");}
	}
	for(int i=0;i<N;i++){
		sum=0;
		for(int k=0;k<K;k++){sum+=g[i][k];}
		model.addConstr(sum>=1,"20");}
	for(int k=0;k<K;k++){
		sum=0;
		for(int i=0;i<N;i++){sum+=p[i]*q[i]*r[i]*g[i][k];}
		model.addConstr(sum<=0.9*length*width*height,"c26");}

	for(int i=0;i<N;i++){
		sum=0;
		for(int j=0;j<N;j++){if(i!=j){sum+=d[j][i];}}
		model.addConstr(sum>=e[i],"c24");
		model.addConstr(sum<=M*e[i],"c24");
		model.addConstr(z[i]<=M*e[i],"c25");
		model.addConstr(z[i]>=e[i],"c25");
	}

																// Optimize model
	model.update();
    model.optimize();

	float percentage=0;
	int counter1=0;
	int counter2=0;
	for(int i=0;i<count;i++){
		for(int k=0;k<K;k++){
			if(g[i][k].get(GRB_DoubleAttr_X)==1){
		counter2++;
		percentage+=p[i]*q[i]*r[i];
		cout<<item[counter1]<<"在棧板"<<k+1<<" 座標(";
		cout<<x[i].get(GRB_DoubleAttr_X)<<",";
		cout<<y[i].get(GRB_DoubleAttr_X)<<",";
		cout<<z[i].get(GRB_DoubleAttr_X)<<")處";
		if(hz[i].get(GRB_DoubleAttr_X)==1){cout<<"橫躺";}
		else if(lz[i].get(GRB_DoubleAttr_X)==1){cout<<"直立";}
		else if(wz[i].get(GRB_DoubleAttr_X)==1){cout<<"側立";}
		cout<<endl;
		if(counter2==cd[counter1]){counter1++;counter2=0;}
			}
		}
	}
	int total_pallet=0;
	for(int k=0;k<K;k++){
		if(pallet[k].get(GRB_DoubleAttr_X)==1){total_pallet++;}
	}
	cout<<"共使用"<<total_pallet<<"個棧板"<<endl;
    cout << "棧板空間使用率" << percentage * 100 / total_pallet / (length* width * height) <<"%"<< endl;//列印最佳解的值
	//結束try
  }
	catch(GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;}
	catch(...) {cout << "Exception during optimization" << endl;}
  system("pause");
  return 0;
}
