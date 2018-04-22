#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include <time.h>

#define NP 64//40 /* The number of colony size (employed bees+onlooker bees)  */
#define FoodNumber NP / 2 /*The number of food sources equals the half of the colony size  */
#define limit 64  /*A food source which could not be improved through "limit" trials is abandoned by its employed bee */
#define maxCycle 1000 /*The number of cycles for foraging {a stopping criteria}            */
#define D 2  
#define lb 0 /*lower bound of the parameters.  */
#define ub 100 /*upper bound of the parameters. lb and ub can be defined as arrays for the problems of which parameters have different bounds*/
#define LEFT 0
#define RIGHT 1
#define NumberOfArrayB 10																							
#define runtime 1                                  
#define experiments 500
#define MAP_point 16
#define MAP_pointA 36
#define MAP_pointB 484

#define MAP_point_xiaomi 16

#define MAP_center_agile_warehouse 4
#define MAP_point_agile_warehouse 20

#define MAP_upper_t_point 12

#define MAP_t_point 8

#define MAP_man_center 2
#define MAP_man_point 12
#define MAP_man_tri 3


#define MAP_length 20
#define MAP_width  2
#define M_PI       3.14159265358979323846   // pi																	  
#define Mosttries 30
#define collision 778.4271247462																						 
#define length    0.1																									 
#define cut_in_distance 10

//#define rockpile_circle_number 18
//double rockpile_circle[rockpile_circle_number][3] = { {10 ,90, 7}, {10, 65, 7}, {10 ,40 ,7}, {10 ,15, 7},      {30 ,80, 7}, {30 ,55, 7},{30, 30, 7},{50, 90, 7},
//{50, 65, 7},{50 , 40, 7},{50 ,15, 7}, {70, 80, 7},           {70 ,55, 7},{70, 30, 7},{90, 90, 7},{90, 65, 7}, {90, 40, 7},{90, 15, 7} };
//#define rockpile_circle_number 14
//double rockpile_circle[rockpile_circle_number][3] = { { 15 ,90, 5 },{ 40, 80, 5 },{ 70 ,85 ,6 },{ 80 ,70, 5 },{ 30 ,65, 6 },{ 40 ,70, 7 },{ 70, 60, 5 },{ 30, 40, 7 },
//{ 35, 45, 4 },{ 60 , 40, 6 },{ 80 ,35, 5 },{ 30, 20, 5 },{ 550 ,25, 6 },{ 80, 30, 7 }};

//#define agile_warehouse_circle_number 4
//#define agile_warehouse_rect_number 5
//double agile_warehouse_circle[agile_warehouse_circle_number][3] = { {30,30, 5},{30, 70, 5},{50,30 ,5}, {50,70, 5} };
//double agile_warehouse_rect[agile_warehouse_rect_number][4] = { {70, 10, 20, 5},{70 ,30, 20, 5},{70, 50, 20, 5},{70, 70, 20, 5},{70, 90, 20 ,5} };//x, y, length, width
 

//#define xiaomi_rect_number 5
//double xiaomi_rect[xiaomi_rect_number][4] = { {40, 20, 5, 30},{80, 20, 5, 45},{20,20, 5, 40},{60, 20, 5 , 45}, {20, 60, 40, 5} };

//#define t_rect_number 2
//double t_rect[t_rect_number][4] = { {50, 30, 4, 30},{30, 60, 40, 4}};

//#define upper_t_rect_number 2
//double upper_t_rect[upper_t_rect_number][4] = { {50, 30, 4, 30},{30, 60, 40, 4}};

//#define tri_wall_tri_number 15
//double tri_wall_tri[tri_wall_tri_number][6] = {
//	{6, 96, 16, 96, 11, 88},   {38, 96, 48, 96, 43, 88},    {70, 96, 80, 96, 75, 88},
//	{6, 76, 16, 76, 11, 68 },{ 38, 76, 48, 76, 43, 68 },{ 70, 76, 80, 76, 75, 68 },
//	{ 6, 56, 16, 56, 11, 48 },{ 38, 56, 48, 56, 43, 48 },{ 70, 56, 80, 56, 75, 48 },
//	{ 6, 36, 16, 36, 11, 28 },{ 38, 36, 48, 36, 43, 28 },{ 70, 36, 80, 36, 75, 28 },
//	{ 6, 16, 16, 16, 11, 8 },{ 38, 16, 48, 16, 43, 8 },{ 70, 16, 80, 16, 75, 8 },
//};

#define workspace7_circle_number 2
#define workspace7_rect_number 3
#define workspace7_tri_number 3
double workspace7_circle[workspace7_circle_number][3] = { { 15 ,40, 7 },{ 85, 40, 7 } };
double workspace7_tri[workspace7_tri_number][6] = { { 16, 90, 26, 66, 11, 80 } };
double workspace7_rect[workspace7_rect_number][4] = { { 20, 11, 60, 4 },{ 50, 40, 10, 20 },{ 40,75, 20, 4 } };

double ALL_COUPLE_POINTS[experiments][3][D];
int    CURRENT_LEFT_INDEX, CURRENT_RIGHT_INDEX;
double Foods[FoodNumber][D]; /*Foods is the population of food sources. Each row of Foods matrix is a vector holding D parameters to be optimized. The number of rows of Foods matrix equals to the FoodNumber*/
double f[FoodNumber];  /*f is a vector holding objective function values associated with food sources                 各食物源关于目标函数的属性 */
double fitness[FoodNumber]; /*fitness is a vector holding fitness (quality) values associated with food sources       各食物源关于目标函数的适应性*/
double trial[FoodNumber]; /*trial is a vector holding trial numbers through which solutions can not be improved       各食物源未被优化是的访问次数*/
double prob[FoodNumber]; /*prob is a vector holding probabilities of food sources (solutions) to be chosen            各食物源被跟随蜂选中的概率*/
double solution[D]; /*New solution (neighbour) produced by v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) j is a randomly chosen parameter and k is a randomlu chosen solution different from i依据随机策略生成的新的食物源*/
double total_length;
int collide[FoodNumber];//  存储个节点冲撞信息
double E_distance[FoodNumber];//存储个节点距离信息
double collide_local;
double E_distance_local;

double map[MAP_pointA][2] = { { 0,0 },{ 100,0 },{ 100,100 },{ 0,100 },
{ 12,0 },{ 22,0 },{ 22,20 },{ 12,20 },{ 12,50 },{ 22,50 },{ 22,100 },{ 12,100 },
{ 34,0 },{ 44,0 },{ 44,50 },{ 34,50 },{ 34,80 },{ 44,80 },{ 44,100 },{ 34,100 },
{ 56,0 },{ 66,0 },{ 66,20 },{ 56,20 },{ 56,50 },{ 66,50 },{ 66,100 },{ 56,100 },
{ 78,0 },{ 88,0 },{ 88,50 },{ 78,50 },{ 78,80 },{ 88,80 },{ 88,100 },{ 78,100 } };

//
//double all_obstacle_lines_xiaomi[20][2][2] = { { {40, 20}, {45, 20} },		{{40, 20}, {40, 50}},     {{45, 20}, {45, 50}},    {{40, 50}, {45, 50}}, 
//{{80, 20},{85, 20}},         {{80, 20},{80, 65}},           {{85, 20},{85, 65}},           {{80, 65},{85, 20}},
//{ {20, 20},{25, 20}},         {{20, 20},{20, 60}},           {{25, 20},{25, 60}},                      {{20, 60}, {25, 60}},
//{ {60, 20},{65, 20}},          {{60, 20},{60, 65}},         {{65, 20},{65, 65}},    {{60, 65}, {65, 65}}, 
//{{20, 60}, {60, 60}},         {{20, 60},{20, 65}},           {{60, 60},{60, 65}},           {{20, 65},{60, 65}} }
//};
//double all_obstacle_circle_xiaomi[] = {};
//double map[MAP_pointB][2] = { { 0,0 },{ 100,0 },{ 100,100 },{ 0,100 },
//{ 0,2 },{ MAP_length,2 },{ MAP_length,MAP_width + 2 },{ 0,MAP_width + 2 },{ 30,2 },{ 30 + MAP_length,2 },{ 30 + MAP_length, MAP_width + 2 },{ 30,MAP_width + 2 },{ 60,2 },{ 60 + MAP_length,2 },{ 60 + MAP_length, MAP_width + 2 },{ 60,MAP_width + 2 },
//{ 100 - MAP_length,11 },{ 100,11 },{ 100,11 + MAP_width },{ 100 - MAP_length,11 + MAP_width },{ 70 - MAP_length,11 },{ 70,11 },{ 70,11 + MAP_width },{ 70 - MAP_length,11 + MAP_width },{ 40 - MAP_length,11 },{ 40,11 },{ 40,11 + MAP_width },{ 40 - MAP_length,11 + MAP_width },
//{ 0,22 },{ MAP_length,22 },{ MAP_length,MAP_width + 22 },{ 0,MAP_width + 22 },{ 30,22 },{ 30 + MAP_length,22 },{ 30 + MAP_length, MAP_width + 22 },{ 30,MAP_width + 22 },{ 60,22 },{ 60 + MAP_length,22 },{ 60 + MAP_length, MAP_width + 22 },{ 60,MAP_width + 22 },
//{ 100 - MAP_length,33 },{ 100,33 },{ 100,33 + MAP_width },{ 100 - MAP_length,33 + MAP_width },{ 70 - MAP_length,33 },{ 70,33 },{ 70,33 + MAP_width },{ 70 - MAP_length,33 + MAP_width },{ 40 - MAP_length,33 },{ 40,33 },{ 40,33 + MAP_width },{ 40 - MAP_length,33 + MAP_width },
//{ 0,44 },{ MAP_length,44 },{ MAP_length,MAP_width + 44 },{ 0,MAP_width + 44 },{ 30,44 },{ 30 + MAP_length,44 },{ 30 + MAP_length, MAP_width + 44 },{ 30,MAP_width + 44 },{ 60,44 },{ 60 + MAP_length,44 },{ 60 + MAP_length, MAP_width + 44 },{ 60,MAP_width + 44 },
//{ 100 - MAP_length,55 },{ 100,55 },{ 100,55 + MAP_width },{ 100 - MAP_length,55 + MAP_width },{ 70 - MAP_length,55 },{ 70,55 },{ 70,55 + MAP_width },{ 70 - MAP_length,55 + MAP_width },{ 40 - MAP_length,55 },{ 40,55 },{ 40,55 + MAP_width },{ 40 - MAP_length,55 + MAP_width },
//{ 0,66 },{ MAP_length,66 },{ MAP_length,MAP_width + 66 },{ 0,MAP_width + 66 },{ 30,66 },{ 30 + MAP_length,66 },{ 30 + MAP_length, MAP_width + 66 },{ 30,MAP_width + 66 },{ 60,66 },{ 60 + MAP_length,66 },{ 60 + MAP_length, MAP_width + 66 },{ 60,MAP_width + 66 },
//{ 100 - MAP_length,77 },{ 100,77 },{ 100,77 + MAP_width },{ 100 - MAP_length,77 + MAP_width },{ 70 - MAP_length,77 },{ 70,77 },{ 70,77 + MAP_width },{ 70 - MAP_length,77 + MAP_width },{ 40 - MAP_length,77 },{ 40,77 },{ 40,77 + MAP_width },{ 40 - MAP_length,77 + MAP_width },
//{ 0,88 },{ MAP_length,88 },{ MAP_length,MAP_width + 88 },{ 0,MAP_width + 88 },{ 30,88 },{ 30 + MAP_length,88 },{ 30 + MAP_length, MAP_width + 88 },{ 30,MAP_width + 88 },{ 60,88 },{ 60 + MAP_length,88 },{ 60 + MAP_length, MAP_width + 88 },{ 60,MAP_width + 88 },
//{ 100 - MAP_length,97 },{ 100,97 },{ 100,97 + MAP_width },{ 100 - MAP_length,97 + MAP_width },{ 70 - MAP_length,97 },{ 70,97 },{ 70,97 + MAP_width },{ 70 - MAP_length,97 + MAP_width },{ 40 - MAP_length,97 },{ 40,97 },{ 40,97 + MAP_width },{ 40 - MAP_length,97 + MAP_width },
//};

double Left_StartandGoal[2][D];																						//存储起点和终点
double Right_StartandGoal[2][D];																																		//double StartandGoal[2][D] = { { 20, 90 },{ 90, 75 } };
double BestLeft[35][D];																									//用于存储选取到的最佳点
double BestRight[35][D];
double ObjValSol; /*Objective function value of new solution														  新食物源的f*/
double FitnessSol; /*Fitness value of new solution																	  新食物源的fitness*/
int neighbour, param2change; /*param2change corrresponds to j, neighbour corresponds to k in equation v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})相当于k和j*/
double GlobalMin; /*Optimum solution obtained by ABC algorithm                                                        某一全局最小值*/
double GlobalParams[D]; /*Parameters of the optimum solution                                                          某一全局最小值的描述参数*/
double GlobalMins[runtime]; /*GlobalMins holds the GlobalMin of each run in multiple runs                             全部全局最小值*/
double r; /*a random number in the range [0,1)                                                                        用于存储每次产生的随机数，此处要注意随机种子的更新*/


		  /*a function pointer returning double and taking a D-dimensional array as argument */
		  /*If your function takes additional arguments then change function pointer definition and lines calling "...=function(solution);" in the code*/
typedef double(*FunctionCallback)(double sol[D]);



double PathFunction(int index);

/*Write your own objective function name instead of sphere*/
FunctionCallback function = &PathFunction;                                         //函数选择
																				   /****************************************需要进行的修改**************************************/
																				   /*需要先刻画地图范围map[map_point]*/
																				   /*我要写个障碍安全判断函数linecollidefree, pointcollidefree*/
																				   /*增加一个距离计算函数EuclideanDistance*/
																				   /*增加一个数组存贮每一轮的节点信息和起点终点信息Best和startandgoal*/
																				   /*增加一个针对路径规划的综合评价函数PathFunction,而且要有对应的权值存储*/

																				   /*我还需要把D改了，适应新的评价体系*/
																				   /*再把引用函数改了*/
																				   /*最后还要把main函数的结构改了，因为要调用多次。*/


																				   /*a algorithm to judge whether a point can see a line is very useful, but should not be too complexity*/
																				   /*this idea wait for time......trash english*/
double EuclideanDistance(double sol[D], double goal_x, double goal_y)//计算某一点和终点的欧几里德距离
{
	double dis;
	dis = sqrt((sol[0] - goal_x) * (sol[0] - goal_x) + (sol[1] - goal_y) * (sol[1] - goal_y));
	return dis;
}

double multi(double x1, double y1, double x2, double y2, double x0, double y0)
{
	return (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
}

//double cross_point(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
//{
//	double x, y, a, b;
//	a = multi(x4, y4, x2, y2, x1, y1);
//	b = multi(x3, y3, x2, y2, x1, y1);
//	x = (a * x3 - b * x4) / (a - b);
//	y = (a * y3 - b * y4) / (a - b);
//}

int point_in_triangle_check(double point_x, double point_y, double vertexa_x, double vertexa_y, double vertexb_x, double vertexb_y, double vertexc_x, double vertexc_y)
{
	int i;
	double v0_x, v0_y, v1_x, v1_y, v2_x, v2_y, u, v;
	v0_x = vertexc_x - vertexa_x;
	v0_y = vertexc_y - vertexa_y;
	v1_x = vertexb_x - vertexa_x;
	v1_y = vertexb_y - vertexa_y;
	v2_x = point_x - vertexa_x;
	v2_y = point_y - vertexa_y;
	u = ((v1_x * v1_x + v1_y * v1_y) * (v2_x * v0_x + v2_y * v0_y) - (v1_x *v0_x + v1_y * v0_y)*(v2_x * v1_x + v2_y * v1_y)) / ((v0_x*v0_x + v0_y * v0_y)*(v1_x*v1_x + v1_y * v1_y) - (v0_x *v1_x + v0_y * v1_y)*(v0_x *v1_x + v0_y * v1_y));
	v = ((v0_x * v0_x + v0_y * v0_y) * (v2_x * v1_x + v2_y * v1_y) - (v1_x *v0_x + v1_y * v0_y)*(v2_x * v0_x + v2_y * v0_y)) / ((v0_x*v0_x + v0_y * v0_y)*(v1_x*v1_x + v1_y * v1_y) - (v0_x *v1_x + v0_y * v1_y)*(v0_x *v1_x + v0_y * v1_y));
	if (u >= 0)
	{
		if (v >= 0)
		{
			if ((u + v) <= 1)
				return 1;
		}
	}
	return 0;
}
int xiaomi_pointcollide(double x, double y)
{
	int i = 0, group;
	for (i = 0; i < workspace7_tri_number; i++)
	{
		if (point_in_triangle_check(x, y, workspace7_tri[i][0], workspace7_tri[i][1], workspace7_tri[i][2], workspace7_tri[i][3], workspace7_tri[i][4], workspace7_tri[i][5]))
			return 1;
	}
	for (i = 0; i < workspace7_circle_number; i++)
	{
		if (sqrt((workspace7_circle[i][0] - x) * (workspace7_circle[i][0] - x) + (workspace7_circle[i][1] - y) * (workspace7_circle[i][1] - y)) < workspace7_circle[i][2])
			return 1;
	}
	for (i = 0; i < workspace7_rect_number; i++)
	{
		if (x > workspace7_rect[i][0] && x < (workspace7_rect[i][0] + workspace7_rect[i][2]) && y > workspace7_rect[i][1] && y < (workspace7_rect[i][0] + workspace7_rect[i][3]))
			return 1;
	}
	if (x < 0 || y < 0 || x > 100 || y > 100)
	{
		printf("wrong nodes!!!!");
		return 1;
	}
	return 0;
}

int Pointcollide(double x, double y)//判断某一点是否在障碍区，若不在，返回0，若在，返回1
{
	int i, group;
	group = MAP_pointA / 4;
	for (i = 1; i <= group; i++)
	{
		if (x > map[4 * i][0] && x < map[4 * i + 1][0] && y > map[4 * i][1] && y < map[4 * i + 3][1])
			return 1;
	}
	if (x < map[0][0] || y < map[0][1] || x > map[1][0] || y > map[3][1])
	{
		printf("wrong nodes!!!!");
		return 1;
	}
	return 0;
}

int Linecollidefree(double sol[D], int last, int direction)//判断某一点和上一个最佳点之间是否有障碍。last为上一点的索引。1有障碍，0无障碍
{
	int number, j;
	double Best[2] = { 0 };
	double distance, distance_x, distance_y, unit_x, unit_y, x, y, danweimidu = 0.5;
	if (direction)
	{
		Best[0] = BestRight[last][0];
		Best[1] = BestRight[last][1];
	}
	else
	{
		Best[0] = BestLeft[last][0];
		Best[1] = BestLeft[last][1];
	}


	distance = EuclideanDistance(sol, Best[0], Best[1]);									//此处可能会有问题，注意
	number = abs((int)(distance / (double)danweimidu));

	distance_x = Best[0] - sol[0];
	distance_y = Best[1] - sol[1];
	unit_x = distance_x / (double)number;
	unit_y = distance_y / (double)number;
	x = Best[0];
	y = Best[1];

	for (j = 1; j < number; j++)
	{
		x = x - unit_x;
		y = y - unit_y;
		if (xiaomi_pointcollide(x, y))
		{
			return 1;
		}

	}
	return 0;
}

double distance_to_line(double p[], double line_s[], double line_e[])
{
	double face;
	if ((p[0] - line_s[0]) * (line_s[1] - line_e[1]) - (p[1] - line_s[1]) * (line_s[0] - line_e[0]) <= 0)
	{
		return EuclideanDistance(p, line_s[0], line_s[1]);
	}
	else if ((p[0] - line_e[0]) * (line_e[0] - line_s[0]) + (p[1] - line_e[1]) * (line_e[1] - line_s[1]) <= 0)
	{
		return EuclideanDistance(p, line_e[0], line_e[1]);
	}
	else
	{
		face = fabs((p[0] - line_e[0]) * (line_s[1] - line_e[1]) - (p[1] - line_s[1]) * (line_s[0] - line_e[0]));
		return face / EuclideanDistance(line_e, line_s[0], line_s[1]);
	}
}
/*check whether the two have cross or not, if cross ,return 1*/
int path_cross_check(double x1, double y1, double x2, double y2, double a1, double b1, double a2, double b2)
{
	double fc, fd, fa, fb;
	fc = (b1 - y1) * (x1 - x2) - (a1 - x1) * (y1 - y2);
	fd = (b2 - y1) * (x1 - x2) - (a2 - x1) * (y1 - y2);
	if ((fc * fd) < 0)
	{
		fa = (y1 - b1) * (a1 - a2) - (x1 - a1) * (b1 - b2);
		fb = (y2 - b1) * (a1 - a2) - (x2 - a1) * (b1 - b2);//？
		if ((fa * fb) < 0)
		{
			return 1;
		}
	}
	return 0;
}

/*check whether two paths have met. If so, return i+1, or 0 for not*/
int path_meet_check(double sol[D], int index, int direction)
{
	int i = 0;
	for (i = 0; i <= index; i++)
	{
		if (!Linecollidefree(sol, i, !direction))
		{
			return i + 1;
		}
	}
	return 0;
}
int path_end_check(void)
{
	if (EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]) < 1)
	{
		printf("%12f:%12f\n%12f:%12f\n", BestLeft[CURRENT_LEFT_INDEX][0], BestLeft[CURRENT_LEFT_INDEX][1], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]);
		return 1;
	}
	return 0;
}

int path_connect_check(double sol[D], int index, int direction)
{
	if (!Linecollidefree(sol, index, !direction))
	{
		return 1;
	}
	return 0;
}
/*Fitness function*/
double CalculateFitness(double fun)
{
	int k = 3;
	double result = 0;
	if (fun >= 0)
	{
		result = k * (1000 - fun + 100.0 / fun);//为什么要选这种计算方式？
	}
	else
	{
		result = 10 + fabs(fun);//fabs计算浮点数的绝对值
	}
	return result;
}

/*The best food source is memorized*/
void MemorizeBestSource()
{
	int i, j;
	for (i = 0; i<FoodNumber; i++)
	{
		if (f[i]<GlobalMin)
		{
			GlobalMin = f[i];
			for (j = 0; j<D; j++)
				GlobalParams[j] = Foods[i][j];
		}
	}
}

/*Variables are initialized in the range [lb,ub]. If each parameter has different range, use arrays lb[j], ub[j] instead of lb and ub */
/* Counters of food sources are also initialized in this function*/
void init(int index, int last, int direction)
{
	int j;
	for (j = 0; j<D; j++)
	{
		r = (double)rand() / ((double)(RAND_MAX)+(double)(1));
		Foods[index][j] = r*(ub - lb) + lb;
		solution[j] = Foods[index][j];
	}
	collide[index] = xiaomi_pointcollide(solution[0], solution[1]);
	if (collide[index])
	{
		trial[index] += limit;// date:12/2 to make sure that a totally wrong node will be delete.
	}
	else
	{
		collide[index] = Linecollidefree(solution, last, direction);
	}
	if (direction)
	{
		E_distance[index] = EuclideanDistance(solution, Right_StartandGoal[1][0], Right_StartandGoal[1][1]);
	}
	else
	{
		E_distance[index] = EuclideanDistance(solution, Left_StartandGoal[1][0], Left_StartandGoal[1][1]);
	}
	f[index] = (double)function(index);
	fitness[index] = CalculateFitness(f[index]);
	trial[index] = 0;
	//printf("index:%2d\n x:%8f\n y:%8f\ncollide:%2d\ndistance:%8f\nfun:%8f\nfit:%8f\n", index, solution[0], solution[1], collide[index], E_distance[index], f[index], fitness[index]);
}


void initial(int last, int direction)
{
	int i;
	for (i = 0; i<FoodNumber; i++)
	{
		init(i, last, direction);
	}
	GlobalMin = f[0];
	for (i = 0; i<D; i++)
		GlobalParams[i] = Foods[0][i];

}

void SendEmployedBees(int direction)
{
	int i, j, collide_local, solution_index = FoodNumber * 2;
	double E_distance_local;
	/*Employed Bee Phase*/
	for (i = 0; i<FoodNumber; i++)
	{
		/*The parameter to be changed is determined randomly*/
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		param2change = (int)(r*D);

																/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		neighbour = (int)(r*FoodNumber);

		/*Randomly selected solution must be different from the solution i*/
		while (neighbour == i)
		{
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			neighbour = (int)(r*FoodNumber);
		}
		for (j = 0; j<D; j++)
			solution[j] = Foods[i][j];

		/*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		solution[param2change] = fabs(Foods[i][param2change] + (Foods[i][param2change] - Foods[neighbour][param2change])*(r - 0.5) * 2);

	/*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
		if (solution[param2change] <= lb)
			solution[param2change] = lb + 0.01;
		if (solution[param2change] >= ub)
			solution[param2change] = solution[param2change] - ub;
		collide_local = xiaomi_pointcollide(solution[0], solution[1]);
		if (!collide_local)
		{
			collide_local = Linecollidefree(solution, 0, direction);
		}
		if (direction)
		{
			E_distance_local = EuclideanDistance(solution, Right_StartandGoal[1][0], Right_StartandGoal[1][1]);
		}
		else
		{
			E_distance_local = EuclideanDistance(solution, Left_StartandGoal[1][0], Left_StartandGoal[1][1]);
		}
		ObjValSol = (double)function(solution_index);//结合所有属性信息获得一个综合评价值
		FitnessSol = CalculateFitness(ObjValSol);//根据属性计算适应度

												 /*a greedy selection is applied between the current solution i and its mutant*/
		if (ObjValSol>f[i])//直接使用function的值判断，更符合最优解的记忆方式
		{
			/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
			trial[i] = 0;//选择新的蜜源，访问次数置零
			for (j = 0; j<D; j++)
				Foods[i][j] = solution[j];
			f[i] = ObjValSol;
			collide[i] = collide_local;
			E_distance[i] = E_distance_local;
			fitness[i] = FitnessSol;
		}
		else
		{
			/*if the solution i can not be improved, increase its trial counter*/
			trial[i] = trial[i] + 1;
		}
	}

	/*end of employed bee phase*/
}

/* A food source is chosen with the probability which is proportioal to its quality*/
/*Different schemes can be used to calculate the probability values*/
/*For example prob(i)=fitness(i)/sum(fitness)*/
/*or in a way used in the metot below prob(i)=a*fitness(i)/max(fitness)+b*/
/*probability values are calculated by using fitness values and normalized by dividing maximum fitness value*/
void CalculateProbabilities()//基于蜜源的质量计算优先概率
{
	int i;
	double maxfit, total_fitness = 0;
	maxfit = fitness[0];
	for (i = 1; i<FoodNumber; i++)//获取已存在的最高适应度
	{
		total_fitness = total_fitness + fitness[i];
		//if (fitness[i]>maxfit)
		//	maxfit = fitness[i];
	}
	for (i = 0; i<FoodNumber; i++)
	{
		prob[i] = fitness[i] / total_fitness;
		//prob[i] = (0.9*(fitness[i] / maxfit)) + 0.1;//把所有适应度放到0.1-1中去
	}

}

void SendOnlookerBees(int direction)//进行跟随蜜蜂的计算过程。
{

	int i, j, t, collide_local, times = 0;//times:try to limit the cycles!!!!!!!!!!!!!!!!!!!
	double E_distance_local;
	i = 0;
	t = 0;

	/*onlooker Bee Phase*/
	while ((t < FoodNumber) && (times < 3))//对所有的食物源进行概率性更新
	{

		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		r = r / 6.0; //加快进度
		if (r<prob[i]) /*choose a food source depending on its probability to be chosen*/
					   //只有优先级大于随机概率的蜜源才会被跟随蜜蜂接受。随机值是每执行一次都会改变的。
		{
			t++;
			//跟随蜜蜂选中蜜源之后便成为雇佣蜂，进行类似操作。
			/*The parameter to be changed is determined randomly*/
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			param2change = (int)(r * D);//

										/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			neighbour = (int)(r*FoodNumber);//k

											/*Randomly selected solution must be different from the solution i*/
			while (neighbour == i)
			{
				r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
				neighbour = (int)(r*FoodNumber);
			}
			for (j = 0; j<D; j++)
				solution[j] = Foods[i][j];

			/*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			solution[param2change] = Foods[i][param2change] + (Foods[i][param2change] - Foods[neighbour][param2change])*(r - 0.5) * 2;

			/*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
			if (solution[param2change]<lb)
				solution[param2change] = lb;
			if (solution[param2change]>ub)
				solution[param2change] = ub;
			collide_local = xiaomi_pointcollide(solution[0], solution[1]);
			if (!collide_local)
			{
				collide_local = Linecollidefree(solution, 0, direction);
			}
			if (direction)
			{
				E_distance_local = EuclideanDistance(solution, Right_StartandGoal[1][0], Right_StartandGoal[1][1]);
			}
			else
			{
				E_distance_local = EuclideanDistance(solution, Left_StartandGoal[1][0], Left_StartandGoal[1][1]);
			}
			/*a greedy selection is applied between the current solution i and its mutant*/
			if (ObjValSol>f[i])//直接使用function的值判断，更符合最优解的记忆方式
			{
				/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
				trial[i] = 0;
				for (j = 0; j<D; j++)
					Foods[i][j] = solution[j];
				f[i] = ObjValSol;
				collide[i] = collide_local;
				E_distance[i] = E_distance_local;
				fitness[i] = FitnessSol;
			}
			else
			{   /*if the solution i can not be improved, increase its trial counter*/
				trial[i] = trial[i] + 1;
			}
		} /*if */
		i++;
		if (i == FoodNumber) {
			i = 0;					//t只有被选中才会自增，而i每次都会自增，二者不同步。
			times++;
		}

	}/*while*/

	 /*end of onlooker bee phase     */
}
/*determine the food sources whose trial counter exceeds the "limit" value. In Basic ABC, only one scout is allowed to occur in each cycle*/
void SendScoutBees(int last, int direction)//判断是否要放弃蜜源
{
	int maxtrialindex, i;
	maxtrialindex = 0;
	for (i = 1; i<FoodNumber; i++)					//得出最大的访问次数的index
	{
		if (trial[i]>trial[maxtrialindex])
			maxtrialindex = i;
	}
	if (trial[maxtrialindex] >= limit)				//判断是否对最大访问次数的蜜源执行放弃操作，然后初始化。
	{
		init(maxtrialindex, last, direction);
	}
}


/*Main program of the ABC algorithm*/
void ABC(int last, int couple, int direction)
{
	int iter, run;
	//srand((unsigned)time(NULL) + couple * couple * couple - 2 * couple * couple + 10 * couple + 100);		//only以当前系统时间获取随机种子的操作,还是不完全随机
	for (run = 0; run < runtime; run++)
	{
		initial(last, direction);								
		MemorizeBestSource();						
		for (iter = 0; iter < maxCycle; iter++)
		{
			SendEmployedBees(direction);
			CalculateProbabilities();
			SendOnlookerBees(direction);
			MemorizeBestSource();
			SendScoutBees(last, direction);
		}
	}
}

void init_all_couple_points(void)
{
	int i;
	double point_x, point_y;
	for (i = 0; i < experiments; i++)
	{
		do {
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_x = r * (ub - lb);
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_y = r * (ub - lb);
		} while (xiaomi_pointcollide(point_x, point_y));
		ALL_COUPLE_POINTS[i][0][0] = point_x;
		ALL_COUPLE_POINTS[i][0][1] = point_y;
		do {
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_x = r * (ub - lb);
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_y = r * (ub - lb);
		} while (xiaomi_pointcollide(point_x, point_y));
		ALL_COUPLE_POINTS[i][1][0] = point_x;
		ALL_COUPLE_POINTS[i][1][1] = point_y;
	}
}

int main(void)
{
	int i, couple, j, k, flag, direction, common_point_index = 0, CURRENT_RIGHT_INDEX = 0, ii = 0, jj = 0, succeed_tries = 0;
	clock_t start_singal, finish_singal, start_double, finish_double;												    //记录时间
	double totaltime, local_length = 0;
	srand((unsigned)time(NULL));
	init_all_couple_points();

	total_length = 0;
	start_singal = clock();
	for (couple = 0; couple < experiments; couple++)
	{
		srand((unsigned)time(NULL) + couple + couple * couple);
		total_length += local_length;
		local_length = 0;
		CURRENT_LEFT_INDEX = 0;
		CURRENT_RIGHT_INDEX = 0;
		Right_StartandGoal[0][0] = ALL_COUPLE_POINTS[couple][1][0];
		Right_StartandGoal[0][1] = ALL_COUPLE_POINTS[couple][1][1];
		Right_StartandGoal[1][0] = ALL_COUPLE_POINTS[couple][0][0];
		Right_StartandGoal[1][1] = ALL_COUPLE_POINTS[couple][0][1];
		Left_StartandGoal[0][0] = ALL_COUPLE_POINTS[couple][0][0];
		Left_StartandGoal[0][1] = ALL_COUPLE_POINTS[couple][0][1];
		Left_StartandGoal[1][0] = ALL_COUPLE_POINTS[couple][1][0];
		Left_StartandGoal[1][1] = ALL_COUPLE_POINTS[couple][1][1];
		for (i = 0; i < D; i++)
		{
			BestLeft[CURRENT_LEFT_INDEX][i] = Left_StartandGoal[0][i];
			BestRight[CURRENT_RIGHT_INDEX][i] = Right_StartandGoal[0][i];
		}
		i = 0;
		for (k = 0; k < Mosttries; k++)
		{
			direction = LEFT;
			ABC(CURRENT_LEFT_INDEX, couple, direction);
			CURRENT_LEFT_INDEX++;
			for (j = 0; j < D; j++)
			{
				BestLeft[CURRENT_LEFT_INDEX][j] = GlobalParams[j];
				Right_StartandGoal[1][j] = GlobalParams[j];
			}
			flag = path_meet_check(BestLeft[CURRENT_LEFT_INDEX], CURRENT_RIGHT_INDEX, direction);
			if (flag)
			{
				for (ii = 0; ii < CURRENT_LEFT_INDEX; ii++)
				{
					local_length += EuclideanDistance(BestLeft[ii], BestLeft[ii + 1][0], BestLeft[ii + 1][1]);
				}
				for (jj = 0; jj <= common_point_index; jj++)
				{
					local_length += EuclideanDistance(BestRight[jj], BestRight[jj + 1][0], BestRight[jj + 1][1]);
				}
				local_length += EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]);
				succeed_tries++;
				printf("%d\n", couple);
				break;
			}
			flag = 0;
		}
	}
	printf("succeed_tries(singal):%d\n", succeed_tries);
	printf("ALL tries: %d\n", experiments);
	printf("total length(singal)%f:\n", total_length);
	finish_singal = clock();
	totaltime = (double)(finish_singal - start_singal) / CLOCKS_PER_SEC;
	printf("the total time(singal) is : %f\n", totaltime);


	succeed_tries = 0;
	total_length = 0;
	start_double = clock();
	for (couple = 0; couple < experiments; couple++)
	{
		total_length += local_length;
		local_length = 0;
		CURRENT_LEFT_INDEX = 0;
		CURRENT_RIGHT_INDEX = 0;
		Right_StartandGoal[0][0] = ALL_COUPLE_POINTS[couple][1][0];
		Right_StartandGoal[0][1] = ALL_COUPLE_POINTS[couple][1][1];
		Right_StartandGoal[1][0] = ALL_COUPLE_POINTS[couple][0][0];
		Right_StartandGoal[1][1] = ALL_COUPLE_POINTS[couple][0][1];
		Left_StartandGoal[0][0] = ALL_COUPLE_POINTS[couple][0][0];
		Left_StartandGoal[0][1] = ALL_COUPLE_POINTS[couple][0][1];
		Left_StartandGoal[1][0] = ALL_COUPLE_POINTS[couple][1][0];
		Left_StartandGoal[1][1] = ALL_COUPLE_POINTS[couple][1][1];
		for (i = 0; i < D; i++)
		{
			BestLeft[CURRENT_LEFT_INDEX][i] = Left_StartandGoal[0][i];
			BestRight[CURRENT_RIGHT_INDEX][i] = Right_StartandGoal[0][i];
		}
		i = 0;
		for (k = 0; k < Mosttries; k++)
		{
			direction = LEFT;
			ABC(CURRENT_LEFT_INDEX, couple, direction);
			CURRENT_LEFT_INDEX++;
			for (j = 0; j < D; j++)
			{
				BestLeft[CURRENT_LEFT_INDEX][j] = GlobalParams[j];
				Right_StartandGoal[1][j] = GlobalParams[j];
			}
			flag = path_meet_check(BestLeft[CURRENT_LEFT_INDEX], CURRENT_RIGHT_INDEX, direction);
			if (flag)
			{
				for (ii = 0; ii < CURRENT_LEFT_INDEX; ii++)
				{
					local_length += EuclideanDistance(BestLeft[ii], BestLeft[ii + 1][0], BestLeft[ii + 1][1]);
				}
				for (jj = 0; jj <= common_point_index; jj++)
				{
					local_length += EuclideanDistance(BestRight[jj], BestRight[jj + 1][0], BestRight[jj + 1][1]);
				}
				local_length += EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]);
				succeed_tries++;
				printf("%d\n", couple);
				break;
			}
			flag = 0;
			k++;
			direction = RIGHT;
			ABC(CURRENT_RIGHT_INDEX, couple, direction);
			CURRENT_RIGHT_INDEX++;
			for (j = 0; j < D; j++)
			{
				BestRight[CURRENT_RIGHT_INDEX][j] = GlobalParams[j];
				Left_StartandGoal[1][j] = GlobalParams[j];
			}
			flag = path_meet_check(BestRight[CURRENT_RIGHT_INDEX], CURRENT_LEFT_INDEX, direction);
			if (flag)
			{
				for (ii = 0; ii < CURRENT_LEFT_INDEX; ii++)
				{
					local_length += EuclideanDistance(BestLeft[ii], BestLeft[ii + 1][0], BestLeft[ii + 1][1]);
				}
				for (jj = 0; jj <= common_point_index; jj++)
				{
					local_length += EuclideanDistance(BestRight[jj], BestRight[jj + 1][0], BestRight[jj + 1][1]);
				}
				local_length += EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]);
				succeed_tries++;
				printf("%d\n", couple);
				break;
			}
			flag = 0;
		}
	}
	printf("succeed_tries(double):%d\n", succeed_tries);
	printf("ALL tries(double): %d\n", experiments);
	printf("total length(double)%f:\n", total_length);
	finish_double = clock();
	totaltime = (double)(finish_double - start_double) / CLOCKS_PER_SEC;
	printf("the total time(double) is : %f\n", totaltime);
	getchar();
}

double PathFunction(int index)//此处属性有er个
{
	double top = 0;
	if (index < FoodNumber)
	{
		top = top + collide[index] * collision + E_distance[index] * length;
	}
	else
	{
		top = top + collide_local * collision + E_distance_local * length;
	}
	return top;
}