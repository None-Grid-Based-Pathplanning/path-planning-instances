#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include <time.h>
#include<string.h>

/*the paramenters of PSO algorithm*/
#define true 1
#define false 0
#define PARAMETER_NUMBER 4//x,y,velocity_x,velocity_y
#define PARTICLE_NUMBER 50
#define V_MAX 2
#define V_MAX_OPPSITE (-2)
#define EPOCH_NUMBER 100
#define RANGE_LOW 0
#define RANGE_UP 100
#define LEFT 0
#define RIGHT 1
#define FLAG 2
#define PATH_END_TYPE0 0
#define PATH_END_TYPE_DISTANCE 4
#define PATH_END_TYPE1 1
#define PATH_END_TYPE2 2
#define POSITION_X 0
#define POSITION_Y 1
#define POSITION_VX 2
#define POSITION_VY 3
#define EXPERIMENT_COUPLES 500
#define EXPERIMENT_TIMES 1//every couple of popints been tried experiment_times times
#define MAP_point 16
#define MAP_pointA 36
#define MAP_pointB 484
#define MAP_length 20
#define MAP_width  2
#define M_PI       3.14159265358979323846
#define Mosttries 30//must be smaller than Bestleft or right nodes numbers!!!
#define collision 778.4271247462																						 
#define length    1
#define length_quality 1
#define collision_quality 789
#define differential_quality 1//discrible the difference between the verticle and distance, if differ too much, the point quality should bee very low.

//#define rockpile_circle_number 18
//double rockpile_circle[rockpile_circle_number][3] = { { 10 ,90, 7 },{ 10, 65, 7 },{ 10 ,40 ,7 },{ 10 ,15, 7 },{ 30 ,80, 7 },{ 30 ,55, 7 },{ 30, 30, 7 },{ 50, 90, 7 },
//{ 50, 65, 7 },{ 50 , 40, 7 },{ 50 ,15, 7 },{ 70, 80, 7 },{ 70 ,55, 7 },{ 70, 30, 7 },{ 90, 90, 7 },{ 90, 65, 7 },{ 90, 40, 7 },{ 90, 15, 7 } };
//#define agile_warehouse_circle_number 4
//#define agile_warehouse_rect_number 5
//double agile_warehouse_circle[agile_warehouse_circle_number][3] = { { 30,30, 5 },{ 30, 70, 5 },{ 50,30 ,5 },{ 50,70, 5 } };
//double agile_warehouse_rect[agile_warehouse_rect_number][4] = { { 70, 10, 20, 5 },{ 70 ,30, 20, 5 },{ 70, 50, 20, 5 },{ 70, 70, 20, 5 },{ 70, 90, 20 ,5 } };//x, y, length, width
//#define xiaomi_rect_number 5
//double xiaomi_rect[xiaomi_rect_number][4] = { { 40, 20, 5, 30 },{ 80, 20, 5, 45 },{ 20,20, 5, 40 },{ 60, 20, 5 , 45 },{ 20, 60, 40, 5 } };
//#define t_rect_number 2
//double t_rect[t_rect_number][4] = { { 50, 30, 4, 30 },{ 30, 60, 40, 4 } };
//#define upper_t_rect_number 2
//double upper_t_rect[upper_t_rect_number][4] = { { 50, 30, 4, 30 },{ 30, 60, 40, 4 } };
//#define tri_wall_tri_number 15
//double tri_wall_tri[tri_wall_tri_number][6] = {
//	{ 6, 96, 16, 96, 11, 88 },{ 38, 96, 48, 96, 43, 88 },{ 70, 96, 80, 96, 75, 88 },
//{ 6, 76, 16, 76, 11, 68 },{ 38, 76, 48, 76, 43, 68 },{ 70, 76, 80, 76, 75, 68 },
//{ 6, 56, 16, 56, 11, 48 },{ 38, 56, 48, 56, 43, 48 },{ 70, 56, 80, 56, 75, 48 },
//{ 6, 36, 16, 36, 11, 28 },{ 38, 36, 48, 36, 43, 28 },{ 70, 36, 80, 36, 75, 28 },
//{ 6, 16, 16, 16, 11, 8 },{ 38, 16, 48, 16, 43, 8 },{ 70, 16, 80, 16, 75, 8 },
//};
//#define workspace4_circle_number 10
//double workspace4_circle[workspace4_circle_number][3] = { { 24 ,63, 7 },{ 24, 15, 6 },{ 60 ,40 ,7 },{ 37 ,42, 5 },{ 40 ,82, 7 },{ 80 ,20, 7 },{ 80, 50, 6 },{ 85, 75, 5 },
//{ 70, 60, 5 },{ 85 , 40, 5 } };
//#define workspace4_tri_number 4
//double workspace4_tri[workspace4_tri_number][6] = {
//	{ 14, 47, 13, 33, 26, 41 },{ 47, 57, 44, 68, 59, 75 },{ 40, 7, 51, 8, 58, 21 },
//{ 63, 89, 66, 80, 80, 90 }
//};
#define workspace7_circle_number 2
#define workspace7_rect_number 3
#define workspace7_tri_number 3
double workspace7_circle[workspace7_circle_number][3] = { { 15 ,40, 7 },{ 85, 40, 7 } };
double workspace7_tri[workspace7_tri_number][6] = { { 16, 90, 26, 66, 11, 80 } };
double workspace7_rect[workspace7_rect_number][4] = { { 20, 11, 60, 4 },{ 50, 40, 10, 20 },{ 40,75, 20, 4 } };

double ALL_COUPLE_POINTS[EXPERIMENT_COUPLES][3][2];//START, END, FLAG(SINGAL_SUCCESS, DOUBLE_SUCCESS)
double Left_StartandGoal[2][PARAMETER_NUMBER - 2] = { { 10, 10 },{ 90, 90 } };																					
double Right_StartandGoal[2][PARAMETER_NUMBER - 2] = { { 90, 90 } ,{ 10, 10 } };
int    CURRENT_LEFT_INDEX;
double BestLeft[35][PARAMETER_NUMBER - 2];		
int    CURRENT_RIGHT_INDEX;
double BestRight[35][PARAMETER_NUMBER - 2];
double endx, endy;//the two dynamic parameter will be used frequently, thus should not define in every function;commmon for left and right
double DIRECTION; //the direction should be changed when changing line and need not to be pass between functions

double PARTICLE[PARTICLE_NUMBER][PARAMETER_NUMBER];//X, Y, V, BEST_value not current value. and the global best value should be one of them.
double PARTICLE_BEST[PARTICLE_NUMBER][PARAMETER_NUMBER];//store the best personal state,the global_best_index should be used here. 
double PARTICLE_COLLIDE[PARTICLE_NUMBER];
double PARTICLE_BEST_COLLIDE[PARTICLE_NUMBER];
double PSO_W; //range from 0.9 to 0.7
double PSO_W_UNIT;
double r;
//double map[MAP_point][2] = { { 0,0 },{ 100,0 },{ 100,100 },{ 0,100 },{ 15,0 },{ 25,0 },{ 25,50 },{ 15,50 },{ 40,50 },{ 50,50 },{ 50,100 },{ 40,100 },{65,0},{75,0},{75,50},{65,50} };
//double map[MAP_pointA][2] = { { 0,0 },{ 100,0 },{ 100,100 },{ 0,100 },
//{ 18,0 },{ 20.5,0 },{ 20.5,20 },{ 18,20 },{ 18,50 },{ 20.5,50 },{ 20.5,100 },{ 18,100 },
//{ 38.5,0 },{ 41,0 },{ 41,50 },{ 38.5,50 },{ 38.5,80 },{ 41,80 },{ 41,100 },{ 38.5,100 },
//{ 59,0 },{ 61.5,0 },{ 61.5,20 },{ 59,20 },{ 59,50 },{ 61.5,50 },{ 61.5,100 },{ 59,100 },
//{ 79.5,0 },{ 82,0 },{ 82,50 },{ 79.5,50 },{ 79.5,80 },{ 82,80 },{ 82,100 },{ 79.5,100 }};
double map[MAP_pointA][2] = { { 0,0 },{ 100,0 },{ 100,100 },{ 0,100 },
{ 12,0 },{ 22,0 },{ 22,20 },{ 12,20 },{ 12,50 },{ 22,50 },{ 22,100 },{ 12,100 },
{ 34,0 },{ 44,0 },{ 44,50 },{ 34,50 },{ 34,80 },{ 44,80 },{ 44,100 },{ 34,100 },
{ 56,0 },{ 66,0 },{ 66,20 },{ 56,20 },{ 56,50 },{ 66,50 },{ 66,100 },{ 56,100 },
{ 78,0 },{ 88,0 },{ 88,50 },{ 78,50 },{ 78,80 },{ 88,80 },{ 88,100 },{ 78,100 } };

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


int point_in_triangle_check(double point_x, double point_y, double vertexa_x, double vertexa_y, double vertexb_x, double vertexb_y, double vertexc_x, double vertexc_y)
{
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
	int i = 0;
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
	//for (i = 0; i < upper_t_rect_number; i++)
	//{
	//	if (x > upper_t_rect[i][0] && x < (upper_t_rect[i][0] + upper_t_rect[i][2]) && y > upper_t_rect[i][1] && y < (upper_t_rect[i][0] + upper_t_rect[i][3]))
	//		return 1;
	//}
	if (x < -10 || y < -10 || x > 110 || y > 110)
	{
		printf("wrong nodes!!!!");
		return 1;
	}
	return 0;
}

void init_all_couple_points(void)
{
	int i;
	double point_x, point_y;
	for (i = 0; i < EXPERIMENT_COUPLES; i++)
	{
		do {
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_x = r * (RANGE_UP - RANGE_LOW);
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_y = r * (RANGE_UP - RANGE_LOW);
		} while (xiaomi_pointcollide(point_x, point_y));
		ALL_COUPLE_POINTS[i][0][0] = point_x;
		ALL_COUPLE_POINTS[i][0][1] = point_y;
		do {
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_x = r * (RANGE_UP - RANGE_LOW);
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			point_y = r * (RANGE_UP - RANGE_LOW);
		} while (xiaomi_pointcollide(point_x, point_y));
		ALL_COUPLE_POINTS[i][1][0] = point_x;
		ALL_COUPLE_POINTS[i][1][1] = point_y;
	}
}

double EuclideanDistance(double point[PARAMETER_NUMBER], double goal_x, double goal_y)//计算某一点和终点的欧几里德距离
{
	double dis;
	dis = sqrt((point[POSITION_X] - goal_x) * (point[POSITION_X] - goal_x) + (point[POSITION_Y] - goal_y) * (point[POSITION_Y] - goal_y));
	return dis;
}

double multi(double x1, double y1, double x2, double y2, double x0, double y0)
{
	return (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
}



int pointcollide(double x, double y)//判断某一点是否在障碍区，若不在，返回0，若在，返回1
{
	int i, group;
	group = MAP_pointA / 4;
	//map[MAP_point][2] = { { 0,0 },{ 100,0 },{ 100,100 },{ 0,100 },{ 20,0 },{ 40,0 },{ 40,55 },{ 20,55 },{ 60,45 },{ 80,45 },{ 80,100 },{ 60,100 } };
	for (i = 1; i <= group; i++)
	{
		if (x > map[4 * i][0] && x < map[4 * i + 1][0] && y > map[4 * i][1] && y < map[4 * i + 3][1])
			return 1;
	}
	if (x < (map[0][0] - 10) || y < (map[0][1] - 10) || x > (map[1][0] + 10) || y > (map[3][1] + 10))
	{
		printf("outside of the big map!!!!\n");
		return 1;
	}
	if (x < map[0][0] || y < map[0][1] || x > map[1][0] || y > map[3][1])
	{
		//printf("outside of map!!!!\n");
		return 1;
	}
	return 0;
}

int Linecollide(double sol[PARAMETER_NUMBER])//判断某一点和上一个最佳点之间是否有障碍。last为上一点的索引。1有障碍，0无障碍
{
	int sample_number, j;
	double Best[2] = { 0 };
	double distance, distance_x, distance_y, unit_x, unit_y, sample_x, sample_y, danweimidu = 0.2;
	if (DIRECTION)//attention!we need the last point here!!!
	{
		Best[0] = BestRight[CURRENT_RIGHT_INDEX][0];
		Best[1] = BestRight[CURRENT_RIGHT_INDEX][1];
	}
	else
	{
		Best[0] = BestLeft[CURRENT_LEFT_INDEX][0];
		Best[1] = BestLeft[CURRENT_LEFT_INDEX][1];
	}

	distance = EuclideanDistance(sol, Best[0], Best[1]);							
	sample_number = abs((int)(distance / (double)danweimidu));

	distance_x = Best[0] - sol[0];
	distance_y = Best[1] - sol[1];
	unit_x = distance_x / (double)sample_number;
	unit_y = distance_y / (double)sample_number;
	sample_x = Best[0];
	sample_y = Best[1];

	for (j = 1; j < sample_number; j++)
	{
		sample_x = sample_x - unit_x;
		sample_y = sample_y - unit_y;
		if (xiaomi_pointcollide(sample_x, sample_y))
		{
			return 1;
		}
	}
	return 0;
}

double function(int index)
{
	double total = 0, local_length =0;
	double  endx, endy, distance_x, distance_y;
	if (!DIRECTION)
	{
		endx = BestRight[CURRENT_RIGHT_INDEX][POSITION_X];
		endy = BestRight[CURRENT_RIGHT_INDEX][POSITION_Y];
	}
	else
	{
		endx = BestLeft[CURRENT_LEFT_INDEX][POSITION_X];
		endy = BestLeft[CURRENT_LEFT_INDEX][POSITION_Y];
	}
	distance_x = endx - PARTICLE[index][POSITION_X];
	distance_y = endy - PARTICLE[index][POSITION_Y];

	PARTICLE_COLLIDE[index] = xiaomi_pointcollide(PARTICLE[index][POSITION_X], PARTICLE[index][POSITION_Y]);
	if (!PARTICLE_COLLIDE[index])
	{
		PARTICLE_COLLIDE[index] = Linecollide(PARTICLE[index]);
	}
	total = (fabs(distance_x) + fabs(distance_y)) * length_quality + PARTICLE_COLLIDE[index] * collision_quality;
	//total = (fabs(distance_x - PARTICLE[index][2]) + fabs(distance_y - PARTICLE[index][3])) * length_quality + PARTICLE_COLLIDE[index] * collision_quality;
	return total;
}

double function_best(int index)
{
	double total = 0, local_length = 0;
	double  endx, endy, distance_x, distance_y;
	if (!DIRECTION)
	{
		endx = BestRight[CURRENT_RIGHT_INDEX][POSITION_X];
		endy = BestRight[CURRENT_RIGHT_INDEX][POSITION_Y];
	}
	else
	{
		endx = BestLeft[CURRENT_LEFT_INDEX][POSITION_X];
		endy = BestLeft[CURRENT_LEFT_INDEX][POSITION_Y];
	}
	distance_x = endx - PARTICLE_BEST[index][POSITION_X];
	distance_y = endy - PARTICLE_BEST[index][POSITION_Y];

	PARTICLE_BEST_COLLIDE[index] = xiaomi_pointcollide(PARTICLE_BEST[index][POSITION_X], PARTICLE_BEST[index][POSITION_Y]);
	if (!PARTICLE_BEST_COLLIDE[index])
	{
		PARTICLE_BEST_COLLIDE[index] = Linecollide(PARTICLE_BEST[index]);
	}
	total = (fabs(distance_x) + fabs(distance_y)) * length_quality + PARTICLE_BEST_COLLIDE[index] * collision_quality;
	//total = (fabs(distance_x - PARTICLE[index][2]) + fabs(distance_y - PARTICLE[index][3])) * length_quality + PARTICLE_COLLIDE[index] * collision_quality;
	return total;
}

void initialize_particle(viod)
{
	int i;
	for (i = 0; i < PARTICLE_NUMBER; i++)
	{
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		PARTICLE[i][POSITION_X] = RANGE_LOW + r * (RANGE_UP - RANGE_LOW);
		
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		PARTICLE[i][POSITION_Y] = RANGE_LOW + r*(RANGE_UP - RANGE_LOW);
		
		PARTICLE[i][POSITION_VX] = 0;
		PARTICLE[i][POSITION_VY] = 0;

		PARTICLE_BEST[i][POSITION_X] = PARTICLE[i][POSITION_X];
		PARTICLE_BEST[i][POSITION_Y] = PARTICLE[i][POSITION_Y];
		PARTICLE_BEST[i][POSITION_VX] = PARTICLE[i][POSITION_VX];
		PARTICLE_BEST[i][POSITION_VY] = PARTICLE[i][POSITION_VY];

	}
}

int get_present_best_index(void)//this structure is very confusing,however, not wrong.save it until understand its special function
{
	int best_index = 0, i = 0;
	for (i = 0; i < PARTICLE_NUMBER; i++)
	{
		if (i != best_index)
		{
			if ( function(i) < function(best_index))
			{
				best_index = i;
			}
		}
	}
	return best_index;
}

void update_velocity(int global_best_index)
{
	int i;
	double velocity_x = 0, velocity_y = 0;

	//best_fitness = function(global_best_index);
	for (i = 0; i < PARTICLE_NUMBER; i++)
	{
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		velocity_x = PARTICLE[i][POSITION_VX] + 2 * r * (PARTICLE_BEST[i][POSITION_X] - PARTICLE[i][POSITION_X]);
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		velocity_x = PSO_W * velocity_x + 2 * r * (PARTICLE_BEST[global_best_index][POSITION_X] - PARTICLE[i][POSITION_X]);
		if (velocity_x > V_MAX)
		{
			PARTICLE[i][POSITION_VX] = V_MAX;
		}
		else if (velocity_x < V_MAX_OPPSITE)
		{
			PARTICLE[i][POSITION_VX] = V_MAX_OPPSITE;
		}
		else 
		{
			PARTICLE[i][POSITION_VX] = velocity_x;
		}

		velocity_y = PARTICLE[i][POSITION_VY] + 2 * r * (PARTICLE_BEST[i][POSITION_Y] - PARTICLE[i][POSITION_Y]);
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		velocity_y = velocity_y + 2 * r * (PARTICLE_BEST[global_best_index][POSITION_Y] - PARTICLE[i][POSITION_Y]);
		if (velocity_y > V_MAX)
		{
			PARTICLE[i][POSITION_VY] = V_MAX;
		}
		else if (velocity_y < V_MAX_OPPSITE)
		{
			PARTICLE[i][POSITION_VY] = V_MAX_OPPSITE;
		}
		else
		{
			PARTICLE[i][POSITION_VY] = velocity_y;
		}
	}
}

void update_particle(int global_best_index)
{
	int i;

	for (i = 0; i < PARTICLE_NUMBER; i++)
	{ 
		if (PARTICLE[i][POSITION_X] != PARTICLE_BEST[global_best_index][POSITION_X])
		{
			PARTICLE[i][POSITION_X] = PARTICLE[i][POSITION_X] + PARTICLE[i][POSITION_VX];
		}
		if (PARTICLE[i][POSITION_Y] != PARTICLE_BEST[global_best_index][POSITION_Y])
		{
			PARTICLE[i][POSITION_Y] = PARTICLE[i][POSITION_Y] + PARTICLE[i][POSITION_VY];
		}

		if (function(i) < function_best(i))
		{
			PARTICLE_BEST[i][POSITION_X] = PARTICLE[i][POSITION_X];
			PARTICLE_BEST[i][POSITION_VX] = PARTICLE[i][POSITION_VX];
			PARTICLE_BEST[i][POSITION_Y] = PARTICLE[i][POSITION_Y];
			PARTICLE_BEST[i][POSITION_VY] = PARTICLE[i][POSITION_VY];

		}
	}
}

int path_meet_check(double sol[PARAMETER_NUMBER], int index)
{
	int i = 0, direction = !DIRECTION;
	for (i = 0; i < index; i++)
	{

		if (!Linecollide(sol))
		{
			return i + 1;
		}
	}

	return 0;
}

int path_end(int path_end_type)//if end return 1
{
	int sample_number, j;
	double left_point[2] = { 0 }, right_point[2] = { 0 };
	double distance, distance_x, distance_y, unit_x, unit_y, sample_x, sample_y, danweimidu = 0.5;
	if (path_end_type == 0)
	{
		if (EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]) < PATH_END_TYPE_DISTANCE)
			return 1;
		return 0;
	}
	else if (path_end_type == 1)
	{
		left_point[POSITION_X] = BestLeft[CURRENT_LEFT_INDEX][POSITION_X];
		left_point[POSITION_Y] = BestLeft[CURRENT_LEFT_INDEX][POSITION_Y];
		right_point[POSITION_X] = BestRight[CURRENT_RIGHT_INDEX][POSITION_X];
		right_point[POSITION_Y] = BestRight[CURRENT_RIGHT_INDEX][POSITION_Y];
		distance = EuclideanDistance(left_point, right_point[POSITION_X], right_point[POSITION_Y]);
		sample_number = abs((int)(distance / (double)danweimidu));

		distance_x = right_point[POSITION_X] - left_point[POSITION_X];
		distance_y = right_point[POSITION_Y] - left_point[POSITION_Y];
		unit_x = distance_x / (double)sample_number;
		unit_y = distance_y / (double)sample_number;
		sample_x = right_point[POSITION_X];
		sample_y = right_point[POSITION_Y];

		for (j = 1; j < sample_number; j++)
		{
			sample_x = sample_x - unit_x;
			sample_y = sample_y - unit_y;
			if (xiaomi_pointcollide(sample_x, sample_y))
			{
				return 0;
			}
		}
		return 1;
	}
	else if (path_end_type == 2)
	{
		if (path_meet_check)
			return 1;
	}
	else
	{
		printf("wrong commend!!!\n");
		return 0;
	}
	return 0;
}

int pso_algorithm()
{
	int epoch = 0, best_fitness_index = 0, global_best_index = 0;
	initialize_particle();
	for(epoch = 0;epoch < EPOCH_NUMBER;epoch++)
	{
		PSO_W = 0.9;
		PSO_W_UNIT = (0.9 - 0.7) / (double)EPOCH_NUMBER;

		best_fitness_index = get_present_best_index();
		if (function(best_fitness_index) < function_best(global_best_index))
		{
			global_best_index = best_fitness_index;
		}
		PSO_W = PSO_W - PSO_W_UNIT;
		update_velocity(global_best_index);
		update_particle(global_best_index);

	}
	if (!DIRECTION)
	{
		CURRENT_LEFT_INDEX++;
		BestLeft[CURRENT_LEFT_INDEX][POSITION_X] = PARTICLE_BEST[global_best_index][POSITION_X];
		BestLeft[CURRENT_LEFT_INDEX][POSITION_Y] = PARTICLE_BEST[global_best_index][POSITION_Y];
		return global_best_index;
	}
	else
	{
		CURRENT_RIGHT_INDEX++;
		BestRight[CURRENT_RIGHT_INDEX][POSITION_X] = PARTICLE_BEST[global_best_index][POSITION_X];
		BestRight[CURRENT_RIGHT_INDEX][POSITION_Y] = PARTICLE_BEST[global_best_index][POSITION_Y];
	}
	return global_best_index;
}

//void left_direction_search(void)
//{
//	return 0;
//}
//
//void double_direction_search(void)
//{
//	return 0;
//}

void main(void)
{
	clock_t START_DOUBLE, END_DOUBLE, START_SINGAL, END_SINGAL;
	double totaltime, total_path_length = 0, local_length;
	int global_best_index, i, experiment, couple_index, success_times = 0, success_flag;

	for (i = 0; i < MAP_point; i++)
	{

	}
	init_all_couple_points();
	printf("init done\n");

	START_SINGAL = clock();
	for (couple_index = 0; couple_index < EXPERIMENT_COUPLES; couple_index++)
	{
		BestLeft[0][POSITION_X] = ALL_COUPLE_POINTS[couple_index][0][POSITION_X];
		BestLeft[0][POSITION_Y] = ALL_COUPLE_POINTS[couple_index][0][POSITION_Y];
		BestRight[0][POSITION_X] = ALL_COUPLE_POINTS[couple_index][1][POSITION_X];
		BestRight[0][POSITION_Y] = ALL_COUPLE_POINTS[couple_index][1][POSITION_Y];
		success_flag = 0;
		local_length = 0;//sometimes total_path_length appears error!
		for (experiment = 0; experiment < EXPERIMENT_TIMES; experiment++)
		{
			srand((unsigned)time(NULL) + experiment + couple_index * couple_index + experiment * experiment * couple_index);
			memset(PARTICLE_COLLIDE, 0, PARTICLE_NUMBER);
			memset(PARTICLE_BEST_COLLIDE, 0, PARTICLE_NUMBER);

			CURRENT_LEFT_INDEX = 0;
			CURRENT_RIGHT_INDEX = 0;
			for (i = 0; i < Mosttries; i++)
			{
				DIRECTION = LEFT;
				global_best_index = pso_algorithm();
				if (path_end(0))
				{
					success_times++;
					success_flag = 1;
					break;
				}//stop check
			}
		}
		if (success_flag)
		{
			for (i = 0; i < (CURRENT_LEFT_INDEX); i++)
			{
				local_length += EuclideanDistance(BestLeft[i], BestLeft[i + 1][0], BestLeft[i + 1][1]);
			}
			for (i = 0; i < (CURRENT_RIGHT_INDEX); i++)
			{
				local_length += EuclideanDistance(BestRight[i], BestRight[i + 1][0], BestRight[i + 1][1]);
			}
			local_length += EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0], BestRight[CURRENT_RIGHT_INDEX][1]);
		}
		total_path_length += local_length;
	}
	END_SINGAL = clock();
	totaltime = (double)(END_SINGAL - START_SINGAL) / CLOCKS_PER_SEC;

	printf("total time(singal):%f\n", totaltime);
	printf("success times(singal):%d\n all times:%d\n", success_times, EXPERIMENT_COUPLES);
	printf("total path length(singal):%f\n", total_path_length);
	printf("average path length(singal):%f\n", total_path_length / (double)success_times);



	success_times = 0;
	total_path_length = 0;

	START_DOUBLE = clock(); 
	for (couple_index = 0; couple_index < EXPERIMENT_COUPLES; couple_index++)
	{
		BestLeft[0][POSITION_X] = ALL_COUPLE_POINTS[couple_index][0][POSITION_X];
		BestLeft[0][POSITION_Y] = ALL_COUPLE_POINTS[couple_index][0][POSITION_Y];
		BestRight[0][POSITION_X] = ALL_COUPLE_POINTS[couple_index][1][POSITION_X];
		BestRight[0][POSITION_Y] = ALL_COUPLE_POINTS[couple_index][1][POSITION_Y];
		success_flag = 0;
		local_length = 0;//sometimes total_path_length appears error!
		for (experiment = 0; experiment < EXPERIMENT_TIMES; experiment++)
		{
			srand((unsigned)time(NULL) + experiment + couple_index * couple_index + experiment * experiment * couple_index);
			memset(PARTICLE_COLLIDE, 0, PARTICLE_NUMBER);
			memset(PARTICLE_BEST_COLLIDE, 0, PARTICLE_NUMBER);

			CURRENT_LEFT_INDEX = 0;
			CURRENT_RIGHT_INDEX = 0;
			for (i = 0; i < Mosttries; i++)
			{
				DIRECTION = LEFT;
				global_best_index = pso_algorithm();
				i++;
				if (path_end(0))
				{
					success_times++;
					success_flag = 1;
					break;
				}//stop check
				DIRECTION = RIGHT;//this part determine the searching strategy.
				global_best_index = pso_algorithm();
				if (path_end(0))
				{
					success_times++;
					success_flag = 1;
					break;
				}//stop check
			}

			//printf("left path:\n");
			//printf("X:\n");
			//for (i = 0; i < CURRENT_LEFT_INDEX; i++)
			//{
			//	printf("%12f", BestLeft[i][0]);
			//}
			//printf("\n");
			//printf("Y:\n");
			//for (i = 0; i < CURRENT_LEFT_INDEX; i++)
			//{
			//	printf("%12f", BestLeft[i][1]);
			//}
			//printf("\n");
			//printf("right path:\n");
			//printf("X:\n");
			//for (i = 0; i < CURRENT_RIGHT_INDEX; i++)
			//{
			//	printf("%12f", BestRight[i][0]);
			//}
			//printf("\n");
			//printf("Y:\n");
			//for (i = 0; i < CURRENT_RIGHT_INDEX; i++)
			//{
			//	printf("%12f", BestRight[i][1]);
			//}
			//printf("\n");
		}
		if (success_flag)
		{
			for (i = 0; i < (CURRENT_LEFT_INDEX); i++)
			{
				local_length += EuclideanDistance(BestLeft[i], BestLeft[i + 1][0], BestLeft[i + 1][1]);
			}
			for (i = 0; i < (CURRENT_RIGHT_INDEX); i++)
			{
				local_length += EuclideanDistance(BestRight[i], BestRight[i + 1][0], BestRight[i + 1][1]);
			}
			local_length += EuclideanDistance(BestLeft[CURRENT_LEFT_INDEX], BestRight[CURRENT_RIGHT_INDEX][0],BestRight[CURRENT_RIGHT_INDEX][1]);
		}
		total_path_length += local_length;
	}
	END_DOUBLE = clock();
	totaltime = (double)(END_DOUBLE - START_DOUBLE) / CLOCKS_PER_SEC;

	printf("total time(double):%f\n", totaltime);
	printf("success times(double):%d\n all times:%d\n", success_times, EXPERIMENT_COUPLES);
	printf("total path length(double):%f\n", total_path_length);
	printf("average path length(singal):%f\n", total_path_length / (double)success_times);


}
