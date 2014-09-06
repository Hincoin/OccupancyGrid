// OccupancyGrid.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#define PI 3.14159265358979323846
#define L_ZERO .1
#define L_OCC .3
#define L_FREE .05
#define ALPHA 3
#define BETA 1
#define MAX_RANGE 15.0
#define RAD PI/180
#define DEG 180/PI
using namespace std;
vector<pair<int,int>> landmarks;
double grid_map[10][10];
inline void add_landmark(const pair<int,int>& pair)
{
	landmarks.push_back(pair);
}
inline pair<double,double> sensor_to(int index, double x ,double y, double theta)
{
		double range = sqrt(pow(landmarks[index].first-x,2) + pow(landmarks[index].second-y,2));
		double bearing = atan2(landmarks[index].second-y,landmarks[index].first-x) - theta;
		return make_pair(range,bearing);
		
}
inline void occupancy_grid_tile(int i, int j, double range_measure, double range_angle,double x ,double y, double theta)
{
	double range_to =   sqrt(pow(x-j,2) + pow(y-i,2));
	double bearing_to = atan2(i-y,j-x) - theta;
	//cout << "Degrees bearing_to = " << bearing_to * DEG << "\nrange_angle in degrees = " << range_angle * DEG << "\n";
	if(range_to > min(MAX_RANGE,range_measure+(ALPHA/2.0)))
	{
		grid_map[i][j] += L_ZERO;
		return;
	}
	double bearing_off = fmod(abs(bearing_to-range_angle),2*PI);
	if((bearing_off > BETA/2.0))
	{
		grid_map[i][j] += L_ZERO;
		return;
	}
	else if(range_measure < MAX_RANGE && (abs(range_to-range_measure) < ALPHA/2))
	{
		grid_map[i][j] += L_OCC;
		return;
	}
	else if(range_to <= range_measure)
	{
		grid_map[i][j] += L_FREE;
		return;
	}

}
double raycast(int degrees, double x, double y)
{
	double radians = degrees * RAD;
	vector<int> landmarks_in_line;
	for(int i =0; i < landmarks.size(); ++i)
	{

		double bearing = atan2(landmarks[i].second - y, landmarks[i].first-x) - radians;
	//	double bearing = atan2( y - landmarks[i].second, x-landmarks[i].first) - radians;
		if(bearing == -2 * PI)
		{ 
			landmarks_in_line.push_back(i);
		}
	}
	double shortest = 1 << 20;
	for(const auto& g : landmarks_in_line)
	{
		double range = sqrt(pow(x-landmarks[g].first,2) + pow(y-landmarks[g].second,2));
		shortest = min(shortest,range);
	}
	return shortest;
}
int main()
{

	//fill(&grid_map[0][0],&grid_map[0][0] + sizeof(grid_map),0.0);





	memset(grid_map,sizeof(grid_map),0);
	//landmarks.push_back(make_pair(2,5));

	landmarks.push_back(make_pair(0,0));


	double x = 1,y = 1,theta = 0;
	for(int i =0 ; i < 3; ++i) // travel 30 steps in the same directions using a spinning laser-range finder
	{
		int current_degree = theta;
		for(int j =0; j <= 360 ;++j)
		{
			current_degree = (theta*DEG)+j;
			double raycast_range =raycast(current_degree,x,y);
			if(raycast_range != 1 << 20) // there's something in the way
			{
				for(int k=0; k < 10; ++k)
				{
					for(int l = 0; l < 10; ++l)
					{
						occupancy_grid_tile(k,l,raycast_range,current_degree * RAD,x,y,theta);
					}
				}
			}

		}
		++x;
		++y;
	}

	cout << "\n\n";
	for(int i =0; i < 10; ++i)
	{
		for(int j =0; j < 10; ++j)
		{
			cout << grid_map[i][j] << "  ";
		}
		cout << "\n";
	}
	int b;
	cin >> b;
}	