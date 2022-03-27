//Traffic scheduling for full version model

#define  _CRT_SECURE_NO_WARNINGS
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<sys/types.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include<fstream>
#include "string.h"

using namespace std;

const int MaxRowNum = 50;// The maximum number of row links.
const int MaxColumnNum = 50;// The maximum number of column links.
const int Max_Sched_Time = 60; // The maximum sampling interval is 60s.
const int Min_Win = 5; // The minimum time window in each scheduling.
const int PopSize = 50;	// The size of population.

int Gen;	//	The number of iterations of the algorithm.
int Repeat;	// The repetitions of the experiments.
int Sched_Time;	// The time of each sampling interval in the experiments.
int Win;	// The time window in each scheduling in the experiments.
int RowNum;  // The number of row links.
int ColumnNum; //The number of column links.
int C_Upper_Row[MaxRowNum][MaxColumnNum]; //  The upper capacity value of row direction (left to right).
int C_Upper_Column[MaxColumnNum][MaxRowNum]; // The upper capacity value of column direction (above to below).
int C_Row_Initial[MaxRowNum][MaxColumnNum]; // The  Initial capacity value of row direction (left to right).
int C_Column_Initial[MaxColumnNum][MaxRowNum]; // The Initial capacity value of column direction (above to below).
int C_Row[MaxRowNum][MaxColumnNum]; // The capacity value of row direction (left to right).
int C_Column[MaxColumnNum][MaxRowNum]; // The capacity value of row direction (above to below).

int C_Upper_Row1[MaxRowNum][MaxColumnNum]; // The upper capacity value of row direction (right to left).
int C_Upper_Column1[MaxColumnNum][MaxRowNum]; // The upper capacity value of column direction (below to above).
int C_Row1_Initial[MaxRowNum][MaxColumnNum]; // The  Initial capacity value of row direction (right to left).
int C_Column1_Initial[MaxColumnNum][MaxRowNum]; // The Initial capacity value of column direction (below to above).
int C_Row1[MaxRowNum][MaxColumnNum]; // The capacity value of row direction (right to left).
int C_Column1[MaxColumnNum][MaxRowNum];	// The capacity value of row direction (below to above).

int f_Row[MaxRowNum][MaxColumnNum];// The flowing rate in row direction (left to right).
int f_Row1[MaxRowNum][MaxColumnNum]; // The  flowing rate in row direction (right to left).
int f_Column[MaxColumnNum][MaxRowNum];// The  flowing rate column direction  (above to below).
int f_Column1[MaxColumnNum][MaxRowNum]; //The  flowing rate in column direction (below to above).

int Pop[PopSize][Max_Sched_Time / Min_Win][MaxRowNum][MaxColumnNum]; //The population (The phase chosen by each individual at each sampling moment)
int New_Pop[PopSize][Max_Sched_Time / Min_Win][MaxRowNum][MaxColumnNum];// Generateing new population
int** New_Pop_Temp = nullptr;
int** New_Pop_2 = nullptr; //Represented population as an array of length PopSize consisting of chromosomes represented as continuous arrays of size (k * RowNum * ColumnNum)
int Best_Pop[Max_Sched_Time / Min_Win][MaxRowNum][MaxColumnNum]; // The best solution
int New_Car_Row_Initial[Max_Sched_Time / Min_Win][MaxRowNum];	// Generate new vehicles in a row direction (right to left) at each sampling time
int New_Car_Column_Initial[Max_Sched_Time / Min_Win][MaxColumnNum];	// Generate new vehicles in a column direction (above to below) at each sampling time
int New_Car_Row1_Initial[Max_Sched_Time / Min_Win][MaxRowNum];	// Generate new vehicles in a row direction (left to right) at each sampling time
int New_Car_Column1_Initial[Max_Sched_Time / Min_Win][MaxColumnNum];	// Generate new vehicles in a column direction (below to above) at each sampling time

int Obj[PopSize]; // The objective function value for each current individual.
int New_Obj; // Record the leaving of the vehicle.
int count2;	// Determine whether the calculated objective function value is the initial population or the new population
int Total_Obj[PopSize]; // The objective function value for each individual.
int K; // Record the sampling moment.
int popi; // Record the population.
int totalobj, I;	// The objective function value of the best solution.
int TotalNum; // total objective value
float ExitRat[MaxRowNum][MaxColumnNum][12];		//	Ratio of vehicles leaving in the left, right above and below directions .
int ExitTurnNum = 2 * 2 * 3;	// Different leaving directions of the vehicles.
float EntraRat[MaxRowNum][MaxColumnNum][8];   //	Ratio of vehicles entering in the left, right above and below directions .
int EntraTurnNum = 2 * 2 * 2;	// Different entering directions of vehicles..
int Lij[5][Max_Sched_Time / Min_Win];	 // The first row is for the turn left maximum speed when there is just one lane;  the second mean the straight spped for one lanes and  for the left turn speed when there are muptiple lanes.
int Lanes[MaxRowNum][MaxColumnNum][2];	// The first value is for left and right direction, the second value is for the above and below direction.
int L[MaxRowNum][MaxColumnNum][2];	//The length of links connected to each intersection, left, right above and below.
int Len;// The average length to store each car in link.
ofstream outfile;
FILE* fp;

void Read_GenRepeat()
{
	fp = fopen("GenRepeat.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no GenRepeat file or no data in GenRepeat file\n");
		system("pause");
	}
	fscanf(fp, "%d%d", &Gen, &Repeat);
	fclose(fp);
}
void Read_Instance()
{
	int i, j;
	string str;
	fp = fopen("Instance.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no Instance file or no data in Instance\n");
		system("pause");
	}
	fscanf(fp, "%d%d%d%d", &Sched_Time, &Win, &RowNum, &ColumnNum);

	// left to right direction
	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Upper_Row[i][j]);
		}
	}
	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Row_Initial[i][j]);
		}
	}

	//right to left direction
	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Upper_Row1[i][j]);
		}
	}
	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Row1_Initial[i][j]);
		}
	}

	// above to below direction
	for (i = 0;i < ColumnNum + 1;i++)
	{
		for (j = 0;j < RowNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Upper_Column[i][j]);
		}
	}
	for (i = 0;i < ColumnNum + 1;i++)
	{
		for (j = 0;j < RowNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Column_Initial[i][j]);
		}
	}

	// below to above direction
	for (i = 0;i < ColumnNum + 1;i++)
	{
		for (j = 0;j < RowNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Upper_Column1[i][j]);
		}
	}
	for (i = 0;i < ColumnNum + 1;i++)
	{
		for (j = 0;j < RowNum + 1;j++)
		{
			fscanf(fp, "%d", &C_Column1_Initial[i][j]);
		}
	}
	fclose(fp);
}
void Read_New_car()
{
	int i, j;
	fp = fopen("New car.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no New car file or no data in New Car file\n");
		system("pause");
	}
	for (i = 0;i < Sched_Time / Win;i++)
	{
		// left to right direction
		for (j = 0;j < RowNum;j++)
		{
			fscanf(fp, "%d", &New_Car_Row_Initial[i][j]);
		}
		// right to left direction
		for (j = 0;j < RowNum;j++)
		{
			fscanf(fp, "%d", &New_Car_Row1_Initial[i][j]);
		}
		// above to beolw direction
		for (j = 0;j < ColumnNum;j++)
		{
			fscanf(fp, "%d", &New_Car_Column_Initial[i][j]);
		}
		// below to above direction
		for (j = 0;j < ColumnNum;j++)
		{
			fscanf(fp, "%d", &New_Car_Column1_Initial[i][j]);
		}
	}
	fclose(fp);
}
void Read_Exit_TurnRatio()
{
	int i, j;
	fp = fopen("Exit Turn Ratio.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no Exit Turn Ratio file or no data in Exit Turn Ratio file\n");
		system("pause");
	}
	for (i = 0;i < RowNum;i++)
	{
		for (j = 0;j < ColumnNum;j++)
		{
			for (int k = 0;k < ExitTurnNum;k++)
			{
				fscanf(fp, "%f", &ExitRat[i][j][k]);
			}
		}
	}
	fclose(fp);
}
void Read_Entrance_Ratio()
{
	int i, j;
	fp = fopen("Entrance Ratio.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no Entrance Ratio file or no data in Entrance Ratio file\n");
		system("pause");
	}
	for (i = 0;i < RowNum;i++)
	{
		for (j = 0;j < ColumnNum;j++)
		{
			for (int k = 0;k < EntraTurnNum;k++)
			{
				fscanf(fp, "%f", &EntraRat[i][j][k]);
			}
		}
	}
	fclose(fp);
}
void Read_Lij()
{
	int i, j;
	fp = fopen("Lij.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no Lij file or no data in Lij file\n");
		system("pause");
	}
	for (i = 0;i < 5;i++)
	{
		for (j = 0;j < Sched_Time / Win;j++)
		{
			fscanf(fp, "%d", &Lij[i][j]);
		}
	}
	fclose(fp);
}
void Read_Lanes()
{
	int i, j;
	fp = fopen("Lanes.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no Lanes file or no data in Lanes file\n");
		system("pause");
	}
	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			fscanf(fp, "%d%d", &Lanes[i][j][0], &Lanes[i][j][1]);
		}
	}
	fclose(fp);
}
void Read_L()
{
	int i, j;
	fp = fopen("L.csv", "r");
	if (fp == NULL)
	{
		printf("\nThere is no L file or no data in L file\n");
		system("pause");
	}
	fscanf(fp, "%d", &Len);
	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			fscanf(fp, "%d%d", &L[i][j][0], &L[i][j][1]);
		}
	}
	fclose(fp);
}
void Calculate_TotalNum()
{
	int i, j;
	TotalNum = 0;
	for (i = 0;i < RowNum;i++)
	{
		for (j = 0;j < ColumnNum;j++)
		{
			TotalNum += C_Row[i][j] + C_Column[j][i] + C_Row1[i + 1][j] + C_Column1[j + 1][i];
		}
	}
}
void Initi_Pop()
{
	int i, j, k, l;
	for (i = 0;i < PopSize;i++)
	{
		for (l = 0;l < Sched_Time / Win;l++)
		{
			for (j = 0;j < RowNum;j++)
			{
				for (k = 0;k < ColumnNum;k++)
				{
					Pop[i][l][j][k] = rand() % 4;
				}
			}
		}
	}
}
void Calculate_single_Obj()
{
	int i, j, k;
	int Ea1, Ea2, Ea3, Eb1, Eb2, Eb3;// exit vechile nubmers at two directions
	int MaxV1, MaxV2, MaxV3, MaxV4, MaxV5, MaxV6;// the maximum vechile numbers that can go through at each directions
	int Entra1, Entra2, Entra3, Entra4, Entra5, Entra6;
	int v1, v2, v3, v4, v5, v6;
	int cc;
	int ExitNum = 0;

	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			f_Row[i][j] = 0;
			f_Row1[i][j] = 0;
			f_Column1[j][i] = 0;
			f_Column[j][i] = 0;
		}
	}
	for (i = 0;i < RowNum;i++)
	{
		for (j = 0;j < ColumnNum;j++)
		{
			if (Pop[popi][K][i][j] == 0)
			{
				//get the exit vechile nubmers of two directions to three different links
				Ea1 = ExitRat[i][j][0] * C_Row[i][j];
				Ea2 = ExitRat[i][j][1] * C_Row[i][j];
				Ea3 = ExitRat[i][j][2] * C_Row[i][j];
				Eb1 = ExitRat[i][j + 1][3] * C_Row1[i][j + 1];
				Eb2 = ExitRat[i][j + 1][4] * C_Row1[i][j + 1];
				Eb3 = ExitRat[i][j + 1][5] * C_Row1[i][j + 1];
				//get the maximum vechile number of two directions to three different links
				if (K == 0)
				{
					switch (Lanes[i][j][0]) {
					case 1: MaxV1 = Lij[Lanes[i][j][0] - 1][K];
						MaxV2 = Lij[Lanes[i][j][0]][K];
						MaxV3 = Lij[Lanes[i][j][0] - 1][K];

						break;
					case 2:  MaxV1 = Lij[Lanes[i][j][0] - 1][K];
						MaxV2 = Lij[Lanes[i][j][0]][K];
						MaxV3 = Lij[Lanes[i][j][0] - 1][K];

						break;
					case 3: MaxV1 = Lij[Lanes[i][j][0] - 2][K];
						MaxV2 = Lij[Lanes[i][j][0]][K];
						MaxV3 = Lij[Lanes[i][j][0] - 2][K];

						break;
					case 4: MaxV1 = Lij[Lanes[i][j][0] - 3][K];
						MaxV2 = Lij[Lanes[i][j][0]][K];
						MaxV3 = Lij[Lanes[i][j][0] - 3][K];

						break;
					}
					switch (Lanes[i][j + 1][0]) {
					case 1:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 1][K];
						MaxV5 = Lij[Lanes[i][j + 1][0]][K];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 1][K];
						break;
					case 2:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 1][K];
						MaxV5 = Lij[Lanes[i][j + 1][0]][K];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 1][K];
						break;
					case 3:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 2][K];
						MaxV5 = Lij[Lanes[i][j + 1][0]][K];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 2][K];
						break;
					case 4:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 3][K];
						MaxV5 = Lij[Lanes[i][j + 1][0]][K];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 3][K];
						break;
					}

				}
				else
				{
					cc = 0;
					for (k = K - 1;k > -1;k--)
					{
						if (Pop[popi][k][i][j] == 0)
						{
							cc++;
						}
						else
						{
							break;
						}
					}
					switch (Lanes[i][j][0]) {
					case 1: MaxV1 = Lij[Lanes[i][j][0] - 1][cc];
						MaxV2 = Lij[Lanes[i][j][0]][cc];
						MaxV3 = Lij[Lanes[i][j][0] - 1][cc];

						break;
					case 2:  MaxV1 = Lij[Lanes[i][j][0] - 1][cc];
						MaxV2 = Lij[Lanes[i][j][0]][cc];
						MaxV3 = Lij[Lanes[i][j][0] - 1][cc];

						break;
					case 3: MaxV1 = Lij[Lanes[i][j][0] - 2][cc];
						MaxV2 = Lij[Lanes[i][j][0]][cc];
						MaxV3 = Lij[Lanes[i][j][0] - 2][cc];

						break;
					case 4: MaxV1 = Lij[Lanes[i][j][0] - 3][cc];
						MaxV2 = Lij[Lanes[i][j][0]][cc];
						MaxV3 = Lij[Lanes[i][j][0] - 3][cc];

						break;
					}
					switch (Lanes[i][j + 1][0]) {
					case 1:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 1][cc];
						MaxV5 = Lij[Lanes[i][j + 1][0]][cc];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 1][cc];
						break;
					case 2:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 1][cc];
						MaxV5 = Lij[Lanes[i][j + 1][0]][cc];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 1][cc];
						break;
					case 3:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 2][cc];
						MaxV5 = Lij[Lanes[i][j + 1][0]][cc];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 2][cc];
						break;
					case 4:
						MaxV4 = Lij[Lanes[i][j + 1][0] - 3][cc];
						MaxV5 = Lij[Lanes[i][j + 1][0]][cc];
						MaxV6 = Lij[Lanes[i][j + 1][0] - 3][cc];
						break;
					}


				}
				//get the maximum vechile number at different entrance links
				if (MaxV2 > Ea2)
				{
					if (L[i][j][1] * Lanes[i][j][1] / Len > C_Column1[j][i])
					{
						Entra1 = EntraRat[i][j][6] * (L[i][j][1] * Lanes[i][j][1] / Len - C_Column1[j][i]);
						Entra3 = EntraRat[i][j][7] * (L[i][j][1] * Lanes[i][j][1] / Len - C_Column1[j][i]);
					}
					else
					{
						Entra1 = 0;
						Entra3 = 0;
					}
				}
				else
				{
					if (L[i][j][1] * Lanes[i][j][1] / Len > C_Column1[j][i])
					{
						Entra1 = L[i][j][1] * Lanes[i][j][1] / Len - C_Column1[j][i];
					}
					else
					{
						Entra1 = 0;
					}
					Entra3 = 0;
					MaxV3 = 0;
				}
				Entra2 = L[i][j + 1][0] * Lanes[i][j + 1][0] / Len - C_Row[i][j + 1];
				if (MaxV5 > Eb2)
				{
					if (L[i + 1][j][1] * Lanes[i + 1][j][1] / Len > C_Column[j][i + 1])
					{
						Entra4 = EntraRat[i + 1][j][4] * (L[i + 1][j][1] * Lanes[i + 1][j][1] / Len - C_Column[j][i + 1]);
						Entra6 = EntraRat[i + 1][j][5] * (L[i + 1][j][1] * Lanes[i + 1][j][1] / Len - C_Column[j][i + 1]);
					}
					else
					{
						Entra4 = 0;
						Entra6 = 0;
					}
				}
				else
				{
					if (L[i + 1][j][1] * Lanes[i + 1][j][1] / Len > C_Column[j][i + 1])
					{
						Entra4 = L[i + 1][j][1] * Lanes[i + 1][j][1] / Len - C_Column[j][i + 1];
					}
					else
					{
						Entra4 = 0;
					}
					Entra6 = 0;
					MaxV6 = 0;
				}
				Entra5 = L[i][j][0] * Lanes[i][j][0] / Len - C_Row1[i][j];
				v1 = min(Ea1, MaxV1);
				v1 = min(v1, Entra1);
				v2 = min(Ea2, MaxV1);
				v2 = min(v2, Entra2);

				v4 = min(Eb1, MaxV4);
				v4 = min(v4, Entra4);
				v5 = min(Eb2, MaxV5);
				v5 = min(v5, Entra5);

				f_Row[i][j] = -v1 - v2;
				f_Row1[i][j] = v5;
				f_Column1[j][i] = v1;
				f_Row[i][j + 1] = v2;
				f_Row1[i][j + 1] = -v4 - v5;
				f_Column[j][i + 1] = v4;
				ExitNum += v1 + v2 + v4 + v5;

			}
			else
			{
				if (Pop[popi][K][i][j] == 1)
				{
					//get the exit vechile nubmers of two directions to three different links
					Ea3 = ExitRat[i][j][2] * C_Row[i][j];

					Eb3 = ExitRat[i][j + 1][5] * C_Row1[i][j + 1];

					//get the maximum vechile number of two directions to three different links
					if (K == 0)
					{
						switch (Lanes[i][j][0]) {
						case 1: MaxV1 = Lij[Lanes[i][j][0] - 1][K];

							break;
						case 2:  MaxV1 = Lij[Lanes[i][j][0] - 1][K];


							break;
						case 3: MaxV1 = Lij[Lanes[i][j][0] - 2][K];


							break;
						case 4: MaxV1 = Lij[Lanes[i][j][0] - 3][K];


							break;
						}
						switch (Lanes[i][j + 1][0]) {
						case 1:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 1][K];

							break;
						case 2:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 1][K];

							break;
						case 3:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 2][K];

							break;
						case 4:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 3][K];

							break;
						}

					}
					else
					{
						cc = 0;
						for (k = K - 1;k > -1;k--)
						{
							if (Pop[popi][k][i][j] == 1)
							{
								cc++;
							}
							else
							{
								break;
							}
						}
						switch (Lanes[i][j][0]) {
						case 1: MaxV1 = Lij[Lanes[i][j][0] - 1][cc];


							break;
						case 2:  MaxV1 = Lij[Lanes[i][j][0] - 1][cc];


							break;
						case 3: MaxV1 = Lij[Lanes[i][j][0] - 2][cc];


							break;
						case 4: MaxV1 = Lij[Lanes[i][j][0] - 3][cc];


							break;
						}
						switch (Lanes[i][j + 1][0]) {
						case 1:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 1][cc];

							break;
						case 2:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 1][cc];

							break;
						case 3:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 2][cc];

							break;
						case 4:
							MaxV4 = Lij[Lanes[i][j + 1][0] - 3][cc];

							break;
						}


					}
					//get the maximum vechile number at different entrance links
					if (L[i + 1][j][1] * Lanes[i + 1][j][1] / Len > C_Column[j][i + 1])
					{
						Entra1 = L[i + 1][j][1] * Lanes[i + 1][j][1] / Len - C_Column[j][i + 1];
					}
					else
					{
						Entra1 = 0;
					}
					if (L[i][j][1] * Lanes[i][j][1] / Len > C_Column1[j][i])
					{
						Entra4 = L[i][j][1] * Lanes[i][j][1] / Len - C_Column1[j][i];
					}
					else
					{
						Entra4 = 0;
					}

					v1 = min(Ea3, MaxV1);
					v1 = min(v1, Entra1);
					v4 = min(Eb3, MaxV4);
					v4 = min(v4, Entra4);
					f_Row[i][j] = -v1;
					f_Column1[j][i] = v4;
					f_Row1[i][j + 1] = -v4;
					f_Column[j][i + 1] = v1;
					ExitNum += v1 + v4;
				}
				else
				{
					if (Pop[popi][K][i][j] == 2)
					{
						//get the exit vechile nubmers of two directions to three different links
						Ea1 = ExitRat[i][j][6] * C_Column[j][i];
						Ea2 = ExitRat[i][j][7] * C_Column[j][i];
						Ea3 = ExitRat[i][j][8] * C_Column[j][i];
						Eb1 = ExitRat[i][j + 1][9] * C_Column1[j][i + 1];
						Eb2 = ExitRat[i][j + 1][10] * C_Column1[j][i + 1];
						Eb3 = ExitRat[i][j + 1][11] * C_Column1[j][i + 1];
						//get the maximum vechile number of two directions to three different links
						if (K == 0)
						{
							switch (Lanes[i][j][1]) {
							case 1: MaxV1 = Lij[Lanes[i][j][1] - 1][K];
								MaxV2 = Lij[Lanes[i][j][1]][K];
								MaxV3 = Lij[Lanes[i][j][1] - 1][K];

								break;
							case 2:  MaxV1 = Lij[Lanes[i][j][1] - 1][K];
								MaxV2 = Lij[Lanes[i][j][1]][K];
								MaxV3 = Lij[Lanes[i][j][1] - 1][K];

								break;
							case 3: MaxV1 = Lij[Lanes[i][j][1] - 2][K];
								MaxV2 = Lij[Lanes[i][j][1]][K];
								MaxV3 = Lij[Lanes[i][j][1] - 2][K];

								break;
							case 4: MaxV1 = Lij[Lanes[i][j][1] - 3][K];
								MaxV2 = Lij[Lanes[i][j][1]][K];
								MaxV3 = Lij[Lanes[i][j][1] - 3][K];

								break;
							}
							switch (Lanes[i + 1][j][1]) {
							case 1:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][K];
								MaxV5 = Lij[Lanes[i + 1][j][1]][K];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 1][K];
								break;
							case 2:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][K];
								MaxV5 = Lij[Lanes[i + 1][j][1]][K];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 1][K];
								break;
							case 3:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 2][K];
								MaxV5 = Lij[Lanes[i + 1][j][1]][K];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 2][K];
								break;
							case 4:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 3][K];
								MaxV5 = Lij[Lanes[i + 1][j][1]][K];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 3][K];
								break;
							}

						}
						else
						{
							cc = 0;
							for (k = K - 1;k > -1;k--)
							{
								if (Pop[popi][k][i][j] == 2)
								{
									cc++;
								}
								else
								{
									break;
								}
							}
							switch (Lanes[i][j][1]) {
							case 1: MaxV1 = Lij[Lanes[i][j][1] - 1][cc];
								MaxV2 = Lij[Lanes[i][j][1]][cc];
								MaxV3 = Lij[Lanes[i][j][1] - 1][cc];

								break;
							case 2:  MaxV1 = Lij[Lanes[i][j][1] - 1][cc];
								MaxV2 = Lij[Lanes[i][j][1]][cc];
								MaxV3 = Lij[Lanes[i][j][1] - 1][cc];

								break;
							case 3: MaxV1 = Lij[Lanes[i][j][1] - 2][cc];
								MaxV2 = Lij[Lanes[i][j][1]][cc];
								MaxV3 = Lij[Lanes[i][j][1] - 2][cc];

								break;
							case 4: MaxV1 = Lij[Lanes[i][j][1] - 3][cc];
								MaxV2 = Lij[Lanes[i][j][1]][cc];
								MaxV3 = Lij[Lanes[i][j][1] - 3][cc];

								break;
							}
							switch (Lanes[i + 1][j][1]) {
							case 1:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][cc];
								MaxV5 = Lij[Lanes[i + 1][j][1]][cc];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 1][cc];
								break;
							case 2:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][cc];
								MaxV5 = Lij[Lanes[i + 1][j][1]][cc];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 1][cc];
								break;
							case 3:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 2][cc];
								MaxV5 = Lij[Lanes[i + 1][j][1]][cc];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 2][cc];
								break;
							case 4:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 3][cc];
								MaxV5 = Lij[Lanes[i + 1][j][1]][cc];
								MaxV6 = Lij[Lanes[i + 1][j][1] - 3][cc];
								break;
							}


						}
						//get the maximum vechile number at different entrance links
						if (MaxV2 > Ea2)
						{
							if (L[i][j + 1][0] * Lanes[i][j + 1][0] / Len > C_Row[i][j + 1])
							{
								Entra1 = EntraRat[i][j + 1][0] * (L[i][j + 1][0] * Lanes[i][j + 1][0] / Len - C_Row[i][j + 1]);
								Entra3 = EntraRat[i][j + 1][1] * (L[i][j + 1][0] * Lanes[i][j + 1][0] / Len - C_Row[i][j + 1]);
							}
							else
							{
								Entra1 = 0;
								Entra3 = 0;
							}
						}
						else
						{
							if (L[i][j + 1][0] * Lanes[i][j + 1][0] / Len > C_Row[i][j + 1])
							{
								Entra1 = L[i][j + 1][0] * Lanes[i][j + 1][0] / Len - C_Row[i][j + 1];
							}
							else
							{
								Entra1 = 0;
							}
							Entra3 = 0;
							MaxV3 = 0;
						}
						Entra2 = L[i + 1][j][1] * Lanes[i + 1][j][1] / Len - C_Column[j][i + 1];
						if (MaxV5 > Eb2)
						{
							if (L[i][j][0] * Lanes[i][j][0] / Len > C_Row1[i][j])
							{
								Entra4 = EntraRat[i][j][2] * (L[i][j][0] * Lanes[i][j][0] / Len - C_Row1[i][j]);
								Entra6 = EntraRat[i][j][3] * (L[i][j][0] * Lanes[i][j][0] / Len - C_Row1[i][j]);
							}
							else
							{
								Entra4 = 0;
								Entra6 = 0;
							}
						}
						else
						{
							if (L[i][j][0] * Lanes[i][j][0] / Len > C_Row1[i][j])
							{
								Entra4 = L[i][j][0] * Lanes[i][j][0] / Len - C_Row1[i][j];
							}
							else
							{
								Entra4 = 0;
							}
							Entra6 = 0;
							MaxV6 = 0;
						}
						Entra5 = L[i][j][1] * Lanes[i][j][1] / Len - C_Column1[j][i];
						v1 = min(Ea1, MaxV1);
						v1 = min(v1, Entra1);
						v2 = min(Ea2, MaxV2);
						v2 = min(v2, Entra2);
						v4 = min(Eb1, Entra4);
						v4 = min(v4, MaxV5);
						v5 = min(Eb2, MaxV5);
						v5 = min(v5, Entra5);
						f_Column[j][i] = -v1 - v2;
						f_Column1[j][i] = v5;
						f_Row[i][j + 1] = v1;
						f_Column[j][i + 1] = v2;
						f_Column1[j][i + 1] = -v4 - v5;
						f_Row1[i][j] = v4;
						ExitNum += v1 + v2 + v4 + v5;
					}
					else
					{
						//get the exit vechile nubmers of two directions to three different links
						Ea3 = ExitRat[i][j][8] * C_Column[j][i];

						Eb3 = ExitRat[i + 1][j][11] * C_Column1[j][i + 1];

						//get the maximum vechile number of two directions to three different links
						if (K == 0)
						{
							switch (Lanes[i][j][1]) {
							case 1: MaxV1 = Lij[Lanes[i][j][1] - 1][K];

								break;
							case 2:  MaxV1 = Lij[Lanes[i][j][1] - 1][K];


								break;
							case 3: MaxV1 = Lij[Lanes[i][j][1] - 2][K];


								break;
							case 4: MaxV1 = Lij[Lanes[i][j][1] - 3][K];


								break;
							}
							switch (Lanes[i + 1][j][1]) {
							case 1:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][K];

								break;
							case 2:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][K];

								break;
							case 3:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 2][K];

								break;
							case 4:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 3][K];

								break;
							}

						}
						else
						{
							cc = 0;
							for (k = K - 1;k > -1;k--)
							{
								if (Pop[popi][k][i][j] == 1)
								{
									cc++;
								}
								else
								{
									break;
								}
							}
							switch (Lanes[i][j][1]) {
							case 1: MaxV1 = Lij[Lanes[i][j][1] - 1][cc];


								break;
							case 2:  MaxV1 = Lij[Lanes[i][j][1] - 1][cc];


								break;
							case 3: MaxV1 = Lij[Lanes[i][j][1] - 2][cc];


								break;
							case 4: MaxV1 = Lij[Lanes[i][j][1] - 3][cc];


								break;
							}
							switch (Lanes[i + 1][j][1]) {
							case 1:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][cc];

								break;
							case 2:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 1][cc];

								break;
							case 3:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 2][cc];

								break;
							case 4:
								MaxV4 = Lij[Lanes[i + 1][j][1] - 3][cc];

								break;
							}


						}
						//get the maximum vechile number at different entrance links
						if (L[i][j][0] * Lanes[i][j][0] / Len > C_Row1[i][j])
						{
							Entra1 = L[i][j][0] * Lanes[i][j][0] / Len > C_Row1[i][j];
						}
						else
						{
							Entra1 = 0;
						}
						if (L[i][j + 1][0] * Lanes[i][j + 1][0] / Len > C_Row[i][j + 1])
						{
							Entra4 = L[i][j + 1][0] * Lanes[i][j + 1][0] / Len > C_Row[i][j + 1];
						}
						else
						{
							Entra4 = 0;
						}
						v1 = min(Ea3, MaxV1);
						v1 = min(v1, Entra1);
						v4 = min(Eb3, MaxV4);
						v4 = min(v4, Entra4);
						f_Column[j][i] = -v1;
						f_Column1[j][i + 1] = -v4;
						f_Row[i][j + 1] = v4;
						f_Row1[i][j] = v1;
						ExitNum += v1 + v4;
					}
				}
			}
		}
	}
	New_Obj = ExitNum;
}
void Caculate_Obj()
{
	New_Obj = 0;
	Calculate_single_Obj();
	if (K == 0)
	{
		Obj[popi] = TotalNum - New_Obj;
	}
	else
	{
		Obj[popi] += TotalNum - New_Obj;
	}

}
void Update_TotalObj()
{
	Total_Obj[K] = TotalNum - totalobj;
	int i, j;
	for (i = 0;i < RowNum;i++)
	{
		for (j = 0;j < ColumnNum;j++)
		{
			Best_Pop[K][i][j] = Pop[I][K][i][j];
		}
	}
}
void Update_C()
{
	int i, j;

	for (i = 0;i < RowNum + 1;i++)
	{
		for (j = 0;j < ColumnNum + 1;j++)
		{
			C_Row[i][j] += f_Row[i][j];
			C_Column[j][i] += f_Column[j][i];
			C_Row1[i][j] += f_Row1[i][j];
			C_Column1[j][i] += f_Column1[j][i];
			if (C_Row[i][j] < 0)
			{
				system("pause");
			}
			if (C_Column[j][i] < 0)
			{
				system("pause");
			}
			if (C_Row1[i][j] < 0)
			{
				system("pause");
			}
			if (C_Column1[j][i] < 0)
			{
				system("pause");
			}
		}
	}
}
void New_Update_C()
{
	int i, r;
	for (i = 0;i < RowNum + 1;i++)
	{
		C_Row[i][0] = C_Row[i][0] + New_Car_Row_Initial[K][i];
		C_Row1[i][RowNum] = C_Row1[i][RowNum] + New_Car_Row1_Initial[K][i];
	}
	for (i = 0;i < ColumnNum;i++)
	{
		C_Column[i][0] = C_Column[i][0] + New_Car_Column_Initial[K][i];
		C_Column1[i][ColumnNum] = C_Column1[i][ColumnNum] + New_Car_Column1_Initial[K][i];
	}
}

/*
int main()
{
	srand((unsigned)time(NULL) + MaxRowNum * MaxColumnNum * 1000000);

	int i, j, k, Sum, rep, l;
	Read_GenRepeat();	// Get the number of iterations of the algorithm and the number of repetitions of the experiment
	for (rep = 0;rep < Repeat;rep++)
	{
		Read_Instance();
		Read_New_car();
		Read_Exit_TurnRatio();
		Read_Entrance_Ratio();
		Read_Lij();
		Read_Lanes();
		Read_L();
		time_t start = clock();
		Initi_Pop();
		for (popi = 0;popi < PopSize;popi++)
		{
			for (i = 0;i < RowNum + 1;i++)
			{
				for (j = 0;j < ColumnNum + 1;j++)
				{
					C_Row[i][j] = C_Row_Initial[i][j];
					C_Column[j][i] = C_Column_Initial[j][i];
					C_Row1[i][j] = C_Row1_Initial[i][j];
					C_Column1[j][i] = C_Column1_Initial[j][i];
				}
			}
			for (K = 0;K < Sched_Time / Win;K++)
			{
				if (K > 0)
				{
					New_Update_C();
				}
				Calculate_TotalNum();
				Caculate_Obj();
				Update_C();

			}
			Total_Obj[popi] = Obj[popi];
			if (popi == 0)
			{
				totalobj = Total_Obj[popi];
				I = popi;
				for (i = 0;i < Sched_Time / Win;i++)
				{
					for (j = 0;j < RowNum;j++)
					{
						for (k = 0;k < ColumnNum;k++)
						{
							Best_Pop[i][j][k] = Pop[popi][i][j][k];
						}
					}
				}
			}
			else
			{
				if (totalobj > Total_Obj[popi])
				{
					totalobj = Total_Obj[popi];
					I = popi;
					for (i = 0;i < Sched_Time / Win;i++)
					{
						for (j = 0;j < RowNum;j++)
						{
							for (k = 0;k < ColumnNum;k++)
							{
								Best_Pop[i][j][k] = Pop[popi][i][j][k];
							}
						}
					}
				}
			}
		}

		outfile.open("Best Solution.csv", ios::app);
		for (l = 0; l < Sched_Time / Win; l++)
		{
			for (i = 0; i < RowNum; i++)
			{
				for (j = 0; j < ColumnNum; j++)
				{
					outfile << Best_Pop[l][i][j] << " ";
				}

				outfile << endl;
			}
		}
		outfile << endl;
		outfile.close();

		outfile.open("Results.csv", ios::app);
		outfile << totalobj  << endl;
		outfile.close();
	}
	return 0;
}
*/


int GetChromosomeSize()
{
	return (Sched_Time / Win) * RowNum * ColumnNum;
}


void AllocateNewRepresentation()
{
	int chromosomeSize = GetChromosomeSize(), i = 0;
	New_Pop_2 = new int*[PopSize];
	for (i = 0; i < PopSize; i++)
	{
		New_Pop_2[i] = new int[chromosomeSize];
		memset(New_Pop_2[i], 0, chromosomeSize * sizeof(int));
	}
}


void DeallocateNewRepresentation()
{
	int i = 0;
	for (i = 0; i < PopSize; i++)
	{
		delete[] New_Pop_2[i];
	}
	delete[] New_Pop_2;
}


void NewPopToNewPop2()
{
	int candidate = 0, k = 0, r = 0, c = 0, newIndex;
	for (candidate = 0; candidate < PopSize; candidate++)
	{
		for (k = 0; k < Sched_Time / Win; k++)
		{
			for (r = 0; r < RowNum; r++)
			{
				for (c = 0; c < ColumnNum; c++)
				{
					newIndex = k * RowNum * ColumnNum + r * ColumnNum + c;
					New_Pop_2[candidate][newIndex] = New_Pop[candidate][k][r][c];
				}
			}
		}
	}
}


void NewPopFromNewPop2()
{
	int candidate = 0, k = 0, r = 0, c = 0, newIndex;
	for (candidate = 0; candidate < PopSize; candidate++)
	{
		for (k = 0; k < Sched_Time / Win; k++)
		{
			for (r = 0; r < RowNum; r++)
			{
				for (c = 0; c < ColumnNum; c++)
				{
					newIndex = k * RowNum * ColumnNum + r * ColumnNum + c;
					New_Pop[candidate][k][r][c] = New_Pop_2[candidate][newIndex];
				}
			}
		}
	}
}


/*
	It evaluates the candidate solutions in Pop[]
	It saves the scores in Total_Obj[]
	It updates the best solution saved in Best_Pop
	It updates the best score in totalobj
*/
void EvaluatePopulationAndUpdateBest()
{
	int i, j, k;
	for (popi = 0; popi < PopSize; popi++)
	{
		for (i = 0; i < RowNum + 1; i++)
		{
			for (j = 0; j < ColumnNum + 1; j++)
			{
				C_Row[i][j] = C_Row_Initial[i][j];
				C_Column[j][i] = C_Column_Initial[j][i];
				C_Row1[i][j] = C_Row1_Initial[i][j];
				C_Column1[j][i] = C_Column1_Initial[j][i];
			}
		}
		for (K = 0; K < Sched_Time / Win; K++)
		{
			if (K > 0)
			{
				New_Update_C();
			}
			Calculate_TotalNum();
			Caculate_Obj();
			Update_C();

		}
		Total_Obj[popi] = Obj[popi];
		if (popi == 0)
		{
			totalobj = Total_Obj[popi];
			I = popi;
			for (i = 0; i < Sched_Time / Win; i++)
			{
				for (j = 0; j < RowNum; j++)
				{
					for (k = 0; k < ColumnNum; k++)
					{
						Best_Pop[i][j][k] = Pop[popi][i][j][k];
					}
				}
			}
		}
		else
		{
			if (totalobj > Total_Obj[popi])
			{
				totalobj = Total_Obj[popi];
				I = popi;
				for (i = 0; i < Sched_Time / Win; i++)
				{
					for (j = 0; j < RowNum; j++)
					{
						for (k = 0; k < ColumnNum; k++)
						{
							Best_Pop[i][j][k] = Pop[popi][i][j][k];
						}
					}
				}
			}
		}
	}
}


/*
* Copies the existing population in the New_Pop vector
*/

int ComputeFitness(int* chromosome) {
	int chromosomeSize = GetChromosomeSize();
	int fitness = 0;
	for (int i = 0; i < chromosomeSize; i++) {
		fitness += chromosome[i];
	}
	return fitness;
}

void RouletteSelect()
{
	struct fitness {
		int index;
		int fitness_;
	};
	struct fitness fitness_computed[PopSize] = { 0 };

	int candidate = 0, k = 0, r = 0, c = 0, newIndex;
	for (candidate = 0; candidate < PopSize; candidate++)
	{
		for (k = 0; k < Sched_Time / Win; k++)
		{
			for (r = 0; r < RowNum; r++)
			{
				for (c = 0; c < ColumnNum; c++)
				{
					newIndex = k * RowNum * ColumnNum + r * ColumnNum + c;
					New_Pop_Temp[candidate][newIndex] = Pop[candidate][k][r][c];
				}
			}
		}
	}


	int sumFitness = 0;

	for (int candidate = 0; candidate < PopSize; candidate++)
	{
		fitness_computed[candidate].index = candidate;
		fitness_computed[candidate].fitness_ = ComputeFitness(New_Pop_Temp[candidate]);
		sumFitness += fitness_computed[candidate].fitness_;
	}

	int index_selected = 0;
	for (int i = 0; i < PopSize; i++) {
		float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / sumFitness));
		float newSum = 0;
		for (int j = 0; j < PopSize; j++) {
			newSum += fitness_computed[j].fitness_;
			//stop when the new sum is higher than the random number
			if (newSum > r) {
				New_Pop_2[fitness_computed[j].index] = New_Pop_Temp[fitness_computed[j].index];
				break;
			}
		}
	}

	

}


void Mutate(int mutation_prob)
{
	int candidate = 0, random_gene = 0, chance = 0;
	int chromosome_size = GetChromosomeSize();
	int* chromosome;
	for (candidate = 0; candidate < PopSize; candidate++)
	{
		chromosome = New_Pop_2[candidate];
		chance = rand() % 100;
		if (chance >= mutation_prob)
		{
			continue;
		}
		random_gene = rand() % chromosome_size;
		chromosome[random_gene] = (chromosome[random_gene] + (rand() % 3 + 1)) % 4;
	}
}


void Crossover(int crossover_prob)
{
	struct chance {
		int index;
		int prob;
	};
	struct chance chances[PopSize] = { 0 };
	auto comparator = [](struct chance first, struct chance second) -> bool {
		return first.prob < second.prob;
	};
	int candidate = 0;
	for (candidate = 0; candidate < PopSize; candidate++)
	{
		chances[candidate].index = candidate;
		chances[candidate].prob = rand() % 100;
	}
	std::sort(std::begin(chances), std::end(chances), comparator);
	int i = 0, j = 0, temp = 0, cutpoint = 0, chromosomeSize = 0;
	chromosomeSize = GetChromosomeSize();
	for (i = 0; i < PopSize - 1; i += 2)
	{
		if (chances[i].prob >= crossover_prob)
			break;
		int* firstParent = New_Pop_2[chances[i].index];
		int* secondParent = New_Pop_2[chances[i + 1].index];
		cutpoint = rand() % (chromosomeSize - 1) + 1;
		for(j = 0; j < cutpoint; j++)
		{
			std::swap(firstParent[i], secondParent[i]);
		}
	}
}


int main()
{
	srand((unsigned)time(NULL) + MaxRowNum * MaxColumnNum * 1000000);

	int i, j, k, Sum, rep, l;
	Read_GenRepeat();	// Get the number of iterations of the algorithm and the number of repetitions of the experiment
	for (rep = 0; rep < Repeat; rep++)
	{
		Read_Instance();
		Read_New_car();
		Read_Exit_TurnRatio();
		Read_Entrance_Ratio();
		Read_Lij();
		Read_Lanes();
		Read_L();
		time_t start = clock();
		
		int generation = 0;
		int MAX_GENERATIONS = 100;
		Initi_Pop();
		EvaluatePopulationAndUpdateBest();
		AllocateNewRepresentation();
		for (generation = 0; generation < MAX_GENERATIONS; generation++)
		{
			RouletteSelect();
			//NewPopToNewPop2(); // am transformat deja in formatul "NewPop2"
			Mutate(5);
			Crossover(30);
			NewPopFromNewPop2();
			memcpy(Pop, New_Pop, sizeof(New_Pop));
			EvaluatePopulationAndUpdateBest();
		}
		DeallocateNewRepresentation();

		outfile.open("Best Solution.csv", ios::app);
		for (l = 0; l < Sched_Time / Win; l++)
		{
			for (i = 0; i < RowNum; i++)
			{
				for (j = 0; j < ColumnNum; j++)
				{
					outfile << Best_Pop[l][i][j] << " ";
				}

				outfile << endl;
			}
		}
		outfile << endl;
		outfile.close();

		outfile.open("Results.csv", ios::app);
		outfile << totalobj << endl;
		outfile.close();
	}
	return 0;
}
