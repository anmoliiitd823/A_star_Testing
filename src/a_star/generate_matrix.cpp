#include<iostream>
#include<fstream>
#include<cmath>
#include<stdlib.h>
#include <sstream>
#include<string>
#include <vector>
using namespace std;

const int MATRIX_WIDTH = 501;
const int MATRIX_LENGTH = 501;
const int OBSTACLE_COST = 9;
const int DIST_EFFECT_OBST = 3;
const int LANE_WIDTH = 3;

string INPUT_MATRIX_PATH = "~/Desktop/A_star_Testing/src/a_star/matrix.txt";

float euclid_dist(int x1,int y1,int x2,int y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void generate_random_binary_matrix(int** a){
	for(int i=0;i<MATRIX_LENGTH;i++){
		for(int j=0;j<MATRIX_WIDTH;j++){
			int x = rand()%1000;
			if(x==10)
				a[i][j]=OBSTACLE_COST;
			else
				a[i][j]=0;
		}
	}
}

void random_lane_costs(int** a){
	int i=0;
	while(i<MATRIX_LENGTH){
		//cout<<i<<endl;
		int temp_cost = rand()%2;
		for(int j=i;j<i+LANE_WIDTH;j++){
			for(int k=0;k<MATRIX_WIDTH;k++){
				if(a[j][k]==0)
					a[j][k] = temp_cost;
			}
		}
		i+=LANE_WIDTH;
	}
}
void get_matrix_from_file(int** a){
	std::vector<std::vector<int> >     data;
    std::ifstream file("/Users/anmol/Desktop/A_star_Testing/src/a_star/matrix.txt",ios::in);
    std::string line;
    char next;
	while(file.get(next))
	{
	    if (next == '\n')  
	    {    break;        
	    }                  
	}
    while(std::getline(file, line))
    {
        std::vector<int>   lineData;
        std::stringstream  lineStream(line);
        int value;
        while(lineStream >> value)
        {
            lineData.push_back(value);
        }
        data.push_back(lineData);
    }
    for(int i=0;i<MATRIX_LENGTH;i++){
    	for(int j=0;j<MATRIX_WIDTH;j++){
    		cout<<data[i][j]<<" ";
    	}
    	cout<<endl;
    }
	
}

void normalize(int** a,int x,int y,float** matrix){
	int i=x,j=y;
	int count=DIST_EFFECT_OBST;

	//LEFT
	while(count>=0 && j>=1){
		j--;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	i=x,j=y;
	count=DIST_EFFECT_OBST;
	//RIGHT
	while(count>=0 && j<MATRIX_WIDTH-1){
		j++;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	i=x,j=y;
	count=DIST_EFFECT_OBST;
	//UP
	while(count>=0 && i>=1){
		i--;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	i=x,j=y;
	count=DIST_EFFECT_OBST;
	//DOWN
	while(count>=0 && i<MATRIX_LENGTH-1){
		i++;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	//DOWN-LEFT
	while(count>=0 && j>=1 && i<MATRIX_LENGTH-1){
		j--;
		i++;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	i=x,j=y;
	count=DIST_EFFECT_OBST;
	//UP-RIGHT
	while(count>=0 && j<MATRIX_WIDTH-1 && i>=1){
		j++;
		i--;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	i=x,j=y;
	count=DIST_EFFECT_OBST;
	//UP-LEFT
	while(count>=0 && i>=1 && j>=1){
		i--;
		j++;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}

	i=x,j=y;
	count=DIST_EFFECT_OBST;
	//DOWN-RIGHT
	while(count>=0 && i<MATRIX_LENGTH-1 && j<MATRIX_WIDTH-1){
		i++;
		j++;
		matrix[i][j]+=OBSTACLE_COST/euclid_dist(i,j,x,y);
		count--;
	}
}

void weighted_matrix(int** a,float** matrix){
	for(int i=0;i<MATRIX_LENGTH;i++){
		for(int j=0;j<MATRIX_WIDTH;j++){
			if(a[i][j]==OBSTACLE_COST){
				normalize(a,i,j,matrix);
			}
		}
	}
}


int main(){
	ofstream outfile;
   	//outfile.open("matrix.txt");

   	int** a = new int*[MATRIX_LENGTH];
   	float** matrix = new float*[MATRIX_LENGTH];
   	for(int i=0;i<MATRIX_LENGTH;i++){
   		a[i] = new int[MATRIX_WIDTH];
   		matrix[i] = new float[MATRIX_WIDTH];
   	}

   	for(int i=0;i<MATRIX_LENGTH;i++){
   		for(int j=0;j<MATRIX_WIDTH;j++){
   			a[i][j]=0;
   			matrix[i][j]=0.0;
   		}
   	}
   	get_matrix_from_file(a);
   	//generate_random_binary_matrix(a);
   	//weighted_matrix(a,matrix);
   	//random_lane_costs(a);
//    	for(int i=0;i<MATRIX_LENGTH;i++){
//    		for(int j=0;j<MATRIX_WIDTH;j++){
//    			a[i][j] = a[i][j]+matrix[i][j];
//    			if(a[i][j]>OBSTACLE_COST){
//    				a[i][j] = OBSTACLE_COST;
//    			}
//    			outfile << a[i][j];
//    			outfile << " ";
//    		}
//    		outfile << endl;
//    	}
//   	outfile.close();
 }