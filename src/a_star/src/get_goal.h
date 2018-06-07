#include<iostream>
#include<fstream>
using namespace std;

void get_goal_from_file(int &x,int &y,const char* GOAL_PATH){
    std::ifstream file(GOAL_PATH,ios::in);
    file>>x;
    file>>y;
    return;
}