/*

*Team Id: 3302
*Author List: MISHAEL THOMAS, SAURAV PANDEY, AYUSH PAREEK, PULKIT SHARMA
*Filename: CB_Task_1_Sandbox.h
*Theme: Construct-O-Bot (eYRC 2019-20)

*Functions:  forward_wls(unsigned char node),left_turn_wls(void),right_turn_wls(void), e_shape(void),Task_1_1(void),Task_1_2(),nod(),
			invert(),broken_line(),insert_at(location* header, int weight, struct adjlist** vertex_list),make_adjlist(),
			follow_wall(),initiate_location(),swap(location** a, location** b),heapify(location* array[], int size, int i),
			insert(location* array[], location* newNum),deleteRoot(location* array[], location* num),
			optimal_dist(location current, location goal),checkclosed_list(location* arr[], location* ptr),
			short_path(location start, location goal),traverse(),pick_obj(),place_obj()

*Global Variables: extern unsigned int color_sensor_pulse_count

*/
#include "CB_Task_1_Predef.h"

extern unsigned int color_sensor_pulse_count;

class location;  // prototype for a class defined later and to be used in struct adjlist

void forward_wls(unsigned char node);
void left_turn_wls(void);
void right_turn_wls(void);
void e_shape(void);
void Task_1_1(void);
void Task_1_2(void);
void nod();
void invert();
void broken_line();
void insert_at(location* header, int weight, struct adjlist** vertex_list); 
void make_adjlist();
void follow_wall(); 
void initiate_location(); void swap(location** a, location** b); 
void heapify(location* array[], int size, int i);
void insert(location* array[], location* newNum); 
void deleteRoot(location* array[], location* num);
float optimal_dist(location current, location goal); 
bool checkclosed_list(location* arr[], location* ptr);
void short_path(location *start, location *goal);
void traverse();
void pick_obj();
void place_obj();
