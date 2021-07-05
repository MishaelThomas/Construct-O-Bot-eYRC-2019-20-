/*

*Team Id: 3302
*Author List: MISHAEL THOMAS, SAURAV PANDEY, AYUSH PAREEK, PULKIT SHARMA
*Filename: CB_Task_1_Sandbox.cpp
*Theme: Construct-O-Bot 
*Functions: forward_wls(unsigned char node),left_turn_wls(void),right_turn_wls(void), e_shape(void),Task_1_1(void),Task_1_2(),nod(),
			invert(),broken_line(),insert_at(location* header, int weight, struct adjlist** vertex_list),make_adjlist(),
			follow_wall(),initiate_location(),swap(location** a, location** b),heapify(location* array[], int size, int i),
			insert(location* array[], location* newNum),deleteRoot(location* array[], location* num),
			optimal_dist(location current, location goal),checkclosed_list(location* arr[], location* ptr),
			short_path(location start, location goal),traverse(),pick_place()

Global Variables: counter, count_node, turning_velocity, forward_velocity,bridge_counter,bridge_limit,size,z,path[16]

*/

#include "CB_Task_1_Sandbox.h"

int counter, count_node = 0, turning_velocity = 60, forward_velocity = 255, bridge_counter = 0, size = 0, z, flag_right = 0, flag_left = 0,goal_x,goal_y;
double bridge_limit = 500000;  int pl13 = 0; int pl24 = 0; int direction = 0;
location* path[16];


/*
VARIABLE DESCRIPTION
	counter: used to count the number of nodes passed by the bot and stop according to the argument of forward_wls()
	count_node: counts the number of nodes passed
	turning_velocity: velocity of motors while taking turns
	forward_velocity:velocity of motors while moving forward
	bridge_counter: oscillates between the limit
	bridge_limit:limit for the angle of turn taken by the bot to find the broken line
	*path[16]: a pointer array that can store the address of an object of class location (this class is defined later in the function)
	size:size of binary heap's array
	z:counter variable of for loop for storing the shortest path in *path[16]
*/


/*

Function Name: nod
* Input:  void
* Output: void
* Logic: increments the value of count_node at the end of the node
* Example Call: nod();

*/
void nod()
{
	// After node has been traversed, the values of counter and count_node variables is incremented

	counter++;
	count_node++;
	
	// passing through the node

	forward();
	_delay_ms(200);

}

/*

Function Name: invert
* Input:  void
* Output: void
* Logic: used to run the bot on the white line according to the sensor values using the same concept of forward_wls()
* Example Call: invert()

*/

void invert()
{
	while (1)
	{
		if (ADC_Conversion(1) == 255 && ADC_Conversion(3) == 0)  // condition when the bot should take right so as to stay in the white line
		{

			right();
			velocity(turning_velocity, turning_velocity);
		}

		else if (ADC_Conversion(3) == 255 && ADC_Conversion(1) == 0)  // condition when the bot should take left so as to stay in the white line
		{

			left();
			velocity(turning_velocity, turning_velocity);
		}

		else if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 0)
			break;   // when black line is to be followed, control is sent back to the forward_wls()

		else if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0)
			nod();

		else  // when above conditions are not satisfied, the bot oves ahead
		{
			forward();
			velocity(forward_velocity, forward_velocity);
		}
	}
}

/*
Function Name: follow_wall
* Input:  void
* Output: void
* Logic: used to run the bot in between the two walls using the values of left and right proximity sensors
* Example Call: follow_wall()
*/
void follow_wall()
{
	
	while (1)
	{
		if (ADC_Conversion(1) == 255 && ADC_Conversion(3) == 0)  // condition when the bot should take soft_left to be in the black line
		{

			soft_left();
			velocity(0, turning_velocity);  // adjusting the velocity to take turn
			_delay_ms(1);
		}

		else if (ADC_Conversion(3) == 255 && ADC_Conversion(1) == 0)  // condition when the bot should take soft_right to be in the black line
		{

			soft_right();
			velocity(turning_velocity, 0);
			_delay_ms(1);
		}
		else if (ADC_Conversion(1) == 255 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 255)  // condition when the bot encounters a node and nod() is called
		{
			nod();
			break;
		}
		else if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 0)  // condition for moving ahead
		{
			forward();
			velocity(forward_velocity, forward_velocity);
		}
		else if ((ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0)&&(ADC_Conversion(5)!=32 && ADC_Conversion(6!=32)))
		{
			
			
				while (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0)
				{
					if (ADC_Conversion(5) == 32 && ADC_Conversion(6)>90)
					{

						soft_left();
						velocity(0, turning_velocity); // adjusting the velocity to take turn
						_delay_ms(1);

					}
					else if (ADC_Conversion(6) == 32 && ADC_Conversion(5) > 80)
					{

						soft_right();
						velocity(turning_velocity, 0); // adjusting the velocity to take turn
						_delay_ms(1);

					}
					if (ADC_Conversion(5) > 110 && ADC_Conversion(6) < 90)
					{

						soft_left();
						velocity(0, turning_velocity);  // adjusting the velocity to take turn
						_delay_ms(1);
					}
					else if (ADC_Conversion(6) > 110 && ADC_Conversion(5) < 90)
					{

						soft_right();
						velocity(turning_velocity, 0); // adjusting the velocity to take turn
						_delay_ms(1);
					}
					else
					{
						forward();
						velocity(forward_velocity, forward_velocity);

					}
					
				}
			
		}

	}

}

/*

* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes

*/

void forward_wls(unsigned char node)
{
	counter = 0;  // counter variable controls the following loop

	// Below provided loop runs until the specified number of nodes have been covered

	while (counter != node)
	{

		if (ADC_Conversion(1) == 255 && ADC_Conversion(3) == 0)  // condition when the bot should take soft_left to be in the black line
		{

			soft_left();
			velocity(0, turning_velocity);  // adjusting the velocity to take turn
			_delay_ms(1);
		}

		else if (ADC_Conversion(3) == 255 && ADC_Conversion(1) == 0)  // condition when the bot should take soft_right to be in the black line
		{

			soft_right();
			velocity(turning_velocity, 0);
			_delay_ms(1);
		}
		else if (ADC_Conversion(1) == 255 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 255)  // condition when the bot encounters a node and nod() is called
			nod();

		else if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 0)  // condition for moving ahead
		{
			forward();
			velocity(forward_velocity, forward_velocity);
		}


		else if (ADC_Conversion(1) == 255 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 255)
			invert();	// calls the invert() in the above given condition


	}
}
/*

Function Name: broken_line
* Input:  void
* Output: void
* Logic: using the values of white line sensors, it turns and searches for line and then moves along the found line
* Example Call: broken_line();

*/

void broken_line()
{
	while (1)
	{
		// concept used in broken line is same as forward_wls() for traversing the black line

		if (ADC_Conversion(1) == 255 && ADC_Conversion(3) == 0)
		{

			soft_left();
			velocity(0, turning_velocity);
			_delay_ms(1);
		}

		else if (ADC_Conversion(3) == 255 && ADC_Conversion(1) == 0)
		{

			soft_right();
			velocity(turning_velocity, 0);
			_delay_ms(1);
		}
		else if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 0)
		{
			forward();
			velocity(forward_velocity, forward_velocity);
		}
		//  below provided condition checks for a black line by turning left and right by few degrees

		else if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0)
		{
			for (bridge_counter = 0; (bridge_counter < 600000) && (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0); bridge_counter++)
			{
				left();
				velocity(100, 100);
			}
			for (bridge_counter = 0; (bridge_counter < 2 * 600000) && (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0); bridge_counter++)
			{
				right();
				velocity(100, 100);
			}
			for (bridge_counter = 0; (bridge_counter < 600000) && (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0); bridge_counter++)
			{
				left();
				velocity(100, 100);
			}
		}
		// when the nod is detected the control is sent back to forward_wls()

		else if (ADC_Conversion(1) == 255 && ADC_Conversion(2) == 255 && ADC_Conversion(3) == 255)
		{
			nod();
			break;
		}
	}
}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/

void left_turn_wls(void)
{
	while (1)
	{
		left();
		velocity(100, 100);
		if (ADC_Conversion(1) == 255 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 0)
			break;
	}
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/

void right_turn_wls(void)
{
	while (1)
	{
		right();
		velocity(100, 100);
		if (ADC_Conversion(1) == 0 && ADC_Conversion(2) == 0 && ADC_Conversion(3) == 255)
			break;
	}
}

/*
*
* Function Name: e_shape
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a e shape path on the arena
* Example Call: e_shape();
*/

void e_shape(void)
{
	forward_wls(1);
	right_turn_wls();
	forward_wls(2);
	right_turn_wls();
	forward_wls(1);
	right_turn_wls();
	forward_wls(1);
	right_turn_wls();
	forward_wls(1);
}

/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*/

void Task_1_1(void)
{
	forward_wls(1);
	right_turn_wls();
	forward_wls(2);
	left_turn_wls();
	
	forward_wls(1);
	
	forward_wls(1);
	left_turn_wls();
	forward_wls(1);
	right_turn_wls();
	
	broken_line();
	left_turn_wls();
	
	forward_wls(3);
	right_turn_wls();
	forward_wls(1);
	right_turn_wls();
	forward_wls(1);
	left_turn_wls();
	
	forward_wls(1);
	right_turn_wls();
	
	forward_wls(2);
	left_turn_wls();
	forward_wls(1);

}

// the below provided structure helps in creating an adjacency list which has connects the vertices in vertex list to the list of their neighbours

struct adjlist
{
	location* data;
	int weight;
	struct adjlist* next;
};

// the below provided class cretes the nodes of the vertex list

class location
{
public:
	int x_cord; // abscissa
	int y_cord; // ordinate
	float global; // global value in A* search algorithm
	char thing; // specification of whether the node is a warehouse, house or a normal node
	adjlist* neighbours; //header pointer to its neighbour..........
	int local; // local value in A* search algorithm
	location* parent; // pointer to the parent in shortest path's list

	location(int abscissa, int ordinate, char specification) // parametrized constructor of the class that initialises the abscissa, ordinate,local value, global value,parent and specification for an oject made
	{
		x_cord = abscissa;
		y_cord = ordinate;
		global = 0.00;
		local = 999;
		parent = NULL;
		thing = specification;
	}
};

// creating a header pointer for each vertex to the adjacency list
struct adjlist* A_neigh = NULL, * B_neigh = NULL, * C_neigh = NULL, * D_neigh = NULL, * E_neigh = NULL, * F_neigh = NULL, * G_neigh = NULL, * H_neigh = NULL, * I_neigh = NULL, * J_neigh = NULL, * K_neigh = NULL, * L_neigh = NULL, * M_neigh = NULL, * N_neigh = NULL, * O_neigh = NULL, * P_neigh = NULL;

// creating an array of objects that represent the nodes
location A[16] = {  {0, 0,'n'},{0, 1,'B'},{0, 2,'1'}, {0, 3,'C'}, {0, 4,'3'},{0, 5,'E'}, {0, 6,'n'}, {1, 6,'5'}, {2, 6,'n'}, {2, 5,'P'}, {2, 4,'4'}, {2, 3,'S'},{2, 2,'2'},{2, 1,'G'},{2, 0,'n'},{1, 0,'n'} };

/*
*
* Function Name: insert_at
* Input: header, weight, vertex_list
* Output: void
* Logic: creates a linked list for a particular header, and every linked list consists of neighbours of a node
* Example Call: insert_at(&A[15], 20, &A_neigh);
*/

void insert_at(location* header, int weight, struct adjlist** vertex_list)
{
	struct adjlist* temp, * temp_node;

	temp = (struct adjlist*)malloc(sizeof(struct adjlist));  // dynamic memory allocation for new node


	if (*vertex_list == NULL) // when first node is inserted
	{
		*vertex_list = temp;
		**vertex_list = { header,weight,NULL };
		return;
	}

	temp_node = *vertex_list;

	while (temp_node->next != NULL)  // traversing towards last node in the list
		temp_node = temp_node->next;

	temp_node->next = temp;
	*temp = { header,weight,NULL };
}

/*
*
* Function Name: make_adjlist()
* Input: void
* Output: void
* Logic: this function simply creates an adjacency list
* Example Call: make_adjlist();
*/

void make_adjlist()
{
	// calling insert_at function so as to create the adjacency list 

	insert_at(&A[15],12, &A_neigh);
	insert_at(&A[1], 10, &A_neigh);
	insert_at(&A[2], 10, &B_neigh);
	insert_at(&A[0], 10, &B_neigh);
	insert_at(&A[3], 10, &C_neigh);
	insert_at(&A[1], 10, &C_neigh);
	insert_at(&A[12], 60, &C_neigh);
	insert_at(&A[2], 10, &D_neigh);
	insert_at(&A[4], 10, &D_neigh);
	insert_at(&A[3], 10, &E_neigh);
	insert_at(&A[5], 10, &E_neigh);
	insert_at(&A[10], 50, &E_neigh);
	insert_at(&A[4], 10, &F_neigh);
	insert_at(&A[6], 10, &F_neigh);
	insert_at(&A[5], 10, &G_neigh);
	insert_at(&A[7], 14, &G_neigh);
	insert_at(&A[6], 14, &H_neigh);
	insert_at(&A[8], 14, &H_neigh);
	insert_at(&A[9], 10, &I_neigh);
	insert_at(&A[7], 14, &I_neigh);
	insert_at(&A[8], 10, &J_neigh);
	insert_at(&A[10], 10, &J_neigh);
	insert_at(&A[11], 10, &K_neigh);
	insert_at(&A[9], 10, &K_neigh);
	insert_at(&A[4], 50, &K_neigh);
	insert_at(&A[10], 10, &L_neigh);
	insert_at(&A[12], 10, &L_neigh);
	insert_at(&A[11], 10, &M_neigh);
	insert_at(&A[13], 10, &M_neigh);
	insert_at(&A[2], 60, &M_neigh);
	insert_at(&A[14], 10, &N_neigh);
	insert_at(&A[12], 10, &N_neigh);
	insert_at(&A[13], 10, &O_neigh);
	insert_at(&A[15], 12, &O_neigh);
	insert_at(&A[0], 12, &P_neigh);
	insert_at(&A[14], 12, &P_neigh);
	A[0].neighbours = A_neigh;
	A[1].neighbours = B_neigh;
	A[2].neighbours = C_neigh;
	A[3].neighbours = D_neigh;
	A[4].neighbours = E_neigh;
	A[5].neighbours = F_neigh;
	A[6].neighbours = G_neigh;
	A[7].neighbours = H_neigh;
	A[8].neighbours = I_neigh;
	A[9].neighbours = J_neigh;
	A[10].neighbours = K_neigh;
	A[11].neighbours = L_neigh;
	A[12].neighbours = M_neigh;
	A[13].neighbours = N_neigh;
	A[14].neighbours = O_neigh;
	A[15].neighbours = P_neigh;
}

/*
*
* Function Name: initiate_location
* Input: void
* Output: void
* Logic: this function is used to restore the global, local and parent values for each vertex
* Example Call: initiate_location();
*/

void initiate_location()
{
	for (int iter_var = 0; iter_var <= 16; iter_var++)
	{
		A[iter_var].global = 0;
		A[iter_var].local = 999;
		A[iter_var].parent = NULL;
	}
}

//for priority que in form of binary heap...................................

/*
*
* Function Name: swap
* Input: **a, **b
* Output: void
* Logic: this function swaps the values of two elements of the array
* Example Call: swap(&array[i], &array[largest]);
*/
void swap(location** a, location** b)
{
	location* temp = *b;
	*b = *a;
	*a = temp;
}

/*
*
* Function Name: heapify
* Input: *array[],size,i
* Output: void
* Logic: searches for the location with the smallest global value in a binary heap and transeferes it to the root of the binary heap
* Example Call: heapify(array, size, largest);
*/
void heapify(location* array[], int size, int i)
{
	if (size != 1)
	{
		int largest = i;
		int l = 2 * i + 1;
		int r = 2 * i + 2;
		if (l < size && (*array[l]).global < (*array[largest]).global)
			largest = l;
		if (r < size && (*array[r]).global < (*array[largest]).global)
			largest = r;
		if (largest != i)
		{
			swap(&array[i], &array[largest]);
			heapify(array, size, largest);
		}
	}
}

/*
*
* Function Name: insert
* Input: *array[],* newNum
* Output: void
* Logic: stores the address of a given location in binary heap
* Example Call: insert(openlist, &start);
*/
void insert(location* array[], location* newNum)
{
	if (size == 0)
	{
		array[0] = newNum;
		size += 1;
	}
	else
	{
		array[size] = newNum;
		size += 1;
		for (int i = (size / 2) - 1; i >= 0; i--)
		{
			heapify(array, size, i);
		}
	}
}

/*
*
* Function Name: deleteroot
* Input: *array[], *num
* Output: void
* Logic: this function removes the processed node from the openlist and heapifies the binary heap to get the address of the location at root having minimum global value having
* Example Call: deleteRoot(openlist, temp);
*/

void deleteRoot(location* array[], location* num)
{
	int i;
	for (i = 0; i < size; i++)
	{
		if (num == array[i])
			break;
	}
	swap(&array[i], &array[size - 1]);
	size -= 1;
	for (int i = (size / 2) - 1; i >= 0; i--)
	{
		heapify(array, size, i);
	}
}

//for A* search algorithm...........................................................

/*
*
* Function Name: optimal_dist
* Input: current,goal
* Output: float
* Logic: calculates the manhattan heuristic value from current location to goal
* Example Call: distance=optimal_dist(start, goal)
*/

float optimal_dist(location current, location goal)
{
	float r = sqrt(pow((current.x_cord - goal.x_cord), 2) + pow((current.y_cord - goal.y_cord), 2));
	return r;
}

/*
*
* Function Name: checkclosed_list
* Input: *arr[],*ptr
* Output: bool
* Logic: checks the closed list that the current location is processed or not
* Example Call: value=checkclosed_list(closedlist, (ptr->data))
*/

bool checkclosed_list(location* arr[], location* ptr)// for checking a closedlist
{
	int i = 0;
	while (i <= size) {
		if (ptr == arr[i])
			return true;
		i++;
	}
	return false;
}


/*
*
* Function Name: short_path
* Input: start,goal
* Output: void
* Logic:determines the shortest path between start location and goal location using A* search algorithm and stores that path in *path array
* Example Call: short_path(A[0],A[3]);
*/
void short_path(location* start, location* goal)
{
	path[16] = { NULL };
	int i = 0;
	size = 0;
	int closize = 0;
	adjlist* ptr;
	location* openlist[10];
	location* closedlist[16];
	(*(start)).local = 0;
	(*(start)).global = (*(start)).local + optimal_dist(*start, *goal);

	insert(openlist, start);
	while (openlist != NULL && !((*openlist[0]).x_cord == (*goal).x_cord && (*openlist[0]).y_cord == (*goal).y_cord))
	{
		location* temp = openlist[0];
		deleteRoot(openlist, temp);  //popping processed node 
		for (ptr = (*temp).neighbours; ptr != NULL; ptr = ptr->next)
		{
			if (!checkclosed_list(closedlist, (ptr->data)))
			{
				int tentative_local = (*temp).local + ptr->weight;
				if (tentative_local < (*(ptr->data)).local)
				{
					(*(ptr->data)).local = tentative_local;
					(*(ptr->data)).global = (*(ptr->data)).local + optimal_dist(*(ptr->data), *goal);
					(*(ptr->data)).parent = temp;
					insert(openlist, ptr->data);

				}
			}
		}
		closedlist[closize] = temp;
		closize++;
	}
	if (openlist == NULL)
		std::cout << "no path";
	else
	{
		path[0] = openlist[0];

		for (z = 0; (*path[z]).parent != NULL; )
			path[++z] = (*path[z - 1]).parent;
	}
}

/*
*
* Function Name: traverse
* Input: void
* Output: void
* Logic: traverses the *path array and processes each node
* Example Call: traverse();
*/
void traverse() {



	for (int k = 0; k < z; k++)
	{
		if ((*path[k]).x_cord == (*path[k + 1]).x_cord)
		{
			if ((*path[k]).y_cord < (*path[k + 1]).y_cord) {
				//	follow_wall(); // moving 
				forward_wls(1);
				direction = 1;
			}
			else {
				left_turn_wls();
				left_turn_wls();
				forward_wls(1);
				direction = 0;
			}
		}
		else {
			if ((*path[k]).x_cord < (*path[k + 1]).x_cord) {
				if (direction == 1) {
					right_turn_wls();
				}
				else {
					left_turn_wls();
				}
			}
			else {
				if (direction == 1) {
					left_turn_wls();
				}
				else {
					right_turn_wls();
				}
			}
			if ((*path[k + 1]).y_cord == 0) {
				forward_wls(2);
			}
			else if ((*path[k + 1]).y_cord == 2) {
				follow_wall();
			}
			else if ((*path[k + 1]).y_cord == 4) {
				forward_wls(1);
			}
			else if ((*path[k + 1]).y_cord == 6) {
				invert();
			}
		}
	}


}
/*
*
* Function Name: pick_place
* Input: void
* Output: void
* Logic: traverses from the graph vertices to the final positions
* Example Call: pick_place();
*/

void pick_obj() {
	if ((*(path[0])).thing == 'B' || (*(path[0])).thing == 'G' || (*(path[0])).thing == 'C' || (*(path[0])).thing == 'P' || (*(path[0])).thing == 'E' || (*(path[0])).thing == 'S') // for left line
	{
		left_turn_wls();
		forward_wls(1);
		forward();
		_delay_ms(15);
		if (ADC_Conversion(4) > 32)
		{
			pick();
			left_turn_wls();
			forward_wls(1);
			if ((*path[0]).y_cord > (*path[1]).y_cord) // when come from below
			{
				flag_right = 1;
				flag_left = 0;
			}
			else if ((*path[0]).y_cord < (*path[1]).y_cord)//when come from above
			{
				flag_right = 0;
				flag_left = 1;

			}



		}
		else
		{
			left_turn_wls();
			forward_wls(2);
			
			pick();
			left_turn_wls();
			forward_wls(1);
			if ((*path[0]).y_cord > (*path[1]).y_cord) // when come from below
			{
				flag_right = 0;
				flag_left = 1;
			}
			else if ((*path[0]).y_cord < (*path[1]).y_cord)//when come from above
			{
				flag_right = 1;
				flag_left = 0;

			}

		}
	}
}

void place_obj()
{
	if ((*(path[0])).thing == '1' || (*(path[0])).thing == '3')
	{
		if (pl13 > 1)
			pl13 = 0;

		if ((*path[0]).y_cord > (*path[1]).y_cord) // when come from below in left line
		{
			left_turn_wls();
			forward_wls(1);
			if (pl13 == 0) {
				soft_left();
				_delay_ms(120);
			}
			else
			{
				soft_right();
				_delay_ms(150);
			}
			pl13++;
			place();
			if (pl13 > 1) {
				soft_left();
				_delay_ms(120);
			}
			left_turn_wls();
			forward_wls(1);

		}
		else if ((*path[0]).y_cord < (*path[1]).y_cord) // when come from above in left line
		{
			right_turn_wls();
			forward_wls(1);
			if (pl13 == 0) {
				soft_left();
				_delay_ms(120);
			}
			else
			{
				soft_right();
				_delay_ms(120);
			}
			pl13++;
			place();
			if (pl13 > 1) {
				soft_left();
				_delay_ms(120);
			}
			left_turn_wls();
			forward_wls(1);
		}
		else if ((*path[0]).y_cord == (*path[1]).y_cord) // when come from above in left line
		{


			forward_wls(1);
			if (pl13 == 0) {
				soft_left();
				_delay_ms(120);
			}
			else
			{
				soft_right();
				_delay_ms(120);

			}

			pl13++;
			place();
			if (pl13 > 1) {
				soft_left();
				_delay_ms(120);

			}
			
			left_turn_wls();
			forward_wls(1);
			
		}
	}
	else if ((*(path[0])).thing == '4' || '2')
	{
		if (pl24 > 1)
			pl24 = 0;

		if ((*path[0]).y_cord > (*path[1]).y_cord) // when come from below in right line
		{
			right_turn_wls();
			forward_wls(1);
			if (pl24 == 0) {
				soft_left();
				_delay_ms(120);
			}
			else
			{
				soft_right();
				_delay_ms(150);
			}

			pl24++;
			place();
			if (pl24 > 1) {
				soft_left();
				_delay_ms(120);
			}
			left_turn_wls();
			forward_wls(1);

		}
		else if ((*path[0]).y_cord < (*path[1]).y_cord) // when come from above in right line
		{
			left_turn_wls();
			forward_wls(1);
			if (pl24 == 0) {
				soft_left();
				_delay_ms(120);
			}
			else
			{
				soft_right();
				_delay_ms(150);
			}

			pl24++;
			place();
			if (pl24 > 1) {
				soft_left();
				_delay_ms(120);
			}
			left_turn_wls();
			forward_wls(1);
		}
		else if ((*path[0]).y_cord == (*path[1]).y_cord) // when come from left line
		{

			forward_wls(1);
			if (pl24 == 0) {
				soft_left();
				_delay_ms(120);
			}
			else
			{
				soft_right();
				_delay_ms(150);
			}
			pl24++;
			place();
			if (pl24 > 1) {
				soft_left();
				_delay_ms(120);
			}
			left_turn_wls();
			forward_wls(1);
		}
	}

}
/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to accomplish Task_1_2
* Example Call: Task_1_2();
*/
void Task_1_2(void)
{
	//write your task 1.2 logic here

	// code for house 3
	forward_wls(1);
	left_turn_wls();
	make_adjlist();
	short_path(&A[10], &A[3]);
	for (z = 0; path[z] != NULL;z++ )
	{
		std::cout << path[z]->thing<<" ";
	}
	/*traverse();
	
	pick_obj();

	initiate_location();
	short_path(&A[12], &A[5]);
	traverse();

	place_obj();
	initiate_location();
	short_path(&A[5], &A[14]);
	traverse();

	pick_obj();
	initiate_location();
	short_path(&A[14], &A[5]);
	traverse();
	place_obj();
	
	*/
}
