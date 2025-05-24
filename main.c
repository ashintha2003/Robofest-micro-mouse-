#define F_CPU 16000000UL    // define CPU frequency as 16 MHz
#include <avr/io.h>         // include the standard AVR header file
#include <util/delay.h>     // include the delay loop functions
#include <avr/interrupt.h>  // include the interrupt file
//#include <stdlib.h>         //include the library
#define BAUD 9600           // Baud rate for serial communication
#include <math.h>
//define row and coloumn
#define row 16
#define coloumn 16
#define final_row 8
#define final_coloumn 8
#define length_and_width_of_the_cell 18
#define callibration_constant length_and_width_of_the_cell / 2 - 5.9
#define decrease_value_in_wall_adjustment 6
//assign the values for PID
#define Ki 0.01  //0.01
#define Kp 0.42  //0.41
#define Kd 0     //omit this part from the equation so let Kd = 0
#define PWM_linear_constant 0.56
//next cell count
#define max_count 653  //580      //rotations of the wheel for next cell
#define PWM_right_motor_max 48  //60
#define PWM_left_motor_max PWM_right_motor_max + 2//7
#define after_turn_PWM 35
//
#define adjustment_before_turn_PWM 25  //24
//rotation gyroscope
#define clockwise_set_value 400
#define anticlockwise_set_value -400
//wall adjustment while moving
#define wall_gap 3.26    //between the object and wall (this is the bit reading of the sensor
#define delay_wall 50  //delay time
//wall center before rotating
#define lowest_gap 313   //3cm
#define highest_gap 283  //3.5cm
#define turn_wheel_for_90_degrees 205
#define turn_wheel_for_180_degrees 440
#define PWM_max_value 160  //this is for the ICR1 register it should be the maximum
//setting the ranges
#define range_min 0.8
#define range_max 8
#define cell_center_adjustment_front_max 1.5
#define cell_center_adjustment_front_min 1.1
#define cell_center_adjustment_back_max 1.91
#define cell_center_adjustment_back_min 1.88
//
int ADC_conversion(uint8_t select_pin);
void fill_matrix();
void function_2();
void obstacle_detection();
void insertion_sort();
void move_pid();
void turn(int final_direction);
void wall_adjustment();
void cell_center_adjustment(uint8_t input);
void motor_movement(char value_2);
void maze_solver_queue_adder();
void create_final_path();
void master_communicate_initialize();
void send_value_to_master(char data);
void callibration();
void final_path_creator();
void test_pid();   ////////////////////////////////////////////////////////////////////////// remove this when you dont need
void test_turn();  ////////////////////////////////////////////////////////////////////////// remove this when you dont need
void adjustment_before_turn();
float callibration_length_reading(uint8_t x);
void clear_que_in_cell();
uint8_t cell_wall_detection_and_return_value(uint8_t X, uint8_t Y, char character);
void  test_wall();
void test_before_floodfill();

volatile uint8_t object_x_value = 0;
volatile uint8_t object_y_value = 0;
volatile uint8_t check = 1;
volatile uint8_t z = 0;
volatile uint8_t m = 0;
volatile uint8_t sort[4] = {0};
volatile uint8_t sort_position = 0;
volatile uint8_t verify_move = 0;
volatile uint8_t A = 0;
uint8_t key = 0;
int queue_pointer = 0;
volatile char master_received_value = '\0';
volatile uint16_t ADC_value_global = 0;
volatile uint8_t valid_1 = 0;
volatile float count_left = 0.0f, count_right = 0.0f, count_error = 0.0f, remainder_1 = 0.0f, count_left_real = 0.0f, count_right_real = 0.0f;
volatile float integral = 0.0f, derivative = 0.0f, previous_error = 0.0f, output = 0.0f;
int initial_direction  = 0;

//starting x and y coordinates
volatile uint8_t x = row, y = 1;

struct CELL {  //declare the structure CELL
	uint8_t obstacle;
	uint8_t visited_cell;
	uint8_t value;
	uint8_t que_added;
};
struct QUEUE {  //declare the structure QUEUE
	uint8_t x_1;
	uint8_t y_1;
};

struct CELL cell[row + 2][coloumn + 2];  // Declare the 2D array
struct QUEUE queue[200];                 //declare the queue with 20 elements

int main(void) {  //main function
	uint8_t check_10 = 0;
	
	//setting the pins
	DDRB = 0xFF;   //make the PORTB into outputs
	DDRC = 0x00;   //PINC (23,24,25,26) as input
	DDRD = 0xF2;   //pin 1, 32 and 30 as input
	PORTC = 0x00;  //set all PINC to LOW
	PORTD = 0x00;  //set all PINC to LOW
	PORTB = 0x00;  //set all PINB to LOW
	//setting external interrupts
	EICRA = 0x0F;  //setting INT0 and INT1 for rising edge
	EIMSK = 0x03;  //enable INT1 and INT2
	sei();         //enable global interrupts
	//setting ADC conversion
	ADMUX = 0x00;   //set ADC0 as the pin
	ADCSRA = 0x87;  //enable ADC conversion and set the division factor as 128 (5V -> 1023)
	//setting PWM for left and right motor
	TCCR1A = 0xA2;
	TCCR1B = 0x19;                //set to fast PWM and TOP as ICR1 register
	TCNT1 = 0;                    //set the initial value to zero
	ICR1 = PWM_max_value;         //set the top value to obtain 100kHz
	OCR1A = PWM_left_motor_max;   //set maximum speed for left motor
	OCR1B = PWM_right_motor_max;  //set maximum speed for left motor
	master_communicate_initialize();
	EIMSK = 0x00;
	/*the following code is used to communicate with the master */
	EIMSK = 0x03;  //enable INT1 and INT2
	_delay_ms(10000);
	//test_wall();
	fill_matrix();
	//motor_movement('F');
	//callibration();
	//test_pid();
	//test_turn();
	//adjustment_before_turn();
	//	while(master_received_value != 'C');
	//test_before_floodfill();
	//while (1);  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//	while (master_received_value != 1);             //wait until the command is received for callibration
	_delay_ms(10);  //give small time
	//	while (master_received_value == 2);  //until the floor is not detected it sends value '2' so until the floor is not detected it has to stay inside the while loop
	_delay_ms(30);
	//	callibration();  //call the function for callibration
	_delay_ms(30);
	//	while (master_received_value != 3);  //this loop is used to wait until it receive the command to enter into the flood fill, 3 value should be sent from the master to enter into the flood fill
	
	///////////////////////////////////////////////now enter to the flood fill
	queue[0].x_1 = 0;
	queue[0].y_1 = 0;  //assign first element x and y to 0

	while (cell[x][y].value != 0) {  //stops entering the while loop after reaching to the destination
		_delay_ms(90);
		if (check != 0) {  //if 1 then go to while loop
			object_x_value = x;  //assign the value of x into object_x_value
			object_y_value = y;  //assign the value of y into object_y_value
		}

		while (queue[1].x_1 != 0 || queue[1].y_1 != 0) {  //queue is not empty then enter the while loop

			check = 0;
			//{  pick the second value and store it in x and y
			x = queue[1].x_1;
			y = queue[1].y_1;
			cell[x][y].que_added = 0;
			//}/////////////////////////////////
			//pop the first value and left shift all the values in queue
			A = 1;  //points the second element in queue
			while (A < queue_pointer) {
				queue[A].x_1 = queue[A + 1].x_1;
				queue[A].y_1 = queue[A + 1].y_1;  // Move the value from the current position (A) in the queue to the previous position (A+1)
				A++;                              //increment 'A'
			}
			queue[queue_pointer].x_1 = 0;
			queue[queue_pointer].y_1 = 0;  //assign x and y values of the pointing element to 0
			queue_pointer--;               //decrement 'queue_pointer'
			/////////////////////
			function_2();  //call the function_2()
		}
		
		check = 1;
		x = object_x_value;
		y = object_y_value;
		function_2();  //call the function_2
	}
	//it has reached to the center position

	send_value_to_master('4');  //this command is sent to master that it has reached to the center of the maze
	while(1);
	_delay_ms(30);
	while (master_received_value != 5);  //wait until the master send the value for the final path solver
	final_path_creator();
}

ISR(INT0_vect) {  //interrupt service routine for INT0
	//handle the left motor count
	valid_1 = 0;                             //give the ability to the micro controller to enter to the PID function
	count_left++;                            //increment the left count of left motor
	count_left_real++;
	count_error = count_left - count_right;  //take the difference as count_error
	//move_pid();  //this was removed by me
	if (((count_left + count_right)/2 + remainder_1) > max_count) {  //check whether it has reached the next cell
		
		//put the values to count_right and count_left without changing the difference
		verify_move = 1;  //assign one to the variable
	}
}

ISR(INT1_vect) {  //interrupt service routine for INT1
	//handle the right motor count
	valid_1 = 0;                                              //give the ability to the micro controller to enter to the PID function
	count_right++;                                            //increment the left count of right motor
	count_right_real++;
	count_error = count_left - count_right;                   //take the difference as count_error

	if (((count_left + count_right)/2 + remainder_1) > max_count) {  //check whether it has reached the next cell
		//put the values to count_right and count_left without changing the difference
		verify_move = 1;  //assign one to the variable
	}
}

void function_2() {  //update the cells
	if (check == 1) {
		cell[x][y].visited_cell = 1;  //mark the cell as visited
		obstacle_detection();         //detect all the obstacles
	}
	key = cell[x][y].value;  //store the value of the cell in variable key

	if (cell[x][y].visited_cell == 1) {                             //check whether the cell is visited
		if (cell[x - 1][y].value < key && cell_wall_detection_and_return_value(x, y, 'F') == 0) {  //check whether no forward obstacle and key is greater
			if (check == 1) {
				x = x - 1;                  //update coordinate of x and y
				turn(0);                    //turn forward
				
				verify_move = 0;            // Initialize the loop control variable to 0 continue the while loop
				
				motor_movement('F');        //move forward
				while (verify_move != 1) {
					move_pid();  //this will happen until it reach the next cell
					
				}
				motor_movement('o');
				//reset the parameters of the PID
				//
			}
			} else if (cell[x][y + 1].value < key && cell_wall_detection_and_return_value(x, y, 'R') == 0) {  //check whether no right obstacle and key is greater
			if (check == 1) {
				
				turn(90);                   //turn right
				y = y + 1;                  //update coordinate of x and y
				verify_move = 0;            // Initialize the loop control variable to 0 continue the while loop
				
				motor_movement('F');        //move forward
				while (verify_move != 1) {
					move_pid();  //this will happen until it reach the next cell
					
				}
				motor_movement('o');
			}
			} else if (cell[x + 1][y].value < key && cell_wall_detection_and_return_value(x, y, 'B') == 0) {  //check whether no back obstacle and key is greater
			if (check == 1) {
				
				turn(180);                  //turn backward
				x = x + 1;                  //update coordinate of x and y
				verify_move = 0;            // Initialize the loop control variable to 0 continue the while loop
				
				motor_movement('F');        //move forward
				while (verify_move != 1) {
					move_pid();  //this will happen until it reach the next cell
					
				}
				
				motor_movement('o');
			}
			} else if (cell[x][y - 1].value < key && cell_wall_detection_and_return_value(x, y, 'L') == 0) {  //check whether no left obstacle and key is greater
			if (check == 1) {
				
				turn(270);                  //turn left
				y = y - 1;                  //update coordinate of x and y
				verify_move = 0;            // Initialize the loop control variable to 0 continue the while loop
				
				motor_movement('F');        //move forward
				while (verify_move != 1) {
					move_pid();  //this will happen until it reach the next cell
				}
				motor_movement('o');
			}
			} else {
			if (cell_wall_detection_and_return_value(x, y, 'L') == 0 && cell[x][y-1].que_added !=1) {
				sort[sort_position] = cell[x][y - 1].value;  //take the left coordinate value to sort array
				sort_position++;                             //increment sort_position
				queue[queue_pointer + 1].x_1 = x;
				queue[queue_pointer + 1].y_1 = y - 1;  //take the left coordinate to queue
				queue_pointer++;                       //increment queue_pointer
				cell[x][y-1].que_added = 1;
			}
			
			if (cell_wall_detection_and_return_value(x, y, 'F') == 0 && cell[x-1][y].que_added !=1) {
				sort[sort_position] = cell[x - 1][y].value;  //take the forward coordinate value to sort array
				sort_position++;                             //increment sort_position
				queue[queue_pointer + 1].x_1 = x - 1;
				queue[queue_pointer + 1].y_1 = y;  //take the left coordinate to queue
				queue_pointer++;                   //increment queue_pointer
				cell[x-1][y].que_added =1;
			}
			
			if (cell_wall_detection_and_return_value(x, y, 'R') == 0 && cell[x][y+1].que_added !=1) {
				sort[sort_position] = cell[x][y + 1].value;  //take the right coordinate value to sort array
				sort_position++;                             //increment sort_position
				queue[queue_pointer + 1].x_1 = x;
				queue[queue_pointer + 1].y_1 = y + 1;  //take the left coordinate to queue
				queue_pointer++;                       //increment queue_pointer
				cell[x][y+1].que_added =1;
			}
			
			if (cell_wall_detection_and_return_value(x, y, 'B') == 0 && cell[x+1][y].que_added !=1) {
				sort[sort_position] = cell[x + 1][y].value;  //take the back coordinate value to sort array
				sort_position++;                             //increment sort_position
				queue[queue_pointer + 1].x_1 = x + 1;
				queue[queue_pointer + 1].y_1 = y;  //take the left coordinate to queue
				queue_pointer++;                   //increment queue_pointer
				cell[x+1][y].que_added =1;
			}
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			insertion_sort();                                //call the function to sort the array
			cell[x][y].value = sort[0] + 1;  // Assign the value to the cell[x][y] by adding one to the lowest value in sort
			for (int i = 0; i < 4; i++) {
				sort[i] = 0;
			}                   // Reset the 'sort' array, preparing it for the next set of values
			sort_position = 0;  // Reset the 'sort_position' variable to 0 for the next sorting operation
		}
		
		} else if (cell[x][y].visited_cell == 0) {  //check whether the cell is not visited
		sort[0] = cell[x][y + 1].value;
		sort[1] = cell[x + 1][y].value;
		sort[2] = cell[x][y - 1].value;
		sort[3] = cell[x - 1][y].value;
		sort_position = 4;

		insertion_sort();  //call the function to sort the sort array
		if (key > sort[0]) {
			//no change
			} else {
			if(cell[x][y-1].value != 100 && cell[x][y-1].que_added != 1){
				queue[queue_pointer + 1].x_1 = x;
				queue[queue_pointer + 1].y_1 = y - 1;  //take the left coordinate to queue
				queue_pointer++;                       //increment queue_pointer
				cell[x][y-1].que_added = 1;
			}

			if(cell[x-1][y].value != 100 && cell[x-1][y].que_added != 1){
				queue[queue_pointer + 1].x_1 = x - 1;
				queue[queue_pointer + 1].y_1 = y;  //take the left coordinate to queue
				queue_pointer++;
				cell[x-1][y].que_added = 1;
			}
			
			if(cell[x][y+1].value != 100 && cell[x][y+1].que_added != 1){
				queue[queue_pointer + 1].x_1 = x;
				queue[queue_pointer + 1].y_1 = y + 1;  //take the left coordinate to queue
				queue_pointer++;
				cell[x][y+1].que_added = 1;
			}
			
			if(cell[x+1][y].value != 100 && cell[x+1][y].que_added != 1){
				queue[queue_pointer + 1].x_1 = x + 1;
				queue[queue_pointer + 1].y_1 = y;  //take the left coordinate to queue
				queue_pointer++;
				cell[x+1][y].que_added = 1;
				
			}
			//
			
			cell[x][y].value = sort[0] + 1;
			
		}
		for (int i = 0; i < 4; i++) {
			sort[i] = 0;
		}  // Reset the 'sort' array, preparing it for the next set of values
		sort_position = 0;
	}
	motor_movement('o');  //stop the motors
}

void obstacle_detection() {
	float value_3 = 0;  //declare the variable

	value_3 = ADC_conversion(0);
	float ADC_value_local_forward = 17.3 * exp(-0.0045 * value_3) + 0.5 * exp(-0.9 * value_3) - exp(-0.3 * value_3);
	value_3 = ADC_conversion(1);
	float ADC_value_local_right = 17.3 * exp(-0.0045 * value_3) + 0.5 * exp(-0.9 * value_3) - exp(-0.3 * value_3);
	value_3 = ADC_conversion(2);
	float ADC_value_local_back = 17.3 * exp(-0.0045 * value_3) + 0.5 * exp(-0.9 * value_3) - exp(-0.3 * value_3);
	value_3 = ADC_conversion(3);
	float ADC_value_local_left = 17.3 * exp(-0.0045 * value_3) + 0.5 * exp(-0.9 * value_3) - exp(-0.3 * value_3) - 0.5;

	//////////////////////////////////////////////////////////////////////////
	if (initial_direction == 0) {  // facing forward
		if (ADC_value_local_forward > range_min && ADC_value_local_forward < range_max) {
			cell[x][y].obstacle |= 1<<2;
		}
		if (ADC_value_local_right > range_min && ADC_value_local_right < range_max) {
			cell[x][y].obstacle |= 1<<1;
		}
		if (ADC_value_local_back > range_min && ADC_value_local_back < range_max) {
			cell[x][y].obstacle |= 1<<0;
		}
		// ADC_value_local = ADC_conversion(1);  // select ADC1
		if (ADC_value_local_left > range_min && ADC_value_local_left < range_max) {
			cell[x][y].obstacle |= 1<<3;
		}
		
		} else if (initial_direction == 90) {  // facing right
		if (ADC_value_local_right > range_min && ADC_value_local_right < range_max) {
			cell[x][y].obstacle |= 1<<0;
		}
		
		// ADC_value_local = ADC_conversion(2);  // select ADC2
		if (ADC_value_local_back > range_min && ADC_value_local_back < range_max) {
			cell[x][y].obstacle |= 1<<3;
		}
		
		// ADC_value_local = ADC_conversion(3);  // select ADC3
		if (ADC_value_local_left > range_min && ADC_value_local_left < range_max) {
			cell[x][y].obstacle |= 1<<2;
		}
		// ADC_value_local = ADC_conversion(1);  // select ADC1
		if (ADC_value_local_forward > range_min && ADC_value_local_forward < range_max) {
			cell[x][y].obstacle |= 1<<1;
		}
		
		} else if (initial_direction == 180) {  // facing back
		if (ADC_value_local_back > range_min && ADC_value_local_back < range_max) {
			cell[x][y].obstacle |= 1<<2;
		}
		
		// ADC_value_local = ADC_conversion(2);  // select ADC2
		if (ADC_value_local_left > range_min && ADC_value_local_left < range_max) {
			cell[x][y].obstacle |= 1<<1;
		}
		
		// ADC_value_local = ADC_conversion(3);  // select ADC3
		if (ADC_value_local_forward > range_min && ADC_value_local_forward < range_max) {
			cell[x][y].obstacle |= 1<<0;
		}
		
		// ADC_value_local = ADC_conversion(1);  // select ADC1
		if (ADC_value_local_right > range_min && ADC_value_local_right < range_max) {
			cell[x][y].obstacle |= 1<<3;
		}
		
		} else if (initial_direction == 270) {  // facing left
		if (ADC_value_local_left > range_min && ADC_value_local_left < range_max) {
			cell[x][y].obstacle |= 1<<0;
		}
		
		// ADC_value_local = ADC_conversion(2);  // select ADC2
		if (ADC_value_local_forward > range_min && ADC_value_local_forward < range_max) {
			cell[x][y].obstacle |= 1<<3;
		}
		
		// ADC_value_local = ADC_conversion(3);  // select ADC3
		if (ADC_value_local_right > range_min && ADC_value_local_right < range_max) {
			cell[x][y].obstacle |= 1<<2;
		}
		// ADC_value_local = ADC_conversion(1);  // select ADC1
		if (ADC_value_local_back > range_min && ADC_value_local_back < range_max) {
			cell[x][y].obstacle |= 1<<1;
		}
	}

	//////////////////////////////////////////////////////////////////////////
}

void fill_matrix() {

	// Initialize all cells in the 2D array with a default value of 100
	for (int i = 0; i < row + 2; i++) {
		for (int j = 0; j < coloumn + 2; j++) {
			cell[i][j].value = 100;
			cell[i][j].visited_cell = 0;
			cell[i][j].obstacle = 0;
			cell[i][j].que_added = 0;
		}
	}

	// Set the value of the cell at the final specified row and column to 0
	cell[final_row][final_coloumn].value = 0;

	// Update the values in the row of the specified final column, increasing from the specified column towards the end
	for (int h = final_coloumn + 1; h <= coloumn; h++) {
		cell[final_row][h].value = cell[final_row][h - 1].value + 1;
	}

	// Update the values in the row of the specified final column, decreasing from the specified column towards the beginning
	for (int h = final_coloumn - 1; h >= 1; h--) {
		cell[final_row][h].value = cell[final_row][h + 1].value + 1;
	}

	// Update values in the columns after the specified final column
	for (int y = final_coloumn; y <= coloumn; y++) {
		// Increase values going upwards from the specified final row
		for (int k = final_row - 1; k > 0; k--) {
			cell[k][y].value = cell[k + 1][y].value + 1;
		}
		// Increase values going downwards from the specified final row
		for (int k = final_row + 1; k <= row; k++) {
			cell[k][y].value = cell[k - 1][y].value + 1;
		}
	}

	// Update values in the columns before the specified final column
	for (int y = final_coloumn; y >= 1; y--) {
		// Increase values going upwards from the specified final row
		for (int k = final_row - 1; k > 0; k--) {
			cell[k][y].value = cell[k + 1][y].value + 1;
		}
		// Increase values going downwards from the specified final row
		for (int k = final_row + 1; k <= row; k++) {
			cell[k][y].value = cell[k - 1][y].value + 1;
		}
	}
	_delay_ms(90);  //give a small delay
}

int ADC_conversion(uint8_t select_pin) {
	ADMUX = 0x40 | select_pin;      //selecting the pin
	ADCSRA = ADCSRA | (1 << ADSC);  //set ADCS bit to '1'
	while (ADCSRA & 0x40)
	;  //wait until the conversion is complete
	ADC_value_global = ADCL;
	ADC_value_global = (ADCH << 8) | (ADC_value_global);  //store the value in variable 'ADC_value_global'
	return ADC_value_global;                              //return the value
}

void insertion_sort() {
	if (sort_position >= 2) {  // Check if there are at least 2 elements in the 'sort' array
		int store, j, q;
		// Perform insertion sort on the 'sort' array
		for (q = 1; q < sort_position; q++) {  //remember always the maximum value of sort position has to be 4
			store = sort[q];  // Store the value in the 'store' variable
			j = q;
			// Move elements of 'sort' array that are greater than 'store' to one position ahead of their current position
			while (j > 0 && sort[j - 1] > store) {
				sort[j] = sort[j - 1];
				j--;
			}                 // end of while
			sort[j] = store;  // Place 'store' at its correct position in the sorted order
		}                   // end of for
	}
}


void turn(int final_direction) {  //consist of clockwise_set_value and anticlockwise_set_value
	motor_movement('o');  //stop the motors
	int difference = 0;
	_delay_ms(80);
	adjustment_before_turn();
	difference = final_direction - initial_direction;  //calculate the difference
	if (difference == 0) {
		//check the pulse difference between two motors and then try to make them zero
		// make count_right, count_left, error zero
		//make integral and differential to zero
		
		return;
	}  //if the difference is zero then there is no change just return

	//reset the values of PID

	//by turning the right or left motor fix the change in wheels finally both left count and right count must come to zero
	
	cell_center_adjustment(1);
	//
	OCR1A = after_turn_PWM+10;
	OCR1B = after_turn_PWM+10;
	if (difference > 180) {
		//turn anticlockwise by 90 degrees
		while ((count_left + count_right) / 2 < turn_wheel_for_90_degrees) {
			motor_movement('A');  //turn the motor anticlockwise
		}

		} else if (difference > 0 && difference < 180) {
		//turn clockwise by 90 degrees
		while ((count_left + count_right) / 2 < turn_wheel_for_90_degrees) {
			motor_movement('C');  //turn the motor clockwise
		}

		} else if (difference == 180 || difference == -180) {
		//turn clockwise by 180
		while ((count_left + count_right) / 2 < (turn_wheel_for_180_degrees)) {
			motor_movement('C');  //turn the motor clockwise
		}

		} else if (difference == -90) {
		//turn anticlockwise by 90 degrees
		while ((count_left + count_right) / 2 < turn_wheel_for_90_degrees) {
			motor_movement('A');  //turn the motor anticlockwise
		}

		} else {
		//no change
	}
	motor_movement('O');  //stop the motors
	cell_center_adjustment(2);
	OCR1A = PWM_left_motor_max;
	OCR1B = PWM_right_motor_max;
	initial_direction = final_direction;  //update the initial_direction
	_delay_ms(90);
}

void move_pid() {
	float dummy_3 = callibration_length_reading(0);
	if(dummy_3 > 0 && dummy_3 < 5){
		verify_move = 1;
	}
	volatile float count_right_temporary = 0, count_left_temporary = 0;
	if (count_error < 0) {                                                         //decrease right motor speed
		OCR1A = PWM_left_motor_max;                                                  //set maximum speed on left motor
		integral = integral - (count_error * 0.1);                                   //take the sum
		derivative = (-count_error - previous_error) / 0.01;                         //get the rate of change by the ratio of error_difference to time interval
		previous_error = -count_error;                                               //assign the initial error value to previous error value for the next turn
		output = (-Kp * count_error) + (Ki * integral) + (Kd * derivative);          //get the total output value
		OCR1B = (uint16_t)((PWM_right_motor_max) - (PWM_linear_constant * output));  //adjusting right motor PWM, 55.3y+100x-5530=0

		} else if (count_error > 0) {                                                 //decrease the left motor
		OCR1B = PWM_right_motor_max;                                                //set maximum speed on right motor
		integral = integral + (count_error * 0.1);                                  //take the sum
		derivative = (count_error - previous_error) / 0.01;                         //get the rate of change by the ratio of error_difference to time interval
		previous_error = count_error;                                               //assign the initial error value to previous error value for the next turn
		output = (Kp * count_error) + (Ki * integral) + (Kd * derivative);          //get the total output value
		OCR1A = (uint16_t)((PWM_left_motor_max) - (PWM_linear_constant * output));  //adjusting left motor PWM, 55.3y+100x-5530=0
		} else {           //check whether it is in the range

		//valid_1 = 1;  //assign one to valid_1 to stop the recursion and increment the count only
		OCR1A = PWM_left_motor_max;
		OCR1B = PWM_right_motor_max;  //set both motors to maximum values
		//reset all the values
		count_error = 0;
		integral = 0;
		previous_error = 0;
		output = 0;
		count_left_temporary = count_left;
		count_right_temporary = count_right;
		wall_adjustment();  //call the function
		if (m == 1) {             //global variable
			_delay_ms(delay_wall);  //in here it suddenly change the phase and when interrupt occurs it count the difference but PID doesn't happen it happens after this delay
			m = 0;
		}
		remainder_1 = remainder_1 + ((count_left - count_left_temporary) + (count_right - count_right_temporary))/2;
		count_left = count_left_temporary;
		count_right = count_right_temporary;
		//valid_1 = 0;  //assign one to valid_1 to stop the recursion and increment the count only
	}
	valid_1 = 1;
}

void wall_adjustment() {
	float left_wall_distance = callibration_length_reading(3);          //take the distance from left sensor to wall
	float right_wall_distance = callibration_length_reading(1);         //take the distance from right sensor to wall
	if(left_wall_distance > 9 && right_wall_distance > 9){return;}

	if(left_wall_distance < 9 && right_wall_distance > 9){
		//give the left adjustment
		if (left_wall_distance > 4.5) {
			OCR1A = OCR1A - (decrease_value_in_wall_adjustment+10);
		}
		if(left_wall_distance < 2.5){
			OCR1B = OCR1B - (decrease_value_in_wall_adjustment);
		}
		m = 1;
		return;
	}
	if(right_wall_distance < 9 && left_wall_distance > 9){
		//give the right adjustment
		if(right_wall_distance > 4.5){
			OCR1B = OCR1B - (decrease_value_in_wall_adjustment);
		}
		if(right_wall_distance < 2.5){
			OCR1A = OCR1A - (decrease_value_in_wall_adjustment+10);
		}
		m = 1;
		return;
	}
	float gap = left_wall_distance - right_wall_distance;  //take the difference
	if (gap < 0 && -gap > wall_gap) {
		//decrease the speed of right motor by half OCR1B
		OCR1B = OCR1B - (decrease_value_in_wall_adjustment);
		m = 1;
		} else if (gap > 0 && gap > wall_gap) {
		//decrease the speed of left motor by half OCR1A
		OCR1A = OCR1A - (decrease_value_in_wall_adjustment+10);
		m = 1;
	}
}

void cell_center_adjustment(uint8_t input) {  //0->head 2->back
	//if you want to adjust from the frnt sensor then give the  input as 1 and if you want to adjust from back sensor then give the input as 2
	//set the the PWM to LOW
	OCR1A = adjustment_before_turn_PWM + 10;
	OCR1B = adjustment_before_turn_PWM + 8;

	if (input == 1) {
		//taking the head_value
		float head_value = callibration_length_reading(0);  //get the distance from the head sensor
		while(head_value > cell_center_adjustment_front_max && head_value < 9){motor_movement('F'); head_value = callibration_length_reading(0);}  //move the motor forward until the error is adjusted
		motor_movement('O');  //stop the motors
		while(head_value < cell_center_adjustment_front_min  && head_value < 9){motor_movement('B'); head_value = callibration_length_reading(0);}  //move the motor back until the error is adjusted
		motor_movement('O');  //stop the motors
		
		} else {
		//take the back_value
		float back_value = callibration_length_reading(2);  //get the distance from the back sensor
		while(back_value > cell_center_adjustment_back_max  && back_value < 9){motor_movement('B'); back_value = callibration_length_reading(2);}
		motor_movement('O');
		while(back_value < cell_center_adjustment_back_min && back_value < 9){motor_movement('F'); back_value = callibration_length_reading(2);}
		motor_movement('O');
		
		//set the PWM to initial value
	}
	OCR1A = PWM_left_motor_max;
	OCR1B = PWM_right_motor_max;
	count_left = 0;
	count_right = 0;
	remainder_1 = 0;
	count_left_real = 0;
	count_right_real = 0;
	_delay_ms(90);
}

void motor_movement(char value_2) {
	if (value_2 == 'F') {
		//move forward
		//left and right both turn forward PD7->1, PB0->0, PD5->0, PD6->1
		PORTD = (PORTD & 0x1F) | 0xC0;
		PORTB = PORTB & 0xFE;
		} else if (value_2 == 'B') {
		//move backward
		//left and right both turn forward PD7->0, PB0->1, PD5->1, PD6->0
		PORTD = (PORTD & 0x1F) | 0x20;
		PORTB = PORTB | 0X01;
		} else if (value_2 == 'C') {
		//turn clockwise
		//right rotate back and left rotate front PD7->1, PB0->0, PD5->1, PD6->0
		PORTD = (PORTD & 0x1F) | 0xA0;
		PORTB = PORTB & 0xFE;
		} else if (value_2 == 'A') {
		//turn anticlockwise
		//right rotate front and left rotate back PD7->0, PB0->1, PD5->0, PD6->1
		PORTD = (PORTD & 0x1F) | 0x40;
		PORTB = PORTB | 0x01;
		} else if (value_2 == 'o' || value_2 == 'O') {
		PORTD = PORTD & 0x1F;
		PORTB = PORTB & 0xFE;
	}
}

void create_final_path() {
	obstacle_detection();  //check the obstacle and mark them
	//mark all the unvisited cell as 100
	for (int i = 1; i < row; i++) {
		for (int j = 1; j < coloumn; j++) {
			if (cell[i][j].visited_cell == 0) {
				cell[i][j].value = 100;  //assign the value 100
			}
		}
	}
	cell[x][y].value = 1;                             //mark the initial cell as 1
	cell[x][y].visited_cell = 0;                      //mark the cell as unvisited
	maze_solver_queue_adder();                        //call the function and add the corresponding cell to queue
	while (queue[1].x_1 != 0 || queue[y].y_1 != 0) {  //while the queue is not empty
		//{  pick the second value and store it in x and y
		x = queue[1].x_1;
		y = queue[1].y_1;
		//}
		//pop the first value and left shift all the values in queue
		A = 1;  //points the second element in queue
		while (A < queue_pointer) {
			queue[A].x_1 = queue[A + 1].x_1;
			queue[A].y_1 = queue[A + 1].y_1;  // Move the value from the current position (A) in the queue to the previous position (A+1)
			A++;                              //increment 'A'
		}
		queue[queue_pointer].x_1 = 0;
		queue[queue_pointer].y_1 = 0;  //assign x and y values of the pointing element to 0
		queue_pointer--;               //decrement 'queue_pointer'
		//check for obstacle and value not 100 and not 0 and not visited
		if (cell_wall_detection_and_return_value(x, y, 'F') == 0 && cell[x - 1][y].value != 100 && cell[x - 1][y].value != 0) {
			sort[sort_position] = cell[x][y].value;
			sort_position++;
		}
		if (cell_wall_detection_and_return_value(x, y, 'R') == 0 && cell[x][y + 1].value != 100 && cell[x][y + 1].value != 0) {
			sort[sort_position] = cell[x][y].value;
			sort_position++;
		}
		if (cell_wall_detection_and_return_value(x, y, 'B') == 0 && cell[x + 1][y].value != 100 && cell[x + 1][y].value != 0) {
			sort[sort_position] = cell[x][y].value;
			sort_position++;
		}
		if (cell_wall_detection_and_return_value(x, y, 'L') == 0 && cell[x][y - 1].value != 100 && cell[x][y - 1].value != 0) {
			sort[sort_position] = cell[x][y].value;
			sort_position++;
		}
		insertion_sort();                //sort the array
		cell[x][y].value = sort[0] + 1;  // Assign the value to the cell[x][y] by adding one to the lowest value in sort
	}
}

void maze_solver_queue_adder() {
	if (cell_wall_detection_and_return_value(x, y, 'F') == 0 && cell[x - 1][y].value != 100 && cell[x - 1][y].value == 0) {
		queue[queue_pointer + 1].x_1 = x - 1;
		queue[queue_pointer + 1].y_1 = y;  //add the cell to the queue
		queue_pointer++;                   //increment the value of queue_pointer
	}
	if (cell_wall_detection_and_return_value(x, y, 'R') == 0 && cell[x][y + 1].value != 100 && cell[x][y + 1].value == 0) {
		queue[queue_pointer + 1].x_1 = x;
		queue[queue_pointer + 1].y_1 = y + 1;  //add the cell to the queue
		queue_pointer++;                       //increment the value of queue_pointer
	}
	if (cell_wall_detection_and_return_value(x, y, 'B') == 0 && cell[x + 1][y].value != 100 && cell[x + 1][y].value == 0) {
		queue[queue_pointer + 1].x_1 = x + 1;
		queue[queue_pointer + 1].y_1 = y;  //add the cell to the queue
		queue_pointer++;                   //increment the value of queue_pointer
	}
	if (cell_wall_detection_and_return_value(x, y, 'L') == 0 && cell[x][y - 1].value != 100 && cell[x][y - 1].value == 0) {
		queue[queue_pointer + 1].x_1 = x;
		queue[queue_pointer + 1].y_1 = y - 1;  //add the cell to the queue
		queue_pointer++;                       //increment the value of queue_pointer
	}
}

void master_communicate_initialize() {
	//Asynchronous normal mode (U2Xn = 0)
	PORTD = PORTD & 0xFC;
	PORTD = PORTD | 0x02;
	UBRR0 = 103;    //set the baud rate
	UCSR0B = 0x98;  //enable the transmitter receiver and also enable interrupt on receiver data
}

void send_value_to_master(char data) {
	while (!(UCSR0A & 0x20));           //wait until the buffer is empty
	UDR0 = data;  //load the value to the buffer
	while (!(UCSR0A & 0x40));  //wait until it is finish transmitting
}

ISR(USART_RX_vect) {
	master_received_value = UDR0;
}

void callibration() {

	///////
	float value_3[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float length_1 = 0;
	uint8_t dummy_1 = 0;
	char dummy_2;
	//the mouse has to get aligned to the center
	//measure the length of the cell, usually this measurement taken by width (in here there are two methods)
	OCR1A = adjustment_before_turn_PWM+10;
	OCR1B = adjustment_before_turn_PWM+10;
	//
	if((callibration_length_reading(2) > range_min && callibration_length_reading(2) < range_max+6) && (callibration_length_reading(3) > range_min && callibration_length_reading(3) < range_max+6) ){
		dummy_1 = 1;  //this will happen when left and back walls are detected
		}else{
		dummy_1 = 2;  //this will happen when right and back walls are detected
	}
	
	if(dummy_1 == 1){
		dummy_2 = 'A';
		}else if(dummy_1 == 2){
		dummy_2 = 'C';
		}else {
		return;
	}
	//
	//turn the mouse left
	while ((count_left + count_right) / 2 < turn_wheel_for_90_degrees) {
		motor_movement(dummy_2);  //turn the motor anticlockwise
	}
	motor_movement('o');
	//end
	//////////////////////this part take the value from the front sensor
	for (uint8_t i = 0; i < 10; i++) {
		value_3[i] = callibration_length_reading(0);
	}
	for (uint8_t i = 0; i < 10; i++) {
		length_1 = length_1 + value_3[i];
	}
	length_1 = length_1 / 10;  //average length
	
	/////////////////////////end taking the value sand store it in the length_1 variable
	///////////////////////////////////
	if (length_1 > callibration_constant - 2.1) {  //////////////////////////////////////////////////////////////////////error
		while (length_1 > callibration_constant) {
			motor_movement('F');
			length_1 = callibration_length_reading(0);
		}
		} else {
		while (length_1 < callibration_constant) {
			motor_movement('B');
			length_1 = callibration_length_reading(0);
		}
	}
	_delay_ms(90);
	motor_movement('o');
	//////////////////////////////////////////////////////////////////////////
	///////////////////////////////
	// turn right
	if(dummy_2 == 'A'){
		dummy_2 = 'C';
		}else{
		dummy_2 = 'A';
	}
	count_left = 0;
	count_right = 0;
	remainder_1 = 0;
	count_left_real = 0;
	count_right_real = 0;
	while ((count_left + count_right) / 2 < turn_wheel_for_90_degrees) {
		motor_movement(dummy_2);  //turn the motor anticlockwise
	}
	motor_movement('o');
	/////////////////////
	for (uint8_t i = 0; i < 10; i++) {
		value_3[i] = callibration_length_reading(2);
	}
	for (uint8_t i = 0; i < 10; i++) {
		length_1 = length_1 + value_3[i];
	}
	length_1 = length_1 / 10;  //average length
	/////////////////////////////
	if (length_1 > callibration_constant - 1.8) {  //////////////////////////////////////////////////////////////////////error
		while (length_1 > callibration_constant) {
			motor_movement('B');
			length_1 = callibration_length_reading(2);
		}
		} else {
		while (length_1 < callibration_constant - 1.8) {
			motor_movement('F');
			length_1 = callibration_length_reading(2);
		}
	}
	motor_movement('o');
	count_left = 0;
	count_right = 0;
	remainder_1 = 0;
	count_left_real = 0;
	count_right_real = 0;
	//finish
	/*when the callibration is done sent a value to the master device */
	send_value_to_master('d');
	_delay_ms(100);
	//end
}

void final_path_creator() {
}

float callibration_length_reading(uint8_t x){
	float return_value;
	uint16_t dummy = ADC_conversion(x);
	if(x != 3){
		return_value =  17.3 * exp(-0.0045 * dummy) + 0.5 * exp(-0.9 * dummy) - exp(-0.3 * dummy);
		}else{
		return_value =  17.3 * exp(-0.0045 * dummy) + 0.5 * exp(-0.9 * dummy) - exp(-0.3 * dummy) - 0.5;
	}
	return return_value;
}

void test_pid() {
	motor_movement('F');  //move forward
	verify_move = 0;      //make the verify_move variable = 0
	while (verify_move != 1) {
		move_pid();
	}

	//turn off the motor
	motor_movement('o');
}

void adjustment_before_turn() {
	_delay_ms(80);
	if (count_left_real > count_right_real) {
		
		OCR1B = adjustment_before_turn_PWM;  //put the PWM of right motor to very low value
		while (count_right_real < count_left_real-35) {
			//turn right motor forward and stop left motor
			PORTD = (PORTD & 0x9F) | 0x40;
		}

		} else if (count_left_real < count_right_real) {
		
		OCR1A = adjustment_before_turn_PWM;  //put the PWM of left motor to very low value
		while (count_left_real < count_right_real-35) {
			//turn left motor forward and stop left motor
			PORTD = PORTD | 0x80;
			PORTB = PORTB & 0xFE;
		}
		} else {
	}
	motor_movement('o');
	OCR1A = PWM_left_motor_max;
	OCR1B = PWM_right_motor_max;
	count_left = 0;
	count_right = 0;
	remainder_1 = 0;
	count_left_real = 0;
	count_right_real = 0;
	_delay_ms(80);
}

void test_turn() {
	count_left = 0;
	count_right = 0;
	OCR1A = after_turn_PWM + 3;
	OCR1B = after_turn_PWM;
	while ((count_left + count_right) / 2 < turn_wheel_for_90_degrees) {
		motor_movement('A');  //turn the motor anticlockwise
	}
	motor_movement('o');
}

void clear_que_in_cell(){
	for(uint8_t i=1; i<=row; i++){
		for(uint8_t j=1; j<=coloumn; j++){
			cell[i][j].que_added = 0;
		}
	}
}

uint8_t cell_wall_detection_and_return_value(uint8_t X, uint8_t Y, char character){

	if (character == 'F') {
		if ((cell[X][Y].obstacle & 0x04) == 0x04) {  // Check if the 3rd bit is set
			return 1;
			} else {
			return 0;
		}
		} else if (character == 'L') {
		if ((cell[X][Y].obstacle & 0x08) == 0x08) {  // Check if the 4th bit is set
			return 1;
			} else {
			return 0;
		}
		} else if (character == 'B') {
		if ((cell[X][Y].obstacle & 0x01) == 0x01) {  // Check if the 1st bit is set
			return 1;
			} else {
			return 0;
		}
		} else if(character == 'R'){
		if((cell[X][Y].obstacle & 0x02) == 0x02){
			return 1;
			}else{
			return 0;
		}
		}else{
		return 0;
	}

}


void  test_wall(){
	initial_direction = 0;
	turn(180);
	_delay_ms(600);
}

void test_before_floodfill(){
	initial_direction = 0;
	//callibration();
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(0);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(90);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(180);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(90);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(180);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(90);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(0);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(0);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(90);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(180);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}

	turn(0);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}

	turn(0);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
	}
	turn(90);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(90);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(0);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(270);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(270);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(270);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(270);
	//
	motor_movement('F');
	verify_move = 0;
	while(verify_move != 1){
		move_pid();
		
	}
	turn(270);
}