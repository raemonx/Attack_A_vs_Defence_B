#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "my_functions.h"

#include "vision_simulation.h"

#include "timer.h"

// function declerations

int activate(image& a, image& b, image& rgb, image& rgb0, image& label, int IMAGE_HEIGHT, int IMAGE_WIDTH);

void image_prep(int tvalue, image& rgb, image& rgb0, image& a, image& b);

void label_size(image& label_image, int label_num, int& N);

int centroids(image a, image rgb, image label, int& nlabels,
	double& r_ic, double& r_jc,
	double& g_ic, double& g_jc,
	double& y_ic, double& y_jc,
	double& b_ic, double& b_jc, double& obs1_ic, double& obs1_jc, double& obs2_ic, double& obs2_jc, int N_obs);


int position_angle(double& ic1, double& ic2, double& ic3, double& ic4, double& jc1, double& jc2, double& jc3, double& jc4,
	double& x1, double& y1, double& x2, double& y2, double& theta1, double& theta2);

int triangle_angles(double x1, double y1, double x2, double y2, double obs1_ic, double obs1_jc,
	double obs2_ic, double obs2_jc, double& d_obs1_a, double& d_obs2_a, double& d_obs_a, double& obs_ic,
	double& obs_jc, double& theta_shoot_gap_abs);

int triangle_angles_d(double& x1, double& y1, double& x2, double& y2, double& obs1_ic, double& obs1_jc,
	double& obs2_ic, double& obs2_jc, double& d_obs1_b, double& d_obs2_b, double& d_obs_b,
	double& obs_ic, double& obs_jc, double& theta_shoot_gap_abs);

int obstacle_avoid(double& g_ic, double& g_jc, double& obs_ic, double& obs_jc, double& d_obs_a,
	double& theta1, double& x2, double& y2, double& p1x, double& p1y,
	double& p2x, double& p2y, int& pw_r, int& pw_l);

int obstacle_avoid_defence(double& y_ic, double& y_jc, double& g_ic, double& g_jc, double& theta2,
	double& d_obs_b, double& obs_ic, double& obs_jc, double& px, double& py, double& p1x, double& p1y,
	double& p2x, double& p2y, int& pw_r, int& pw_l);

int complete_defence(double& y_ic, double& y_jc, double& g_ic, double& g_jc, double& theta2,
	double& px, double& py, int& pw_r, int& pw_l);

void control_robot(double& x1, double& y1, double& x2, double& y2,
	double& theta1, double& theta_gap, double& theta,
	double& distance_a_b, int& pw_r, int& pw_l, int& laser);

void image_prep_optional(int tvalue, image& rgb, image& rgb0, image& a, image& b);



//-----------------------------------------------------------------------------
// Name: 
// Desc: //this function sets the type and size of images
//-----------------------------------------------------------------------------
int activate(image& a, image& b, image& rgb, image& rgb0, image& label, int IMAGE_HEIGHT, int IMAGE_WIDTH) {

	// set the type and size of the images
	a.type = GREY_IMAGE;
	a.width = IMAGE_WIDTH;
	a.height = IMAGE_HEIGHT;

	b.type = GREY_IMAGE;
	b.width = IMAGE_WIDTH;
	b.height = IMAGE_HEIGHT;

	rgb.type = RGB_IMAGE;
	rgb.width = IMAGE_WIDTH;
	rgb.height = IMAGE_HEIGHT;

	rgb0.type = RGB_IMAGE;
	rgb0.width = IMAGE_WIDTH;
	rgb0.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;

	allocate_image(a);
	allocate_image(b);
	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(label);

	return 0;

}



//-----------------------------------------------------------------------------
// Name: 
// Desc: // This function performs all the necessary image processing functions
//-----------------------------------------------------------------------------
void image_prep(int tvalue, image& rgb, image& rgb0, image& a, image& b) {
	// Image processing
	copy(rgb, a); //grey scale 

	scale(a, b);// scale the image to enhance contrast

	copy(b, a); // put result back into image a

	lowpass_filter(a, b); // Applying low pass filter

	copy(b, a);

	threshold(a, b, tvalue);

	copy(b, a);

	invert(a, b);

	copy(b, a);

	erode(a, b);

	copy(b, a);

	dialate(a, b);

	copy(b, a);

	dialate(a, b);

	copy(b, a);

}


//-----------------------------------------------------------------------------
// Name: 
// Desc: // this function calculates the centroid of all robots and obstacle
//-----------------------------------------------------------------------------
int centroids(
	image a, image rgb, image label, int& nlabels,
	double& r_ic, double& r_jc,
	double& g_ic, double& g_jc,
	double& y_ic, double& y_jc,
	double& b_ic, double& b_jc, double& obs1_ic, double& obs1_jc, double& obs2_ic, double& obs2_jc, int N_obs) {

	int N, k;
	int initializer = 0;
	ibyte* p, * pc; // pc gives current value in rgb image and pac gives current value in binary image using ic, jc
	ibyte B, G, R;
	int width = a.width;
	int height = a.height;
	

	int obs_ic, obs_jc, ic, jc;
	double m1, m2, m3, m4, mi1, mi2, mi3, mi4, mj1, mj2, mj3, mj4;
	double eps = 1.0e-10;
	mi1 = mj1 = mi2 = mj2 = mi3 = mj3 = mi4 = mj4 = m1 = m2 = m3 = m4 = 0.0;
	



	p = (ibyte*)rgb.pdata;							// points to first pixel in rgb image							// points to first pixel in binary image

	for (int n = 1; n <= nlabels; n++) {

		int k, lbl;
		int h, w; // ints are 4 bytes on the PC
		i2byte* pl; // pointer to the label image

		height = label.height;
		width = label.width;
		pl = (i2byte*)label.pdata;

		N = 0;

		for (k = 0; k < h * w; k++) { // loop for kth pixel

			lbl = *pl;

			// collect data if the pixel has the label of interest
			if (lbl == n) {
				N++;
			}
			// increment pointers
			pl++;

		}


		double obs_ic, obs_jc , i, j, ic, jc;

		
		if (N < 1999 && N>1000) {

			for (j = 0; j < height; j++) { // j coord

				for (i = 0; i < width; i++) { // i coord

					k = i + width * j;
					pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

					B = *pc;
					G = *(pc + 1);
					R = *(pc + 2);

					// find RED pixels and calculate their centroid
					if ((B < 100) && (R > 200) && (G < 100)) {

						R = 255;
						G = 0;
						B = 0;

						m1 += R;

						mi1 += i * R ; // (i moment of mk) = mk * i
						mj1 += j * R ; // (j moment of mk) = mk * j

					}
					// find GREEN pixels and calculate their centroid
					else if ((B < 150) && (R < 100) && (G > 150))
					{

						R = 0;
						G = 255;
						B = 0;

						m2 += G;

						// calculate total moments in the i and j directions
						mi2 += i * G; // (i moment of mk) = mk * i
						mj2 += j * G; // (j moment of mk) = mk * j
					}

					// find BLUE pixels and calculate their centroid
					else if ((B > 210) && (R < 50) && (G < 170))
					{

						R = 0;
						G = 0;
						B = 255;

						m3 += B;

						// calculate total moments in the i and j directions
						mi3 += i * B; // (i moment of mk) = mk * i
						mj3 += j * B; // (j moment of mk) = mk * j
					}

					// find YELLOW pixels and calculate their centroid
						//double R1, G1, RG;
					else if ((B < 150) && (R > 180) && (G > 180))
					{

						R = 200;
						G = 200;
						B = 0;

						m4 += (R + G) / 2;

						// calculate total moments in the i and j directions
						mi4 += i * (R + G) / 2; // (i moment of mk) = mk * i
						mj4 += j * (R + G) / 2; // (j moment of mk) = mk * j
					}

					r_ic = int(mi1 / (m1 + eps));
					r_jc = int(mj1 / (m1 + eps));

					//cout << "\nic1 = " << ic1 << " , jc1 = " << jc1;
					g_ic = int(mi2 / (m2 + eps));
					g_jc = int(mj2 / (m2 + eps));

					//cout << "\nic2 = " << ic2 << " , jc2 = " << jc2;
					b_ic = int(mi3 / (m3 + eps));
					b_jc = int(mj3 / (m3 + eps));

					//cout << "\nic3 = " << ic3 << " , jc3 = " << jc3;
					y_ic = int(mi4 / (m4 + eps));
					y_jc = int(mj4 / (m4 + eps));
				}
			}
		}
		else if (N > 2000) {
			if (initializer == 0) {

				centroid(a, label, n, obs_ic, obs_jc);
//				cout << "\n N_label = " << n << " Size= " << N;
//				cout << "\nobject1_ic = " << obs_ic << " object1_jc = " << obs_jc;
				//				draw_point_rgb(rgb, obs_ic, obs_jc, 255, 0, 0);
				obs1_ic = obs_ic;
				obs1_jc = obs_jc;
				initializer = 2;
			}
			else if (initializer == 2) {

				centroid(a, label, n, obs_ic, obs_jc);
//				cout << "\n N_label = " << n << " Size= " << N;
//				cout << "\nobject1_2c = " << obs_ic << " object1_2c = " << obs_jc;
				//				draw_point_rgb(rgb, obs_ic, obs_jc, 255, 0, 0);
				obs2_ic = obs_ic;
				obs2_jc = obs_jc;
				initializer = 0;

			}

		}
		else {}
	}
	return 0;
}


//This function calculates the position of both robots and the angle they make with x-axis

		
/*int position_angle(double& ic1, double& ic2, double& ic3, double& ic4, double& jc1, double& jc2, double& jc3, double& jc4,
	double& x1, double& y1, double& x2, double& y2, double& theta1, double& theta2){
	x1 = (ic1 + ic2) / 2;
	y1 = (jc1 + jc2) / 2;
	x2 = (ic3 + ic4) / 2;
	y2 = (jc3 + jc4) / 2;
	//calculating theta
	double j1_dif = jc2 - jc1;
	double i1_dif = ic2 - ic1;
	theta1 = atan2(j1_dif, i1_dif);
	theta1 = (theta1 * 180 / 3.1415) + (theta1 > 0 ? 0 : 360);
	//theta1 = (theta1 * 180 / 3.1415)+180;
	theta2 = atan2(jc4 - jc3, ic4 - ic3);
	theta2 = (theta2 * 180 / 3.1415) + (theta2 > 0 ? 0 : 360);
	//theta_obs1 = (theta_obs1 * 180 / 3.1415) + (theta_obs1 > 0 ? 0 : 360);
	//theta2 = (theta2 * 180 / 3.1415)+180;
			
	return 0;
}
*/

void position_angle(double ic1, double ic2, double ic3, double ic4, double jc1, double jc2, double jc3, double jc4,
					double &x1, double &y1, double &x2, double &y2, double &theta1, double &theta2)
{
	double M_PI = 3.14159;
	// Calculate centroids
	x1 = (ic1 + ic2) / 2;
	y1 = (jc1 + jc2) / 2;
	x2 = (ic3 + ic4) / 2;
	y2 = (jc3 + jc4) / 2;

	// Calculate angles
	double j1_dif = jc2 - jc1;
	double i1_dif = ic2 - ic1;
	theta1 = atan2(j1_dif, i1_dif);
	if (theta1 < 0)
	{
		theta1 += 2 * M_PI;
	}
	theta1 = theta1 * 180 / M_PI;

	double j2_dif = jc4 - jc3;
	double i2_dif = ic4 - ic3;
	theta2 = atan2(j2_dif, i2_dif);
	if (theta2 < 0)
	{
		theta2 += 2 * M_PI;
	}
	// This function is used to check whether obstacle is inbetween two robots

	// angles of triangle = obstacle 1/obstacle2 + robot a + robot b

	int triangle_angles(double x1, double y1, double x2, double y2, double obs1_ic, double obs1_jc,
						double obs2_ic, double obs2_jc, double &d_obs1_a, double &d_obs2_a, double &d_obs_a, double &obs_ic,
						double &obs_jc, double &theta_shoot_gap_abs)
	{

		d_obs1_a = sqrt(abs(pow(obs1_ic - x1, 2) + pow(obs1_jc - y1, 2)));
		d_obs2_a = sqrt(abs(pow(obs2_ic - x1, 2) + pow(obs2_jc - y1, 2)));
		double theta_obs;
		if (d_obs1_a < d_obs2_a)
		{
			if ((obs1_ic >= 140) && (obs1_ic <= 500) && (obs1_jc <= 140) && (obs1_jc <= 340))
			{
				obs_ic = obs1_ic;
				obs_jc = obs1_jc;
				d_obs_a = d_obs1_a;
			}
		}
		else
		{
			obs_ic = obs2_ic;
			obs_jc = obs2_jc;
			d_obs_a = d_obs2_a;
		}

		double theta_obs_a, theta_obs_b, theta_shoot_gap;
		theta_obs_a = (atan2(obs_jc - y1, obs_ic - x1));
		theta_obs_a = (theta_obs_a * 180 / 3.1415) + (theta_obs_a > 0 ? 0 : 360);

		theta_obs_b = (atan2(obs_jc - y2, obs_ic - x2));
		theta_obs_b = (theta_obs_b * 180 / 3.1415) + (theta_obs_b > 0 ? 0 : 360);

		theta_shoot_gap = theta_obs_a - theta_obs_b;
		theta_shoot_gap_abs = abs(theta_shoot_gap);

		return 0;
	}

	// This function is used to check whether obstacle is inbetween two robots

	// triangle angles of defence-------------------

	int triangle_angles_d(double &x1, double &y1, double &x2, double &y2, double &obs1_ic, double &obs1_jc,
						  double &obs2_ic, double &obs2_jc, double &d_obs1_b, double &d_obs2_b, double &d_obs_b,
						  double &obs_ic, double &obs_jc, double &theta_shoot_gap_abs)
	{

		d_obs1_b = sqrt(abs(pow(obs1_ic - x2, 2) + pow(obs1_jc - y2, 2)));
		d_obs2_b = sqrt(abs(pow(obs2_ic - x2, 2) + pow(obs2_jc - y2, 2)));
		double theta_obs;
		if (d_obs1_b < d_obs2_b)
		{
			if ((obs1_ic >= 140) && (obs1_ic <= 500) && (obs1_jc <= 140) && (obs1_jc <= 340))
			{
				obs_ic = obs1_ic;
				obs_jc = obs1_jc;
				d_obs_b = d_obs1_b;
			}
			else
			
		}
		else
		{
			obs_ic = obs2_ic;
			obs_jc = obs2_jc;
			d_obs_b = d_obs2_b;
		}

		double theta_obs_a, theta_obs_b, theta_shoot_gap;
		theta_obs_a = (atan2(obs_jc - y1, obs_ic - x1));
		theta_obs_a = (theta_obs_a * 180 / 3.1415) + (theta_obs_a > 0 ? 0 : 360);

		theta_obs_b = (atan2(obs_jc - y2, obs_ic - x2));
		theta_obs_b = (theta_obs_b * 180 / 3.1415) + (theta_obs_b > 0 ? 0 : 360);

		theta_shoot_gap = theta_obs_a - theta_obs_b;
		theta_shoot_gap_abs = abs(theta_shoot_gap);

		return 0;
	}

	// This function avoid the obstacle by setting up two points at 90 deg and the move towards it once
	// reaches within a predefined distance

	// ==============two point based approach----------------------------
	int obstacle_avoid(double &g_ic, double &g_jc, double &obs_ic, double &obs_jc, double &d_obs_a,
					   double &theta1, double &x2, double &y2, double &p1x, double &p1y,
					   double &p2x, double &p2y, int &pw_r, int &pw_l)
	{

		double d_obs, theta_obs;

		theta_obs = (atan2(obs_jc - g_jc, obs_ic - g_ic));
		cout << "\n theta_obs = " << theta_obs;
		// theta_obs = (theta_obs * 180 / 3.1415) + (theta_obs > 0 ? 0 : 360);
		double c = 250;

		p1x = (d_obs_a / 6) * cos(theta_obs) + c * cos((theta_obs + 1.5708)) + g_ic;
		p1y = (d_obs_a / 6) * sin(theta_obs) + c * sin((theta_obs + 1.5708)) + g_jc;

		p2x = (d_obs_a / 6) * cos(theta_obs) + c * cos(theta_obs + 4.71239) + g_ic;
		p2y = (d_obs_a / 6) * sin(theta_obs) + c * sin(theta_obs + 4.71239) + g_jc;

		double d_p1_b, d_p2_b, pxx, pyy, d_p_b;

		d_p1_b = sqrt(abs(pow(p1x - x2, 2) + pow(p1y - y2, 2)));
		d_p2_b = sqrt(abs(pow(p2x - x2, 2) + pow(p2y - y2, 2)));

		double d_p1_a, d_p2_a, d_p_a;
		d_p1_a = sqrt(abs(pow(p1x - g_ic, 2) + pow(p1y - g_jc, 2)));
		d_p2_a = sqrt(abs(pow(p2x - g_ic, 2) + pow(p2y - g_jc, 2)));

		if (d_p1_b < d_p2_b)
		{
			pxx = p1x;
			pyy = p1y;
			d_p_b = d_p1_b;
			d_p_a = d_p1_a;
		}
		else
		{
			pxx = p2x;
			pyy = p2y;
			d_p_b = d_p2_b;
			d_p_a = d_p2_a;
		}
		// cout << "\n d_p_a = " << d_p_a;
		double theta_p_a, theta_pxx_b_gap, theta_pxx_b_gap_abs;
		theta_p_a = (atan2(pyy - g_jc, pxx - g_ic));
		theta_p_a = (theta_p_a * 180 / 3.1415) + (theta_p_a > 0 ? 0 : 360);

		theta_pxx_b_gap = theta_p_a - theta1;
		theta_pxx_b_gap_abs = abs(theta_pxx_b_gap);

		if ((theta_pxx_b_gap_abs > 7) && (theta_pxx_b_gap > 0))
		{
			// rotate left
			pw_l = 2000;
			pw_r = 2000;
		}
		else if ((theta_pxx_b_gap_abs > 7) && (theta_pxx_b_gap < 0))
		{
			// rotate right
			pw_l = 1000;
			pw_r = 1000;
		}
		else if ((d_p_a > 0))
		{
			// forward
			pw_l = 1000;
			pw_r = 2000;
		}
		else{}
	return 0;
}


//--------------------TWO POINT BASED APPROACH FOR DEFENCE---------------------------------------
// This function avoid the obstacle by setting up two points at 90 deg and the move towards it once 
// reaches within a predefined distance


int obstacle_avoid_defence(double& y_ic, double& y_jc, double& g_ic, double& g_jc, double& theta2,
	double& d_obs_b, double& obs_ic, double& obs_jc, double& px, double& py, double& p1x, double& p1y,
	double& p2x, double& p2y, int& pw_r, int& pw_l) {

	
	double obs_ic_d, obs_jc_d, d_obs_d, theta_obs;
	

	theta_obs = (atan2(obs_jc - y_jc, obs_ic - y_ic));
	//cout << "\n theta_obs = " << theta_obs;
	//theta_obs = (theta_obs * 180 / 3.1415) + (theta_obs > 0 ? 0 : 360);
	double c = 250;

	p1x = (d_obs_b / 6) * cos(theta_obs) + c * cos((theta_obs + 1.5708)) + y_ic;
	p1y = (d_obs_b / 6) * sin(theta_obs) + c * sin((theta_obs + 1.5708)) + y_jc;

	p2x = (d_obs_b / 6) * cos(theta_obs) + c * cos(theta_obs + 4.71239) + y_ic;
	p2y = (d_obs_b / 6) * sin(theta_obs) + c * sin(theta_obs + 4.71239) + y_jc;

	double d_p1_p, d_p2_p, pxx, pyy, d_p_b, theta_obs_a;


	d_p1_p = sqrt(abs(pow(p1x - px, 2) + pow(p1y - py, 2)));
	d_p2_p = sqrt(abs(pow(p2x - px, 2) + pow(p2y - py, 2)));

	double d_p1_b, d_p2_b, d_p_p;
	d_p1_b = sqrt(abs(pow(p1x - y_ic, 2) + pow(p1y - y_jc, 2)));
	d_p2_b = sqrt(abs(pow(p2x - y_ic, 2) + pow(p2y - y_jc, 2)));

	if (d_p1_p < d_p2_p) {
		pxx = p1x;
		pyy = p1y;
		d_p_p = d_p1_p;
		d_p_b = d_p1_b;
	}
	else {
		pxx = p2x;
		pyy = p2y;
		d_p_p = d_p2_p;
		d_p_b = d_p2_b;
	}
	//cout << "\n d_p_a = " << d_p_p;
	double theta_p_b, theta_pxx_b_gap, theta_pxx_b_gap_abs;
	theta_p_b = (atan2(pyy - y_jc, pxx - y_ic));
	theta_p_b = (theta_p_b * 180 / 3.1415) + (theta_p_b > 0 ? 0 : 360);

	theta_pxx_b_gap = theta_p_b - theta2;
	theta_pxx_b_gap_abs = abs(theta_pxx_b_gap);

	if ((theta_pxx_b_gap_abs > 5) && (theta_pxx_b_gap > 0)) {
		// rotate left
		pw_l = 2000;
		pw_r = 2000;
	}
	else if ((theta_pxx_b_gap_abs > 5) && (theta_pxx_b_gap < 0)) {
		// rotate right
		pw_l = 1000;
		pw_r = 1000;
	}
	else if ((d_p_b > 0)) {
		// forward
		pw_l = 1000;
		pw_r = 2000;
	}
	else {
		pw_l = 1500;
		pw_r = 1500;
	}

	return 0;
}


//--------------------complete defence------------------

// This function makes the robot move towards a point px, py (st up in program)
// so that it can save itself by hiding behind obstacle

////----------------------- working ---------------------------
int complete_defence(double& y_ic, double& y_jc, double& g_ic, double& g_jc, double& theta2,
	double& px, double& py, int& pw_r, int& pw_l) {

	// finds the obstcle
	double theta_obs_gap_abs, d_obs_o;

	double theta_px_b, theta_px_b_gap, theta_px_b_gap_abs, distance_p_b;
	double tolerance = 7;
	theta_px_b = (atan2(py - y_jc, px - y_ic));
	theta_px_b = (theta_px_b * 180 / 3.1415) + (theta_px_b > 0 ? 0 : 360);
	theta_px_b_gap = theta_px_b - theta2;
	theta_px_b_gap_abs = abs(theta_px_b_gap);

	distance_p_b = sqrt((pow(px - y_ic, 2) + pow(py - y_jc, 2)));

	if ((theta_px_b_gap_abs > tolerance) && (theta_px_b_gap < 0)) {

		// rotate right
		pw_l = 1000;
		pw_r = 1000;
	}
	else if ((theta_px_b_gap_abs > tolerance) && (theta_px_b_gap >= 0)) {
		// rotate left
		pw_l = 2000;
		pw_r = 2000;
	}
	else if ((theta_px_b_gap_abs <= tolerance) && (distance_p_b > 0)) {
		// forward
		pw_l = 1000;
		pw_r = 2000;
	}
	//	else if (distance_p_b < 0) {
	//		// backward
	//		pw_l_o = 2000;
	//		pw_r_o = 1000;
	//	}
	else {
		pw_l = 1500;
		pw_r = 1500;
	}

	return 0;
}

// This function makes robot align itself and move towards opponent in order to attack
// once the robot is aligned, it fires laser at opponent


void control_robot(double& x1, double& y1, double& x2, double& y2,
	double& theta1, double& theta_gap, double& theta, 
	double& distance_a_b, int& pw_r, int& pw_l, int& laser)
{
	double dpw = 500;
	//int distance;
	int tolerance = 3;
	//double theta_new;
	double distance_o1, distance_o2;
	//double y_dif = y2 - y1;
	//double x_dif = x2 - x1;

	theta = (atan2(y2 - y1, x2 - x1));
	theta = (theta * 180 / 3.1415) + (theta > 0 ? 0 : 360);

	distance_a_b = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
	
	theta_gap = theta - theta1;
	double theta_gap_abs = abs(theta_gap);


	    if ((theta_gap < 0) && (theta_gap_abs > tolerance)) {
			// rotate right
			pw_l = 1500 - dpw;
			pw_r = 1500 - dpw;
			
		}

		else if ((theta_gap >= 0) && (theta_gap_abs > tolerance)) {
			// rotate left
			pw_l = 1500 + dpw;
			pw_r = 1500 + dpw;
			
		}
		
	    else if (distance_a_b > 250) {
			// forward
			pw_l = 1500 - dpw;
			pw_r = 1500 + dpw;
		}
		else if (distance_a_b < 150) {
		//	// backward
			pw_l = 1500 + dpw;
			pw_r = 1500 - dpw;
		}
		else if ((theta_gap_abs < tolerance) && (distance_a_b <= 250) && (distance_a_b >= 105)) {
			//stop
			pw_l = 1500;
			pw_r = 1500;
			laser = 1;
		}
		else {}
	
}



void image_prep_optional(int tvalue, image& rgb, image& rgb0, image& a, image& b) {

	// I have coded this for the optional part. We can use a textured image now or a white image with black pattern. This fxn will hopefully work 
	// 	   //
		// testing start
	copy(rgb, a); //grey scale 

	scale(a, b);// scale the image to enhance contrast

	copy(b, a); // put result back into image a

	//
	copy(a, rgb);
	view_rgb_image(rgb);
	pause();
	//

	lowpass_filter(a, b); // Applying low pass filter

	copy(b, rgb);
	view_rgb_image(rgb);
	pause();

	lowpass_filter(b, a);

	copy(a, rgb);
	view_rgb_image(rgb);
	pause();




	copy(b, a);



	threshold(a, b, tvalue);

	copy(b, a);

	invert(a, b);

	copy(b, a);

	erode(a, b);

	copy(b, a);

	dialate(a, b);

	copy(b, a);

	dialate(a, b);

	copy(b, a);
	// testing end
	//

	copy(a, rgb);
}



