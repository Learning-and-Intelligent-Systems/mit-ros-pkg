#pragma once

const float PI = 3.14159;

float angle_mean(float theta1, float theta2) {
	float a1 = (theta1 + theta2) / 2;
	float a2 = a1 + PI;
	float a;
	if (fabs(angle_diff(a1, theta1)) < fabs(angle_diff(a2, theta1))
		a = a1;
	else
		a = a2;
	return a;
}

float angle_diff(float theta1, float theta2) {
	difference = theta2 - theta1;
	while (difference <= -PI)
		difference += 2*PI;
	while (difference > PI)
		difference -= 2*PI;
	return difference;
}

float line_point_dist(float r, float theta, float x, float y) {
	return 0;
}

void line_clip(float r, float theta, float x0, float y0, float x1, float y1, float &edge_x0, float &edge_x1, float &edge_y0, float &edge_y1) {
}

void edge2polar(float x0, float y0, float x1, float y1, float &r, float &theta) {
}

void book_edge_dist(float leftR, float leftTheta, float rightR, float rightTheta, float &dr, float &dtheta) {
}
