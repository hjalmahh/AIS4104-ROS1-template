//
// Created by hilde on 13.05.2025.
//

#ifndef STRUCT_H
#define STRUCT_H

#include <vector>
#include<iostream>
#include<string.h>
#include<stdio.h>
#include<math.h>

constexpr double PI = 3.1415926535898;
//#define t(a) ((a)*PI/180)
inline double toRadians(double degrees) { return degrees * PI / 180.0; }
#define treshold_a 6
//#define distance(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))
inline double euclideanDistance(double a1, double a2, double b1, double b2) {
    return sqrt((a1 - b1) * (a1 - b1) + (a2 - b2) * (a2 - b2));
}
//#define max(a,b) (a>b)?a:b
inline double maxValue(double a, double b) {
    return (a > b) ? a : b;
}



struct POINT
{
    double x;
    double y;
};

struct CSData
{
    std::vector<unsigned int> index;
    std::vector<double> bearings;
    std::vector<double> cos_value;
    std::vector<double> sin_value;
};

struct RangeData
{
    std::vector<double> ranges;
    std::vector<double> xs;
    std::vector<double> ys;
};

struct Params
{
    double angle_increment;
    double angle_start;
    double least_thresh;
    double min_line_length;
    double predict_distance;
    unsigned int min_line_points;
    unsigned int seed_line_points;

};

struct word_params
{
    double _role;
    double _theta_one;
    double _theta_two;
};

struct signal_params
{
    double distance_signal;
};

struct line_segment
{
    double a;
    double b;
    double c;
    int left;
    int right;
    POINT p1;
    POINT p2;
    bool inte[2];
};

struct least
{
    double a;
    double b;
    double c;
};

struct point
{
    double role;
    double theta;
    double m_x;
    double m_y;
    double distance;
    double m_gradient;
    bool flag;
};


struct gline
{
    double x1;
    double y1;

    double x2;
    double y2;
};

struct signal
{
    double _angle1_radian;
    double _angle2_radian;
    double _angle1_degree;
    double _angle2_degree;
    double role;
};

struct feature_point
{
    POINT _point;
    double _angle;
};

struct keyword
{
    int _index_role;
    int _index_theta_one;
    int _index_theta_two;
    std::vector<int> _frame_index;
};

#endif //STRUCT_H
