#ifndef _LINE_H_
#define _LINE_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "struct.h"

/*
#define PI 3.1415926535898
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
}*/

namespace line_detection
{
    class line_detector
    {
        public:
        line_detector();
        ~line_detector();
        void setCosSinData(const std::vector<double>&,  const std::vector<double>&, const std::vector<double>&, const std::vector<unsigned int>&);
        void setRangeData(const std::vector<double>&);
        void extractline(std::vector<line_segment>& temp_line1, std::vector<gline>&);
        void set_angle_increment(double);
        void set_angle_start(double);
        void set_least_threshold(double);
        void set_min_line_length(double);
        void set_predict_distance(double);
        void set_min_line_points(unsigned int);
        void set_seed_line_points(unsigned int);

        private:
        least leastsquare(int,int,int);
        bool detectline(const int, const int);
        int detectfullline(const int);
        void cleanline();
        bool delete_short_line(const int, const int);
        void generate(std::vector<gline>& temp_line2);

        private:
        CSData cs_data_;
        RangeData range_data_;
        Params params_;
        std::vector<unsigned int> point_num_;
        std::vector<line_segment> m_line;
        least m_least;

        double mid1;
        double mid2;
        double mid3;
        double mid4;
        double mid5;
    };
}
#endif //_LINE_H_