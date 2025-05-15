#include <vector>

#include "../include/line_detect.h"
#include <chrono>

#include "../include/struct.h"
#include <algorithm>


auto now = std::chrono::high_resolution_clock::now();

namespace line_detection {
    line_detector::line_detector()
    {
    }


    line_detector::~line_detector()
    {

    }

    //set params

    void line_detector::set_angle_increment(double angle_increment)
    {
        params_.angle_increment = angle_increment;
    }

    void line_detection::line_detector::set_angle_start(double angle_start)
    {
        params_.angle_start = angle_start;
    }
    void line_detection::line_detector::set_least_threshold(double least_thresh)
    {
        params_.least_thresh = least_thresh;
    }

    void line_detection::line_detector::set_min_line_length(double  min_line_length)
    {
        params_.min_line_length = min_line_length;
    }

    void line_detection::line_detector::set_predict_distance(double predict_distance)
    {
        params_.predict_distance= predict_distance;
    }

    void line_detection::line_detector::set_min_line_points(unsigned int min_line_points)
    {
        params_.min_line_points= min_line_points;
    }

    void line_detection::line_detector::set_seed_line_points(unsigned int seed_line_points)
    {
        params_.seed_line_points = seed_line_points;
    }

    //---------

    void line_detection::line_detector::setCosSinData(const std::vector<double>& bearings, const std::vector<double>& cos_value, const std::vector<double>& sin_value , const std::vector<unsigned int>& index)
    {
        cs_data_.index = index;
        cs_data_.cos_value=cos_value;
        cs_data_.sin_value=sin_value;
        cs_data_.bearings=bearings;
    }

    void line_detection::line_detector::setRangeData(const std::vector<double>& ranges)
    {

        range_data_.ranges = ranges;
        range_data_.xs.clear();
        range_data_.ys.clear();
        for(std::vector<unsigned int>::const_iterator cit = cs_data_.index.begin(); cit !=cs_data_.index.end(); ++cit)
        {
            range_data_.xs.push_back(cs_data_.cos_value[*cit]*ranges[*cit]);
            range_data_.ys.push_back(cs_data_.sin_value[*cit]*ranges[*cit]);
        }
        std::cout << "[DEBUG] setRangeData: index size = " << cs_data_.index.size()
                  << ", range size = " << ranges.size() << std::endl;

        std::cout << "First 10 (x, y) points:\n";
        for (size_t i = 0; i < std::min<size_t>(10, range_data_.xs.size()); ++i) {
            std::cout << "(" << range_data_.xs[i] << ", " << range_data_.ys[i] << ")\n";
        }

        /*chatbot forslag
        {
            range_data_.ranges = ranges;
            range_data_.xs.resize(ranges.size());
            range_data_.ys.resize(ranges.size());

            for (size_t i = 0; i < ranges.size(); ++i) {
                range_data_.xs[i] = cs_data_.cos_value[i] * ranges[i];
                range_data_.ys[i] = cs_data_.sin_value[i] * ranges[i];
            }
        }
    */

    }



    least line_detection::line_detector::leastsquare(int start, int end,int firstfit)
    {
        double  w1=0,
                w2=0,
                w3=0;
        least temp;
        double n = end-start+1;

        if (firstfit==1)
        {
            mid1=0;
            mid2=0;
            mid3=0;
            mid4=0;
            mid5=0;
            int k = 0;
            for (k=start; k <=end;k++)
            {
                mid1+=range_data_.xs[k];
                mid2+=range_data_.ys[k];
                mid3+=range_data_.xs[k]*range_data_.xs[k];
                mid4+=range_data_.ys[k]*range_data_.ys[k];
                mid5+=range_data_.xs[k]*range_data_.ys[k];
            }
        }
        else
        {
            if(firstfit==2)
            {
                mid1+=range_data_.xs[end];
                mid2+=range_data_.ys[end];
                mid3+=range_data_.xs[end]*range_data_.xs[end];
                mid4+=range_data_.ys[end]*range_data_.ys[end];
                mid5+=range_data_.xs[end]*range_data_.ys[end];
            }
            else
            {
                mid1+=range_data_.xs[start];
                mid2+=range_data_.ys[start];
                mid3+=range_data_.xs[start]*range_data_.xs[start];
                mid4+=range_data_.ys[start]*range_data_.ys[start];
                mid5+=range_data_.xs[start]*range_data_.ys[start];
            }
        }
        w1=n*mid5-mid1*mid2;
        w2=mid2*mid2-n*mid4-mid1+n*mid3;
        w3=mid1*mid2-n*mid5;

        if(w1==0)
        {
            temp.a = -1;
            temp.b=0;
            temp.c=mid1/n;
        }
        else
        {
            temp.a=(-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
            temp.b=-1;
            temp.c=(mid2-temp.a*mid1)/n;
        }
        return temp;
    }

    bool line_detection::line_detector::detectline(const int start,const int num)
    {
        bool flag=false;
        double error1=0;
        double error2=0;
        int k=0;

        POINT m_pn;
        m_pn.x=0;
        m_pn.y=0;

        double kp=0;
        double theta = 0;

        for(k=start; k<start+num;k++)
        {
            error1=fabs(((m_least.a)*range_data_.xs[k]+(m_least.b)*range_data_.ys[k]+m_least.c))/sqrt((1+(m_least.a)*(m_least.a)));

            if (error1>params_.least_thresh)
            {
                flag=true;
                break;
            }
            theta+=params_.angle_increment*k+ params_.angle_start;
            if(fabs((fabs(theta)-PI/2))<1e-05)
            {
                m_pn.x= 0;
                m_pn.y=m_least.c;
            }
            else
            {
                kp=tan(theta);
                m_pn.x = (m_least.c)/(kp-m_least.a);
                m_pn.y=kp*m_pn.x;
            }
            error2=euclideanDistance(range_data_.xs[k],range_data_.ys[k], m_pn.x,m_pn.y);
            if(error2>params_.predict_distance)
            {
                flag=true;
                break;
            }
        }
        if(flag)
            return false;
        else
            return true;
    }

    int line_detection::line_detector::detectfullline(const int start)
    {
        line_segment m_temp;

        bool flag1 =true;
        bool flag2 =true;
        int n1=0;
        int n2=0;
        double a= 0;
        double b= 0;
        double c= 0;

        a= m_least.a;
        b= m_least.b;
        c= m_least.c;

        n2=start+ params_.seed_line_points;
        least m_result;
        m_result.a=0;
        m_result.b=0;
        m_result.c=0;

        while(flag2)
        {
            if((fabs(a*range_data_.xs[n2]+b*range_data_.ys[n2]+c)/(sqrt(1+a*a)))<params_.least_thresh)
            {
                m_least=leastsquare(start,n2,2);
                if(n2<(point_num_.size()-1))
                {
                    n2=n2+1;
                    a=m_least.a;
                    b=m_least.b;
                    c=m_least.c;
                }
                else
                {
                    flag2=false;
                }
            }
            else
            {
                flag2=false;
            }
        }
        n2=n2-1;

        n1=start-1;
        if(n1<0)
        {
            flag1=false;
        }
        while(flag1)
        {
            if((fabs(a*range_data_.xs[n1]+b*range_data_.ys[n1]+c)/(sqrt(1+a*a)))<params_.least_thresh)
            {
                m_least=leastsquare(n1,n2,3);
                if(n1>0)
                {
                    n1=n1-1;
                    a=m_least.a;
                    b=m_least.b;
                    c=m_least.c;
                }
                else
                {
                    flag1=false;
                }
            }
            else
            {
                flag1=false;
            }
        }
        n1=n1+1;

        m_temp.left=n1;
        m_temp.right=n2;

        m_result=leastsquare(n1,n2,1);
        m_temp.a=m_result.a;
        m_temp.b=m_result.b;
        m_temp.c=m_result.c;

        if((n2-n1)>params_.min_line_points)
        {
            if(delete_short_line(n1,n2))
            {
                m_line.push_back(m_temp);
            }
            return n2;
        }
        else
        {
            return start;
        }
    }

    void line_detection::line_detector::cleanline()
    {
        if(m_line.size()<2)
        {
            return;
        }

        int m=0;
        int n=0;
        int m_iter=0;
        double error1=0;
        double error2=0;
        int line_temp=0;
        least temp_least;
        temp_least.a=0;
        temp_least.b=0;
        temp_least.c=0;

        double theta_one_=0;
        double theta_two_=0;
        double theta_d_=0;
        std::size_t q = 0, p=0;

        for(p=0;p<m_line.size();p++)
        {
            m=m_line[p].right;
            for(q=p+1;q  < m_line.size();q++)
            {
                n=m_line[q].left;
                if(m>=n)
                {
                    theta_one_=atan(m_line[p].a);
                    theta_two_=atan(m_line[q].a);

                    theta_d_=fabs(theta_one_ - theta_two_);

                    if((theta_d_<0.1)||(theta_d_>(PI - 0.1)))
                    {
                        int _left= std::min(m_line[p].left,m_line[q].left);

                        least m_temp = leastsquare(_left,m_line[q].right,1);

                        m_line[p].a=m_temp.a;
                        m_line[p].b=m_temp.b;
                        m_line[p].c=m_temp.c;

                        m_line[p].left=_left;
                        m_line[p].right=m_line[q].right;

                        m_line.erase(m_line.begin()+q);

                        m=m_line[p].right;
                        q=q-1;
                    }
                }
            }
        }
        for(p=0; p<m_line.size()-1;p++)
        {
            q=p+1;
            m=m_line[p].right;
            n=m_line[q].left;
            if(m>=n)
            {
                for(m_iter=n;m_iter<=m;m_iter++)
                {
                    line_temp=m_iter;
                    error1=fabs(((m_line[p].a)*range_data_.xs[m_iter]+(m_line[p].b*range_data_.ys[m_iter]+m_line[p].c))/sqrt((1+(m_line[p].a)*m_line[p].a)));
                    error2=fabs(((m_line[q].a)*range_data_.xs[m_iter]+(m_line[q].b)*range_data_.ys[m_iter]+m_line[q].c))/sqrt((1+(m_line[q].a)*m_line[q].a));
                    if(error1>error2)
                    {
                        break;
                    }


                }
                m_line[p].right = m_iter-1;
                temp_least=leastsquare(m_line[p].left,m_line[p].right,1);
                m_line[p].a=temp_least.a;
                m_line[p].b=temp_least.b;
                m_line[p].c=temp_least.c;

                m_line[q].left=m_iter;
                temp_least=leastsquare(m_line[q].left,m_line[q].right,1);
                m_line[q].a=temp_least.a;
                m_line[q].b=temp_least.b;
                m_line[q].c=temp_least.c;
            }
        }
    }

    bool line_detection::line_detector::delete_short_line(const int n1, const int n2)
    {
        if(euclideanDistance(range_data_.xs[n1],range_data_.ys[n1],range_data_.xs[n2],range_data_.ys[n2])<params_.min_line_length)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void line_detection::line_detector::generate(std::vector<gline>& temp_line2)
    {
        gline line_temp;
        std::vector<gline> output;
        POINT endpoint1;
        POINT endpoint2;
        int m=0 , n=0;
        double k1=0,k2=0;
        for(int i=0; i<m_line.size(); i++)
        {
            m=m_line[i].left;
            n=m_line[i].right;

            if(m_line[i].b!=0)
            {
                endpoint1.x=(range_data_.xs[m]/m_line[i].a+range_data_.ys[m]-m_line[i].c)/(m_line[i].a+1.0/(m_line[i].a));
                endpoint1.y=m_line[i].a*endpoint1.x+m_line[i].c;
            }
            else
            {
                endpoint1.x=range_data_.ys[m];
                endpoint1.y=m_line[i].c/m_line[i].a;
            }

            line_temp.x1=endpoint1.x;
            line_temp.y1=endpoint1.y;

            m_line[i].p1=endpoint1;

            if(m_line[i].b!=0)
            {
                endpoint2.x=(range_data_.xs[n]/m_line[i].a + range_data_.ys[n] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
                endpoint2.y=m_line[i].a*endpoint2.x+m_line[i].c;
            }
            else
            {
                endpoint2.x=range_data_.ys[n];
                endpoint2.y=m_line[i].c/m_line[i].a;
            }

            line_temp.x2=endpoint2.x;
            line_temp.y2=endpoint2.y;

            m_line[i].p2=endpoint2;

            output.push_back(line_temp);
        }
        temp_line2=output;
    }
    /*chatbot forslag----------------------------------------
    void line_detection::line_detector::generate(std::vector<gline>& temp_line2)
    {
        gline line_temp;
        std::vector<gline> output;

        for (const auto& seg : m_line)
        {
            unsigned int left = seg.left;
            unsigned int right = seg.right;

            std::cout << "Line index range: left=" << left << " right=" << right << std::endl;

            if (left >= range_data_.xs.size() || right >= range_data_.xs.size())
                continue;

            line_temp.x1 = range_data_.xs[left];
            line_temp.y1 = range_data_.ys[left];
            line_temp.x2 = range_data_.xs[right];
            line_temp.y2 = range_data_.ys[right];

            output.push_back(line_temp);

            std::cout << "Line from (" << line_temp.x1 << ", " << line_temp.y1 << ") to ("
              << line_temp.x2 << ", " << line_temp.y2 << ")\n";

            if (left >= range_data_.xs.size() || right >= range_data_.xs.size())
                continue;

            std::cout << "Point at left=" << left << ": (" << range_data_.xs[left] << ", " << range_data_.ys[left] << ")\n";
            std::cout << "Point at right=" << right << ": (" << range_data_.xs[right] << ", " << range_data_.ys[right] << ")\n";


        }

        temp_line2 = output;
    }*/
    //-------------------------------

    void line_detection::line_detector::extractline(std::vector<line_segment>& temp_line1, std::vector<gline>& temp_line2)
    {
        int line_include=0;
        m_line.clear();
        point_num_ = cs_data_.index;

        if(point_num_.size()<params_.min_line_points)
        {
            return;
        }
        for(unsigned int i=0; i<(point_num_.size()-params_.min_line_points);i++)
        {
            m_least=leastsquare(i,i+params_.seed_line_points -1,1);
            if(detectline(i, params_.seed_line_points))
            {
                line_include=detectfullline(i);
                i=line_include;
            }
        }
        cleanline();
        for(int p=0; p<m_line.size();p++)
        {
            if(!delete_short_line(m_line[p].left,m_line[p].right))
            {
                m_line.erase(m_line.begin()+p);
            }
        }
        generate(temp_line2);
        temp_line1=m_line;
    }
}






