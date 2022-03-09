#ifndef GEODESY_H
#define GEODESY_H

#include <gps_common/conversions.h>

class Geodesy
{
  public:
    Geodesy()
    {
      this->lat1 = 0;
      this->lon1 = 0;
    };
    
    void setOrigin(double lat, double lon)
    {
      this->lat1 = lat*M_PI/180;;
      this->lon1 = lon*M_PI/180;
      
      double a = gps_common::WGS84_A;
      double b = gps_common::WGS84_B;
      
      this->R = sqrt( (a*a*a*a*cos(this->lat1)*cos(this->lat1) + b*b*b*b*sin(this->lat1)*sin(this->lat1)) / (a*a*cos(this->lat1)*cos(this->lat1) + b*b*sin(this->lat1)*sin(this->lat1)) );
    };
    
    void LLtoEuclidean(double lat2, double lon2, double &x, double &y)
    {
      lat2 *= M_PI/180;
      lon2 *= M_PI/180;
      
      double dlat = lat2 - this->lat1;
      double dlon = lon2 - this->lon1;
      
      double A = sin(dlat/2)*sin(dlat/2) +sin(dlon/2)*sin(dlon/2)*cos(this->lat1)*cos(lat2);
      double C = 2*atan2(sqrt(A), sqrt(1-A));
      
      double dist = this->R * C;
      
      double n = sin(dlon)*cos(lat2);
      double m = cos(this->lat1)*sin(lat2) - sin(this->lat1)*cos(lat2)*cos(dlon);
      double bearing = atan2(n,m);
      
      x = dist*sin(bearing);
      y = dist*cos(bearing);
  };
    
  private:
    double lat1, lon1;
    double R;
    
};

#endif
