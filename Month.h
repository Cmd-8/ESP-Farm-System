#ifndef MONTH_H
#define MONTH_H

//This makes a class call Month 
class Month {
public:
  int hours;        //Hours of sunlight 
  int days;         //How many days
  int sleep;        //Hours of night time 
  float Rise;       //Degree of sun-rise
  float Set;        //Degree of sun-set
  float Elevation;  //Max elevation of sun 

  Month(int month_hours, int month_days, int month_sleep, float month_Rise_Azimuth, float month_Set_Azimuth, float month_Elevation){
    hours = month_hours;
    days = month_days;
    sleep = month_sleep;
    Rise = month_Rise_Azimuth;
    Set = month_Set_Azimuth;
    Elevation = month_Elevation;
  }
  void display();
};

#endif