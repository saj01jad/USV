/******AUV defination******/
#include <Arduino.h>
#include <UbxGpsNavPvt.h>
#include <Servo.h>

// definition of GPS.
#define GPS_BAUDRATE 115200L
#define PC_BAUDRATE 9600L

#define DATETIME_FORMAT "%04d.%02d.%02d %02d:%02d:%02d"
#define DATETIME_LENGTH 20

UbxGpsNavPvt<HardwareSerial> gps(Serial3);


char datetime[DATETIME_LENGTH];

double haversineDistance(double lat1, double lon1, double lat2, double lon2);

// make instance of Servo
Servo myservoh1;
Servo myservoh2;
Servo myservof;
Servo myservo0;
Servo myservo1;

// defination of variabel
double heading = 0,pre_heading,avrage_heading;//heading of GPS
double cur_lat, cur_lon;//current lat and lon
int dis_counter = 0, dis_number = 6;//number of distanation and counter of it
double dis_lat[10] = { 318392714.0, 318393827.0, 318394163.0, 318394585.0, 318393920.0, 318393547.0, 0.0, 0.0, 0.0, 0.0} , dis_lon[10] = { 543580863.0, 543582892.0, 543582911.0, 543582419.0, 543581474.0, 543580988.0, 0.0, 0.0, 0.0, 0.0};//distanation lat and lon;
double dis_lat_temp, dis_lon_temp;//temparary of distanation;
double orgin_lat, orgin_lon;//firest lat and lon to shift data and make it smaller
double diff_lat, diff_lon;// diffrance of lat and lon
double angle, d_angle; // find angle bettwen current point and distantion point and angle error
int margin_angle = 5;// margin of angle to go forward
double distance;
double array_lat[5] = {0,0,0,0,0};// moving avrage array lat
double array_lon[5] = {0,0,0,0,0};// moving avrage array lon
double avrage_lat, avrage_lon;// avrage lat and lon
double taw,F;//rotation and forward value
double k_taw = 3,k_f = 30;//Controller coefficients
double max_duty_auv = 1700, min_duty_auv = 1400;// min and max of motor speed
double motor[2];// value of motors
int counter = 0;//counter of moving avrage fillter
int check = 0;//check to start controller
bool auv_state = 1,auv_state_loop = 1;//check state of mode
char mode = 'b';// detarmin mode of controller
void setup() {
  Serial.begin(PC_BAUDRATE);
  gps.begin(GPS_BAUDRATE);
  Serial.println("please wait ...");
  //detarmin pin of each motors
  myservoh1.attach(10);
  myservoh2.attach(11);
  myservof.attach(12);
  myservo0.attach(9);
  myservo1.attach(8);

  //set motors on off mode
  myservoh1.writeMicroseconds(1500);
  myservoh2.writeMicroseconds(1500);
  myservof.writeMicroseconds(1500);
  myservo0.writeMicroseconds(1500);
  myservo1.writeMicroseconds(1500);

  //print user gide
  Serial.println("a => USV ");
  Serial.println("R => reset");
}
void loop() {
  //get mode of controller
  if(Serial.available()>0){
   mode = (char) Serial.read();
   Serial.read();
  }
  if (mode == 'a'){
    while(1){
      if(auv_state){
        Serial.println("*** USV ***");
        Serial.println("s => start");
        Serial.println("l => loop");
        Serial.println("b => break");
        Serial.println("R => reset");
        auv_state = 0;
      }

      // robot is allways at surface of water 
      myservoh1.writeMicroseconds(1580);
      myservoh2.writeMicroseconds(1580);
      myservof.writeMicroseconds(1630);

      if(Serial.available()>0){
       mode = (char) Serial.read();
       Serial.read();
      }
      //to break mode
      if (mode == 'b'){
        Serial.println("a => USV ");
        Serial.println("R => reset");
        myservoh1.writeMicroseconds(1500);
        myservoh2.writeMicroseconds(1500);
        myservof.writeMicroseconds(1500);
        myservo0.writeMicroseconds(1500);
        myservo1.writeMicroseconds(1500);
        auv_state = 1;
        break;
      }
      //start controller
      else if (mode == 's'){
        Serial.println("its started");

        //get orgin lat and lon
        orgin_lat = gps.lat;
        orgin_lon = gps.lon;

        //shifting distanation data
        dis_lat_temp = dis_lat[dis_counter] - orgin_lat;
        dis_lon_temp = dis_lon[dis_counter] - orgin_lon;

        //get data of heading and array of fillter
        while (1){
          if (gps.ready()){
            array_lat[counter] = gps.lat;
            array_lon[counter] = gps.lon;
            counter++;
          }
          if (counter == 5){
            //set forward to get heading;
            motor[0] = 1550;
            motor[1] = 1550;
            myservo0.writeMicroseconds(motor[0]);
            myservo1.writeMicroseconds(motor[1]);
            delay(2000);
            pre_heading = gps.heading / 100000.0;
            mode = 'j';
            check = 2;
            break;
          }
        }
      }
      //reset of micro controller
      else if(mode == 'R'){
        Serial.println("reset");
        mode = 'j';
        pinMode(32 , OUTPUT);
        digitalWrite(32 , HIGH);
        delay(100);
        digitalWrite(32 ,LOW);
        delay(100);
      }
      //in loop can set needed
      else if(mode == 'l'){
        Serial.println("its loop");
        check = 0;
        motor[0] = 1500;
        motor[1] = 1500;
        myservo0.writeMicroseconds(motor[0]);
        myservo1.writeMicroseconds(motor[1]);
        while(1){
          if (auv_state_loop){
            Serial.println("t => k_taw");
            Serial.println("f => k_f");
            Serial.println("u => max_duty");
            Serial.println("l => min_duty");
            Serial.println("n => number of distanation");
            Serial.println("d => distance lat an lon");
            Serial.println("b => break");
            auv_state_loop = 0;
          }
          if (Serial.available()>0){
            mode = Serial.read();
            Serial.read();
            /*k_taw*/
            if (mode == 't'){
              Serial.println("enter the k taw");
              while (1){
                if (Serial.available()>0){
                  k_taw = Serial.parseFloat();
                  Serial.read();
                  Serial.print("k_taw is : ");
                  Serial.println(k_taw);
                  mode = 'j';
                  break;
                }
              }
            }
            /*k_f*/
            else if (mode == 'f'){
              Serial.println("enter the k forward");
               while (1){
                if (Serial.available()>0){
                  k_f = Serial.parseFloat();
                  Serial.read();
                  Serial.print("k_f is : ");
                  Serial.println(k_f);
                  mode = 'j';
                  break;
                }
              }
            }
            /*max_duty*/
            else if (mode == 'u'){
              Serial.println("enter the max duty");
              while (1){
                if (Serial.available()>0){
                  max_duty_auv= Serial.parseFloat();
                  Serial.read();
                  Serial.print("max_duty is : ");
                  Serial.println(max_duty_auv);
                  mode = 'j';
                  break;
                }
              }
            }
            /*min_duty*/
            else if (mode == 'l'){
              Serial.println("enter the min duty");
              while (1){
                if (Serial.available()>0){
                  min_duty_auv = Serial.parseFloat();
                  Serial.read();
                  Serial.print("min_duty is : ");
                  Serial.println(min_duty_auv);
                  mode = 'j';
                  break;
                }
              }
            }
            /*number of dis*/
            else if (mode == 'n'){
              Serial.println("enter the number of distance");
              while (1){
                if (Serial.available()>0){
                  dis_number = Serial.parseInt();
                  Serial.read();
                  Serial.print("number of distance is : ");
                  Serial.println(dis_number);
                  mode = 'j';
                  break;
                }
              }
            }
            /*lat and lon of distantion*/
            else if (mode == 'd'){
              bool lat_not_lon = true;
              int temp = 0;
              Serial.println("enter the lat and lon");
              while (1){
                if (lat_not_lon){
                  Serial.println("enter the lat : ");
                  while(1){
                    if (Serial.available()>0){
                      dis_lat[temp] = Serial.parseFloat();
                      Serial.read();
                      lat_not_lon = false;
                      break;
                    }
                  }
                }
                else {
                  Serial.println("enter the lon : ");
                  while (1){
                    if (Serial.available()>0){
                      dis_lon[temp] = Serial.parseFloat();
                      Serial.read();
                      lat_not_lon = true;
                      temp ++; 
                      break;
                    }
                  }
                  
                }
                if (temp == dis_number){
                  for (int i=0; i<dis_number; i++){
                    Serial.print(dis_lat[i]);
                    Serial.print(", ");
                    Serial.println(dis_lon[i]);
                  }
                  break;
                }
              }
            }
            /*break*/
            else if (mode == 'b'){
              auv_state_loop = 1;
              auv_state = 1;
              mode = 'j';
              break;
            }
          }
        }
      }
      //start main part of code to find dis and angle and controlling motors
      if (check == 2){
        if (gps.ready())
        {
          avrage_lat = 0;
          avrage_lon = 0;

          //shifting moving avrage fillter with new data
          for (int i=0 ; i<counter-1 ; i++){
            array_lat[i] = array_lat[i+1];
            avrage_lat += array_lat[i];
            array_lon[i] = array_lon[i+1];
            avrage_lon += array_lon[i];
          }
          //get data from gps
          cur_lat = gps.lat;
          cur_lon = gps.lon;
          //put data in fillter array
          array_lat[counter-1] = cur_lat;
          array_lon[counter-1] = cur_lon;
          avrage_lat += cur_lat;
          avrage_lon += cur_lon;

          //calculat of avrage data
          avrage_lat /= counter;
          avrage_lon /= counter;
          
          //get current heading
          heading = gps.heading / 100000.0;
          //calculate avrage heading
          avrage_heading = (heading+pre_heading)/2;
          //shifting heading 
          pre_heading = heading;
          
          //convert heading bettwen -180,0,180
          if (avrage_heading > 180){
            avrage_heading = avrage_heading - 360;
          }

          //make avrage data smaller 
          avrage_lat = avrage_lat - orgin_lat;
          avrage_lon = avrage_lon - orgin_lon;
          
          //calculat diffrance data
          diff_lat = dis_lat_temp - avrage_lat;
          diff_lon = dis_lon_temp - avrage_lon;

          //calculate angle of current point and distantion point
          angle = atan2(diff_lon,diff_lat)*180/PI;

          //calculate error angle
          d_angle = angle - avrage_heading;

          //check data be in range
          if ( d_angle<-180){
            d_angle = 360.0 + d_angle;
          }
          else if( d_angle>180){
            d_angle = d_angle - 360.0;
          }

          //calculate distance
          distance = haversineDistance(avrage_lat/10000000.0, avrage_lon/10000000.0, dis_lat_temp/10000000.0, dis_lon_temp/10000000.0);

          //print data
          Serial.print("d_angle : ");
          Serial.print(d_angle, 5);
          Serial.print(" , ");
          Serial.print("disatance : ");
          Serial.print(distance);
          Serial.print(",");
          
          //claulate rotation and forward value
          taw = k_taw * abs(d_angle);
          F = k_f * distance;

          // start controlling of robot
          /*if distance be bettwen 1.5 and 10 controller will control robot with F and 
          taw but if be farrer then 10 just taw control robot and if be nearer then 1.5 
          it's distanation of robot*/
          if (distance>1.5 && distance<=10){
            motor[0] = 1500;
            motor[1] = 1500;
            motor[0] = motor[0] + F/2;
            motor[1] = motor[1] + F/2;
            if (motor[0] > max_duty_auv){
              motor[0] = max_duty_auv;
            }
            else if(motor[0]<min_duty_auv){
              motor[0] = min_duty_auv;
            }
            if (motor[1] > max_duty_auv){
              motor[1] = max_duty_auv;
            }
            else if(motor[1]<min_duty_auv){
              motor[1] = min_duty_auv;
            }
            if (margin_angle>d_angle && d_angle>-1*(margin_angle)){
              motor[0] = motor[0];
              motor[1] = motor[1];
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              Serial.print("motor0 : ");
              Serial.print(motor[0]);
              Serial.print(",");
              Serial.print("motor1 : ");
              Serial.println(motor[1]);
            }
            else if (margin_angle<d_angle && d_angle<180){
              motor[0] = motor[0] + taw/2;
              motor[1] = motor[1] - taw/2;
              if (motor[0] > max_duty_auv){
                motor[0] = max_duty_auv;
              }
              else if(motor[0]<min_duty_auv){
                motor[0] = min_duty_auv;
              }
              if (motor[1] > max_duty_auv){
                motor[1] = max_duty_auv;
              }
              else if(motor[1]<min_duty_auv){
                motor[1] = min_duty_auv;
              }
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              Serial.print("motor0 : ");
              Serial.print(motor[0]);
              Serial.print(",");
              Serial.print("motor1 : ");
              Serial.println(motor[1]);
            }
            else if (-180<d_angle && d_angle<-1*(margin_angle)){
              motor[0] = motor[0] - taw/2;
              motor[1] = motor[1] + taw/2;
              if (motor[0] > max_duty_auv){
                motor[0] = max_duty_auv;
              }
              else if(motor[0]<min_duty_auv){
                motor[0] = min_duty_auv;
              }
              if (motor[1] > max_duty_auv){
                motor[1] = max_duty_auv;
              }
              else if(motor[1]<min_duty_auv){
                motor[1] = min_duty_auv;
              }
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              Serial.print("motor0 : ");
              Serial.print(motor[0]);
              Serial.print(",");
              Serial.print("motor1 : ");
              Serial.println(motor[1]);
            }
          }
          else if (distance>10){
            motor[0] = max_duty_auv;
            motor[1] = max_duty_auv;
            if (-1*(margin_angle)<taw && taw<margin_angle){
              motor[0] = motor[0];
              motor[1] = motor[1];
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              Serial.print("motor0 : ");
              Serial.print(motor[0]);
              Serial.print(",");
              Serial.print("motor1 : ");
              Serial.println(motor[1]);
            }
            else if (margin_angle<d_angle && d_angle<180){
              motor[0] = motor[0] + taw/2;
              motor[1] = motor[1] - taw/2;
              if (motor[0] > max_duty_auv){
                motor[0] = max_duty_auv;
              }
              else if(motor[0]<min_duty_auv){
                motor[0] = min_duty_auv;
              }
              if (motor[1] > max_duty_auv){
                motor[1] = max_duty_auv;
              }
              else if(motor[1]<min_duty_auv){
                motor[1] = min_duty_auv;
              }
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              Serial.print("motor0 : ");
              Serial.print(motor[0]);
              Serial.print(",");
              Serial.print("motor1 : ");
              Serial.println(motor[1]);
            }
            else if (-180<d_angle && d_angle<-1*(margin_angle)){
              motor[0] = motor[0] - taw/2;
              motor[1] = motor[1] + taw/2;
              if (motor[0] > max_duty_auv){
                motor[0] = max_duty_auv;
              }
              else if(motor[0]<min_duty_auv){
                motor[0] = min_duty_auv;
              }
              if (motor[1] > max_duty_auv){
                motor[1] = max_duty_auv;
              }
              else if(motor[1]<min_duty_auv){
                motor[1] = min_duty_auv;
              }
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              Serial.print("motor0 : ");
              Serial.print(motor[0]);
              Serial.print(",");
              Serial.print("motor1 : ");
              Serial.println(motor[1]);
            }
          }
          else{
            Serial.println();
            Serial.println("you are in distanation");
            dis_counter++;
            if (dis_counter == dis_number){
              motor[0] = 1500;
              motor[1] = 1500;
              myservo0.writeMicroseconds(motor[0]);
              myservo1.writeMicroseconds(motor[1]);
              mode = 'b';
              counter = 0;
              dis_counter = 0;
            }
            else{
              dis_lat_temp = dis_lat[dis_counter] - orgin_lat;
              dis_lon_temp = dis_lon[dis_counter] - orgin_lon;
            }
          }
        }  
      }
    }
  }
  //reset of microcontroller
  else if(mode == 'R'){
    Serial.println("reset");
    mode = 'j';
    pinMode(32 , OUTPUT);
    digitalWrite(32 , HIGH);
    delay(100);
    digitalWrite(32 ,LOW);
    delay(100);
  }
}

//haversin formula
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371.0;
  
  double dLat = (lat2 - lat1)*PI/180;
  double dLon = (lon2 - lon1)*PI/180;
  
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos((lat1)*PI/180) * cos((lat2)*PI/180) *
             sin(dLon / 2) * sin(dLon / 2);
  
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = R * c *1000;
  
  return distance;
}