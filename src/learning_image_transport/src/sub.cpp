#include <ros/ros.h>
#include "serial.h"
#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <math.h>
#include <vector>
#include "geometry_msgs/Twist.h"
#include <ros/console.h>  
#include <sstream>
#include <algorithm>
#include <fstream>
using namespace std;

#define FRAME_HEADER       0xAA
#define FRAME_GROUP_NUMBER 0x00
#define FRAME_ROBOT_NUMBER 0x0000

#define CMD_SET_WHEELS          0x01
#define CMD_SET_LED             0x02
#define CMD_SET_WHEELS_AND_ODOM 0x03

#define MODE_MOTORS_OFF    0x00
#define MODE_MOTORS_ON     0x03
#define MODE_SET_ODOMETRY  0x04

float x_pos, y_pos;
ofstream fs;
double theta = 0, Etheta = 0, Ex = 0, Ey = 0, theta_d = 0, xd = 0, yd = 0, coeff = 0, w1 = 0, w2 = 0, u1 = 0, u2 = 0, ka = 0, kp = 0, nu = 0, e_d = 0, sigma = 0, e = 0, coeff1 = 0, pi = 0, r = 0, b = 0, mu = 0, theta_Next = 0, theta_Disc = 0,
       v1 = 0, v2 = 0, hx = 0, hy = 0, d_t = 0, d_theta_a = 0, theta_a = 0, e_a = 0, eq_bar =0, hx_d = 0, hy_d = 0, theta_ad = 0, w_r = 0, w_l = 0, theta__aNext = 0, theta_aDisc = 0, data;

struct Frame {
  Frame() : header(FRAME_HEADER), num_data_bytes(22), group_number(FRAME_GROUP_NUMBER), robot_number(FRAME_ROBOT_NUMBER) {}

  unsigned char header;
  unsigned char num_data_bytes;
  unsigned char group_number;
  unsigned char command;
  unsigned short int robot_number;
  unsigned short int mode;
  short int w_l;    // Left wheel angular velocity
  short int w_r;
  float x;        // X coordinate
  float y;
  float theta;    // Orientation
  uint16_t crc;
};
Frame rx_frame, tx_frame;

unsigned short int CRC16(const unsigned char *data, int len)
  {
    static const unsigned short int crc_table[] = {
      0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,
      0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
      0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,
      0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
      0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,
      0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
      0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,
      0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
      0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,
      0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
      0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,
      0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
      0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,
      0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
      0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,
      0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
      0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,
      0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
      0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,
      0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
      0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,
      0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
      0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,
      0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
      0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,
      0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
      0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,
      0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
      0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,
      0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
      0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,
      0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
    };

    unsigned short int crc_word = 0xFFFF;

    while (len--)
      crc_word = (crc_word << 8) ^ crc_table[(((crc_word >> 8) & 0x00FF) ^ *data++) & 0x00FF];

    return crc_word;
  }

void prepareFrame(unsigned short int mode, unsigned char command)
  {
    tx_frame.command = command;
    tx_frame.mode = mode;

    // Omit header byte and include num_data_bytes byte for crc
    // The crc has different endianess
    unsigned short int crc = CRC16(((uint8_t*)&tx_frame) + 1, (int)(tx_frame.num_data_bytes + 1));
    unsigned char crc1 = crc >> 8;
    unsigned char crc2 = 0xFF & crc;
    tx_frame.crc = crc1 + crc2 * 256;
   }


void setVelocity(float w_l, float w_r)
  {
    tx_frame.w_l =  (short int) (w_l * 256.0f);
    tx_frame.w_r =  (short int) (w_r * 256.0f);    

    prepareFrame(MODE_MOTORS_ON, CMD_SET_WHEELS_AND_ODOM);
  //  writeFrame();
  }



bool myfunction (int i,int j) { return (i<j); }


struct myclass {
  bool operator() (int i,int j) { return (i<j);}
} myobject;

cv::Mat imageCopy;
cv::Mat cameraMatrix, distCoeffs;
vector< cv::Vec3d> rvecs, tvecs;
cv::Ptr<cv::aruco::DetectorParameters> parameters;
cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
vector<int> ids;
vector<vector<cv::Point2f> > corners;
cv::Mat rotationMatrix(3,3,cv::DataType<float>::type);

//Serial robot;


int fd, error;
 short int vl, vr;
unsigned char send_buffer[12];
//double ROBOT_BASE(0.145), WHEEL_RADIUS(0.025);

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Processed";
image_transport::Publisher pub;
image_transport::Subscriber sub;
ros::Publisher velocities;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
cv_bridge::CvImageConstPtr cv_ptr;	
  try
  {
    //cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

//  //<<<MARKER DETECTION>>>

 cv_ptr->image.copyTo(imageCopy);

     cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0)
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

//     //<<<POSE ESTIMATION>>>

    //rvecs.reserve(20);
    //tvecs.reserve(20);
    //int iii=-1;
   if (ids.size() > 0)
    {
   // 	for (int it=0; it<ids.size(); it++)
   //      {
   //         if (ids[it]==446) iii=it;
   //      }

   //     if (iii>=0)
   //     {
    cv::aruco::estimatePoseSingleMarkers(corners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);
		
		//cout << "tvecs0:" << tvecs[0];
		//ROS_INFO_STREAM("rvecs");
		//ROS_INFO_STREAM("tvecs");
		cv::Rodrigues(rvecs[0], rotationMatrix);
		//cout << "rotationMatrix" << rotationMatrix;
		//float roll = atan2(rotationMatrix.at<float>(2,1), rotationMatrix.at<float>(2,2));
        //float theta = atan2(-rotationMatrix.at<double>(2,0), sqrt(pow(rotationMatrix.at<double>(2,1),2.0) + pow(rotationMatrix.at<double>(2,2),2.0)));
        //float yaw = atan2(rotationMatrix.at<float>(1,0), rotationMatrix.at<float>(0,0))s;
           // cout << "theta:" << theta << endl;

		//theta continuity
    theta_Next = atan2(-rotationMatrix.at<double>(2,0), sqrt(pow(rotationMatrix.at<double>(2,1),2.0) + pow(rotationMatrix.at<double>(2,2),2.0))) - pi/2;
    cout << "theta_Next is :" << theta_Next<< endl;
    theta_Disc = atan2(sin(theta),cos(theta));
        // cout << "t_i is :" << t_i << endl;
    d_t = theta_Next - theta_Disc;
    cout << "d_t is :" << d_t << endl;
    if (d_t > pi)
            d_theta_a = d_t - (2*pi);
        else if (d_t < -pi)
            d_theta_a = d_t + (2*pi);
        else
            d_theta_a = d_t;
        cout << "d_theta_a is :" << d_theta_a << endl;
    theta = theta + d_theta_a;
    cout << "theta is :" << theta<< endl;

        float x = tvecs[0][0];
        float y = tvecs[0][1];
        float z = pow(tvecs[0][2], 2.0);
        float z1 = tvecs[0][2];
        x_pos = cos(theta_Next)*x;
        //float x_pos = sin(90-theta)*z1;
            cout << "X position is :" << x_pos << endl;
        y_pos = sqrt(z - pow(x_pos, 2.0));
            cout << "Y position is :" << y_pos << endl;

    //while (theta_d < -pi/2 || theta_d > pi/2);
     //while (xd  < -1 || xd > 1);
     theta_d = (-pi*95)/180, xd = 0, yd = 0.7;
    Etheta = theta_d - theta;
    Ex = xd - x_pos;
    Ey = yd - y_pos;

    r = 0.026;
    b = 0.033;
    ka = 2;
    kp = 1;
    nu = 0.8;
    pi = 3.142;
    e_d = ((Ex*cos(theta_d)) + (Ey*sin(theta_d)));
    cout << "e_d is :" << e_d << endl;
    if (e_d > 1)
        sigma = 1;
        //cout << "sigmapos is :" << sigma << endl;

    else if (e_d < 1)
        sigma = 1;
        //cout << "sigmaneg is :" << sigma << endl;
    else sigma = 1;
   cout << "sigma is :" << sigma << endl;
    e = sqrt(Ex*Ex + Ey*Ey);
    coeff1 = -e*nu*sigma;
    v1 = coeff1*cos(theta_d);
       cout << "v1 is :" << v1 << endl;
    v2 = coeff1*sin(theta_d);
       cout << "v2 is :" << v2 << endl;
    hx = Ex*kp + coeff1*cos(theta_d);
    hy = Ey*kp + coeff1*sin(theta_d);
    u2 = ((hx*cos(theta)) + (hy*sin(theta)));
       cout << "u2 is :" << u2 << endl;

    //theta_a continuity
    theta__aNext = atan2(sigma*hy, sigma*hx);
    theta_aDisc = atan2(sin(theta_a),cos(theta_a));
        // cout << "t_i is :" << t_i << endl;
    d_t = theta__aNext - theta_aDisc;
    cout << "d_t is :" << d_t << endl;
    if (d_t > pi)
            d_theta_a = d_t - (2*pi);
        else if (d_t < -pi)
            d_theta_a = d_t + (2*pi);
        else
            d_theta_a = d_t;
        cout << "d_theta_a is :" << d_theta_a << endl;
    theta_a = theta_a + d_theta_a;
    cout << "theta_a is :" << theta_a << endl;

    //e_a 
    e_a = theta_a - theta_Next;
     //e_a = 1.02;
       cout << "e_a is :" << e_a << endl;
    eq_bar = Ex*u2*cos(theta) + Ey*u2*sin(theta);
       cout << "eq_bar is :" << eq_bar << endl;
    hx_d = (-kp*u2*cos(theta)) + nu*sigma*eq_bar/(sqrt(Ex*Ex + Ey*Ey))*cos(theta_d);
       cout << "hx_d is :" << hx_d << endl;
    hy_d = (-kp*u2*sin(theta)) + nu*sigma*eq_bar/(sqrt(Ex*Ex + Ey*Ey))*sin(theta_d);
       cout << "hy_d is :" << hy_d << endl;
    theta_ad = (hy_d*hx) - (hy*hx_d)/(hx*hx + hy*hy);
       cout << "theta_ad is :" << theta_ad << endl;
    u1 =((ka*e_a) + theta_ad);
       cout << "u1 is :" << u1 << endl;
    w1 = (u2 + b*u1)/r;
       cout << "w1 is :" << w1 << endl;
    w2 = (u2 - b*u1)/r;
       cout << "w2 is :" << w2 << endl;
       // VSB = fmax(w1/0.1,w2/0.1);
    //       cout << "VSB is :" << VSB << endl;
    // int W[] = {w1, w2};
      // mu = 1/VSB*W[1];
    //       cout << "mu is :" << mu << endl;

    w1 = (u2 + b*u1)/r;
    w2 = (u2 - b*u1)/r;
       // w1 = (u2 + u1) * WHEEL_RADIUS / 2.0;
       // w2 = (u2 - u1) * WHEEL_RADIUS / ROBOT_BASE;
   double myints[] = {1,abs(w1/2), abs(w2/2)};
    vector<double> myvector (myints, myints+3);
    sort (myvector.begin(), myvector.end());
    for (vector<double>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
        cout << ' ' << *it;
      cout << '\n';

      mu = 1/myvector[2];
      cout << "mu" << mu << endl;
      w_l = w1*mu;
          cout << "wl" << w_l << endl;
      w_r = w2*mu;
          cout << "wr" << w_r << endl;

          setVelocity(w_l, w_r);
float data[11] = {theta_Next, x_pos, y_pos, w_l, w_r, Etheta, Ex, Ey, theta_d, xd, yd};
   data[0] = theta_Next;
   data[1] = x_pos;
   data[2] = y_pos;
   data[3] = w_l;
   data[4] = w_r;
   data[5] = Etheta;
   data[6] = Ex;
   data[7] = Ey;
   data[8] = theta_d;
   data[9] = xd;
   data[10] = yd;
 std_msgs::Header h = msg->header;

    //fs.open("/home/manu/image_transport/src/learning_image_transport/src/plot.txt");
    fs << data[0] << ',' ;
	fs << data[1] << ',' ;
	fs << data[2] << ',' ;
	fs << data[3] << ',' ;
	fs << data[4] << ',' ;
	fs << data[5] << ',' ;
	fs << data[6] << ',' ;
	fs << data[7] << ',' ;
	fs << data[8] << ',' ;
	fs << data[9] << ',' ;
	fs << data[10] << ',' ;
	fs << h.stamp.sec << endl;
    // fs << "x_pos" << x_pos << endl;
    // fs << "y_pos" << y_pos << endl;
    //fs.close();

    write (fd, (unsigned char*)&tx_frame, sizeof(Frame));
 //}
    //velocities.publish(new_vel);
	}
    for (unsigned int i=0; i<ids.size(); i++)
     cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
     cv::imshow(WINDOW, imageCopy);
     //cv::waitKey(30);
     pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char **argv)
{
cv::FileStorage fs2("/home/manu/image_transport/src/learning_image_transport/src/cam.xml", cv::FileStorage::READ);

  //use Filenode::operator >>
  fs2["camera_matrix"] >> cameraMatrix;
  fs2["distortion_coefficients"] >> distCoeffs;
  fs2.release();

    char *portname = "/dev/ttyUSB0";
    /* fp = fopen (portname, "w");
    if (!fp){
        perror("File opening failed");
             return EXIT_FAILURE;
   }
   */
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    error = set_interface_attribs (fd, B921600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    //write (fd, send_buffer, 12);
    //close (fd);
     fs.open("/home/manu/image_transport/src/learning_image_transport/src/plot12.txt");

  ros::init(argc, argv, "sub");

  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  sub = it.subscribe("/image_raw", 1, imageCallback);
  //cv::destroyWindow(WINDOW);
  //velocities = nh.advertise<geometry_msgs::Twist>("new_vel", 1000);
   //pub = it.advertise("/image_processed/output_video", 1);
  ros::spin();
  cv::destroyWindow(WINDOW);

  //cv::destroyWindow("view");
  return 0;
}

const unsigned int crc_lookup[256] = {
    0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,
    0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
    0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,
    0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
    0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,
    0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
    0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,
    0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
    0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,
    0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
    0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,
    0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
    0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,
    0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
    0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,
    0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
    0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,
    0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
    0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,
    0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
    0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,
    0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
    0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,
    0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
    0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,
    0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
    0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,
    0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
    0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,
    0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
    0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,
    0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
    };

unsigned short int Crc16Value;

void CRC16Calc( unsigned char DataByte )
{
    // Calculating the CRC16 by using a LookUp table
    Crc16Value = (unsigned short int)(Crc16Value << 8) ^ (short unsigned int)(crc_lookup[ (unsigned short int) ( ((Crc16Value >> 8) & 0xFF) ^ DataByte) ]);
}

void CRC16Init( void )
{
    // This is the initial Value of the CRC16
    Crc16Value = 0xFFFF;
}

unsigned short int CRC16GetValue( void )
{
    return Crc16Value;
}

