#include "sar/FollowClass.h"


// CONSTRUCTOR
// The method which gets called when the object is created.
// ----------------------------------------------------------------------------------------------------
//  Input | Type            | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1     | NodeHandle      | nh_     | Links the method to the specified node, making it accessible
//  2     | ImageTransport  | it_     | Links the method to the imagetransport node, letting the method access the images

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  N/A   | N/A   | N/A     | Creates the subscriber node, and sets up the publisher for velocity commands. Within the class, 
//  N/A   | N/A   | N/A     |  the GUI is set up and variables are initialized.

FollowClass::FollowClass(ros::NodeHandle nh_, image_transport::ImageTransport it_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &FollowClass::imageCb, this, image_transport::TransportHints("compressed"));
    pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    OPENCV_WINDOW1 = "Live Feed";
    OPENCV_WINDOW2 = "Thresholded Image";
    OPENCV_WINDOW3 = "Sliders";
    cv::namedWindow(OPENCV_WINDOW1, CV_WINDOW_NORMAL);
    cv::namedWindow(OPENCV_WINDOW2, CV_WINDOW_NORMAL);
    cv::namedWindow(OPENCV_WINDOW3, CV_WINDOW_NORMAL);

    iLowHa =0;
    iHighHa = 179;
    iLowSa = 0; 
    iHighSa = 255;
    iLowVa = 0;
    iHighVa = 255;

    Colour = 0;
    morphsize = 7;
    go = 0;
    Slider_Init_Area = 0;
    ObjectInitArea = 0;
    
    cv::createTrackbar("LowH", OPENCV_WINDOW3, &iLowHa, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", OPENCV_WINDOW3, &iHighHa, 179);
    cv::createTrackbar("LowS", OPENCV_WINDOW3, &iLowSa, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", OPENCV_WINDOW3, &iHighSa, 255);
    cv::createTrackbar("LowV", OPENCV_WINDOW3, &iLowVa, 255);//Value (0 - 255)
    cv::createTrackbar("HighV", OPENCV_WINDOW3, &iHighVa, 255);
    cv::createTrackbar("Area", OPENCV_WINDOW3, &Slider_Init_Area, 1);
    cv::createTrackbar("Morphsize", OPENCV_WINDOW3, &morphsize, 21);
    cv::createTrackbar("Colour", OPENCV_WINDOW3, &Colour, 2);
    cv::createTrackbar("Go", OPENCV_WINDOW3, &go, 1);
}

// DECONSTRUCTOR
// The method which gets called when the object is stopped.
// ----------------------------------------------------------------------------------------------------
//  
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  N/A   | N/A   | N/A     | N/A
//

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  N/A   | N/A   | N/A     | When called through Ctrl-C, closes windows that were created in constructor.
FollowClass::~FollowClass()
{
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
}


// METHOD
// This method computes the following mathematical operation: number^power
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name      | Description
// ----------------------------------------------------------------------------------------------------
//  1   | float   | number    | the number we want to operate on
//  2   | int     | power     | the value of the power intended for the operation
//

// Output | Type   | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1     | float  | number  | returns the value after the operation

float FollowClass::divisor(float number, int power)
{
  for(int i = 0; i < power; ++i)
    number = number*0.1;

  return number;
}

// METHOD
// This method retrieves the image transported over the ros node, processes it, and displays it.
// ----------------------------------------------------------------------------------------------------
//  Input | Type                        | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1     | sensor_msgs::ImageConstPtr& | msg     | this is the link to message sent over the node from the bot
//        |                             |         |   containing the image taken by kinect
//

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A     | N/A

void FollowClass::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr; 

  //define the Mat structures that will hold the HSV and processed images.
  cv::Mat imgHSV, threshold_Image; 

  try
  {
    //allows the image to be copied in to a variable and modified in this method
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
  }
  
  //Any errors that causes the above try to fail will be printed
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

//Trackbar "Colour" links to the Colour variable, and the FollowClassing settings link to the corresponding int values
    // 0: Allow user to slide HSV high and low trackbars manually
    // 1: Lock HSV values for yellow brick
    // 2: Lock HSV values for purple glove 
  switch(Colour) 
  {
    case 0 :
              iLowH = iLowHa ;
              iHighH = iHighHa;
              iLowS = iLowSa; 
              iHighS = iHighSa;
              iLowV = iLowVa;
              iHighV = iHighVa;
              break;

    case 1 : //Yellow Brick
              iLowH =16;
              iHighH = 35;
              iLowS = 148; 
              iHighS = 235;
              iLowV = 0;
              iHighV = 255;
              ObjectInitArea = 2267720;
              break;

    case 2 :// Purple Glove
              iLowH =120;
              iHighH = 163;
              iLowS = 21; 
              iHighS = 100;
              iLowV = 0;
              iHighV = 255;
              ObjectInitArea = 2342430;
              break;       
  }

  //converts the image's BGR encoding to HSV encoding for suitable processing
  cv::cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV); 
  //filters the image to show only the desired colour
  cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), threshold_Image);  

  // perform morphology operations, in this case opening
  cv::erode(threshold_Image, threshold_Image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  cv::dilate(threshold_Image, threshold_Image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

  // more morphology, to reduce noise in the image, can be user customisable via trackbar "Morph Size"
  cv::morphologyEx(threshold_Image, threshold_Image, 2, cv::getStructuringElement( 2, cv::Size(morphsize+1, morphsize+1)));

  //instantiates a Moments object to calculate moments from the image
  cv::Moments oMoments = cv::moments(threshold_Image);

  // define zeroth, first and second order moments
  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  dArea = oMoments.m00;

  // if the user sets trackbar "Area" to 1, the variable is set to current area, to save this, set to 0 (basicallly a button to save the area).
  if(Slider_Init_Area == 1)
  {
    ObjectInitArea = oMoments.m00;
    std::cout << "ObjectInitArea: " << ObjectInitArea<< "\n";
  }

  // checks to see if the image is above a certain threshold for noise, as noise adds area
  if (dArea > 150000)
  {
    // determine centroid of object
    posX = dM10 / dArea;
    posY = dM01 / dArea;

    //when user sets trackbar "Go" to 1, the movement method is called
    if(go == 1)
      movement();

    //Store centre of image for reference to movement of centroid of object
    iLastX = 331;
  }
  //overlay circle over "Live Feed" with the coordinates of the object centroid
  cv::circle(cv_ptr->image, cv::Point(posX, posY), 10, CV_RGB(255,0,0)); 
  
  //display processed image
  cv::imshow(OPENCV_WINDOW2, threshold_Image); 

  //display live feed with circle overlay
  cv::imshow(OPENCV_WINDOW1, cv_ptr->image); 
      
  cv::waitKey(1);
}

// METHOD
// This method computes the velocities that are published to the turtlebot
// ----------------------------------------------------------------------------------------------------
//  Input | Type   | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1     | N/A    | N/A     | N/A


// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A     | N/A

void FollowClass::movement()
{
  //initialize the velocity variables
  cmd_stored.linear.x = cmd_stored.linear.y = cmd_stored.linear.z = cmd_stored.angular.x = cmd_stored.angular.y = cmd_stored.angular.z = 0;

  //calculate the position of the object centroid in relation to the image centroid
  int vector = posX - iLastX; 
  
  const float constant_right = 0.0020089377;
  const float intercept_right = 0.1951411556;
  const float constant_left = -0.001715662;
  const float intercept_left = 0.2168756237;

  //calculates angular speed based on position of object centroid (equation derived in excel file)
  float angular_right = constant_right*vector + intercept_right; 
  float angular_left = constant_left*vector + intercept_left;

  //calculates linear speed based on area of object (equation derived in excel file)
  float linear_backward = 0.85*(divisor(-1.6331969,15)*dArea*dArea + divisor(5.085534,8)*dArea + 0.0975220864);
  float linear_forward = 0.5*(0.070586926*log (dArea) - 0.5313718272);

  //when vector negative turn bot left
  if(vector<-50)
    cmd_stored.angular.z = 1.3*angular_left; //TURN LEFT

  //when vector positive turn bot right
  else if(vector>50)
    cmd_stored.angular.z = -1.3*angular_right; //TURN RIGHT

  //when area is less than original area, move forward
  if(0.85*ObjectInitArea>dArea && dArea>0.2*ObjectInitArea)
    cmd_stored.linear.x = linear_forward;

  //when area is more than original area, move backward
  else if(dArea > 1.2*ObjectInitArea)
    cmd_stored.linear.x = -linear_backward;


  else
    cmd_stored.linear.x = 0;

    //publish velocity commands
    pub_vel.publish(cmd_stored);
}

// MAIN METHOD
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1   | int   | argc      | N/A
//  2   | char  | argv      | N/A

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | void  | N/A     | N/A

int main(int argc, char** argv)
{
  //initialize ros
  ros::init(argc, argv, "image_converter");

  //define ros nodehandle
  ros::NodeHandle nh_;

  //define image transporter over the ros node handle
  image_transport::ImageTransport it_(nh_);

  //as long as a roscore or minimal launch is running, continue to loop
  while(ros::ok)
  {
    //initialize class, calls constructor, using image transporter and nodehandle
    FollowClass follow(nh_, it_);
    //loop unless Ctrl-C is used in terminal
    ros::spin();
    break;
  }
  return 0;
}
