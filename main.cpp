#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;
vector <Mat> serie_img;
Mat img, gray,undistorted;
Size im_size;
int avanti=0;
void setup_calibration(int board_width, int board_height, int num_imgs, 
                       float square_size, char* imgs_filename,
                       char* extension) {
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int k = 1; k <= num_imgs; k++) {
    char img_file[300];
	

    sprintf(img_file, "%s (%d)%s", imgs_filename, k, extension);
    img = imread(img_file, CV_LOAD_IMAGE_COLOR) ;
	serie_img.push_back(img);
	 if(! img.data )                              // Check for invalid input
    {
        cout <<  "immagini? bohhh" << std::endl ;
		waitKey();
        exit(1);
    }
	 else{
		 
	 }
	
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, board_size, corners,CALIB_CB_ADAPTIVE_THRESH| CALIB_CB_FILTER_QUADS);

	if(found==false){
		cout<<"non riesco a trovare una griglia "<<board_width<<" x "<<board_height<<endl;
	}
   else if (found)
    {
      cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(img, board_size, corners, found);
	    
		
	
    }
    
    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      cout<<"immagine: "<<k << endl;
      image_points.push_back(corners);
      object_points.push_back(obj);
	       namedWindow("Display window2",WINDOW_AUTOSIZE );
	      imshow( "Display window2", img );
		   sprintf(img_file, "griglia (%d)%s",k, extension);
		  imwrite( img_file, img );
		  waitKey();
    }
  }
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char const **argv)
{
  int punti_orizz=9;
  int punti_vert=6;
  int num_imgs;
  float lato_quadrato=20.00;
 
  char nome_imgs[10];
  char file_salvataggio[30];
  char estensione[10];
  float errore_riproi;
  cout<<"inserisci numero corner orizzontali";
  cin>>punti_orizz;
  cout<<"inserisci numero corner verticali";
  cin>>punti_vert;
  cout<<"inserisci nome serie immagini:"<<endl;
  cin>>&nome_imgs[0];
  cout<<"inserisci estensione immagini:"<<endl;
  cin>>&estensione[0];
  cout<<"inserisci numero immagini"<<endl;
  cin>>num_imgs;
  cout<<"inserisci dimensione lato [mm]"<<endl;
  cin>>lato_quadrato;
  cout<<"inserisci file slvataggio"<<endl;
  cin>>&file_salvataggio[0];
 

  int c;
 

  setup_calibration(punti_orizz, punti_vert, num_imgs, lato_quadrato, nome_imgs, estensione);
  cout<<"setup_finito, inizio calibrazione"<<endl;


  Mat K;
  Mat D;
  vector< Mat > vR, vT;
  int flag = 0;
  flag |= CALIB_FIX_K4;
  flag |= CALIB_FIX_K5;

  calibrateCamera(object_points, image_points, img.size(), K, D, vR, vT, flag);

   errore_riproi=computeReprojectionErrors(object_points, image_points, vR, vT, K, D);
   cout << "Errore Riproiezione:" <<errore_riproi<<endl;
   cout<<"matrice di calibrazione\n:"<<K<<endl;

  FileStorage fs(file_salvataggio, FileStorage::WRITE);
  fs << "Matrice_K_di_calibrazione" << K;
  fs << "Matrice_D_coefficienti_di_distorsione" << D;
  fs <<  "errore_riproiezione"<< errore_riproi;
  fs << "numero_punti_orizzontali_griglia" << punti_orizz;
  fs << "numero_punti_verticali_griglia" << punti_vert;
  fs << "lato_quadrato_mm" << lato_quadrato;
  fs << "vettore_traslazione" << vT;
  fs << "vettore_rotazione" << vR;
  
  cout<<"desideri vedere la undistorsione??[1/si][2/no]"<<endl;
  
  int ans;

  cin>>ans;
  if(ans==1)
  for(int i=0;i<num_imgs;i++)
  {
          
  char img_file[200];
	  undistort(serie_img[i], undistorted, K, D);
          sprintf(img_file, "undistorted (%d).png",i);
		  imwrite( img_file, undistorted );
     /*   namedWindow("Display original",WINDOW_AUTOSIZE );
        imshow( "Display original", serie_img[i] );
  
  	   namedWindow("Display undistort",WINDOW_AUTOSIZE );
  	   imshow( "Display undistort", undistorted );
  
  */
  	    waitKey();
  
  }




  waitKey();
  return 0;
}