#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstring> 
#include <limits>
#include <cstdlib>

#define COLOR_CHANNEL 3
#define RGB_FULL 255

// Maximum step when for normal calculation
static const int limit = 3;
// x, y, z, normal_x, normal_y, normal_z block size
static const int block = 6;

static const float EPS = 0.001;

struct point
{
  float WorldX;
  float WorldY;
  float WorldZ;

  float Norm_X;
  float Norm_Y;
  float Norm_Z;

public:
  point() : WorldX(0), WorldY(0), WorldZ(0),
	    Norm_X(0), Norm_Y(0), Norm_Z(0)
  {
  }
};

void GetNormal(int x, int y, point* PointCloud, float* depth, 
	       int ImWidth, int ImHeight, int limit,
	       float default_normal[3], float MinDepth, float MaxDepth);

void ReadConfig(float K[3][3], float R[3][3], float T[3], 
		std::string RefImName, std::string ConfigName);

float det3x3(float a1, float a2, float a3, float a4, float a5,
	     float a6, float a7, float a8, float a9);

void inv_3x3(float a[3][3], float inv[3][3]);

void MatMul3_3x3_3(float a[3][3], float b[3][3], float c[3][3]);

void MatMul3_3x3_1(float a[3][3], float b[3], float c[3]);

void translate(point& a_point, float a[3][3], float b[3], int ImgX, int ImgY, float depth);

void cross_product(float a[3], float b[3], float c[3]);


int main(int argc, char** argv)
{
  int ArgCount = 1;
  
  std::string ConfigName   = argv[ArgCount++];

  std::string DepthMapName = argv[ArgCount++];
  std::string RefImName    = argv[ArgCount++];

  int ImWidth  = atoi(argv[ArgCount++]);
  int ImHeight = atoi(argv[ArgCount++]);

  float MinDepth = atof(argv[ArgCount++]);
  float MaxDepth = atof(argv[ArgCount++]);

  std::string OutputName = argv[ArgCount++];

  // const float EPS = std::numeric_limits<float>::epsilon();

  // Read camera matrices
  float K[3][3], R[3][3], T[3];
  ReadConfig(K, R, T, RefImName, ConfigName);

  // Read depth map
  std::ifstream ifm;

  ifm.open(DepthMapName.c_str(), std::ios::binary | std::ios::in);

  if(!ifm.good())
    {
      std::cerr << "Error openning " << DepthMapName << std::endl;
      return 1;
    }

  // Allocate memory for depth map
  float* depth = new float[ImWidth*ImHeight];

  ifm.read((char*)depth, ImWidth*ImHeight*sizeof(float));

  ifm.close();
  
  // Calculate inverse matrix of K and R
  float InvR[3][3], InvK[3][3];
  inv_3x3(R, InvR);
  inv_3x3(K, InvK);

  // InvR*InvK
  float InvR_InvK[3][3];
  MatMul3_3x3_3(InvR, InvK, InvR_InvK);

  // InvR*T
  float InvR_T[3];
  MatMul3_3x3_1(InvR, T, InvR_T);

  // M = K*R
  float M[3][3];
  MatMul3_3x3_3(K, R, M);

  // Principal axis is det(M)*M_3, in which M_3 
  // is the third row of M, so default normal of
  // point cloud will be set to -det(M)*M3
  float default_normal[3];
  float DetM = det3x3(M[0][0], M[0][1], M[0][2], 
		      M[1][0], M[1][1], M[1][2],
		      M[2][0], M[2][1], M[2][2]);

  default_normal[0] = -DetM*M[2][0];
  default_normal[1] = -DetM*M[2][1];
  default_normal[2] = -DetM*M[2][2];

  // Allocate memory for point cloud
  point* PointCloud = new point[ImWidth*ImHeight];
  
  // Translate coordinates of all points from 2D to 3D world
  int pt_counter = 0;
  for(int i=0; i<ImHeight; i++)
    {
      for(int j=0; j<ImWidth; j++)
	{
	  translate(PointCloud[pt_counter], InvR_InvK, InvR_T, 
		    j, i, depth[pt_counter]);
	  pt_counter++;
	}
    }

  // Calculate normal for all points
  pt_counter = 0;
  for(int i=0; i<ImHeight; i++)
    {
      for(int j=0; j<ImWidth; j++)
	{	  
	  if( (depth[pt_counter] > (MinDepth+EPS)) && 
	      (depth[pt_counter] < (MaxDepth-EPS)) )
	    {
	      GetNormal(j, i, PointCloud, depth, ImWidth, ImHeight, 
			limit, default_normal, MinDepth, MaxDepth);

	      pt_counter++;
	    }
	  else
	    {
	      pt_counter++;
	      continue;
	    }
	}
    }

  // Get rid of points with invalid depth, namely, points whose depth
  // is either MinDepth or MaxDepth
  float* xyz_norm = new float[ImWidth*ImHeight*block];

  pt_counter = 0;
  int counter = 0;
  for(int i=0; i<ImHeight; i++)
    {
      for(int j=0; j<ImWidth; j++)
	{
	  if( (depth[pt_counter] > (MinDepth+EPS)) && 
	      (depth[pt_counter] < (MaxDepth-EPS)) )
	    {
	      xyz_norm[counter] = PointCloud[pt_counter].WorldX;
	      counter++;
	      xyz_norm[counter] = PointCloud[pt_counter].WorldY;
	      counter++;
	      xyz_norm[counter] = PointCloud[pt_counter].WorldZ;
	      counter++;
	      xyz_norm[counter] = PointCloud[pt_counter].Norm_X;
	      counter++;
	      xyz_norm[counter] = PointCloud[pt_counter].Norm_Y;
	      counter++;
	      xyz_norm[counter] = PointCloud[pt_counter].Norm_Z;
	      counter++;
	    }

	  pt_counter++;
	}
    }

  std::ofstream ofm;
  ofm.open(OutputName.c_str(), std::ios::trunc | std::ios::binary);
  ofm.write((char*)xyz_norm, ImWidth*ImHeight*block*sizeof(float));
  ofm.close();

  delete[] xyz_norm;
  delete[] depth;
  delete[] PointCloud;

  return 0;
}

// limit constraints how deep the recursion goes
void GetNormal(int x, int y, point* PointCloud, float* depth, 
	       int ImWidth, int ImHeight, int limit,
	       float default_normal[3], float MinDepth, float MaxDepth)
{
  // const float EPS = std::numeric_limits<float>::epsilon();
  const int pos_o = y*ImWidth + x;
  float a[3], b[3], c[3];
  a[0] = a[1] = a[2] = 0;
  b[0] = b[1] = b[2] = 0;
  c[0] = c[1] = c[2] = 0;

  bool LimitReached = false;
  
  int pos;
  for(int i=1; i<=limit; i++)
    {
      if(y+i<ImHeight)
	{
	  pos = (y+i)*ImWidth + x;
	  
	  if( (depth[pos] > (MinDepth+EPS)) && 
	      (depth[pos] < (MaxDepth-EPS)) )
	    {
	      b[0] = PointCloud[pos].WorldX - PointCloud[pos_o].WorldX;
	      b[1] = PointCloud[pos].WorldY - PointCloud[pos_o].WorldY;
	      b[2] = PointCloud[pos].WorldZ - PointCloud[pos_o].WorldZ;

	      break;
	    }

	  if(i == limit)
	    LimitReached = true;
	}
    }

  for(int i=1; i<=limit; i++)
    {
      if(x+i<ImWidth)
	{
	  pos = y*ImWidth + x+i;
	  
	  if( (depth[pos] > (MinDepth+EPS)) && 
	      (depth[pos] < (MaxDepth-EPS)) )
	    {
	      a[0] = PointCloud[pos].WorldX - PointCloud[pos_o].WorldX;
	      a[1] = PointCloud[pos].WorldY - PointCloud[pos_o].WorldY;
	      a[2] = PointCloud[pos].WorldZ - PointCloud[pos_o].WorldZ;

	      break;
	    }

	  if(i == limit)
	    LimitReached = true;
	}
    }

  if( LimitReached )
    {
      PointCloud[pos_o].Norm_X = default_normal[0];
      PointCloud[pos_o].Norm_Y = default_normal[1];
      PointCloud[pos_o].Norm_Z = default_normal[2];
    }
  else
    {
      cross_product(a, b, c);

      PointCloud[pos_o].Norm_X = c[0];
      PointCloud[pos_o].Norm_Y = c[1];
      PointCloud[pos_o].Norm_Z = c[2];
    }
}

void ReadConfig(float K[3][3], float R[3][3], float T[3], 
		std::string RefImName, std::string ConfigName)
{
  // Get rid of the path in front of RefImName
  size_t pos = RefImName.rfind('/');

  if(pos != std::string::npos)
    RefImName  = RefImName.substr(pos+1); 

  std::ifstream ifm;
	
  int TotalImgNum;
	
  // Open the configuration file
  ifm.open(ConfigName.c_str());
	
  // Exit if the file cannot be openned
  if(!ifm.good())
    {
      std::cerr<<"Cannot open config.ini !!!"<<std::endl;
      exit(1);
    }
	
  float temp, scale;
	
  // Read number of images
  ifm >> TotalImgNum >> scale;
	
  // Start reading camera matrices
  for(int i=0; i<TotalImgNum; i++)
    {
      // Firstly read the image name
      std::string ImageName;
      ifm >> ImageName;
		
      if(ImageName == RefImName)
	{
	  ifm >>  K[0][0] >> K[0][1] >> K[0][2]
	      >>  K[1][0] >> K[1][1] >> K[1][2]
	      >>  K[2][0] >> K[2][1] >> K[2][2];

	  K[0][0] *= scale; K[0][1] *= scale; K[0][2] *= scale;
	  K[1][0] *= scale; K[1][1] *= scale; K[1][2] *= scale;
			
			
	  ifm >>  R[0][0] >> R[0][1] >> R[0][2]
	      >>  R[1][0] >> R[1][1] >> R[1][2]
	      >>  R[2][0] >> R[2][1] >> R[2][2];
			
	  ifm >>  T[0] >> T[1] >> T[2];
	}
      else 
	{
	  ifm >>  temp >> temp >> temp
	      >>  temp >> temp >> temp
	      >>  temp >> temp >> temp;
			
			
	  ifm >>  temp >> temp >> temp
	      >>  temp >> temp >> temp
	      >>  temp >> temp >> temp;
			
	  ifm >>  temp >> temp >> temp;
	}	
    }
	
  ifm.close();
}

// Compute the determinant of a 3*3 matrix
// [ a1 a2 a3;
//   a4 a5 a6;
//   a7 a8 a9; ]
inline float det3x3(float a1, float a2, float a3, float a4, float a5,
			   float a6, float a7, float a8, float a9)
{
  return a1*a5*a9 + a4*a8*a3 + a7*a2*a6
    - a7*a5*a3 - a4*a2*a9 - a1*a8*a6 ;
}


// Inverse matrix of a 3x3 matrix a
inline void inv_3x3(float a[3][3], float inv[3][3])
{
  float det = det3x3(a[0][0], a[0][1], a[0][2], a[1][0], a[1][1],
		     a[1][2], a[2][0], a[2][1], a[2][2]);
	
  inv[0][0] =   (a[1][1]*a[2][2] - a[2][1]*a[1][2])/det;
  inv[1][0] = - (a[1][0]*a[2][2] - a[2][0]*a[1][2])/det;
  inv[2][0] =   (a[1][0]*a[2][1] - a[2][0]*a[1][1])/det;
	
  inv[0][1] = - (a[0][1]*a[2][2] - a[2][1]*a[0][2])/det;
  inv[1][1] =   (a[0][0]*a[2][2] - a[2][0]*a[0][2])/det;
  inv[2][1] = - (a[0][0]*a[2][1] - a[2][0]*a[0][1])/det;
	
  inv[0][2] =   (a[0][1]*a[1][2] - a[1][1]*a[0][2])/det;
  inv[1][2] = - (a[0][0]*a[1][2] - a[1][0]*a[0][2])/det;
  inv[2][2] =   (a[0][0]*a[1][1] - a[1][0]*a[0][1])/det;
}

// c = a*b
inline void MatMul3_3x3_3(float a[3][3], float b[3][3], float c[3][3])
{
  c[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
  c[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
  c[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];
	
  c[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
  c[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
  c[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];
	
  c[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
  c[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
  c[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];
}

// c = a*b;
inline void MatMul3_3x3_1(float a[3][3], float b[3], float c[3])
{
  c[0] = a[0][0]*b[0] + a[0][1]*b[1] + a[0][2]*b[2];
  c[1] = a[1][0]*b[0] + a[1][1]*b[1] + a[1][2]*b[2];
  c[2] = a[2][0]*b[0] + a[2][1]*b[1] + a[2][2]*b[2];
}

// Translate point position into world coordinate system
inline void translate(point& a_point, float a[3][3], float b[3], int ImgX, int ImgY, float depth)
{
  float CamPos[3];
  CamPos[0] = ImgX*depth;
  CamPos[1] = ImgY*depth;
  CamPos[2] = depth;
  
  float temp[3];
  
  MatMul3_3x3_1(a, CamPos, temp);
  
  a_point.WorldX = temp[0] - b[0];
  a_point.WorldY = temp[1] - b[1];
  a_point.WorldZ = temp[2] - b[2];
}


inline void cross_product(float a[3], float b[3], float c[3])
{
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}
