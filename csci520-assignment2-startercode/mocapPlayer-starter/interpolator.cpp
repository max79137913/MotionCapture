#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <ctime>

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}
// create self-made funciton
void Interpolator::MultiplyMatrix(RotMatrix firstM, RotMatrix secondM,RotMatrix outputM){
    int i,j,k;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
		outputM[i][j]=0;
		}
	}
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				outputM[i][j]=outputM[i][j]+firstM[i][k]*secondM[k][j];
			}
		}
	}

}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

   std::clock_t start;
   double duration;
   start = std::clock();


  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  printf("%lf\n",duration);
  system("pause");

}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	double Radian[3];
	Radian[0]=Degree2Radian(angles[0]);
	Radian[1]=Degree2Radian(angles[1]);
	Radian[2]=Degree2Radian(angles[2]);

	RotMatrix D={
		cos(Radian[2]),	-sin(Radian[2]),	0.0,
		sin(Radian[2]),	cos(Radian[2]),	0.0,
		0.0,	0.0,	1.0               };
	RotMatrix C={
		cos(Radian[1]),	0.0,	sin(Radian[1]),
		0.0,	1.0,	0.0, 
		-sin(Radian[1]),	0.0,	cos(Radian[1])};
	RotMatrix B={
		1.0,	0.0,	0.0,
		0.0,	cos(Radian[0]),	-sin(Radian[0]),
		0.0,	sin(Radian[0]),	cos(Radian[0])};
   RotMatrix intermediate, result;

	MultiplyMatrix(D,C,intermediate);
	MultiplyMatrix(intermediate,B,result); 
	for (int i = 0; i < 3; i++)
	{
		for (int j =  0; j < 3; j++)
		{
			R[j + 3 * i] = result[i][j];
		}
	}
	
 
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  
   std::clock_t start; // time counter
   double duration;
   start = std::clock();


  vector p0,p1,p2,p3; // key points 
  vector a1,b2;       // control points 

  vector rp0,rp1,rp2,rp3;// rotation key points
  vector ra1,rb2;       // rotational control points 

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

	int previousKeyframe=startKeyframe-1-N; // for Bezier interpolation 
	int nextKeyframe=endKeyframe+1+N;
	
    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
	Posture * previous=NULL;
	Posture * next = NULL;

	if (previousKeyframe >= 0)
	{
	previous = pInputMotion->GetPosture(previousKeyframe); // Setup previous and next bone_rotation 
	}
	if (nextKeyframe < inputLength)
	{
	next = pInputMotion->GetPosture(nextKeyframe);
	}
	

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
	  p1=startPosture->root_pos;  // p1,p2 are start,end frame root pos
	  p2=endPosture->root_pos;
	  vector a2,temp;


	  if (startKeyframe == 0) // first frame doesn't have previous frame
	  {  
		  p3=next->root_pos;  
		  a1=p2-p3+p2;      // find a1
		  a1 = p1 + (a1 - p1) * (1.0 / 3); // set point to 1/3 length followed by an'=pn+k*(an-pn)
		
		  //cacaulate seconde point
		  temp = p2 - p1 + p2;
		  a2 = (temp + p3) * 0.5;
		  a2 = p2 + (a2 - p2) * (1.0 / 3); // set point to 1/3 length
		  b2 = p2 - a2 + p2;


	  }
	  else if(nextKeyframe>inputLength)  //last frame doesn't have the next frame
	  {
		  p0 = previous->root_pos;
		  temp = p1 - p0 + p1; 
		  // caculate a1;
		  a1 = (temp + p2) * 0.5;
		  a1 = p1 + (a1 - p1) * (1.0 / 3) ; // set point to 1/3 length 

     	  // calculate b2
		  a2 = p1 - p0 + p1;
		  b2 =p2+ (a2 - p2) * (1.0 / 3) ;// set point to 1/3 length 
	  }
	  else
	  {
		  p0=previous->root_pos;
		  p3=next->root_pos;

		  temp=p1-p0+p1;
		  a1=(temp+p2)*0.5; // find _a1 
		  a1=p1+(a1-p1)*(1.0/3);// set point to 1/3 length 

		  // caculate b2
		  temp=p2-p1+p2;
		  a2=(temp+p3)*0.5; // find _a2
		  a2=p2+(a2-p2)*(1.0/3);// set point to 1/3 length 
		  b2=p2-a2+p2;
	  }
	  interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

      // interpolate bone rotations
	  for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
	  {
		
	  rp1=startPosture->bone_rotation[bone];  // rp1,rp2 are start,end frame bone rotation 
	  rp2=endPosture->bone_rotation[bone];
	  vector ra2,rtemp;

			if(startKeyframe==0)
			{
			  rp3=next->bone_rotation[bone];  
			  ra1=rp2-rp3+rp2;
			  ra1 =rp1+ (ra1 - rp1) * (1.0 / 3) ; // set point to 1/3 length 
		
			  //cacaulate seconde point
			  rtemp = rp2 - rp1 + rp2;
			  ra2 = (rtemp + rp3) * 0.5;
			  ra2 = rp2+(ra2 - rp2) * (1.0 / 3); // set point to 1/3 length
			  rb2 = rp2 - ra2 + rp2;  
	        }
			else if(nextKeyframe>inputLength)  //last frame doesn't have the next frame
		   {
			rp0 = previous->bone_rotation[bone];
			rtemp = rp1 - rp0 + rp1;
			 // caculate a1;
			ra1 = (rtemp + rp2) * 0.5;
			ra1 = rp1+(ra1 - rp1) * (1.0 / 3) ; // set point to 1/3 length 

     		// calculate b2
			ra2 = rp1 - rp0 + rp1;
			rb2 = rp2+ (ra2 - rp2) * (1.0 / 3);// set point to 1/3 length 
	       }
		   else
	       {
			   rp0=previous->bone_rotation[bone];
			   rp3=next->bone_rotation[bone];

			  rtemp=rp1-rp0+rp1;
			  ra1=(rtemp+rp2)*0.5; // find a1 
			  ra1=rp1+(ra1-rp1)*(1.0/3);// set point to 1/3 length 

			  // caculate b2
			  rtemp=rp2-rp1+rp2;
			  ra2=(rtemp+rp3)*0.5;
			  ra2=rp2+(ra2-rp2)*(1.0/3);// set point to 1/3 length 
			  rb2=rp2-ra2+rp2;
		   }
			interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, rp1, ra1, rb2, rp2);
	  }
      

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));


  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  printf("%lf\n",duration);
  system("pause");

}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

   std::clock_t start; // time counter
   double duration;
   start = std::clock();


  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){ //interpolate quaternion
   
		  // interpolatedPosture.bone_rotation[bone] 

          Quaternion<double> qStart,qEnd,qInterpolate;
		  Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
		  Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
		  qInterpolate = Slerp(t, qStart, qEnd);
          Quaternion2Euler(qInterpolate,interpolatedPosture.bone_rotation[bone].p);

	  }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  printf("%lf\n",duration);
  system("pause");

}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  
   std::clock_t start; // time counter
   double duration;
   start = std::clock();


  vector p0,p1,p2,p3; // key points 
  vector a1,b2;       // control points 

  Quaternion<double> rp0,rp1,rp2,rp3;// rotation key points
  Quaternion<double> ra1,rb2;       // rotational control points 

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

	int previousKeyframe=startKeyframe-1-N; // for Bezier interpolation 
	int nextKeyframe=endKeyframe+1+N;
	
    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
	Posture * previous=NULL;
	Posture * next = NULL;

	if (previousKeyframe >= 0)
	{
	previous = pInputMotion->GetPosture(previousKeyframe); // Setup previous and next bone_rotation 
	}
	if (nextKeyframe < inputLength)
	{
	next = pInputMotion->GetPosture(nextKeyframe);
	}
	

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
	  p1=startPosture->root_pos;  // p1,p2 are start,end frame root pos
	  p2=endPosture->root_pos;
	  vector a2,temp;


	  if (startKeyframe == 0) // first frame doesn't have previous frame
	  {  
		  p3=next->root_pos;  
		  a1=p2-p3+p2;
		  a1 = p1 + (a1 - p1) * (1.0 / 3); // set point to 1/3 length 
		
		  //cacaulate seconde point
		  temp = p2 - p1 + p2;
		  a2 = (temp + p3) * 0.5;
		  a2 = p2 + (a2 - p2) * (1.0 / 3); // set point to 1/3 length
		  b2 = p2 - a2 + p2;


	  }
	  else if(nextKeyframe>inputLength)  //last frame doesn't have the next frame
	  {
		  p0 = previous->root_pos;
		  temp = p1 - p0 + p1; 
		  // caculate a1;
		  a1 = (temp + p2) * 0.5;
		  a1 = p1 + (a1 - p1) * (1.0 / 3) ; // set point to 1/3 length 

     	  // calculate b2
		  a2 = p1 - p0 + p1;
		  b2 =p2+ (a2 - p2) * (1.0 / 3) ;// set point to 1/3 length 
	  }
	  else
	  {
		  p0=previous->root_pos;
		  p3=next->root_pos;

		  temp=p1-p0+p1;
		  a1=(temp+p2)*0.5; // find a1 
		  a1=p1+(a1-p1)*(1.0/3);// set point to 1/3 length 

		  // caculate b2
		  temp=p2-p1+p2;
		  a2=(temp+p3)*0.5;
		  a2=p2+(a2-p2)*(1.0/3);// set point to 1/3 length 
		  b2=p2-a2+p2;
	  }
	  interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

      // interpolate bone rotations
	  for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
	  {
		  Euler2Quaternion(startPosture->bone_rotation[bone].p,rp1);	 // change start,end euler rotation to Quaternion
		  Euler2Quaternion(endPosture->bone_rotation[bone].p,rp2);	
	
	      Quaternion<double> ra2,rtemp;
		

			if(startKeyframe==0)
			{
			  Euler2Quaternion(next->bone_rotation[bone].p,rp3); // change next frame rotation to Quaternion
			  rtemp=Slerp(2.0,rp3,rp2);
			  ra1=Slerp(1.0/3,rp1,rtemp);  // set point to 1/3 length
		
			  rtemp=Slerp(2.0,rp1,rp2);
			  ra2=Slerp(0.5,rtemp,rp3);   // make the _ra2
			  rb2=Slerp(-1.0/3,rp2,ra2);

	        }
			else if(nextKeyframe>inputLength)  //last frame doesn't have the next frame
		   {
			Euler2Quaternion(previous->bone_rotation[bone].p,rp0); // change previous frame rotation to Quaternion
			rtemp=Slerp(2.0,rp0,rp1);
			 // caculate a1;
			ra1=Slerp(0.5,rtemp,rp2);
			ra1 = Slerp(1.0/3,rp1,ra1); ; // set point to 1/3 length 

     		// calculate b2
			ra2=Slerp(2.0,rp0,rp1);
			rb2=Slerp(1.0/3,rp2,ra2);

	       }
		   else
	       {
			   Euler2Quaternion(previous->bone_rotation[bone].p,rp0); // change previous frame rotation to Quaternion
			   Euler2Quaternion(next->bone_rotation[bone].p,rp3);     // change next frame rotation to Quaternion
			  
			  rtemp=Slerp(2.0,rp0,rp1);
			  ra1=Slerp(0.5,rtemp,rp2); //set up _ra1 
			  ra1=Slerp(1.0/3,rp1,ra1); //set point to 1/3 length

			  // caculate b2
			  rtemp=Slerp(2.0,rp1,rp2);
			  ra2=Slerp(0.5,rtemp,rp3); //set up _ra2
			  rb2=Slerp(-1.0/3,rp2,ra2);
		   }
			Quaternion<double> interpolateQuaternion;
			interpolateQuaternion = DeCasteljauQuaternion(t, rp1, ra1, rb2, rp2);
			Quaternion2Euler(interpolateQuaternion,interpolatedPosture.bone_rotation[bone].p);
	  }
      

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  printf("%lf\n",duration);
  system("pause");

}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
	double R[9];
	Euler2Rotation(angles,R); // convert Euler to rotation matrix
	q = Quaternion<double>::Matrix2Quaternion(R); // convert matrix to quaternion
	q.Normalize();
    
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  double R[9];

  q.Quaternion2Matrix(R); // quaternion to rotation matrix 
  Rotation2Euler(R,angles); // rotation matrix to euler
}
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
	double DotProduct;//cos(theta)
	Quaternion<double> qCloser;
	Quaternion<double> result;

	DotProduct=qStart.Gets()*qEnd_.Gets()+qStart.Getx()*qEnd_.Getx()+qStart.Gety()*qEnd_.Gety()+qStart.Getz()*qEnd_.Getz();

	if(DotProduct<0){ // change the quaternion since -q is more closer
		DotProduct=-DotProduct;
		qCloser.Set(-qEnd_.Gets(),-qEnd_.Getx(),-qEnd_.Gety(),-qEnd_.Getz());
	}else{ 
	qCloser=qEnd_;
	}
	float degree = acosf(DotProduct);
    if(degree == 0)return qEnd_;
	
	result=(sinf((1-t)*degree)/sinf(degree))*qStart+(sinf(t*degree)/sinf(degree))*qCloser;

	result.Normalize();
 
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;
  double dotproduct;
  
 
  dotproduct= p.Gets()*q.Gets()+p.Getx()*q.Getx()+p.Gety()*q.Gety()+p.Getz()*q.Getz();

  result= 2*(dotproduct)*q- p;

  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector q0,q1,q2,r0,r1,result;

  q0=p1*t+p0*(1-t);
  q1=p2*t+p1*(1-t);
  q2=p3*t+p2*(1-t);
  r0=q1*t+q0*(1-t);
  r1=q2*t+q1*(1-t);
  result=r1*t+r0*(1-t);

  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  Quaternion<double> q0,q1,q2,r0,r1,result;
  
  q0=Slerp(t,p0,p1);
  q1=Slerp(t,p1,p2);
  q2=Slerp(t,p2,p3);
  r0=Slerp(t,q0,q1);
  r1=Slerp(t,q1,q2);
  result=Slerp(t,r0,r1);

  return result;
}

