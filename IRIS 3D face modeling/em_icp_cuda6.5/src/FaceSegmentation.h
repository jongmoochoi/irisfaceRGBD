#include <opencv/cxcore.hpp>
#include "src/DATA.h"
#include <iostream>

using namespace std;


class FaceSegmentation
{

public:
	std::vector<cv::Point2f> mChinMark;
	std::vector<float> mSlope;
	std::vector<float> mConstant;
	int eyebrow_x;
	int eyebrow_y;

	FaceSegmentation(std::vector<cv::Point2f> sLandmarkPts)
	{
		mChinMark.clear();
		for (int i = 3; i < 15; i++)
		{
		//	cout << " x: " << sLandmarkPts[i].x << "y: " << sLandmarkPts[i].y << endl;
			mChinMark.push_back(sLandmarkPts[i]);
		}
		for (std::vector<cv::Point2f>::iterator begin = mChinMark.begin(); begin != mChinMark.end(); begin++)
		{
			float x = begin->x;
			float y = begin->y;
			std::vector<cv::Point2f>::iterator next = begin + 1;
			if (next != mChinMark.end())
			{
				float x2 = next->x;
				float y2 = next->y;

				float slope = (y2 - y) / (x2 - x);
				float constant = y - slope*x;
				mSlope.push_back(slope);
				mConstant.push_back(constant);
			}
		}
	}

	~FaceSegmentation()
	{
		mChinMark.clear();
	}

	bool underChinBorder(float sX, float sY)
	{
		int i = this->findChinIndex(sX, sY);
		if (i >= 0)
		{ 
			float slope = mSlope[i];
			float constant = mConstant[i];

			float y = slope*sX + constant;
			//cout << "real y = " << sY << " estimated y = " << y << " " << endl;
			if (sY > y)
			{
//				cout << "real y = " << sY << " estimated y = " << y << " " << endl;
				return true;
			}
		}
		return false;
	}



	bool aboveEyeBrow(float sX, float sY)
	{

		if (sY < this->eyebrow_y)
		{
				//				cout << "real y = " << sY << " estimated y = " << y << " " << endl;
				return true;
			
		}
		return false;
	}


	void setEyebrow(int x, int y)
	{
		eyebrow_x = x;
		eyebrow_y = y;
	}


	int findChinIndex(float sX, float sY)
	{
		int i = 0;
		if (sX < mChinMark[0].x)
		{
			return 0;
		}
		else if (sX > mChinMark[mChinMark.size() - 1].x)
		{
			return mChinMark.size() - 2;
		}

		for (std::vector<cv::Point2f>::iterator begin = mChinMark.begin(); begin != mChinMark.end(); begin++)
		{
			float x = begin->x;
			float y = begin->y;
			std::vector<cv::Point2f>::iterator next = begin + 1;
			if (next != mChinMark.end())
			{
				float x2 = next->x;
				float y2 = next->y;
				if (x <= sX && sX <= x2)
				{
			//		cout << "find index = " << i << endl;
					return i;
				}
			}

			i++;
		}

		return -1;
	}

	void deletePointUnderChin(float *h_X, int nb_X, float centroidX, float centroidY, float centroidZ,float NORMALIZATION_FACTOR, int &nb_point)
	{
		float X, Y, Z = 0;
		int	i_x = 0,
			i_y = nb_X,
			i_z = 2 * nb_X;
		float x = 0.0f,
			y = 0.0f,
			z = 0.0f;
		float avg_z = 0.0f;
		int number_z = 0;
		for (int i = 0; i<nb_X; i++) {
			X = 0; Y = 0; Z = 0;

			x = h_X[i_x];
			y = h_X[i_y];
			z = h_X[i_z];

			if (z != 0)
			{
				x = x * NORMALIZATION_FACTOR + centroidX;
				y = y * NORMALIZATION_FACTOR + centroidY;
				z = z * NORMALIZATION_FACTOR + centroidZ;
				X = (((x / XtoZ) / z) + 0.5f) * 640;
				Y = (((y / YtoZ) / z) - 0.5f) * 480 * (-1);
				//cout << "X = " << X << " Y = " << Y << " " << endl;

				if (this->underChinBorder(X, Y))
				{
					x = 0;
					y = 0;
					z = 0;
					h_X[i_x] = 0;
					h_X[i_y] = 0;
					h_X[i_z] = 0;
					
				}

				else if (this ->aboveEyeBrow(X,Y))
				{
					h_X[i_x] = 0;
					h_X[i_y] = 0;
					h_X[i_z] = 0;
				}

				else if (z > centroidZ)
				{
			
					h_X[i_x] = 0;
					h_X[i_y] = 0;
					h_X[i_z] = 0;

				}

			}

			i_x++;
			i_y++;
			i_z++;
		}

	}
	

	void rearrangeArray(float **h_X, int nb_X, int &nb_point)
	{
		int	i_x = 0,
			i_y = nb_X,
			i_z = 2 * nb_X;
		float x = 0.0f,
			y = 0.0f,
			z = 0.0f;
		int nbPoint = 0;

		float *h_temp = new float[3*nb_X];
		memset(h_temp, 0, 3 * nb_X);

		for (int i = 0; i<nb_X; i++) {

			x = h_X[0][i_x];
			y = h_X[0][i_y];
			z = h_X[0][i_z];

			if (x == 0 && y == 0 && z == 0)
			{

			}

			else
			{
				h_temp[nbPoint] = x;
				h_temp[nbPoint + nb_X] = y;
				h_temp[nbPoint + (2 * nb_X)] = z;
				nbPoint++;
			}

			i_x++;
			i_y++;
			i_z++;
		}
		nb_point = nbPoint;
		delete *h_X;
		*h_X = h_temp;
	}


};