#include "PlyFilter.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

PlyFilter::PlyFilter() 
{
}

void PlyFilter::MakeGaussTab(float * coeff, float s)
{
	for (int i = 0; i < 16384; i++)
	{
		float tmp = (float)(exp(0.0) / (3.1415926f * s));
		float i2 = (float)i*i;
		coeff[i] = (float)(((exp(-(i2) / s)) / (3.1415926 * s)) / tmp);
	}
}

void PlyFilter::MakeMeanTab(float * coeff, float s)
{
	for (int i = 0; i < 16384; i++)
	{
		coeff[i] = (i <= (int)s) ? 1.0f : 0.0f;
	}
}

void PlyFilter::WMedFilter(unsigned char * pLImg, unsigned short * pDImgIn, unsigned short * pDImgOut, float * coeff, int width, int height, WMedFilterParam * pWMedFilterP)
{
	/*
	const int xran = 33;
	const int yran = 33;		//initial value 33. up, cpu loading up.
	const int xc = xran / 2;
	const int yc = yran / 2;
	int src = 1;				//0
	int mode = 0;				//0
	int filter_hf_en = 1;		//1
	int filter_hf_ratio = 4;	//4
	*/

	WMedFilterParam WMedFilterP = *pWMedFilterP;
	const int xran = pWMedFilterP->xran;
	const int yran = pWMedFilterP->yran;					//initial value 33. up, cpu loading up.
	const int xc = pWMedFilterP->xc;
	const int yc = pWMedFilterP->yc;
	int src = pWMedFilterP->src;							//0
	int mode = pWMedFilterP->mode;						//0
	int filter_hf_en = pWMedFilterP->filter_hf_en;		//1
	int filter_hf_ratio = pWMedFilterP->filter_hf_ratio;	//4

	int i, j;
	unsigned short *dptr = pDImgIn;
	unsigned short *dout = pDImgOut;
	float wgt;
	int hist_map[16384]; /*2^ 14 bit*/
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int center = x + y * width; // address of the center pixel	
			int yctr = src ? dptr[center] : pLImg[center];
			float non_zero_sum = 0;
			float all_sum = 0;
			const int mem_size = xran * yran;
			HIST * hist = (HIST*)malloc(sizeof(HIST)*mem_size);
			memset(hist, 0x0, sizeof(HIST)*mem_size);
			//HIST hist[mem_size] = { 0 };
			memset(hist_map, 0xff, sizeof(hist_map));
			int idx_cnt = 0;
			for (j = 0; j < yran; j++)
			{
				int yidx = y + j - yc;
				if (yidx >= 0 && yidx < height)
				{
					for (i = 0; i < xran; i++)
					{
						int xidx = x + i - xc;
						if (xidx >= 0 && xidx < width)
						{
							int ysur = src ? dptr[yidx*width + xidx] : pLImg[yidx*width + xidx];
							int diff = abs(yctr - ysur);
							wgt = coeff[diff];
							if (wgt)
							{
								if (dptr[yidx*width + xidx])
								{
									int d = dptr[yidx*width + xidx];
									if (hist_map[d] == -1)
									{
										if (idx_cnt < mem_size)
										{
											hist_map[d] = idx_cnt;
											hist[idx_cnt].idx = d;
											hist[idx_cnt].wgt_sum = wgt;
											idx_cnt++;
										}
									}
									else //value existed
									{
										hist[hist_map[d]].wgt_sum += wgt;
									}
									non_zero_sum += wgt;
								}
								all_sum += wgt;
							}
						}
					}
				}
			}
			if (dptr[center] == 0 && (!filter_hf_en || non_zero_sum < (all_sum*filter_hf_ratio / 8.0)))
			{
				dout[center] = 0;
			}
			else
			{
				int idx = 0;
				if (mode == 0) //median
				{
					float hist_cnt = 0;
					while (hist_cnt < (non_zero_sum / 2.0))
					{
						idx++;
						if (hist_map[idx] >= 0)
							hist_cnt += hist[hist_map[idx]].wgt_sum;
					}
					dout[center] = idx;
				}
				else if (mode == 1) //mode
				{
					float max_wgt = 0;
					for (i = 0; i < idx_cnt; i++)
					{
						if (hist[i].wgt_sum > max_wgt)
						{
							max_wgt = hist[i].wgt_sum;
							idx = hist[i].idx;
						}
					}
					dout[center] = idx;
				}
				else //mean
				{
					float a = 0, b = 0;
					for (i = 0; i < idx_cnt; i++)
					{
						a += hist[i].idx*hist[i].wgt_sum;
						b += hist[i].wgt_sum;
					}
					dout[center] = (b > 0.0) ? (unsigned short)(a / b) : 0;
				}
			}
			free(hist);
		}
	}
}

void PlyFilter::WMedFilter_1(unsigned char * pLImg, unsigned short * pDImgIn, unsigned short * pDImgOut, float * coeff, int width, int height, WMedFilter_1Param * pWMedFilter_1P)
{
	/*
	const int xran = 17;
	const int yran = 17;		//initial value 33. up, cpu loading up.
	const int xc = xran / 2;
	const int yc = yran / 2;
	int src = 0;				//0
	int mode = 0;				//0
	int filter_hf_en = 1;		//1
	int filter_hf_ratio = 4;	//4
	*/

	WMedFilter_1Param WMedFilter_1P = *pWMedFilter_1P;
	const int xran = pWMedFilter_1P->xran;
	const int yran = pWMedFilter_1P->yran;					//initial value 33. up, cpu loading up.
	const int xc = pWMedFilter_1P->xc;
	const int yc = pWMedFilter_1P->yc;
	int src = pWMedFilter_1P->src;							//0
	int mode = pWMedFilter_1P->mode;							//0
	int filter_hf_en = pWMedFilter_1P->filter_hf_en;			//1
	int filter_hf_ratio = pWMedFilter_1P->filter_hf_ratio;	//4
	
	int i, j;
	unsigned short *dptr = pDImgIn;
	unsigned short *dout = pDImgOut;
	float wgt;
	int hist_map[16384];
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (x == 53 && y == 279)
				x = x;
			int center = x + y * width; // address of the center pixel	
			int yctr = src ? dptr[center] : pLImg[center];
			float non_zero_sum = 0;
			float all_sum = 0;
			const int mem_size = xran * yran;
			HIST * hist = (HIST*)malloc(sizeof(HIST)*mem_size);
			memset(hist, 0x0, sizeof(HIST)*mem_size);
			//HIST hist[mem_size] = { 0 };
			memset(hist_map, 0xff, sizeof(hist_map));
			int idx_cnt = 0;
			for (j = 0; j < yran; j++)
			{
				int yidx = y + j - yc;
				if (yidx >= 0 && yidx < height)
				{
					for (i = 0; i < xran; i++)
					{
						int xidx = x + i - xc;
						if (xidx >= 0 && xidx < width)
						{
							int ysur = src ? dptr[yidx*width + xidx] : pLImg[yidx*width + xidx];
							int diff = abs(yctr - ysur);
							wgt = coeff[diff];
							if (wgt)
							{
								if (dptr[yidx*width + xidx])
								{
									int d = dptr[yidx*width + xidx];
									if (hist_map[d] == -1)
									{
										if (idx_cnt < mem_size)
										{
											hist_map[d] = idx_cnt;
											hist[idx_cnt].idx = d;
											hist[idx_cnt].wgt_sum = wgt;
											idx_cnt++;
										}
									}
									else //value existed
									{
										hist[hist_map[d]].wgt_sum += wgt;
									}
									non_zero_sum += wgt;
								}
								all_sum += wgt;
							}
						}
					}
				}
			}
			if (dptr[center] == 0 && (!filter_hf_en || non_zero_sum < (all_sum*filter_hf_ratio / 8.0)))
			{
				dout[center] = 0;
			}
			else
			{
				int idx = 0;
				if (mode == 0) //median
				{
					float hist_cnt = 0;
					while (hist_cnt < (non_zero_sum / 2.0))
					{
						idx++;
						if (hist_map[idx] >= 0)
							hist_cnt += hist[hist_map[idx]].wgt_sum;
					}
					dout[center] = idx;
				}
				else if (mode == 1) //mode
				{
					float max_wgt = 0;
					for (i = 0; i < idx_cnt; i++)
					{
						if (hist[i].wgt_sum > max_wgt)
						{
							max_wgt = hist[i].wgt_sum;
							idx = hist[i].idx;
						}
					}
					dout[center] = idx;
				}
				else //mean
				{
					float a = 0, b = 0;
					for (i = 0; i < idx_cnt; i++)
					{
						a += hist[i].idx*hist[i].wgt_sum;
						b += hist[i].wgt_sum;
					}
					dout[center] = (b > 0.0) ? (unsigned short)(a / b) : 0;
				}
			}
			free(hist);
		}
	}
}

void PlyFilter::WMeanFilter(unsigned char * pLImg, unsigned short * pDImgIn, unsigned short * pDImgOut, float * coeff, int width, int height, WMeanFilterParam * pWMeanP)
{
	/*
	const int xran = 9;			// initial 9
	const int yran = 9;
	const int xc = xran / 2;
	const int yc = yran / 2;
	int src = 1;				// 1
	int mode = 2;				// 2 0~2
	int filter_hf_en = 0;		// 0 
	int filter_hf_ratio = 2;	// 2 0~8
	*/
	
	WMeanFilterParam WMeanP = *pWMeanP;
	const int xran = WMeanP.xran;					// initial 9
	const int yran = WMeanP.yran;
	const int xc = WMeanP.xc;
	const int yc = WMeanP.yc;
	int src = WMeanP.src;							// 1
	int mode = WMeanP.mode;							// 2 0~2
	int filter_hf_en = WMeanP.filter_hf_en;			// 0 
	int filter_hf_ratio = WMeanP.filter_hf_ratio;	// 2 0~8
	
	int i, j;
	unsigned short *dptr = pDImgIn;
	unsigned short *dout = pDImgOut;
	float wgt;
	int hist_map[16384];
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int center = x + y * width; // address of the center pixel	
			int yctr = src ? dptr[center] : pLImg[center];
			float non_zero_sum = 0;
			float all_sum = 0;
			const int mem_size = xran * yran;
			HIST * hist = (HIST*)malloc(sizeof(HIST)*mem_size);
			memset(hist, 0x0, sizeof(HIST)*mem_size);
			//HIST hist[mem_size] = { 0 };
			memset(hist_map, 0xff, sizeof(hist_map));
			int idx_cnt = 0;
			for (j = 0; j < yran; j++)
			{
				int yidx = y + j - yc;
				if (yidx >= 0 && yidx < height)
				{
					for (i = 0; i < xran; i++)
					{
						int xidx = x + i - xc;
						if (xidx >= 0 && xidx < width)
						{
							int ysur = src ? dptr[yidx*width + xidx] : pLImg[yidx*width + xidx];
							int diff = abs(yctr - ysur);
							wgt = coeff[diff];
							if (wgt)
							{
								if (dptr[yidx*width + xidx])
								{
									int d = dptr[yidx*width + xidx];
									if (hist_map[d] == -1)
									{
										if (idx_cnt < mem_size)
										{
											hist_map[d] = idx_cnt;
											hist[idx_cnt].idx = d;
											hist[idx_cnt].wgt_sum = wgt;
											idx_cnt++;
										}
									}
									else //value existed
									{
										hist[hist_map[d]].wgt_sum += wgt;
									}
									non_zero_sum += wgt;
								}
								all_sum += wgt;
							}
						}
					}
				}
			}
			if (dptr[center] == 0 && (!filter_hf_en || non_zero_sum < (all_sum*filter_hf_ratio / 8.0)))
			{
				dout[center] = 0;
			}
			else
			{
				int idx = 0;
				if (mode == 0) //median
				{
					float hist_cnt = 0;
					while (hist_cnt < (non_zero_sum / 2.0))
					{
						idx++;
						if (hist_map[idx] >= 0)
							hist_cnt += hist[hist_map[idx]].wgt_sum;
					}
					dout[center] = idx;
				}
				else if (mode == 1) //mode
				{
					float max_wgt = 0;
					for (i = 0; i < idx_cnt; i++)
					{
						if (hist[i].wgt_sum > max_wgt)
						{
							max_wgt = hist[i].wgt_sum;
							idx = hist[i].idx;
						}
					}
					dout[center] = idx;
				}
				else //mean
				{
					float a = 0, b = 0;
					for (i = 0; i < idx_cnt; i++)
					{
						a += hist[i].idx*hist[i].wgt_sum;
						b += hist[i].wgt_sum;
					}
					dout[center] = (b > 0.0) ? (unsigned short)(a / b) : 0;
				}
			}
			free(hist);
		}
	}
}

void PlyFilter::WMeanFilterFloat(float * pDImgIn, float * pDImgOut, float * coeff, int width, int height, WMeanFilterFloatParam * pWMeanFloatP)
{

	/*
	const int xran = 9;		// initial 9
	const int yran = 9;
	const int xc = xran / 2;
	const int yc = yran / 2;
	*/
	
	WMeanFilterFloatParam WMeanFloatP = *pWMeanFloatP;
	const int xran = pWMeanFloatP->xran;		// initial 9
	const int yran = pWMeanFloatP->yran;
	const int xc = pWMeanFloatP->xc;
	const int yc = pWMeanFloatP->yc;

	int i, j;
	float *dptr = pDImgIn;
	float *dout = pDImgOut;
	float wgt;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int center = x + y * width; // address of the center pixel	
			float dctr = dptr[center];
			float wgt_sum = 0.0;
			float all_sum = 0.0;
			for (j = -yc; j <= yc; j++)
			{
				for (i = -xc; i <= xc; i++)
				{
					int yidx = y + j;
					if (yidx >= 0 && yidx < height)
					{
						int xidx = x + i;
						if (xidx >= 0 && xidx < width)
						{
							float dsur = dptr[yidx*width + xidx];
							int diff = (int)(60 * abs(dctr - dsur)); //100 can be any interger as you wish

							if (diff >= 16384) diff = 16384 - 1;
							wgt = coeff[diff];

							if (dsur && dctr)
							{
								wgt_sum += wgt;
								all_sum += (wgt*dsur);
							}
						}
					}
				}
			}

			dout[center] = (wgt_sum > 0.0) ? (all_sum / wgt_sum) : 0;
		}
	}
}

void PlyFilter::CF_FILTER(
	std::vector<unsigned char>&bufDepth,
	std::vector<unsigned char> bufColorRGB,
	int widthDepth,
	int heightDepth,
	int widthColor,
	int heightColor,
	std::vector<float>&imgFloatBufOut,
	eSPCtrl_RectLogData* pRectifyLogData
)
{
	WMedFilter_1Param WMedFilter_1P;
	WMedFilterParam WMedFilterP;
	WMeanFilterFloatParam WMeanFloatP;

	GetFilterParams(&WMedFilter_1P, &WMedFilterP, &WMeanFloatP);
	CF_FILTER(bufDepth, bufColorRGB, widthDepth, heightDepth, widthColor, heightColor, imgFloatBufOut, &WMedFilter_1P, &WMedFilterP, &WMeanFloatP, pRectifyLogData);
}

void PlyFilter::CF_FILTER (
	std::vector<unsigned char>&bufDepth, 
	std::vector<unsigned char> bufColorRGB, 
	int widthDepth, 
	int heightDepth, 
	int widthColor, 
	int heightColor,
	std::vector<float>&imgFloatBufOut, 
	WMedFilter_1Param * pWMedFilter_1P, 
	WMedFilterParam * pWMedFilterP, 
	WMeanFilterFloatParam * pWMeanFloatP, 
	eSPCtrl_RectLogData* pRectifyLogData
)
{
	float ratio_color_to_depth = ((float)widthColor * heightColor) / (widthDepth * heightDepth);
	std::vector<unsigned char> bufColorGraylevel;
	bufColorGraylevel.resize(bufColorRGB.size() / 3 / ratio_color_to_depth);
	for (int i = 0; i < bufColorGraylevel.size(); i++)
	{
		bufColorGraylevel[i] = (bufColorRGB[((int)(i * ratio_color_to_depth)) * 3 + 0] + bufColorRGB[((int)(i * ratio_color_to_depth)) * 3 + 1] + bufColorRGB[((int)(i * ratio_color_to_depth)) * 3 + 2]) / 3;
	}

	std::vector<unsigned char> bufDepthTmp, bufDepthTmpout;
	bufDepthTmp.clear();
	bufDepthTmpout.clear();
	bufDepthTmp.assign(bufDepth.begin(), bufDepth.end());
	bufDepthTmpout.resize(bufDepth.size());
	unsigned char * pLImg = &bufColorGraylevel[0];
	unsigned short * pDImgIn = (unsigned short *)&bufDepthTmp[0];
	unsigned short * pDImgOut = (unsigned short *)&bufDepthTmpout[0];

	float ratio = (float)heightDepth / pRectifyLogData->OutImgHeight;
	float focalLength = pRectifyLogData->ReProjectMat[11] * ratio;   //1600
	float baseline = 1.0f / pRectifyLogData->ReProjectMat[14] * ratio; //50
    printf("F*B/ const :  %f", 5000000.f / (focalLength*baseline));
	//CF's filter						

	float coeff[16384];
	//Median
	MakeMeanTab(coeff, 1000);   //400 
	WMedFilter_1(pLImg, pDImgIn, pDImgOut, coeff, widthDepth, heightDepth, pWMedFilter_1P);
	memcpy(pDImgIn, pDImgOut, bufDepth.size());

	pDImgIn = (unsigned short *)&bufDepthTmp[0];
	MakeMeanTab(coeff, 80);   //400 
	WMedFilter(pLImg, pDImgIn, pDImgOut, coeff, widthDepth, heightDepth, pWMedFilterP);
	memcpy(pDImgIn, pDImgOut, bufDepth.size());

	// Disparity to distance (d to z)
	std::vector<float> imgFloatBufIn;
	imgFloatBufIn.resize(bufDepth.size());
	imgFloatBufOut.resize(bufDepth.size());
	float * pImgFloatIn = &imgFloatBufIn[0];
	float * pImgFloatOut = &imgFloatBufOut[0];

	for (int j = 0; j < heightDepth; j++)
	{
		for (int i = 0; i < widthDepth; i++)
		{
			imgFloatBufIn[j*widthDepth + i] = pDImgIn[j*widthDepth + i] ? (focalLength*baseline) / pDImgIn[j*widthDepth + i] : 0; /* F*B*CONSTANT = 5000000 */
		}
	}

	//Mean
	MakeMeanTab(coeff, 140);
	WMeanFilterFloat(pImgFloatIn, pImgFloatOut, coeff, widthDepth, heightDepth, pWMeanFloatP);

	bufDepth.clear();
	bufDepth.assign(bufDepthTmpout.begin(), bufDepthTmpout.end());
}

void PlyFilter::GetFilterParams(WMedFilter_1Param* pWMedFilter_1P, WMedFilterParam* pWMedFilterP, WMeanFilterFloatParam* pWMeanFloatP)
{
    //parameters for WMedFilter_1
	pWMedFilter_1P->xran = 17;
	pWMedFilter_1P->yran = 17;					//initial value 33. up, cpu loading up.
	pWMedFilter_1P->xc = pWMedFilter_1P->xran / 2;
	pWMedFilter_1P->yc = pWMedFilter_1P->yran / 2;
	pWMedFilter_1P->src = 0;						//0
	pWMedFilter_1P->mode = 0;						//0
	pWMedFilter_1P->filter_hf_en = 1;				//1
	pWMedFilter_1P->filter_hf_ratio = 4;			//4

	//parameters for WMedFilter
	pWMedFilterP->xran = 33;
	pWMedFilterP->yran = 33;					//initial value 33. up, cpu loading up.
	pWMedFilterP->xc = pWMedFilterP->xran / 2;
	pWMedFilterP->yc = pWMedFilterP->yran / 2;
	pWMedFilterP->src = 1;						//0
	pWMedFilterP->mode = 0;						//0
	pWMedFilterP->filter_hf_en = 1;				//1
	pWMedFilterP->filter_hf_ratio = 4;			//4

	//parameters for WMeanFilterFloat
	pWMeanFloatP->xran = 9;			// initial 9
	pWMeanFloatP->yran = 9;
	pWMeanFloatP->xc = pWMeanFloatP->xran / 2;
	pWMeanFloatP->yc = pWMeanFloatP->yran / 2;
}

void PlyFilter::CF_FILTER_Z14(
	std::vector<unsigned char>&bufDepth,
	std::vector<unsigned char> bufColorRGB,
	int widthDepth,
	int heightDepth,
	int widthColor,
	int heightColor,
	std::vector<float>&imgFloatBufOut,
	eSPCtrl_RectLogData* pRectifyLogData
)
{
	WMedFilter_1Param WMedFilter_1P;
	WMedFilterParam WMedFilterP;
	WMeanFilterFloatParam WMeanFloatP;

	GetFilterParams(&WMedFilter_1P, &WMedFilterP, &WMeanFloatP);
	CF_FILTER_Z14(bufDepth, bufColorRGB, widthDepth, heightDepth, widthColor, heightColor, imgFloatBufOut, &WMedFilter_1P, &WMedFilterP, &WMeanFloatP, pRectifyLogData);
}

void PlyFilter::CF_FILTER_Z14(
	std::vector<unsigned char>&bufDepth,
	std::vector<unsigned char> bufColorRGB,
	int widthDepth,
	int heightDepth,
	int widthColor,
	int heightColor,
	std::vector<float>&imgFloatBufOut,
	WMedFilter_1Param * pWMedFilter_1P,
	WMedFilterParam * pWMedFilterP,
	WMeanFilterFloatParam * pWMeanFloatP,
	eSPCtrl_RectLogData* pRectifyLogData
)
{
	float ratio_color_to_depth = ((float)widthColor * heightColor) / (widthDepth * heightDepth);
	std::vector<unsigned char> bufColorGraylevel;
	bufColorGraylevel.resize(bufColorRGB.size() / 3 / ratio_color_to_depth);
	for (int i = 0; i < bufColorGraylevel.size(); i++)
	{
		bufColorGraylevel[i] = (bufColorRGB[((int)(i * ratio_color_to_depth)) * 3 + 0] + bufColorRGB[((int)(i * ratio_color_to_depth)) * 3 + 1] + bufColorRGB[((int)(i * ratio_color_to_depth)) * 3 + 2]) / 3;
	}

	std::vector<unsigned char> bufDepthTmp, bufDepthTmpout;
	bufDepthTmp.clear();
	bufDepthTmpout.clear();
	bufDepthTmp.assign(bufDepth.begin(), bufDepth.end());
	bufDepthTmpout.resize(bufDepth.size());
	unsigned char * pLImg = &bufColorGraylevel[0];
	unsigned short * pDImgIn = (unsigned short *)&bufDepthTmp[0];
	unsigned short * pDImgOut = (unsigned short *)&bufDepthTmpout[0];

	float ratio = (float)heightDepth / pRectifyLogData->OutImgHeight;
	float focalLength = pRectifyLogData->ReProjectMat[11] * ratio;   //1600
	float baseline = 1.0f / pRectifyLogData->ReProjectMat[14] * ratio; //50
    printf("(5000000/(F*B)) = const :  %f\n", 5000000.f / (focalLength*baseline));
	//CF's filter						

	float coeff[16384];
	//Median
	MakeMeanTab(coeff, 1000);   //400 
	WMedFilter_1(pLImg, pDImgIn, pDImgOut, coeff, widthDepth, heightDepth, pWMedFilter_1P);
	memcpy(pDImgIn, pDImgOut, bufDepth.size());

	pDImgIn = (unsigned short *)&bufDepthTmp[0];
	MakeMeanTab(coeff, 80);   //400 
	WMedFilter(pLImg, pDImgIn, pDImgOut, coeff, widthDepth, heightDepth, pWMedFilterP);
	memcpy(pDImgIn, pDImgOut, bufDepth.size());

	std::vector<float> imgFloatBufIn;
	imgFloatBufIn.resize(bufDepth.size());
	imgFloatBufOut.resize(bufDepth.size());
	float * pImgFloatIn = &imgFloatBufIn[0];
	float * pImgFloatOut = &imgFloatBufOut[0];

	for (int j = 0; j < heightDepth; j++)
	{
		for (int i = 0; i < widthDepth; i++)
		{
			/* F*B*CONSTANT = 5000000 */
			imgFloatBufIn[j*widthDepth + i] = pDImgIn[j*widthDepth + i] ?(float)(pDImgIn[j*widthDepth + i])/8 : 0;
		}
	}

	//Mean
	MakeMeanTab(coeff, 140);
	WMeanFilterFloat(pImgFloatIn, pImgFloatOut, coeff, widthDepth, heightDepth, pWMeanFloatP);

	bufDepth.clear();
	bufDepth.assign(bufDepthTmpout.begin(), bufDepthTmpout.end());
}

void PlyFilter::UnavailableDisparityCancellation(
	std::vector<unsigned char>&bufDepth,
	int widthDepth,
	int heightDepth,
	int maxDisparity
)
{
	std::vector<unsigned char> bufDepthTmp;
	bufDepthTmp.clear();
	bufDepthTmp.assign(bufDepth.begin(), bufDepth.end());
	unsigned short * pDImgIn = (unsigned short *)&bufDepthTmp[0];
	for (int j = 0; j < heightDepth; j++)
	{
		for (int i = 0; i < widthDepth; i++)
		{
			//int tmp = pDImgIn[j*widthDepth + i];
			if (pDImgIn[j*widthDepth + i])
				if ((pDImgIn[j*widthDepth + i]) > maxDisparity)
					pDImgIn[j*widthDepth + i] = maxDisparity;
		}
	}

	bufDepth.clear();
	bufDepth.assign(bufDepthTmp.begin(), bufDepthTmp.end());
}
