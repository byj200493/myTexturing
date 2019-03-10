#include "generate_nnf.h"
/*
description:
compute the distance term
created at 08/16/2017
modified at 08/21/2017
*/
float distanceNNF(NNF_Data* nnf_p, float prev_dist, int xs, int ys, int xt, int yt)
{
	/*long double distance = 0;
	long double wsum = 0, ssdmax = 9 * 255 * 255;
	int dy, dx, band;
	int xks, yks;
	int xkt, ykt;
	long double ssd;
	//long res;
	int s_value, t_value, s_gx, t_gx, s_gy, t_gy;

	// for each pixel in the source patch
	for (dy = -nnf_p->patch_size/2; dy <= nnf_p->patch_size/2; ++dy) {
		for (dx = -nnf_p->patch_size/2; dx <= nnf_p->patch_size/2; ++dx) {

			xks = xs + dx;
			yks = ys + dy;
			xkt = xt + dx;
			ykt = yt + dy;
			wsum++;

			if (xks<1 || xks >= nnf_p->source->width - 1) { distance++; continue; }
			if (yks<1 || yks >= nnf_p->source->height - 1) { distance++; continue; }

			// corresponding pixel in the target patch
			if (xkt<1 || xkt >= nnf_p->dest->width - 1) { distance++; continue; }
			if (ykt<1 || ykt >= nnf_p->dest->height - 1) { distance++; continue; }

			ssd = 0;
			for (band = 0; band<3; ++band) {
				// pixel values
				s_value = CV_IMAGE_ELEM(nnf_p->source, unsigned char, yks, 3 * xks + band);//getSampleMaskedImage(source, xks, yks, band);
				t_value = CV_IMAGE_ELEM(nnf_p->dest, unsigned char, ykt, 3 * xkt + band);//getSampleMaskedImage(source, xkt, ykt, band);

				// pixel horizontal gradients (Gx)
				s_gx = 128 + (CV_IMAGE_ELEM(nnf_p->source, unsigned char, yks, 3 * (xks+1) + band) - CV_IMAGE_ELEM(nnf_p->source, unsigned char, yks, 3 * (xks-1) + band)) / 2;
				t_gx = 128 + (CV_IMAGE_ELEM(nnf_p->dest, unsigned char, ykt, 3 * (xkt + 1) + band) - CV_IMAGE_ELEM(nnf_p->dest, unsigned char, ykt, 3 * (xkt - 1) + band)) / 2;

				// pixel vertical gradients (Gy)
				s_gy = 128 + (CV_IMAGE_ELEM(nnf_p->source, unsigned char, yks + 1, 3 * xks + band) - CV_IMAGE_ELEM(nnf_p->source, unsigned char, yks - 1, 3 * xks + band)) / 2;//(getSampleMaskedImage(source, xks, yks + 1, band) - getSampleMaskedImage(source, xks, yks - 1, band)) / 2;
				t_gy = 128 + (CV_IMAGE_ELEM(nnf_p->dest, unsigned char, ykt + 1, 3 * xkt + band) - CV_IMAGE_ELEM(nnf_p->dest, unsigned char, ykt - 1, 3 * xkt + band)) / 2;//(getSampleMaskedImage(target, xkt, ykt + 1, band) - getSampleMaskedImage(target, xkt, ykt - 1, band)) / 2;

				ssd += pow((long double)s_value - t_value, 2); // distance between values in [0,255^2]
				ssd += pow((long double)s_gx - t_gx, 2); // distance between Gx in [0,255^2]
				ssd += pow((long double)s_gy - t_gy, 2); // distance between Gy in [0,255^2]
			}

			// add pixel distance to global patch distance
			distance += ssd / ssdmax;
		}
	}
	distance /= wsum;
	return distance;*/
	IplImage* src_img = nnf_p->source;
	IplImage* targ_img = nnf_p->dest;
	int width = nnf_p->source->width;
	int height = nnf_p->source->height;
	int patch_size = nnf_p->patch_size;
	float sum = patch_size*patch_size;
	float distance = 0.0f;
	for (int iy = -patch_size / 2; iy < patch_size / 2; iy++)
	{
		for (int ix = -patch_size / 2; ix < patch_size / 2; ix++)
		{
			/*int sx = MAX(0, MIN(ix + xs, width-1));
			int sy = MAX(0, MIN(iy + ys, height-1));
			int tx = MAX(0, MIN(ix + xt, width-1));
			int ty = MAX(0, MIN(iy + yt, height-1));*/
			int sx = ix + xs;
			if (sx < 0 || sx > width - 1)
				return FLT_MAX;
			int sy = iy + ys;
			if (sy < 0 || sy > height - 1)
				return FLT_MAX;
			int tx = ix + xt;
			if (tx < 0 || tx > width - 1)
				return FLT_MAX;
			int ty = iy + yt;
			if (ty < 0 || ty > height - 1)
				return FLT_MAX;
			float fdist[3];
			fdist[0] = CV_IMAGE_ELEM(src_img, unsigned char, sy, 3*sx) - CV_IMAGE_ELEM(targ_img, unsigned char, ty, 3*tx);
			fdist[1] = CV_IMAGE_ELEM(src_img, unsigned char, sy, 3 * sx+1) - CV_IMAGE_ELEM(targ_img, unsigned char, ty, 3 * tx+1);
			fdist[2] = CV_IMAGE_ELEM(src_img, unsigned char, sy, 3 * sx+2) - CV_IMAGE_ELEM(targ_img, unsigned char, ty, 3 * tx+2);
			distance += (fdist[0]*fdist[0] + fdist[1] * fdist[1] + fdist[2] * fdist[2])/sum;
			if (distance > prev_dist)
				return FLT_MAX;
		}
	}
	return distance;
}
/*
created at 08/17/2017
*/
void randomize(IplImage* nnf, float r)
{
	int width = nnf->width;
	int height = nnf->height;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int px = x + fmod(rand(), 2 * r) - r;
			int py = y + fmod(rand(), 2 * r) - r;
			px = MAX(0, MIN(width - 1, px));
			py = MAX(0, MIN(height - 1, py));
			CV_IMAGE_ELEM(nnf, float, y, 3 * x) = px;
			CV_IMAGE_ELEM(nnf, float, y, 3 * x + 1) = py;
			CV_IMAGE_ELEM(nnf, float, y, 3 * x + 2) = FLT_MAX;
		}
	}
}
/*
created at 08/16/2017
*/
NNF_Data* init_NNF(IplImage* source, IplImage* dest, int patch_size)
{
	NNF_Data* nnf_p = new NNF_Data;
	nnf_p->patch_size = patch_size;
	nnf_p->source = source;
	nnf_p->dest = dest;
	int width = source->width;
	int height = source->height;
	nnf_p->radius = 0.1f*powf((float)width*height, 0.5f);
	nnf_p->nnf = cvCreateImage(cvSize(width, height),IPL_DEPTH_32F,3);
	randomize(nnf_p->nnf, nnf_p->radius);
	return nnf_p;
}
/*
created at 08/17/2017
*/
// minimize a single link (see "PatchMatch" - page 4)
void minimizeLinkNNF(NNF_Data* nnf_p, int x, int y, int dir)
{
	int width = nnf_p->nnf->width;
	int height = nnf_p->nnf->height;
	float xp, yp, dp, wi, xpi, ypi;
	float current_dist = CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x + 2);
	//Propagation Up/Down
	if (y - dir>0 && y - dir<nnf_p->nnf->height) {
		xp = CV_IMAGE_ELEM(nnf_p->nnf, float, y - dir, 3 * x);
		yp = CV_IMAGE_ELEM(nnf_p->nnf, float, y - dir, 3 * x + 1) + dir;
		yp = MAX(0, MIN(yp, height-1));
		dp = distanceNNF(nnf_p, current_dist, x, y, xp, yp);
		if (dp < current_dist) {
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x) = xp;
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x+1) = yp;
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x+2) = dp;
			current_dist = dp;
		}
	}
	//Propagation Left/Right
	if (x - dir>0 && x - dir<nnf_p->source->width) {
		xp = CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * (x-dir)) + dir;
		yp = CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * (x-dir)+1);
		xp = MAX(0, MIN(xp, width - 1));
		dp = distanceNNF(nnf_p, current_dist, x, y, xp, yp);
		if (dp < current_dist) {
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x) = xp;
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x + 1) = yp;
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x + 2) = dp;
			current_dist = dp;
		}
	}
	//Random search
	wi = nnf_p->radius;//nnf->output->image->width;
	xpi = CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x);// nnf->field[x][y][0];
	ypi = CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x+1);// nnf->field[x][y][1];
	int r = 0;
	while (wi>1) {
		r = fmod(rand(), 2*wi) - wi;//(rand()%(2 * wi)) - wi;
		xp = xpi + r;
		r = fmod(rand(), 2 * wi) - wi;//r = (rand() % (2 * wi)) - wi;
		yp = ypi + r;
		xp = int(MAX(0, MIN(nnf_p->source->width - 1, xp)));
		yp = int(MAX(0, MIN(nnf_p->source->height - 1, yp)));
		current_dist = CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x + 2);
		dp = distanceNNF(nnf_p, current_dist, x, y, xp, yp);
		if (dp < current_dist) {
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x) = xp;
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x+1) = yp;
			CV_IMAGE_ELEM(nnf_p->nnf, float, y, 3 * x+2) = dp;
		}
		wi /= 2;
	}
}
/*
description:
multi-pass NN-field minimization (see "PatchMatch" - page 4)
created at 08/16/2017
*/
IplImage* minimizeNNF(IplImage* src_img, IplImage* targ_img, int patch_size, int pass)
{
	NNF_Data* nnf_p = init_NNF(src_img, targ_img, patch_size);
	IplImage* nnf_img = nnf_p->nnf;
	int i, y, x;
	int min_x = 1, min_y = 1, max_x = src_img->width - 1, max_y = src_img->height - 1;
	// multi-pass minimization
	for (i = 0; i<pass; i++) {
		// scanline order
		for (y = min_y; y<max_y; ++y)
			for (x = min_x; x <= max_x; ++x)
				if (CV_IMAGE_ELEM(nnf_img,float,y,3*x+2)>0)
					minimizeLinkNNF(nnf_p, x, y, +1);
		// reverse scanline order
		for (y = max_y; y >= min_y; y--)
			for (x = max_x; x >= min_x; x--)
				if (CV_IMAGE_ELEM(nnf_img, float, y, 3 * x + 2)>0)
					minimizeLinkNNF(nnf_p, x, y, -1);
	}
	return nnf_img;
}
/*
created at 08/19/2017
*/
IplImage* generate_compeletness(IplImage* src_img, IplImage* targ_img, int patch_size, int nPass)
{
	int width = src_img->width;
	int height = src_img->height;
	IplImage* nnf_img = minimizeNNF(src_img, targ_img, patch_size, nPass);
	IplImage* compeletness_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 4);
	cvZero(compeletness_img);
	float a = (float)1.0f/patch_size*patch_size;
	for (int y = patch_size/2; y < height - patch_size/2; y++)
	{
		for (int x = patch_size/2; x < width - patch_size/2; x++)
		{
			int xp = CV_IMAGE_ELEM(nnf_img, float, y, 3 * x);
			int yp = CV_IMAGE_ELEM(nnf_img, float, y, 3 * x+1);
			CV_IMAGE_ELEM(compeletness_img, float, yp, 4 * xp) += a*CV_IMAGE_ELEM(src_img, unsigned char, y, 3 * x);
			CV_IMAGE_ELEM(compeletness_img, float, yp, 4 * xp+1) += a*CV_IMAGE_ELEM(src_img, unsigned char, y, 3 * x+1);
			CV_IMAGE_ELEM(compeletness_img, float, yp, 4 * xp+2) += a*CV_IMAGE_ELEM(src_img, unsigned char, y, 3 * x+2);
			CV_IMAGE_ELEM(compeletness_img, float, yp, 4 * xp + 3) += a;
		}
	}
	return compeletness_img;
}
/*
created at 08/19/2017
*/
IplImage* generate_coherence(IplImage* src_img, IplImage* targ_img, int patch_size, int nPass)
{
	int width = src_img->width;
	int height = src_img->height;
	float wi = 0.1f*powf(width*height, 0.5f);
	IplImage* nnf_img = minimizeNNF(targ_img, src_img, patch_size, nPass);
	IplImage* coherence_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 4);
	cvZero(coherence_img);
	float a = (float)2.0f / patch_size*patch_size;
	for (int y = patch_size / 2; y < height - patch_size / 2; y++)
	{
		for (int x = patch_size / 2; x < width - patch_size / 2; x++)
		{
			int xp = CV_IMAGE_ELEM(nnf_img, float, y, 3 * x);
			int yp = CV_IMAGE_ELEM(nnf_img, float, y, 3 * x + 1);
			CV_IMAGE_ELEM(coherence_img, float, y, 4 * x) += a*CV_IMAGE_ELEM(src_img, unsigned char, yp, 3 * xp);
			CV_IMAGE_ELEM(coherence_img, float, y, 4 * x + 1) += a*CV_IMAGE_ELEM(src_img, unsigned char, yp, 3 * xp + 1);
			CV_IMAGE_ELEM(coherence_img, float, y, 4 * x + 2) += a*CV_IMAGE_ELEM(src_img, unsigned char, yp, 3 * xp + 2);
			CV_IMAGE_ELEM(coherence_img, float, y, 4 * x + 3) += a;
		}
	}
	return coherence_img;
}
