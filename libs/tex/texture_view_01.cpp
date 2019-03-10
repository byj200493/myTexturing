/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <list>

#include <math/matrix.h>
#include <mve/image_io.h>
#include <mve/image_tools.h>

#include "texture_view.h"
#include <iostream>
#include "texturing.h"
#include "generate_nnf.h"
TEX_NAMESPACE_BEGIN

TextureView::TextureView(std::size_t id, mve::CameraInfo const & camera,
    std::string const & image_file)
    : id(id), image_file(image_file) {
	mve::image::ImageHeaders header;
    try {
         header = mve::image::load_file_headers(image_file);
	} catch (util::Exception e) {
        std::cerr << "Could not load image header of " << image_file << std::endl;
        std::cerr << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }

	width = header.width;
	height = header.height;
	rate = (float)width / height;
	camera.fill_calibration01(*projection);
	camera.fill_calibration01(*original_projection);//08/09/2017
	camera.fill_camera_pos(*pos);
    camera.fill_viewing_direction(*viewdir);
    camera.fill_world_to_cam(*world_to_cam);
	camera.fill_cam_to_world(*cam_to_world);//07/28/2017
	camera.fill_inverse_calibration(*inverse_calibration, width, height);
}

void
TextureView::generate_validity_mask(void) {
    assert(image != NULL);
	validity_mask.resize(width * height, true);
    mve::ByteImage::Ptr checked = mve::ByteImage::create(width, height, 1);

    std::list<math::Vec2i> queue;

    /* Start from the corners. */
    queue.push_back(math::Vec2i(0,0));
    checked->at(0, 0, 0) = 255;
    queue.push_back(math::Vec2i(0, height - 1));
    checked->at(0, height - 1, 0) = 255;
    queue.push_back(math::Vec2i(width - 1, 0));
    checked->at(width - 1, 0, 0) = 255;
    queue.push_back(math::Vec2i(width - 1, height - 1));
    checked->at(width - 1, height - 1, 0) = 255;

    while (!queue.empty()) {
        math::Vec2i pixel = queue.front();
        queue.pop_front();

        int const x = pixel[0];
        int const y = pixel[1];

        int sum = 0;
        for (int c = 0; c < image->channels(); ++c) {
            sum += image->at(x, y, c);
        }

        if (sum == 0) {
            validity_mask[x + y * width] = false;

            std::vector<math::Vec2i> neighbours;
            neighbours.push_back(math::Vec2i(x + 1, y));
            neighbours.push_back(math::Vec2i(x, y + 1));
            neighbours.push_back(math::Vec2i(x - 1, y));
            neighbours.push_back(math::Vec2i(x, y - 1));

            for (std::size_t i = 0; i < neighbours.size(); ++i) {
                math::Vec2i npixel = neighbours[i];
                int const nx = npixel[0];
                int const ny = npixel[1];
                if (0 <= nx && nx < width && 0 <= ny && ny < height) {
                    if (checked->at(nx, ny, 0) == 0) {
                        queue.push_front(npixel);
                        checked->at(nx, ny, 0) = 255;
                    }
                }
            }
        }
    }
}

void
TextureView::load_image(void) {
    if(image != NULL) return;
	image = mve::image::load_file(image_file);
}

void
TextureView::generate_gradient_magnitude(void) {
    assert(image != NULL);
    mve::ByteImage::Ptr bw = mve::image::desaturate<std::uint8_t>(image, mve::image::DESATURATE_LUMINANCE);
    gradient_magnitude = mve::image::sobel_edge<std::uint8_t>(bw);
}

void
TextureView::erode_validity_mask(void) {
    std::vector<bool> eroded_validity_mask(validity_mask);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (x == 0 || x == width - 1 || y == 0 || y == height - 1) {
                validity_mask[x + y * width] = false;
                continue;
            }

            if (validity_mask[x + y * width]) continue;
            for (int j = -1; j <= 1; ++j) {
                for (int i = -1; i <= 1; ++i) {
                    int const nx = x + i;
                    int const ny = y + j;
                    eroded_validity_mask[nx + ny * width] = false;
                }
            }
        }
    }

    validity_mask.swap(eroded_validity_mask);
}

void
TextureView::get_face_info(math::Vec3f const & v1, math::Vec3f const & v2,
    math::Vec3f const & v3, FaceProjectionInfo * face_info, Settings const & settings) const {

    assert(image != NULL);
    assert(settings.data_term != DATA_TERM_GMI || gradient_magnitude != NULL);

    math::Vec2f p1 = get_pixel_coords(v1);
    math::Vec2f p2 = get_pixel_coords(v2);
    math::Vec2f p3 = get_pixel_coords(v3);

    assert(valid_pixel(p1) && valid_pixel(p2) && valid_pixel(p3));

    Tri tri(p1, p2, p3);
    float area = tri.get_area();

    if (area < std::numeric_limits<float>::epsilon()) {
        face_info->quality = 0.0f;
        return;
    }

    std::size_t num_samples = 0;
    math::Vec3d colors(0.0);
    double gmi = 0.0;

    bool sampling_necessary = settings.data_term != DATA_TERM_AREA || settings.outlier_removal != OUTLIER_REMOVAL_NONE;

    if (sampling_necessary && area > 0.5f) {
        /* Sort pixels in ascending order of y */
        while (true)
            if(p1[1] <= p2[1])
                if(p2[1] <= p3[1]) break;
                else std::swap(p2, p3);
            else std::swap(p1, p2);

        /* Calculate line equations. */
        float const m1 = (p1[1] - p3[1]) / (p1[0] - p3[0]);
        float const b1 = p1[1] - m1 * p1[0];

        /* area != 0.0f => m1 != 0.0f. */
        float const m2 = (p1[1] - p2[1]) / (p1[0] - p2[0]);
        float const b2 = p1[1] - m2 * p1[0];

        float const m3 = (p2[1] - p3[1]) / (p2[0] - p3[0]);
        float const b3 = p2[1] - m3 * p2[0];

        bool fast_sampling_possible = std::isfinite(m1) && m2 != 0.0f && std::isfinite(m2) && m3 != 0.0f && std::isfinite(m3);

        Rect<float> aabb = tri.get_aabb();
        for (int y = std::floor(aabb.min_y); y < std::ceil(aabb.max_y); ++y) {
            float min_x = aabb.min_x - 0.5f;
            float max_x = aabb.max_x + 0.5f;

            if (fast_sampling_possible) {
                float const cy = static_cast<float>(y) + 0.5f;

                min_x = (cy - b1) / m1;
                if (cy <= p2[1]) max_x = (cy - b2) / m2;
                else max_x = (cy - b3) / m3;

                if (min_x >= max_x) std::swap(min_x, max_x);

                if (min_x < aabb.min_x || min_x > aabb.max_x) continue;
                if (max_x < aabb.min_x || max_x > aabb.max_x) continue;
            }

            for (int x = std::floor(min_x + 0.5f); x < std::ceil(max_x - 0.5f); ++x) {
                math::Vec3d color;

                const float cx = static_cast<float>(x) + 0.5f;
                const float cy = static_cast<float>(y) + 0.5f;
                if (!fast_sampling_possible && !tri.inside(cx, cy)) continue;

                if (settings.outlier_removal != OUTLIER_REMOVAL_NONE) {
                    for (std::size_t i = 0; i < 3; i++){
                         color[i] = static_cast<double>(image->at(x, y, i)) / 255.0;
                    }
                    colors += color;
                }

                if (settings.data_term == DATA_TERM_GMI) {
                    gmi += static_cast<double>(gradient_magnitude->at(x, y, 0)) / 255.0;
                }
                ++num_samples;
            }
        }
    }

    if (settings.data_term == DATA_TERM_GMI) {
        if (num_samples > 0) {
            gmi = (gmi / num_samples) * area;
        } else {
            double gmv1 = static_cast<double>(gradient_magnitude->linear_at(p1[0], p1[1], 0)) / 255.0;
            double gmv2 = static_cast<double>(gradient_magnitude->linear_at(p2[0], p2[1], 0)) / 255.0;
            double gmv3 = static_cast<double>(gradient_magnitude->linear_at(p3[0], p3[1], 0)) / 255.0;
            gmi = ((gmv1 + gmv2 + gmv3) / 3.0) * area;
        }
    }

    if (settings.outlier_removal != OUTLIER_REMOVAL_NONE) {
        if (num_samples > 0) {
            face_info->mean_color = colors / num_samples;
        } else {
            math::Vec3d c1, c2, c3;
            for (std::size_t i = 0; i < 3; ++i) {
                 c1[i] = static_cast<double>(image->linear_at(p1[0], p1[1], i)) / 255.0;
                 c2[i] = static_cast<double>(image->linear_at(p2[0], p2[1], i)) / 255.0;
                 c3[i] = static_cast<double>(image->linear_at(p3[0], p3[1], i)) / 255.0;
            }
            face_info->mean_color = ((c1 + c2 + c3) / 3.0);
        }
    }

    switch (settings.data_term) {
        case DATA_TERM_AREA: face_info->quality = area; break;
        case DATA_TERM_GMI:  face_info->quality = gmi; break;
    }
}

bool
TextureView::valid_pixel(math::Vec2f pixel) const {
    float const x = pixel[0];
    float const y = pixel[1];
	/* The center of a pixel is in the middle. */
    bool valid = (x >= 0.0f && x < static_cast<float>(width - 1)
        && y >= 0.0f && y < static_cast<float>(height - 1));

    if (valid && validity_mask.size() == static_cast<std::size_t>(width * height)) {
        /* Only pixel which can be correctly interpolated are valid. */
        float cx = std::max(0.0f, std::min(static_cast<float>(width - 1), x));
        float cy = std::max(0.0f, std::min(static_cast<float>(height - 1), y));
        int const floor_x = static_cast<int>(cx);
        int const floor_y = static_cast<int>(cy);
        int const floor_xp1 = std::min(floor_x + 1, width - 1);
        int const floor_yp1 = std::min(floor_y + 1, height - 1);

        /* We screw up if weights would be zero
         * e.g. we lose valid pixel in the border of images... */

        valid = validity_mask[floor_x + floor_y * width] &&
                validity_mask[floor_x + floor_yp1 * width] &&
                validity_mask[floor_xp1 + floor_y * width] &&
                validity_mask[floor_xp1 + floor_yp1 * width];
    }

    return valid;
}

void
TextureView::export_triangle(math::Vec3f v1, math::Vec3f v2, math::Vec3f v3,
    std::string const & filename) const {
    assert(image != NULL);
    math::Vec2f p1 = get_pixel_coords(v1);
    math::Vec2f p2 = get_pixel_coords(v2);
    math::Vec2f p3 = get_pixel_coords(v3);

    assert(valid_pixel(p1) && valid_pixel(p2) && valid_pixel(p3));

    Tri tri(p1, p2, p3);

    Rect<float> aabb = tri.get_aabb();
    const int width = ceil(aabb.width());
    const int height = ceil(aabb.height());
    const int left = floor(aabb.min_x);
    const int top = floor(aabb.max_y);

    assert(width > 0 && height > 0);
    mve::image::save_png_file(mve::image::crop(image, width, height, left, top,
        *math::Vec3uc(255, 0, 255)), filename);
}

void
TextureView::export_validity_mask(std::string const & filename) const {
    assert(validity_mask.size() == static_cast<std::size_t>(width * height));
    mve::ByteImage::Ptr img = mve::ByteImage::create(width, height, 1);
    for (std::size_t i = 0; i < validity_mask.size(); ++i) {
        img->at(static_cast<int>(i), 0) = validity_mask[i] ? 255 : 0;
    }
    mve::image::save_png_file(img, filename);
}
/*
description:
perform sampling of face and update weight map.
created at 08/03/2017
*/
void TextureView::face_to_samples(math::Vec3f* v, math::Vec3f &face_normal, size_t t)
{
	math::Vec2f pix_coords[3];
	for (int i = 0; i < 3; i++)
	{
		pix_coords[i] = get_pixel_coords(v[i]);
		if (pix_coords[i][0] > width - 1 || pix_coords[i][1] > height - 1)
			return;
	}
	math::Vec2f vec2f[2];
	vec2f[0] = pix_coords[1] - pix_coords[0];
	vec2f[1] = pix_coords[2] - pix_coords[0];
	int count[2];
	count[0] = vec2f[0].norm() + 0.5f;
	count[1] = vec2f[1].norm() + 0.5f;
	math::Vec3f e[2];
	e[0] = v[1] - v[0];
	e[1] = v[2] - v[0];
	for (int j = 0; j < count[1]; j++)
	{
		for (int i = 0; i < count[0]; i++)
		{
			float a = (float)i / count[0];
			float b = (float)j / count[1];
			if (a + b > 1.0f)
				continue;
			Sample_Point sample;
			sample.world_coord = v[0] + a*e[0] + b*e[1];
			sample.pix_coord = get_pixel_coords(sample.world_coord);
			bool valid = (sample.pix_coord[0] >= 0.0f && sample.pix_coord[0] < static_cast<float>(width - 1)
				&& sample.pix_coord[1] >= 0.0f && sample.pix_coord[1] < static_cast<float>(height - 1));
			if (valid == false)
				continue;
			if (CV_IMAGE_ELEM(weight_map, float, (int)sample.pix_coord[1], (int)sample.pix_coord[0]) == 0.0f)
			{
				sample.t = t;
				math::Vec3f face_to_view = pos - sample.world_coord;
				float dist = face_to_view.norm();
				face_to_view.normalize();
				float w = face_to_view.dot(face_normal) / dist;
				CV_IMAGE_ELEM(weight_map, float, (int)sample.pix_coord[1], (int)sample.pix_coord[0]) = w;
				samples.push_back(sample);
			}
		}
	}
}
/*
perform sampling of faces and generate weight map.
created at 08/03/2017
*/
void 
TextureView::generate_samples(mve::TriangleMesh::ConstPtr mesh)
{
	std::vector<unsigned int> const & faces = mesh->get_faces();
	std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
	mve::TriangleMesh::NormalList const & face_normals = mesh->get_face_normals();
	for (size_t t = 0; t < face_project_infos->size(); t++)
	{
		for (int i = 0; i < face_project_infos->at(t).view_ids.size(); i++)
		{
			if (id == face_project_infos->at(t).view_ids[i])
			{
				math::Vec3f vec3f[3];
				vec3f[0] = vertices[faces[3 * t] + 0];
				vec3f[1] = vertices[faces[3 * t] + 1];
				vec3f[2] = vertices[faces[3 * t] + 2];
				math::Vec3f normal = face_normals.at(t);
				face_to_samples(vec3f, normal, t);
				break;
			}
		}
	}
	original_weight_map = cvCloneImage(weight_map);
	std::cout << "sample-count:" << samples.size() << std::endl;
}
/*
created 08/03/2017
*/
bool TextureView::get_pixel_coord_sample(Sample_Point &sample, math::Vec2f &pix_coord)
{
	size_t t = sample.t;
	for (int i = 0; i < face_project_infos->at(t).view_ids.size(); i++)
	{
		if (id == face_project_infos->at(t).view_ids[i])
		{
			pix_coord = get_pixel_coords(sample.world_coord);
			return true;
		}
	}
	return false;
}
/*
description:
reconstruct texture_map by blending target images from multiple views
created 08/03/2017
*/
void TextureView::update_texture_map(std::vector<TextureView> &texture_views)
{
	//std::cout << "start update texture map:" << id << std::endl;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float w = get_weight_val(math::Vec2f(x, y));
			math::Vec3f color = w*get_target_pixel_val(math::Vec2f(x, y));
			float w_sum = w;
			for (int i = 0; i < texture_views.size(); i++)
			{
				if (i == id)
					continue;
				math::Vec2f pix_coord;
				pix_coord[0] = CV_IMAGE_ELEM(pixel_maps[i], float, y, 2 * x);
				pix_coord[1] = CV_IMAGE_ELEM(pixel_maps[i], float, y, 2 * x + 1);
				if (pix_coord[0] < 0 || pix_coord[0] > width - 1)
				{
					std::cout << "error::x:" << pix_coord[0] << std::endl;
					continue;
				}
				if (pix_coord[1] < 0 || pix_coord[1] > height - 1)
				{
					std::cout << "error::y:" << pix_coord[1] << std::endl;
					continue;
				}
				w = texture_views[i].get_weight_val(pix_coord);
				color += w*texture_views[i].get_target_pixel_val(pix_coord);
				w_sum += w;
			}
			CV_IMAGE_ELEM(texture_map, unsigned char, y, 3 * x) = color[0] / w_sum;
			CV_IMAGE_ELEM(texture_map, unsigned char, y, 3 * x + 1) = color[1] / w_sum;
			CV_IMAGE_ELEM(texture_map, unsigned char, y, 3 * x + 2) = color[2] / w_sum;
		}
	}
	/*for (size_t i = 0; i < samples.size(); i++)
	{
		float cur_weight = 0.0f;//get_weight_val(cur_pix_coord);
		math::Vec3f cur_pixel(0.0f,0.0f,0.0f);// = cur_weight*get_target_pixel_val(cur_pix_coord);
		size_t t = samples[i].t;
		for (int j = 0; j < face_project_infos->at(t).view_ids.size(); j++)
		{
			int view_id = face_project_infos->at(t).view_ids[j];
			math::Vec2f pix_coord = texture_views[view_id].get_pixel_coords(samples[i].world_coord);
			bool valid = (pix_coord[0] >= 0.0f && pix_coord[0] < static_cast<float>(width - 1)
				&& pix_coord[1] >= 0.0f && pix_coord[1] < static_cast<float>(height - 1));
			if (valid == false)
				continue;
			math::Vec3f pixel = texture_views[view_id].get_target_pixel_val(pix_coord);
			float w = texture_views[view_id].get_weight_val(pix_coord);
			cur_pixel += w*pixel;
			cur_weight += w;
		}
		math::Vec2f cur_pix_coord = get_pixel_coords(samples[i].world_coord);
		unsigned char* dst = &CV_IMAGE_ELEM(texture_map, unsigned char, (int)cur_pix_coord[1], 3 * (int)cur_pix_coord[0]);
		*dst = cur_pixel[0]/cur_weight + 0.5f;	dst++;
		*dst = cur_pixel[1]/cur_weight + 0.5f;	dst++;
		*dst = cur_pixel[2]/cur_weight + 0.5f;
	}
	//std::cout << "compelete update texture map:" << id << std::endl;*/
}
/*
created 08/03/2017
*/
math::Vec3f TextureView::get_target_pixel_val(math::Vec2f &pix_coord)
{
	unsigned char* ptr = &CV_IMAGE_ELEM(target_img, unsigned char, (int)pix_coord[1], 3*(int)pix_coord[0]);
	math::Vec3f color;
	color[0] = *ptr;	ptr++;
	color[1] = *ptr;	ptr++;
	color[2] = *ptr;
	return color;
}
/*
created 08/03/2017
*/
float TextureView::get_weight_val(math::Vec2f &pix_coord)
{
	return CV_IMAGE_ELEM(weight_map, float, (int)pix_coord[1], (int)pix_coord[0]);
}
/*
created 08/03/2017
*/
void TextureView::initialize(int patchWidth, int patchHeight)
{
	patch_width = patchWidth;
	patch_height = patchHeight;
	alpha = 2.0f;
	lamda = 0.1f;
	primary_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	origin_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	target_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	texture_map = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	weight_map = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
	cvZero(weight_map);
	/*sample_map = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 4);
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			CV_IMAGE_ELEM(sample_map, float, y, 4 * x) = -1.0f;
		}
	}*/
	if (image == NULL)
	{
		load_image();
	}
	unsigned char* src_ptr = (unsigned char*)image->get_byte_pointer();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			unsigned r, g, b;
			r = *src_ptr; src_ptr++;
			g = *src_ptr; src_ptr++;
			b = *src_ptr; src_ptr++;
			unsigned char* primary_ptr = &CV_IMAGE_ELEM(primary_img, unsigned char, y, 3 * x);
			unsigned char* origin_ptr = &CV_IMAGE_ELEM(origin_img, unsigned char, y, 3 * x);
			unsigned char* targ_ptr = &CV_IMAGE_ELEM(target_img, unsigned char, y, 3 * x);
			unsigned char* texture_ptr = &CV_IMAGE_ELEM(target_img, unsigned char, y, 3 * x);
			*primary_ptr = *origin_ptr = *targ_ptr = *texture_ptr = b;
			primary_ptr++;  targ_ptr++; texture_ptr++; origin_ptr++;
			*primary_ptr = *origin_ptr = *targ_ptr = *texture_ptr = g;
			primary_ptr++;  targ_ptr++; texture_ptr++; origin_ptr++;
			*primary_ptr = *origin_ptr = *targ_ptr = *texture_ptr = r;
		}
	}
	release_image();
}
/*reconstruct target image aligned*/
/*
description:
take a sub image from image
created at 07/25/2017
*/
void TextureView::get_sub_mat(int x0, int y0, int x1, int y1, IplImage* src_img, cv::Mat &mat)
{
	int cols = x1 - x0;
	int rows = y1 - y0;
	for (int y = 0; y < rows; y++)
	{
		for (int x = 0; x < cols; x++)
		{
			int sx = x0 + x;
			int sy = y0 + y;
			unsigned char* src_ptr = &CV_IMAGE_ELEM(src_img, unsigned char, sy, 3 * sx);
			mat.at<cv::Vec3f>(x, y)[0] = *src_ptr; src_ptr++;
			mat.at<cv::Vec3f>(x, y)[1] = *src_ptr; src_ptr++;
			mat.at<cv::Vec3f>(x, y)[2] = *src_ptr;
		}
	}
}
/*
description:
compute the completeness of source image and target image
created at 07/26/2017
modified at 08/02/2017
*/
IplImage* TextureView::compute_compeleteness()
{
	IplImage* compele_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 4);
	cvZero(compele_img);
	float a = (float)1.0f / (patch_width*patch_height);
	cv::Mat result = cv::Mat::zeros(patch_width, patch_height, CV_32FC1);
	cv::Mat patch = cv::Mat::zeros(patch_width, patch_height, CV_32FC3);
	cv::Mat dst = cv::Mat::zeros(2*patch_width, 2*patch_height, CV_32FC3);
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	for (int y = patch_height; y < height - 2 * patch_height; y++)
	{
		for (int x = patch_width; x < width - 2 * patch_width; x++)
		{
			get_sub_mat(x, y, x+patch_width, y+patch_height, origin_img, patch);
			int dst_x0 = x - 0.5f*patch_width;
			int dst_y0 = y - 0.5f*patch_height;
			int dst_x1 = dst_x0 + 2.0f*patch_width;
			int dst_y1 = dst_y0 + 2.0f*patch_height;
			get_sub_mat(dst_x0, dst_y0, dst_x1, dst_y1, target_img, dst);
			cv::matchTemplate(dst, patch, result, CV_TM_SQDIFF);
			//cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
			cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
			minLoc.x = dst_x0 + minLoc.x;
			minLoc.y = dst_y0 + minLoc.y;
			for (int iy = 0; iy < patch.rows; iy++)
			{
				for (int ix = 0; ix < patch.cols; ix++)
				{
					int cx = minLoc.x + ix;
					int cy = minLoc.y + iy;
					CV_IMAGE_ELEM(compele_img, float, cy, 4 * cx + 0) += (float)CV_IMAGE_ELEM(origin_img, unsigned char, y+iy, 3 * (x+ix))*a;
					CV_IMAGE_ELEM(compele_img, float, cy, 4 * cx + 1) += (float)CV_IMAGE_ELEM(origin_img, unsigned char, y+iy, 3 * (x+ix)+1)*a;
					CV_IMAGE_ELEM(compele_img, float, cy, 4 * cx + 2) += (float)CV_IMAGE_ELEM(origin_img, unsigned char, y+iy, 3 * (x+ix)+2)*a;
					CV_IMAGE_ELEM(compele_img, float, cy, 4 * cx + 3) += (float)a;
				}
			}
			result.release();
		}
	}
	//std::cout << "succeed compeleteness image viewid:" << id << std::endl;*/
	return compele_img;
}
/*
description:
compute the conherence of source image and target image
created at 07/26/2017
modified at 08/02/2017
*/
IplImage* TextureView::compute_coherence()
{
	IplImage* coherence_img = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 4);
	cvZero(coherence_img);
	float L = patch_width*patch_height;
	float a = alpha / L;
	cv::Mat result = cv::Mat::zeros(patch_width, patch_height, CV_32FC1);
	cv::Mat patch = cv::Mat::zeros(patch_width, patch_height, CV_32FC3);
	cv::Mat dst = cv::Mat::zeros(2 * patch_width, 2 * patch_height, CV_32FC3);
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	for (int y = patch_height; y < height - 2 * patch_height; y++) {
		for (int x = patch_width; x < width - 2 * patch_width; x++)
		{
			get_sub_mat(x, y, x + patch_width, y + patch_height, target_img, patch);
			int dst_x0 = x - 0.5f*patch_width;
			int dst_y0 = y - 0.5f*patch_height;
			int x1 = dst_x0 + 2.0f*patch_width;
			int y1 = dst_y0 + 2.0f*patch_height;
			get_sub_mat(dst_x0, dst_y0, x1, y1, origin_img, dst);
			cv::matchTemplate(dst, patch, result, CV_TM_SQDIFF);
			//cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
			cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
			for (int iy = 0; iy < patch_height; iy++)
			{
				for (int ix = 0; ix < patch_width; ix++)
				{
					int sx = dst_x0 + minLoc.x + ix;
					int sy = dst_y0 + minLoc.y + iy;
					CV_IMAGE_ELEM(coherence_img, float, y + iy, 4 * (x + ix) + 0) += (float)CV_IMAGE_ELEM(origin_img, unsigned char, sy, 3 * sx)*a;
					CV_IMAGE_ELEM(coherence_img, float, y + iy, 4 * (x + ix) + 1) += (float)CV_IMAGE_ELEM(origin_img, unsigned char, sy, 3 * sx + 1)*a;
					CV_IMAGE_ELEM(coherence_img, float, y + iy, 4 * (x + ix) + 2) += (float)CV_IMAGE_ELEM(origin_img, unsigned char, sy, 3 * sx + 2)*a;
					CV_IMAGE_ELEM(coherence_img, float, y + iy, 4 * (x + ix) + 3) += a;
				}
			}
			result.release();
		}
	}
	return coherence_img;
}
/*
created 08/03/2017
*/
math::Vec3f TextureView::get_pixel_val_texture(math::Vec2f &pix_coord)
{
	unsigned char* ptr = &CV_IMAGE_ELEM(texture_map, unsigned char, (int)pix_coord[1], 3 * (int)pix_coord[0]);
	math::Vec3f color;
	color[0] = *ptr;	ptr++;
	color[1] = *ptr;	ptr++;
	color[2] = *ptr;
	return color;
}
/*
description:
blend texture maps from multiple views by projecting pixels of current view into multiple views.
pixels of current view are projected into multiple view by maping the samples of current view into the pixels of multiple views.
created at 08/03/2017
*/
IplImage* TextureView::blend_texture_maps(std::vector<TextureView> &texture_views)
{
	IplImage* blend_texture_map = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	cvZero(blend_texture_map);
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float w = get_weight_val(math::Vec2f(x, y));
			math::Vec3f color = w*get_pixel_val_texture(math::Vec2f(x, y));
			int count = 0;
			for (int i = 0; i < texture_views.size(); i++)
			{
				if (i == id)
					continue;
				math::Vec2f pix_coord;
				pix_coord[0] = CV_IMAGE_ELEM(pixel_maps[i], float, y, 2 * x);
				pix_coord[1] = CV_IMAGE_ELEM(pixel_maps[i], float, y, 2 * x+1);
				if (pix_coord[0] > width - 1 || pix_coord[1] > height - 1)
					continue;
				if (pix_coord[0] < 0 || pix_coord[1] < 0)
					continue;
				color += texture_views[i].get_pixel_val_texture(pix_coord);
				count++;
			}
			CV_IMAGE_ELEM(blend_texture_map, float, y, 3 * x) = lamda*w*color[0] / count;
			CV_IMAGE_ELEM(blend_texture_map, float, y, 3 * x+1) = lamda*w*color[1] / count;
			CV_IMAGE_ELEM(blend_texture_map, float, y, 3 * x+2) = lamda*w*color[2] / count;
		}
	}
	return blend_texture_map;
	/*IplImage* blend_texture_map = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
	cvZero(blend_texture_map);
	for (size_t s = 0; s < samples.size(); s++)
	{
		Face_Project_Info &face_info = face_project_infos->at(samples[s].t);
		int view_count = face_info.view_ids.size();
		math::Vec3f color(0.0f, 0.0f, 0.0f);
		int visual_count = 0;
		for (int i = 0; i < view_count; i++)
		{
			int view_id = face_info.view_ids[i];
			math::Vec2f pix_coord = texture_views[view_id].get_pixel_coords(samples[s].world_coord);
			bool valid = (pix_coord[0] >= 0.0f && pix_coord[0] < static_cast<float>(width - 1)
				&& pix_coord[1] >= 0.0f && pix_coord[1] < static_cast<float>(height - 1));
			if (valid == false)
				continue;
			color += texture_views[view_id].get_pixel_val_texture(pix_coord);
			visual_count++;
		}
		math::Vec2f pix_coord = get_pixel_coords(samples[s].world_coord);
		int x = pix_coord[0];
		int y = pix_coord[1];
		if (x > width - 1 || y > height - 1)
			continue;
		color = lamda*get_weight_val(pix_coord)*color/visual_count;
		float* dst = &CV_IMAGE_ELEM(blend_texture_map, float, y, 3*x);
		*dst = color[0];	dst++;
		*dst = color[1];	dst++;
		*dst = color[2];
	}
	return blend_texture_map;*/
}
/*
description:
update target image
created at 07/26/2017
modified at 07/31/2017
modified at 08/03/2017
*/
void TextureView::update_target_image(std::vector<TextureView> &texture_views)
{
	IplImage* compele_img = generate_compeletness(origin_img, target_img, patch_width, 5);//compute_compeleteness();
	IplImage* coherence_img = generate_coherence(origin_img, target_img, patch_width, 5);//compute_coherence();
	//std::cout << "start blend texture maps:" << id << std::endl;
	IplImage* blend_texture_map = blend_texture_maps(texture_views);
	//std::cout << "pass blend texture maps:" << id << std::endl;
	float lamda = 0.1f;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float* compele_ptr = &CV_IMAGE_ELEM(compele_img, float, y, 4*x);
			float* coherence_ptr = &CV_IMAGE_ELEM(coherence_img, float, y, 4*x);
			float* blend_ptr = &CV_IMAGE_ELEM(blend_texture_map, float, y, 3*x);
			float b = *compele_ptr + *coherence_ptr +*blend_ptr;
			compele_ptr++;	coherence_ptr++;  blend_ptr++;
			float g = *compele_ptr + *coherence_ptr +*blend_ptr*lamda;
			compele_ptr++;	coherence_ptr++;  blend_ptr++;
			float r = *compele_ptr + *coherence_ptr +*blend_ptr*lamda;
			compele_ptr++;	coherence_ptr++;
			float w1 = lamda*CV_IMAGE_ELEM(weight_map, float, y, x);
			float w2 = *compele_ptr + *coherence_ptr +w1;
			if (w2 == 0.0f)
				w2 = 1.0f;
			unsigned char* dst_ptr = &CV_IMAGE_ELEM(target_img, unsigned char, y, 3*x);
			*dst_ptr = (unsigned char)(b / w2 + 0.5f);	dst_ptr++;
			*dst_ptr = (unsigned char)(g / w2 + 0.5f);	dst_ptr++;
			*dst_ptr = (unsigned char)(r / w2 + 0.5f);
		}
	}
	//std::cout << "pass update target image" << id << std::endl;
	cvReleaseImage(&compele_img);
	cvReleaseImage(&coherence_img);
	cvReleaseImage(&blend_texture_map);
}
/*
created at 08/03/2017
*/
void TextureView::set_face_project_info(std::vector<Face_Project_Info>* pFace_Project_Infos)
{
	face_project_infos = pFace_Project_Infos;
}
/*
created at 08/03/2017
*/
void TextureView::save_target_image()
{
	cvSaveImage(image_file.c_str(), target_img);
	cvReleaseImage(&target_img);
	cvReleaseImage(&weight_map);
	cvReleaseImage(&texture_map);
}
/*
created at 08/03/2017
*/
void TextureView::save_texture_map()
{
	cvSaveImage(image_file.c_str(), texture_map);
	cvReleaseImage(&target_img);
	cvReleaseImage(&weight_map);
	cvReleaseImage(&texture_map);
}
/*
created at 08/08/2017
*/
IplImage* TextureView::down_sampling_img(IplImage* src_img, int nType, int nChannel)
{
	IplImage* down_img = cvCreateImage(cvSize(src_img->width/2, src_img->height/2), nType, nChannel);
	cvZero(down_img);
	cvPyrDown(src_img, down_img, IPL_GAUSSIAN_5x5);
	cvReleaseImage(&src_img);
	return down_img;
}
/*
created at 08/09/2017
*/
void TextureView::down_sampling(int down_res)
{
	height = down_res;
	width = rate*height;
	float scale = (float)height/original_height;
	scaling_projection(scale);
	IplImage* origin_tmp = down_sampling_img(origin_img, width, height, IPL_DEPTH_8U, 3);
	cvReleaseImage(&origin_img);
	origin_img = origin_tmp;
	IplImage* target_tmp = down_sampling_img(target_img, width, height, IPL_DEPTH_8U, 3);
	cvReleaseImage(&target_img);
	target_img = target_tmp;
	IplImage* texture_tmp = down_sampling_img(texture_map, width, height, IPL_DEPTH_8U, 3);
	cvReleaseImage(&texture_map);
	texture_map = texture_tmp;
	IplImage* weight_tmp = down_sampling_img(weight_map, width, height, IPL_DEPTH_32F, 1);
	cvReleaseImage(&weight_map);
	weight_map = weight_tmp;
	//init_nnfs();
}
/*
created at 08/09/2017
*/
IplImage* TextureView::up_sampling_img(IplImage* src_img, int up_width, int up_height)
{
	IplImage* up_img = cvCreateImage(cvSize(up_width, up_height), IPL_DEPTH_8U, 3);
	cvZero(up_img);
	for (int y = 0; y < up_height; y++)
	{
		for (int x = 0; x < up_width; x++)
		{
			unsigned char* up_ptr = &CV_IMAGE_ELEM(up_img, unsigned char, y, 3 * x);
			int sx = src_img->width*(float)x / up_width;
			int sy = src_img->height*(float)y / up_height;
			unsigned char* src_ptr = &CV_IMAGE_ELEM(src_img, unsigned char, sy, 3 * sx);
			*up_ptr = *src_ptr;
			up_ptr++; src_ptr++;
			*up_ptr = *src_ptr;
			up_ptr++; src_ptr++;
			*up_ptr = *src_ptr;
		}
	}
	return up_img;
}
/*
created at 08/09/2017
modified at 08/10/2017
*/
void TextureView::up_sampling(int up_res)
{
	height = up_res;
	width = rate*height;
	float scale = (float)height / original_height;
	scaling_projection(scale);
	std::cout << "pass scaling:" << id << std::endl;
	cvReleaseImage(&origin_img);
	cvReleaseImage(&weight_map);
	std::cout << "pass release:" << id << std::endl;
	origin_img = cvCloneImage(primary_img);
	weight_map = cvCloneImage(original_weight_map);
	IplImage* origin_tmp = down_sampling_img(origin_img, width, height, IPL_DEPTH_8U, 3);
	cvReleaseImage(&origin_img);
	std::cout << "pass release origin_img:" << id << std::endl;
	origin_img = origin_tmp;
	IplImage* weight_tmp = down_sampling_img(weight_map, width, height, IPL_DEPTH_32F, 1);
	cvReleaseImage(&weight_map);
	std::cout << "pass release weight_map:" << id << std::endl;
	weight_map = weight_tmp;
	IplImage* target_tmp = up_sampling_img(target_img, width, height);
	cvReleaseImage(&target_img);
	target_img = target_tmp;
	std::cout << "pass reset target_img:" << id << std::endl;
	IplImage* texture_tmp = up_sampling_img(texture_map, width, height);
	std::cout << "pass upsampling texture_map:" << id << std::endl;
	cvReleaseImage(&texture_map);
	std::cout << "pass release texture_map:" << id << std::endl;
	texture_map = texture_tmp;
	std::cout << "pass upsampling:" << id << std::endl;
}
/*
created at 08/08/2017
*/
void TextureView::scaling_projection(float s)
{
	for (int i = 0; i < 6; i++)
	{
		projection[i] = s*original_projection[i];
	}
}
/*
created at 08/12/2017
*/
void TextureView::reset_projection()
{
	for (int i = 0; i < 9; i++)
	{
		projection[i] = original_projection[i];
	}
}
/*
created at 08/10/2017
*/
IplImage* TextureView::down_sampling_img(IplImage* src_img, int down_width, int down_height, int nType, int nChannel)
{
	IplImage* down_img = cvCreateImage(cvSize(down_width, down_height), nType, nChannel);
	IplImage* sum_img = cvCreateImage(cvSize(down_width, down_height), IPL_DEPTH_32F, nChannel);
	IplImage* count_img = cvCreateImage(cvSize(down_width, down_height), IPL_DEPTH_16U, 1);
	cvZero(down_img);
	cvZero(sum_img);
	cvZero(count_img);
	if (nType == IPL_DEPTH_8U)
	{
		for (int y = 0; y < src_img->height; y++)
		{
			for (int x = 0; x < src_img->width; x++)
			{
				int down_x = down_width*(float)x / src_img->width;
				int down_y = down_height*(float)y / src_img->height;
				float* sum_ptr = &CV_IMAGE_ELEM(sum_img, float, down_y, 3 * down_x);
				unsigned char* src_ptr = &CV_IMAGE_ELEM(src_img, unsigned char, y, 3 * x);
				CV_IMAGE_ELEM(count_img, uint16_t, down_y, down_x) += 1;
				*sum_ptr += *src_ptr; sum_ptr++; src_ptr++;
				*sum_ptr += *src_ptr; sum_ptr++; src_ptr++;
				*sum_ptr += *src_ptr; 
			}
		}
		for (int y = 0; y < down_height; y++)
		{
			for (int x = 0; x < down_width; x++)
			{
				unsigned char* down_ptr = &CV_IMAGE_ELEM(down_img, unsigned char, y, 3 * x);
				float* sum_ptr = &CV_IMAGE_ELEM(sum_img, float, y, 3 * x);
				uint16_t count = CV_IMAGE_ELEM(count_img, uint16_t, y, x);
				if (count > 0.0f)
				{
					*down_ptr = sum_ptr[0] / count; down_ptr++;
					*down_ptr = sum_ptr[1] / count; down_ptr++;
					*down_ptr = sum_ptr[2] / count;
				}
			}
		}
	}
	if (nType == IPL_DEPTH_32F)
	{
		for (int y = 0; y < src_img->height; y++)
		{
			for (int x = 0; x < src_img->width; x++)
			{
				int down_x = down_width*(float)x / src_img->width;
				int down_y = down_height*(float)y / src_img->height;
				CV_IMAGE_ELEM(sum_img, float, down_y, down_x) += CV_IMAGE_ELEM(src_img, float, y, x);
				CV_IMAGE_ELEM(count_img, uint16_t, down_y, down_x) += 1;
			}
		}
		for (int y = 0; y < down_height; y++)
		{
			for (int x = 0; x < down_width; x++)
			{
				CV_IMAGE_ELEM(down_img, float, y, x) = CV_IMAGE_ELEM(sum_img, float, y, x) / CV_IMAGE_ELEM(count_img, uint16_t, y, x);
			}
		}
	}
	cvReleaseImage(&sum_img);
	cvReleaseImage(&count_img);
	return down_img;
}
/*
created 08/22/2017
*/
math::Vec3f TextureView::pixel_to_world(int x, int y)
{
	math::Vec3f uv_coord;
	uv_coord[0] = x - projection[2];
	uv_coord[1] = y - projection[5];
	uv_coord[2] = projection[0];//focal_length_x
	//math::Vec3f cam_coord = inverse_calibration.mult(uv_coord);
	math::Vec4f cam_coord_h(uv_coord[0], uv_coord[1], uv_coord[2], 1.0f);
	math::Vec4f world_coord = cam_to_world.mult(cam_coord_h);
	return math::Vec3f(world_coord[0], world_coord[1], world_coord[2]);
}
/*
description:
compute the weight map of current view and pixel maps from current view to other views
created at 08/22/2017
*/
void TextureView::generate_pixel_maps(mve::TriangleMesh::ConstPtr mesh, std::vector<TextureView> &texture_views)
{
	pixel_maps.resize(texture_views.size());
	for (int i = 0; i < texture_views.size(); i++)
	{
		if (i == id)
		{
			pixel_maps[i] = NULL;
			continue;
		}
		pixel_maps[i] = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 2);
		cvZero(pixel_maps[i]);
	}
	std::vector<unsigned int> const & faces = mesh->get_faces();
	std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
	mve::TriangleMesh::NormalList const & face_normals = mesh->get_face_normals();
	BVHTree bvh_tree(faces, vertices); 
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//computing weight of pixel
			math::Vec3f targ_pos = pixel_to_world(x, y);
			BVHTree::Ray ray;
			ray.origin = get_pos();
			ray.dir = targ_pos - ray.origin;
			ray.tmax = ray.dir.norm();
			ray.tmin = ray.tmax * 0.0001f;
			ray.dir.normalize();
			BVHTree::Hit hit;
			if (!bvh_tree.intersect(ray, &hit)) {
				continue;
			}
			math::Vec3f intersect_pos = ray.origin + hit.t*ray.dir;
			math::Vec3f face_normal = face_normals[hit.idx];
			math::Vec3f face_to_view_vec = (ray.origin - intersect_pos).normalized();
			//if (face_to_view_vec.dot(face_normal) < 0.0f)
				//std::cout << "dot error" << std::endl;
			CV_IMAGE_ELEM(weight_map,float,y,x) = abs(face_to_view_vec.dot(face_normal));
			for (int i = 0; i < texture_views.size(); i++)
			{
				if (i == id)
					continue;
				//check visibility of point
				ray.origin = intersect_pos;
				ray.dir = texture_views[i].get_pos() - ray.origin;
				ray.tmax = ray.dir.norm();
				ray.tmin = ray.tmax * 0.0001f;
				ray.dir.normalize();
				ray.origin += 0.00001f*ray.dir;
				BVHTree::Hit hit;
				if (bvh_tree.intersect(ray, &hit)) {
					continue;
				}
				math::Vec2f pix_coord = texture_views[i].get_pixel_coords(ray.origin);
				if (pix_coord[0] < 1 || pix_coord[1] < 1)
					continue;
				if (pix_coord[0] > width - 1 || pix_coord[1] > height - 1)
					continue;
				CV_IMAGE_ELEM(pixel_maps[i], float, y, 2 * x) = pix_coord[0];
				CV_IMAGE_ELEM(pixel_maps[i], float, y, 2 * x + 1) = pix_coord[1];
			}
		}
	}
}
TEX_NAMESPACE_END
