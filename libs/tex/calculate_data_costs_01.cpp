/*
 * Copyright (C) 2015, Nils Moehrle, Michael Waechter
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <numeric>

#include <mve/image_color.h>
#include <acc/bvh_tree.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "util.h"
#include "histogram.h"
#include "texturing.h"
#include "sparse_table.h"
#include "progress_counter.h"

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;

TEX_NAMESPACE_BEGIN

/**
 * Dampens the quality of all views in which the face's projection
 * has a much different color than in the majority of views.
 * Returns whether the outlier removal was successfull.
 *
 * @param infos contains information about one face seen from several views
 * @param settings runtime configuration.
 */
bool
photometric_outlier_detection(std::vector<FaceProjectionInfo> * infos, Settings const & settings) {
    if (infos->size() == 0) return true;

    /* Configuration variables. */

    double const gauss_rejection_threshold = 6e-3;

    /* If all covariances drop below this we stop outlier detection. */
    double const minimal_covariance = 5e-4;

    int const outlier_detection_iterations = 10;
    int const minimal_num_inliers = 4;

    float outlier_removal_factor = std::numeric_limits<float>::signaling_NaN();
    switch (settings.outlier_removal) {
        case OUTLIER_REMOVAL_NONE: return true;
        case OUTLIER_REMOVAL_GAUSS_CLAMPING:
            outlier_removal_factor = 1.0f;
        break;
        case OUTLIER_REMOVAL_GAUSS_DAMPING:
            outlier_removal_factor = 0.2f;
        break;
    }

    Eigen::MatrixX3d inliers(infos->size(), 3);
    std::vector<std::uint32_t> is_inlier(infos->size(), 1);
    for (std::size_t row = 0; row < infos->size(); ++row) {
        inliers.row(row) = mve_to_eigen(infos->at(row).mean_color).cast<double>();
    }

    Eigen::RowVector3d var_mean;
    Eigen::Matrix3d covariance;
    Eigen::Matrix3d covariance_inv;

    for (int i = 0; i < outlier_detection_iterations; ++i) {

        if (inliers.rows() < minimal_num_inliers) {
            return false;
        }

        /* Calculate the inliers' mean color and color covariance. */
        var_mean = inliers.colwise().mean();
        Eigen::MatrixX3d centered = inliers.rowwise() - var_mean;
        covariance = (centered.adjoint() * centered) / double(inliers.rows() - 1);

        /* If all covariances are very small we stop outlier detection
         * and only keep the inliers (set quality of outliers to zero). */
        if (covariance.array().abs().maxCoeff() < minimal_covariance) {
            for (std::size_t row = 0; row < infos->size(); ++row) {
                if (!is_inlier[row]) infos->at(row).quality = 0.0f;
            }
            return true;
        }

        /* Invert the covariance. FullPivLU is not the fastest way but
         * it gives feedback about numerical stability during inversion. */
        Eigen::FullPivLU<Eigen::Matrix3d> lu(covariance);
        if (!lu.isInvertible()) {
            return false;
        }
        covariance_inv = lu.inverse();

        /* Compute new number of inliers (all views with a gauss value above a threshold). */
        for (std::size_t row = 0; row < infos->size(); ++row) {
            Eigen::RowVector3d color = mve_to_eigen(infos->at(row).mean_color).cast<double>();
            double gauss_value = multi_gauss_unnormalized<double, 3>(color, var_mean, covariance_inv);
            is_inlier[row] = (gauss_value >= gauss_rejection_threshold ? 1 : 0);
        }
        /* Resize Eigen matrix accordingly and fill with new inliers. */
        inliers.resize(std::accumulate(is_inlier.begin(), is_inlier.end(), 0), Eigen::NoChange);
        for (std::size_t row = 0, inlier_row = 0; row < infos->size(); ++row) {
            if (is_inlier[row]) {
                inliers.row(inlier_row++) = mve_to_eigen(infos->at(row).mean_color).cast<double>();
            }
        }
    }

    covariance_inv *= outlier_removal_factor;
    for (FaceProjectionInfo & info : *infos) {
        Eigen::RowVector3d color = mve_to_eigen(info.mean_color).cast<double>();
        double gauss_value = multi_gauss_unnormalized<double, 3>(color, var_mean, covariance_inv);
        assert(0.0 <= gauss_value && gauss_value <= 1.0);
        switch(settings.outlier_removal) {
            case OUTLIER_REMOVAL_NONE: return true;
            case OUTLIER_REMOVAL_GAUSS_DAMPING:
                info.quality *= gauss_value;
            break;
            case OUTLIER_REMOVAL_GAUSS_CLAMPING:
                if (gauss_value < gauss_rejection_threshold) info.quality = 0.0f;
            break;
        }
    }
    return true;
}

void
calculate_face_projection_infos(mve::TriangleMesh::ConstPtr mesh,
    std::vector<TextureView> * texture_views, Settings const & settings,
    FaceProjectionInfos * face_projection_infos) {

    std::vector<unsigned int> const & faces = mesh->get_faces();
    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
    mve::TriangleMesh::NormalList const & face_normals = mesh->get_face_normals();

    std::size_t const num_views = texture_views->size();

    util::WallTimer timer;
    std::cout << "\tBuilding BVH from " << faces.size() / 3 << " faces... " << std::flush;
    BVHTree bvh_tree(faces, vertices);
    std::cout << "done. (Took: " << timer.get_elapsed() << " ms)" << std::endl;
	
    ProgressCounter view_counter("\tCalculating face qualities", num_views);
	#pragma omp parallel
    {
        std::vector<std::pair<std::size_t, FaceProjectionInfo> > projected_face_view_infos;
		#pragma omp for schedule(dynamic)
		#if !defined(_MSC_VER)
        for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
		#else
        for (std::int32_t j = 0; j < num_views; ++j) {
		#endif
			view_counter.progress<SIMPLE>();
			TextureView * texture_view = &texture_views->at(j);
			texture_view->load_image();
			texture_view->generate_validity_mask();
			if (settings.data_term == DATA_TERM_GMI) {
                texture_view->generate_gradient_magnitude();
                texture_view->erode_validity_mask();
            }
			math::Vec3f const & view_pos = texture_view->get_pos();
            math::Vec3f const & viewing_direction = texture_view->get_viewing_direction();
			//math::Matrix4f mat = texture_view->getWorld_To_Cam();
			int visual_count = 0;
			for (std::size_t i = 0; i < faces.size(); i += 3) {
                std::size_t face_id = i / 3;

                math::Vec3f const & v1 = vertices[faces[i]];
                math::Vec3f const & v2 = vertices[faces[i + 1]];
                math::Vec3f const & v3 = vertices[faces[i + 2]];
                math::Vec3f const & face_normal = face_normals[face_id];
                math::Vec3f const face_center = (v1 + v2 + v3) / 3.0f;

                /* Check visibility and compute quality */

                math::Vec3f view_to_face_vec = (face_center - view_pos).normalized();
                math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();

                /* Backface and basic frustum culling */
                float viewing_angle = face_to_view_vec.dot(face_normal);
                if (viewing_angle < 0.0f || viewing_direction.dot(view_to_face_vec) < 0.0f)
                    continue;

                if (std::acos(viewing_angle) > MATH_DEG2RAD(75.0f))
                    continue;
				/* Projects into the valid part of the TextureView? */
				if (!texture_view->inside(v1, v2, v3))
                    continue;
				visual_count++;
				if (settings.geometric_visibility_test) {
                    /* Viewing rays do not collide? */
                    bool visible = true;
                    math::Vec3f const * samples[] = {&v1, &v2, &v3};
                    // TODO: random monte carlo samples...

                    for (std::size_t k = 0; k < sizeof(samples) / sizeof(samples[0]); ++k) {
                        BVHTree::Ray ray;
                        ray.origin = *samples[k];
                        ray.dir = view_pos - ray.origin;
                        ray.tmax = ray.dir.norm();
                        ray.tmin = ray.tmax * 0.0001f;
                        ray.dir.normalize();

                        BVHTree::Hit hit;
                        if (bvh_tree.intersect(ray, &hit)) {
                            visible = false;
                            break;
                        }
                    }
                    if (!visible) continue;
                }

                FaceProjectionInfo info = {j, 0.0f, math::Vec3f(0.0f, 0.0f, 0.0f)};

                /* Calculate quality. */
                texture_view->get_face_info(v1, v2, v3, &info, settings);

                if (info.quality == 0.0) continue;

                /* Change color space. */
                mve::image::color_rgb_to_ycbcr(*(info.mean_color));

                std::pair<std::size_t, FaceProjectionInfo> pair(face_id, info);
                projected_face_view_infos.push_back(pair);
            }
			std::wcout << "visual_count:" << visual_count << std::endl;
            texture_view->release_image();
            texture_view->release_validity_mask();
            if (settings.data_term == DATA_TERM_GMI) {
                texture_view->release_gradient_magnitude();
            }
            view_counter.inc();
        }
		
        //std::sort(projected_face_view_infos.begin(), projected_face_view_infos.end());

        #pragma omp critical
        {
            for (std::size_t i = projected_face_view_infos.size(); 0 < i; --i) {
                std::size_t face_id = projected_face_view_infos[i - 1].first;
                FaceProjectionInfo const & info = projected_face_view_infos[i - 1].second;
                face_projection_infos->at(face_id).push_back(info);
            }
            projected_face_view_infos.clear();
        }
    }
}

void
postprocess_face_infos(Settings const & settings,
        FaceProjectionInfos * face_projection_infos,
        DataCosts * data_costs) {

    ProgressCounter face_counter("\tPostprocessing face infos",
        face_projection_infos->size());
    #pragma omp parallel for schedule(dynamic)
#if !defined(_MSC_VER)
    for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {
#else
    for (std::int64_t i = 0; i < face_projection_infos->size(); ++i) {
#endif
        face_counter.progress<SIMPLE>();

        std::vector<FaceProjectionInfo> & infos = face_projection_infos->at(i);
        if (settings.outlier_removal != OUTLIER_REMOVAL_NONE) {
            photometric_outlier_detection(&infos, settings);

            infos.erase(std::remove_if(infos.begin(), infos.end(),
                [](FaceProjectionInfo const & info) -> bool {return info.quality == 0.0f;}),
                infos.end());
        }
        std::sort(infos.begin(), infos.end());

        face_counter.inc();
    }

    /* Determine the function for the normlization. */
    float max_quality = 0.0f;
    for (std::size_t i = 0; i < face_projection_infos->size(); ++i)
        for (FaceProjectionInfo const & info : face_projection_infos->at(i))
            max_quality = std::max(max_quality, info.quality);

    Histogram hist_qualities(0.0f, max_quality, 10000);
    for (std::size_t i = 0; i < face_projection_infos->size(); ++i)
        for (FaceProjectionInfo const & info : face_projection_infos->at(i))
            hist_qualities.add_value(info.quality);

    float percentile = hist_qualities.get_approx_percentile(0.995f);

    /* Calculate the costs. */
    for (std::uint32_t i = 0; i < face_projection_infos->size(); ++i) {
        for (FaceProjectionInfo const & info : face_projection_infos->at(i)) {

            /* Clamp to percentile and normalize. */
            float normalized_quality = std::min(1.0f, info.quality / percentile);
            float data_cost = (1.0f - normalized_quality) * MRF_MAX_ENERGYTERM;
            data_costs->set_value(i, info.view_id, data_cost);
        }

        /* Ensure that all memory is freeed. */
        face_projection_infos->at(i) = std::vector<FaceProjectionInfo>();
    }

    std::cout << "\tMaximum quality of a face within an image: " << max_quality << std::endl;
    std::cout << "\tClamping qualities to " << percentile << " within normalization." << std::endl;
}

void
calculate_data_costs(mve::TriangleMesh::ConstPtr mesh, std::vector<TextureView> * texture_views,
    Settings const & settings, DataCosts * data_costs) {

    std::size_t const num_faces = mesh->get_faces().size() / 3;
    std::size_t const num_views = texture_views->size();

    if (num_faces > std::numeric_limits<std::uint32_t>::max())
        throw std::runtime_error("Exeeded maximal number of faces");
    if (num_views > std::numeric_limits<std::uint16_t>::max())
        throw std::runtime_error("Exeeded maximal number of views");
    static_assert(MRF_MAX_ENERGYTERM <= std::numeric_limits<float>::max(),
        "MRF_MAX_ENERGYTERM has to be within float limits");

    FaceProjectionInfos face_projection_infos(num_faces);
    calculate_face_projection_infos(mesh, texture_views, settings, &face_projection_infos);
    postprocess_face_infos(settings, &face_projection_infos, data_costs);
}
/*
description:
compute the face projection infos with mesh and texture_views.
created at 07/26/2017
*/
void generate_face_infos(mve::TriangleMesh::ConstPtr mesh,	std::vector<TextureView> * texture_views, 
							  FaceProjectionInfos * face_projection_infos) {

	std::vector<unsigned int> const & faces = mesh->get_faces();
	std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
	mve::TriangleMesh::NormalList const & face_normals = mesh->get_face_normals();

	std::size_t const num_views = texture_views->size();

	util::WallTimer timer;
	std::cout << "\tBuilding BVH from " << faces.size() / 3 << " faces... " << std::flush;
	BVHTree bvh_tree(faces, vertices);
	std::cout << "done. (Took: " << timer.get_elapsed() << " ms)" << std::endl;

	ProgressCounter view_counter("\tCalculating face qualities", num_views);
#pragma omp parallel
	{
		std::vector<std::pair<std::size_t, FaceProjectionInfo> > projected_face_view_infos;
#pragma omp for schedule(dynamic)
#if !defined(_MSC_VER)
		for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
#else
		for (std::int32_t j = 0; j < num_views; ++j) {
#endif
			view_counter.progress<SIMPLE>();
			TextureView * texture_view = &texture_views->at(j);
			math::Vec3f const & view_pos = texture_view->get_pos();
			math::Vec3f const & viewing_direction = texture_view->get_viewing_direction();
			math::Matrix4f mat = texture_view->getWorld_To_Cam();
			int visual_count = 0;
			for (std::size_t i = 0; i < faces.size(); i += 3) {
				std::size_t face_id = i / 3;

				math::Vec3f const & v1 = vertices[faces[i]];
				math::Vec3f const & v2 = vertices[faces[i + 1]];
				math::Vec3f const & v3 = vertices[faces[i + 2]];
				math::Vec3f const & face_normal = face_normals[face_id];
				math::Vec3f const face_center = (v1 + v2 + v3) / 3.0f;

				/* Check visibility and compute quality */

				math::Vec3f view_to_face_vec = (face_center - view_pos).normalized();
				math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();

				/* Backface and basic frustum culling */
				float viewing_angle = face_to_view_vec.dot(face_normal);
				if (viewing_angle < 0.0f || viewing_direction.dot(view_to_face_vec) < 0.0f)
					continue;

				if (std::acos(viewing_angle) > MATH_DEG2RAD(75.0f))
					continue;
				/* Projects into the valid part of the TextureView? */
				if (!texture_view->inside(v1, v2, v3))
					continue;
				visual_count++;
				/* Viewing rays do not collide? */
				bool visible = true;
				math::Vec3f const * samples[] = { &v1, &v2, &v3 };
				// TODO: random monte carlo samples...
				for (std::size_t k = 0; k < sizeof(samples) / sizeof(samples[0]); ++k) {
					BVHTree::Ray ray;
					ray.origin = *samples[k];
					ray.dir = view_pos - ray.origin;
					ray.tmax = ray.dir.norm();
					ray.tmin = ray.tmax * 0.0001f;
					ray.dir.normalize();
					BVHTree::Hit hit;
					if (bvh_tree.intersect(ray, &hit)) {
						visible = false;
						break;
					}
				}
				if (!visible) continue;
				FaceProjectionInfo info = { j, viewing_angle, math::Vec3f(0.0f, 0.0f, 0.0f) };
				std::pair<std::size_t, FaceProjectionInfo> pair(face_id, info);
				projected_face_view_infos.push_back(pair);
			}
			std::wcout << "visual_count:" << visual_count << std::endl;
			view_counter.inc();
		}
		//std::sort(projected_face_view_infos.begin(), projected_face_view_infos.end());
#pragma omp critical
		{
			for (std::size_t i = projected_face_view_infos.size(); 0 < i; --i) {
				std::size_t face_id = projected_face_view_infos[i - 1].first;
				FaceProjectionInfo const & info = projected_face_view_infos[i - 1].second;
				face_projection_infos->at(face_id).push_back(info);
			}
			projected_face_view_infos.clear();
		}
		}
	}
/*
description:
update the vertex infos with mesh and face projection infos
input: mesh, face_projection_infos
output: vertex_view_infos
created at 07/27/2017
*/
void generate_vertex_infos(mve::TriangleMesh::ConstPtr mesh,
						 FaceProjectionInfos &face_projection_infos,
						 VertexViewInfos &vertex_view_infos)
{
	for (unsigned int t = 0; t < face_projection_infos.size(); t++)
	{
		for (int i = 0; i < face_projection_infos[t].size(); i++)
		{
			int view_id = face_projection_infos[t][i].view_id;
			float dot = face_projection_infos[t][i].quality;
			for (int j = 0; j < 3; j++)
			{
				size_t v = mesh->get_faces()[3 * t + j];
				if (vertex_view_infos[v].size() == 0)
				{/*if the info of current vertex is empty, add current projection info of face to the vertex*/
					VertexViewInfo vertex_info;
					vertex_info.avg_dot = face_projection_infos[t][i].quality;
					vertex_info.view_id = face_projection_infos[t][i].view_id;
					vertex_info.count = 1;
					vertex_view_infos[v].push_back(vertex_info);
				}
				else if(vertex_view_infos[v].size() > 0)
				{
					for (int k = 0; k < vertex_view_infos[v].size(); k++)
					{
						if (vertex_view_infos[v][k].view_id == view_id)
						{/*update the vertex info corresponding to the current view*/
							int count = vertex_view_infos[v][k].count;
							vertex_view_infos[v][k].avg_dot = (vertex_view_infos[v][k].avg_dot*count + dot) / (count + 1);
							vertex_view_infos[v][k].count = count + 1;
						}
					}
				}
			}
		}
	}
}
/*
description:
update the weight image of a view with weights of 3 vertices of a face
created at 07/27/2017
*/
void update_weight_image_view(float* weights, math::Vec2f* pix_coords, IplImage *weight_img)
{
	math::Vec2f p[6];
	float w[6];
	for (int i = 0; i < 3; i++)
	{
		p[i] = pix_coords[i];
		w[i] = weights[i];
	}
	math::Vec2f vec2[6];
	vec2[0] = p[1] - p[0];
	vec2[1] = p[2] - p[0];
	vec2[2] = p[2] - p[1];
	int u_count = vec2[0].norm() + 0.5f;
	int v_count = vec2[1].norm() + 0.5f;
	int s_count = vec2[2].norm() + 0.5f;
	vec2[0].normalize();
	vec2[1].normalize();
	vec2[2].normalize();
	for (int v = 0; v < v_count; v++)
	{
		for (int u = 0; u < u_count; u++)
		{
			p[5] = p[0] + (float)u*vec2[0] + (float)v*vec2[1];
			p[3] = p[0] + (float)v*vec2[0];
			p[4] = p[1] + (float)s_count*(float)(v / v_count)*vec2[2];
			w[3] = w[0] * (float)(v_count - v) / v_count + w[2] * (float)v / v_count;
			w[4] = w[1]*(float)(v_count - v) / v_count + w[2] * (float)v / v_count;
			vec2[3] = p[4] - p[3];
			vec2[4] = p[5] - p[3];
			vec2[5] = p[4] - p[5];
			w[5] = w[3] * vec2[5].norm() / vec2[3].norm() + w[4]*vec2[4].norm()/vec2[3].norm();
			int x = p[5][0] + 0.5f;
			int y = p[5][1] + 0.5f;
			CV_IMAGE_ELEM(weight_img, float, y, x) = w[5];
 		}
	}
}
/*
created at 07/27/2017
*/
void generate_weight_image_view(mve::TriangleMesh::ConstPtr mesh, 
								TextureView &texture_view, 
								FaceProjectionInfos face_projection_infos,
								VertexViewInfos vertex_view_infos, 
								IplImage *weight_img_view)
{
	//IplImage* img = cvCreateImage(cvSize(10, 10), IPL_DEPTH_32F, 1);
	int view_id = texture_view.get_id();
	for (size_t t = 0; t < face_projection_infos.size(); t++)
	{
		for (int i = 0; i < face_projection_infos[t].size(); i++)
		{
			if (face_projection_infos[t][i].view_id == view_id)
			{
				math::Vec3f vec3[3];
				math::Vec2f pix_coords[3];
				float weights[3];
				for (int j = 0; j < 3; j++)
				{
					size_t v = mesh->get_faces()[3 * t + j];
					vec3[j] = mesh->get_vertices()[v];
					pix_coords[j] = texture_view.get_pixel_coords(vec3[j]);
					weights[j] = 0.0f;
					for (int k = 0; k < vertex_view_infos[v].size(); k++)
					{
						if (vertex_view_infos[v][k].view_id == view_id)
						{
							weights[j] = vertex_view_infos[v][k].avg_dot;
							break;
						}
					}
				}
				update_weight_image_view(weights, pix_coords, weight_img_view);
				break;
			}
		}
	}
}
/*
created at 07/27/2017
*/
void generate_weight_image_views(mve::TriangleMesh::ConstPtr mesh, 
							std::vector<TextureView> &texture_views,
							std::vector<IplImage*> &weight_img_views)
{
	size_t view_count = texture_views.size();
	size_t face_count = mesh->get_faces().size() / 3;
	size_t vertex_count = mesh->get_vertices().size() / 3;
	FaceProjectionInfos face_projection_infos;
	face_projection_infos.resize(face_count);
	VertexViewInfos vertex_view_infos;
	vertex_view_infos.resize(vertex_count);
	generate_face_infos(mesh, &texture_views, &face_projection_infos);
	generate_vertex_infos(mesh, face_projection_infos, vertex_view_infos);
	for (int i = 0; i < view_count; i++)
	{
		generate_weight_image_view(mesh, texture_views[i], face_projection_infos, vertex_view_infos, weight_img_views[i]);
	}
}
TEX_NAMESPACE_END
