
#include <iostream>
#include <fstream>
#include <vector>

#include <util/timer.h>
#include <util/system.h>
#include <util/file_system.h>
#include <mve/mesh_io_ply.h>

#include "tex/util.h"
#include "tex/timer.h"
#include "tex/debug.h"
#include "tex/texturing.h"
#include "tex/progress_counter.h"

#include "arguments.h"

#include "texture_api.h"
#include <opencv\cv.h>
#include <opencv\highgui.h>

void create_texMesh(std::string &in_scene, const char* plyFileName,	std::string &tex_mesh_name, int nThreshold, bool bFill)
{
	static int model_count = 0;
	//util::system::print_build_timestamp(argv[0]);
	util::system::register_segfault_handler();

#ifdef RESEARCH
	std::cout << "******************************************************************************" << std::endl
		<< " Due to use of the -DRESEARCH=ON compile option, this program is licensed " << std::endl
		<< " for research purposes only. Please pay special attention to the gco license." << std::endl
		<< "******************************************************************************" << std::endl;
#endif

	Timer timer;
	util::WallTimer wtimer;

	Arguments conf;
	conf.in_scene = in_scene.c_str();//args.get_nth_nonopt(0);
	conf.in_mesh = plyFileName;// args.get_nth_nonopt(1);
	conf.out_prefix = tex_mesh_name.c_str();//util::fs::sanitize_path(args.get_nth_nonopt(2));
	//conf.iteration = nIterations;//std::stoi(args.get_nth_nonopt(3));
	//conf.patch_width = patch_width;
	//conf.patch_height = patch_height;
	/* Set defaults for optional arguments. */
	conf.data_cost_file = "";
	conf.labeling_file = "";
	conf.settings.hole_filling = bFill;//added at 02/23/2019
	conf.write_timings = false;
	conf.write_intermediate_results = true;
	conf.write_view_selection_model = false;
	/*try {
		conf = parse_args(argc, argv);
	}
	catch (std::invalid_argument & ia) {
		std::cerr << ia.what() << std::endl;
		std::exit(EXIT_FAILURE);
	}*/

	if (!util::fs::dir_exists(util::fs::dirname(conf.out_prefix).c_str())) {
		std::cerr << "Destination directory does not exist!" << std::endl;
		std::exit(EXIT_FAILURE);
	}

	std::cout << "Load and prepare mesh: " << std::endl;
	mve::TriangleMesh::Ptr mesh;
	try {
		mesh = mve::geom::load_ply_mesh(conf.in_mesh);
	}
	catch (std::exception& e) {
		std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
		std::exit(EXIT_FAILURE);
	}
	mve::MeshInfo mesh_info(mesh);
	tex::prepare_mesh(&mesh_info, mesh);

	std::cout << "Generating texture views: " << std::endl;
	tex::TextureViews texture_views;
	tex::generate_texture_views(conf.in_scene, &texture_views);
	//if(conf.iteration > 0)
		//tex::generate_target_images(mesh, texture_views, conf.iteration, conf.patch_width, conf.patch_height);
	write_string_to_file(conf.out_prefix + ".conf", conf.to_string());
	timer.measure("Loading");

	std::size_t const num_faces = mesh->get_faces().size() / 3;

	std::cout << "Building adjacency graph: " << std::endl;
	tex::Graph graph(num_faces);
	tex::build_adjacency_graph(mesh, mesh_info, &graph);

	if (conf.labeling_file.empty()) {
		std::cout << "View selection:" << std::endl;
		util::WallTimer rwtimer;

		tex::DataCosts data_costs(num_faces, texture_views.size());
		if (conf.data_cost_file.empty()) {
			tex::calculate_data_costs(mesh, &texture_views, conf.settings, &data_costs);

			if (conf.write_intermediate_results) {
				std::cout << "\tWriting data cost file... " << std::flush;
				tex::DataCosts::save_to_file(data_costs, conf.out_prefix + "_data_costs.spt");
				std::cout << "done." << std::endl;
			}
		}
		else {
			std::cout << "\tLoading data cost file... " << std::flush;
			try {
				tex::DataCosts::load_from_file(conf.data_cost_file, &data_costs);
			}
			catch (util::FileException e) {
				std::cout << "failed!" << std::endl;
				std::cerr << e.what() << std::endl;
				std::exit(EXIT_FAILURE);
			}
			std::cout << "done." << std::endl;
		}
		timer.measure("Calculating data costs");

		tex::view_selection(data_costs, &graph, conf.settings);
		timer.measure("Running MRF optimization");
		std::cout << "\tTook: " << rwtimer.get_elapsed_sec() << "s" << std::endl;

		/* Write labeling to file. */
		if (conf.write_intermediate_results) {
			std::vector<std::size_t> labeling(graph.num_nodes());
			for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
				labeling[i] = graph.get_label(i);
			}
			vector_to_file(conf.out_prefix + "_labeling.vec", labeling);
		}
	}
	else {
		std::cout << "Loading labeling from file... " << std::flush;

		/* Load labeling from file. */
		std::vector<std::size_t> labeling = vector_from_file<std::size_t>(conf.labeling_file);
		if (labeling.size() != graph.num_nodes()) {
			std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
			std::exit(EXIT_FAILURE);
		}

		/* Transfer labeling to graph. */
		for (std::size_t i = 0; i < labeling.size(); ++i) {
			const std::size_t label = labeling[i];
			if (label > texture_views.size()) {
				std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
				std::exit(EXIT_FAILURE);
			}
			graph.set_label(i, label);
		}

		std::cout << "done." << std::endl;
	}

	tex::TextureAtlases texture_atlases;
	{
		/* Create texture patches and adjust them. */
		tex::TexturePatches texture_patches;
		tex::VertexProjectionInfos vertex_projection_infos;
		std::cout << "Generating texture patches:" << std::endl;
		tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
			conf.settings, &vertex_projection_infos, &texture_patches);

		if (conf.settings.global_seam_leveling) {
			std::cout << "Running global seam leveling:" << std::endl;
			tex::global_seam_leveling(graph, mesh, mesh_info, vertex_projection_infos, &texture_patches);
			timer.measure("Running global seam leveling");
		}
		else {
			ProgressCounter texture_patch_counter("Calculating validity masks for texture patches", texture_patches.size());
#pragma omp parallel for schedule(dynamic)
#if !defined(_MSC_VER)
			for (std::size_t i = 0; i < texture_patches.size(); ++i) {
#else
			for (std::int64_t i = 0; i < texture_patches.size(); ++i) {
#endif
				texture_patch_counter.progress<SIMPLE>();
				TexturePatch::Ptr texture_patch = texture_patches[i];
				std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
				texture_patch->adjust_colors(patch_adjust_values);
				texture_patch_counter.inc();
			}
			timer.measure("Calculating texture patch validity masks");
			}

		if (conf.settings.local_seam_leveling) {
			std::cout << "Running local seam leveling:" << std::endl;
			tex::local_seam_leveling(graph, mesh, vertex_projection_infos, &texture_patches);
		}
		timer.measure("Running local seam leveling");

		/* Generate texture atlases. */
		std::cout << "Generating texture atlases:" << std::endl;
		tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
		}

	/* Create and write out obj model. */
	{
		std::cout << "Building objmodel:" << std::endl;
		tex::Model model;
		bool bFlg = false;
		if (nThreshold > 0) bFlg = true;
		tex::build_model(mesh, texture_atlases, &model, bFlg);
		timer.measure("Building OBJ model");
		std::cout << "\tSaving model... " << std::flush;
		tex::Model::save(model, conf.out_prefix);
		std::cout << "done." << std::endl;
		timer.measure("Saving");
	}

	std::cout << "Whole texturing procedure took: " << wtimer.get_elapsed_sec() << "s" << std::endl;
	timer.measure("Total");
	if (conf.write_timings) {
		timer.write_to_file(conf.out_prefix + "_timings.csv");
	}

	if (conf.write_view_selection_model) {
		texture_atlases.clear();
		std::cout << "Generating debug texture patches:" << std::endl;
		{
			tex::TexturePatches texture_patches;
			generate_debug_embeddings(&texture_views);
			tex::VertexProjectionInfos vertex_projection_infos; // Will only be written
			tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
				conf.settings, &vertex_projection_infos, &texture_patches);
			tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
		}

		std::cout << "Building debug objmodel:" << std::endl;
		{
			/*tex::Model model;
			tex::build_model(mesh, texture_atlases, &model);
			std::cout << "\tSaving model... " << std::flush;
			tex::Model::save(model, conf.out_prefix + "_view_selection");
			std::cout << "done." << std::endl;*/
		}
	}
}
/*
created at 03/04/2019
*/
void createHoleSamplePoints(std::vector<std::vector<std::vector<float>>> &holeSamples)
{
	//model.createHoleSamples(holeSamples);
}
/*
created at 03/04/2019
*/
void getHolePatches(std::vector<std::vector<std::vector<float>>> &patchVertices,
					std::vector<std::vector<std::vector<float>>> &patchNormals,
					std::vector<std::vector<std::vector<int>>> &patchIndices)
{
	///model.getHolePatches(patchVertices, patchNormals, patchIndices);
}
/*
creatd at 03/04/2019
*/
void save(std::string fileName)
{
	//model.jointPatches();
	//tex::Model::save(model, fileName);
}
/*
crerated at 03/04/2019
*/
void createHolePatch(std::vector<math::Vec3f> &vertSamples, std::vector<math::Vec3f> &patchVertices, std::vector<math::Vec3f> &patchNormals, std::vector<math::Vec3i> &patchIndices)
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < vertSamples.size(); ++i)
	{
		pcl::PointXYZ p;
		p.x = vertSamples[i][0];
		p.y = vertSamples[i][1];
		p.z = vertSamples[i][2];
		cloud->push_back(p);
	}
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(5);//20
	n.compute(*normals);
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	PCL_INFO("pass concatenate\n");
	//* cloud_with_normals = cloud + normals
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);//0.025
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	pcl::PointCloud<pcl::PointNormal> cloud1;
	pcl::fromPCLPointCloud2(triangles.cloud, cloud1);
	for (int i = 0; i < cloud1.size(); ++i)
	{
		math::Vec3f vt, normal;
		vt[0] = cloud1[i].x;
		vt[1] = cloud1[i].y;
		vt[2] = cloud1[i].z;
		normal[0] = cloud1[i].normal[0];
		normal[1] = cloud1[i].normal[1];
		normal[2] = cloud1[i].normal[2];
		patchVertices.push_back(vt);
		patchNormals.push_back(normal);
	}
	for (int i = 0; i < triangles.polygons.size(); ++i)
	{
		math::Vec3i indices;
		indices[0] = triangles.polygons[i].vertices[0];
		indices[1] = triangles.polygons[i].vertices[1];
		indices[2] = triangles.polygons[i].vertices[2];
		patchIndices.push_back(indices);
	}*/
}