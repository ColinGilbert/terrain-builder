#include <memory>
#include <fstream>
#include <time.h>

#include "Noise.h"

#include "format.h"

#include <PolyVox/RawVolume.h>
//#include <PolyVox/CubicSurfaceExtractor.h>
#include <PolyVox/MarchingCubesSurfaceExtractor.h>
#include <PolyVox/Mesh.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Tools/Decimater/CollapseInfoT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>
#include <OpenMesh/Tools/Decimater/ModEdgeLengthT.hh>
#include <OpenMesh/Tools/Decimater/ModHausdorffT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalDeviationT.hh>
#include <OpenMesh/Tools/Decimater/ModNormalFlippingT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <OpenMesh/Tools/Decimater/ModProgMeshT.hh>
#include <OpenMesh/Tools/Decimater/ModIndependentSetsT.hh>
#include <OpenMesh/Tools/Decimater/ModRoundnessT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;
typedef OpenMesh::PolyMesh_ArrayKernelT<> PolyMesh;

// Adapted from http://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another
double translate(double value, double from_min, double from_max, double to_min, double to_max)
{
	double from_span = from_max - from_min;
	double to_span = to_max - to_min;
	
	// Convert value to something between 0-1 
	double scaled_value = value - from_min / from_span;

	double result = to_min + (scaled_value * to_span);
	
	return result;
}


int main()
{
	clock_t t;
	std::cerr << "Begin!" << std::endl;
	
	// YOUR CODE HERE
	
	const size_t volume_width = 512;
	const size_t volume_height = 256;
	const size_t volume_depth = 512;

	noisepp::ThreadedPipeline3D* pipeline;
	noisepp::ElementID id;
	noisepp::Cache* cache;
	noisepp::PipelineElement3D *element;
	
	try
	{
		pipeline = new noisepp::ThreadedPipeline3D(8);
		noisepp::RidgedMultiModule ridged;
		//ridged.setScale(0.5);
	 	// ridged.setOffset(5.0);
		//ridged.setOctaveCount(7);
		//ridged.setQuality(noisepp::NOISE_QUALITY_HIGH);
		id = ridged.addToPipeline(pipeline);
		//cache = pipeline->createCache();
		element = pipeline->getElement(id);
		cache = pipeline->createCache();
	}

	catch (noisepp::Exception &e)
	{
		std::cerr << "exception thrown: " << e.getDescription() << std::endl;
	}


	// NO NEED TO TOUCH CODE BENEATH THIS LINE

	std::unique_ptr<PolyVox::RawVolume<uint8_t>> world = std::unique_ptr<PolyVox::RawVolume<uint8_t>>(new PolyVox::RawVolume<uint8_t>(PolyVox::Region(0, 0, 0, volume_width, volume_height, volume_depth)));
	std::unique_ptr<PolyVox::RawVolume<double>> doubles_world = std::unique_ptr<PolyVox::RawVolume<double>>(new PolyVox::RawVolume<double>(PolyVox::Region(0, 0, 0, volume_width, volume_height, volume_depth)));

	double max, min = 0.0;
	std::cerr << "Calculating 3D noise." << std::endl;
	
	t = clock();

	const size_t extents_x = volume_width - 5;
	const size_t extents_y = volume_height - 5;
	const size_t extents_z = volume_depth - 5;

	for (size_t x = 5; x < extents_x; ++x)
	{
		for (size_t y = 5; y < extents_y; ++y)
		{
			for (size_t z = 5; z < extents_z; ++z)
			{
				
				noisepp::Real value = element->getValue(static_cast<double>(x)/static_cast<double>(extents_x), static_cast<double>(y)/static_cast<double>(extents_y), static_cast<double>(z)/static_cast<double>(extents_z), cache);
				doubles_world->setVoxel(x, y, z, value);
				max = std::max(value, max);
				min = std::min(value, min);
			}
		}
	}

	t = clock() - t;
	std::cerr << "3D noise calculated. Min = " << min << ", max = " << max << ". Took " << static_cast<float>(t)/CLOCKS_PER_SEC << " seconds." << std::endl;

	for (size_t x = 5; x < extents_x; ++x)
	{
		for (size_t y = 5; y < extents_y; ++y)
		{
			for (size_t z = 5; z < extents_z; ++z)
			{
				world->setVoxel(x, y, z, static_cast<uint8_t>(translate(doubles_world->getVoxel(x, y, z), min, max, 0.0, 255.0)));
			}
		}
	}

	t = clock() - t;
	std::cerr << "Voxel world filled! Took " << static_cast<float>(t)/CLOCKS_PER_SEC << " seconds." << std::endl;


	PolyVox::Region bounding_box = world->getEnclosingRegion();

	auto mesh = extractMarchingCubesMesh(&*world, bounding_box);
	auto decoded_mesh = PolyVox::decodeMesh(mesh);
	auto num_indices = decoded_mesh.getNoOfIndices();
	auto num_vertices = decoded_mesh.getNoOfVertices();

	t = clock() - t;
	std::cerr << "Mesh ready! Took " << static_cast<float>(t)/CLOCKS_PER_SEC << " seconds." << std::endl;


	fmt::MemoryWriter w;
	w << "OFF\n" << num_vertices << " " << num_indices / 3 << " 0\n";

	for (auto i = 0; i < num_vertices; i++)
	{
		auto vertex = decoded_mesh.getVertex(i);
		w << vertex.position.getX() << " " << vertex.position.getY() << " " << vertex.position.getZ() << "\n";
	}

	for (auto i = 0; i < num_indices; i += 3)
	{
		w << "3 " << decoded_mesh.getIndex(i) << " " << decoded_mesh.getIndex(i+1) << " " << decoded_mesh.getIndex(i+2) << "\n";
	}

	std::ofstream file("terrain-full.off");
	file << w.str();

	t = clock() - t;
	std::cerr << "OFF file written. Took " << static_cast<float>(t)/CLOCKS_PER_SEC << " seconds." << std::endl;

	TriMesh half_edges;
	OpenMesh::IO::read_mesh(half_edges, "terrain-full.off");
	// logger::log(fmt::format("[Mesh] decimating - Half edges generated. num verts = {0}", half_edges.n_vertices()));

	OpenMesh::Decimater::DecimaterT<TriMesh> decimator(half_edges);
	OpenMesh::Decimater::ModQuadricT<TriMesh>::Handle mod;
	decimator.add(mod);
	decimator.initialize();

	auto testval = decimator.initialize();
	// if (!testval) logger::log("Decimator init failed!");

	size_t verts_removed = decimator.decimate_to(std::min(num_vertices / 10, static_cast<unsigned int>(5000)));

	decimator.mesh().garbage_collection();

	half_edges.garbage_collection();
	OpenMesh::IO::write_mesh(half_edges, "terrain-decimated.off");
	t = clock() - t;
	std::cerr << "Decimated! Took " << static_cast<float>(t)/CLOCKS_PER_SEC << " seconds." << std::endl;
}
