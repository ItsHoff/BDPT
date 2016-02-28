#pragma once

#include <memory>
#include <vector>

#include "3d/CameraControls.hpp"
#include "3d/Mesh.hpp"
#include "base/MulticoreLauncher.hpp"
#include "base/Random.hpp"
#include "gui/Image.hpp"

#include "AreaLight.hpp"
#include "RayTracer.hpp"
#include "RaycastResult.hpp"
#include "RTTriangle.hpp"


namespace FW
{ namespace BDPT 
{

typedef Mesh<VertexPNTC>	MeshWithColors;

// Defines a block which is rendered by a single thread as a single task.
struct PathTracerBlock {
    U32 x;      // X coordinate of the leftmost pixel of the block.
    U32 y;      // Y coordinate of the topmost pixel of the block.
    U32 width;  // Pixel width of the block.
    U32 height; // Pixel height of the block.
};


struct PathTracerContext {
    const MeshWithColors* scene;
    const CameraControls* camera;
    AreaLight* light;
    RayTracer* rt;
    std::unique_ptr<Image> image;   // Image for storing the results
    Image* dest_image;      // Image for displaying the results
    std::vector<PathTracerBlock> blocks; // Render blocks for rendering tasks. Index by .idx.
    bool force_exit;    // Set to true if we want to end tracing immediately
    U32 pass;    // Pass number, increased by one for each full render iteration.
    U32 bounces;    // Number of bounces before quitting tracing or starting rr

    PathTracerContext() : 
      force_exit(false),
      scene(nullptr),
      rt(nullptr),
      light(nullptr),
      pass(0),
      bounces(0),
      dest_image(0),
      camera(nullptr) {
    }
    ~PathTracerContext() {};
};

// Get a cosine sampled new_dir.
Vec3f cosineSampleDirection(const Vec3f& n, Random& R);

// This class contains functionality to render pictures using a ray tracer.
class PathTraceRenderer {
 public:
    PathTraceRenderer() {}
    ~PathTraceRenderer() {
        stop();
    }

    // Get the texture parameters for the hit
	static void	getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular);
    // Start the interactive tracing process
    void startPathTracingProcess(const MeshWithColors* scene, AreaLight*, RayTracer* rt, Image* dest, U32 bounces, const CameraControls& camera);
    // Trace a block of the image given by task
	static void pathTraceBlock(MulticoreLauncher::Task& task);
    // Trace the path of the given ray. b_nodes is a pre-allocated traversal stack
#ifdef ISPC
	static Vec3f traceRay(const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, U32 current_bounce, std::vector<ISPCCheckNode>& b_nodes);
#else
    static Vec3f traceRay(const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, U32 current_bounce, std::vector<CheckNode>& b_nodes);
#endif
    // Update the dest image to match the current traced image
    void updatePicture(Image* dest);
    void checkFinish();
    void stop();
    // are we still processing?
    bool isRunning() const {
        return m_launcher.getNumTasks() > 0; 
    }
	void setNormalMapped(bool b) { 
        m_normal_mapped = b; 
    }
	void setRussianRoulette(bool b) {
        m_russian_roulette = b; 
    }

 protected:
    MulticoreLauncher m_launcher;   // Launcher used to launch the path tracing process
	PathTracerContext m_context;    // Context holding the tracing information
	static bool m_normal_mapped;    
	static bool m_russian_roulette;
};

}	// namespace BDPT
}	// namespace FW
