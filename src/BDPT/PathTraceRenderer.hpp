#pragma once

#include "3d/CameraControls.hpp"
#include "3d/Mesh.hpp"
#include "gui/Image.hpp"
#include "base/Random.hpp"
#include "base/MulticoreLauncher.hpp"

#include <vector>
#include <memory>

#include "AreaLight.hpp"
#include "RayTracer.hpp"
#include "RaycastResult.hpp"
#include "RTTriangle.hpp"


namespace FW
{ namespace BDPT 
{

//------------------------------------------------------------------------

typedef Mesh<VertexPNTC>	MeshWithColors;

//------------------------------------------------------------------------



/// Defines a block which is rendered by a single thread as a single task.
struct PathTracerBlock
{
    int m_x;      ///< X coordinate of the leftmost pixel of the block.
    int m_y;      ///< Y coordinate of the topmost pixel of the block.
    int m_width;  ///< Pixel width of the block.
    int m_height; ///< Pixel height of the block.
};


struct PathTracerContext
{
    PathTracerContext();
    ~PathTracerContext();
    
    std::vector<PathTracerBlock> m_blocks; ///< Render blocks for rendering tasks. Index by .idx.

    bool						m_bForceExit;
	bool						m_bResidual;
    const MeshWithColors*		m_scene;
    RayTracer*	                m_rt;
    AreaLight*                  m_light;
    int							m_pass;    ///< Pass number, increased by one for each full render iteration.
    int							m_bounces;
    std::unique_ptr<Image>		m_image;
    Image*	                	m_destImage;
    const CameraControls*		m_camera;
};



// This class contains functionality to render pictures using a ray tracer.
class PathTraceRenderer
{
public:
    PathTraceRenderer();
    ~PathTraceRenderer();

    // are we still processing?
    bool				isRunning							( void ) const		{ return m_launcher.getNumTasks() > 0; }

    void				startPathTracingProcess				( const MeshWithColors* scene, AreaLight*, RayTracer* rt, Image* dest, int bounces, const CameraControls& camera );
	static void			pathTraceBlock						( MulticoreLauncher::Task& t);
#ifdef ISPC
	static Vec3f		traceRay							( const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, int current_bounce, std::vector<ISPCCheckNode>& b_nodes);
#else
    static Vec3f        traceRay(const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, int current_bounce, std::vector<CheckNode>& b_nodes);
#endif
	static void			getTextureParameters				( const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular );
    void				updatePicture						( Image* display );	// normalize by 1/w
    void				checkFinish							( void );
    void				stop								( void );
	void				setNormalMapped						( bool b ){ m_normalMapped = b; }
	void				setRussianRoulette					( bool b ){ m_russian_roulette = b; }



protected:
    __int64						m_s64TotalRays;
    float						m_raysPerSecond;

    MulticoreLauncher			m_launcher;
	PathTracerContext			m_context;
	static bool					m_normalMapped;
	static bool					m_russian_roulette;
};

void cosineSampleDirection(const Vec3f& n, Vec3f& new_dir, float& pdf, Random& R);

}	// namespace BDPT
}	// namespace FW
