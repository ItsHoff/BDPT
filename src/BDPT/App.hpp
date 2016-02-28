#pragma once

#include <memory>
#include <vector>

#include "3d/CameraControls.hpp"
#include "gpu/Buffer.hpp"
#include "gui/CommonControls.hpp"
#include "gui/Image.hpp"
#include "gui/Window.hpp"

#include "AreaLight.hpp"
#include "PathTraceRenderer.hpp"
#include "RayTracer.hpp"


namespace FW 
{ namespace BDPT
{

enum Action {
    Action_None,

    Action_LoadMesh,
    Action_ReloadMesh,
    Action_SaveMesh,

    Action_ResetCamera,
    Action_EncodeCameraSignature,
    Action_DecodeCameraSignature,

    Action_NormalizeScale,
    Action_FlipXY,
    Action_FlipYZ,
    Action_FlipZ,

    Action_NormalizeNormals,
    Action_FlipNormals,
    Action_RecomputeNormals,

    Action_FlipTriangles,

    Action_CleanMesh,
    Action_CollapseVertices,
    Action_DupVertsPerSubmesh,
    Action_FixMaterialColors,
    Action_DownscaleTextures,
    Action_ChopBehindNear,

    Action_PathTraceMode,
    Action_PlaceLightSourceAtCamera
};

enum CullMode {
    CullMode_None,
    CullMode_CW,
    CullMode_CCW,
};

class App : public Window::Listener, public CommonControls::StateObject {
 public:
	App(std::vector<std::string>& cmd_args);
    virtual ~App();

    virtual bool handleEvent(const Window::Event& ev);
    virtual void readState(StateDump& d);
    virtual void writeState(StateDump& d) const;

 private:
    Window m_window;                
    CommonControls m_common_ctrl;
    CameraControls m_camera_ctrl;

    Action m_action;
    String m_mesh_file_name;
    CullMode m_cull_mode;

    std::unique_ptr<RayTracer> m_rt;
	std::vector<Vec3f> m_rt_vertex_positions; // kept only for MD5 checksums
    std::vector<RTTriangle> m_rt_triangles;

    std::unique_ptr<MeshWithColors> m_mesh;
	std::unique_ptr<AreaLight> m_area_light;
    std::unique_ptr<PathTraceRenderer> m_pathtrace_renderer;
	S32 m_num_bounces;
	F32 m_light_size;
	Timer m_update_clock;

	bool m_rt_mode;
	bool m_use_russian_roulette;
	bool m_normal_mapped;
	Image m_img;

	// this structure holds the necessary arguments when rendering using command line parameters
	struct {
		bool batch_render;
		SplitMode split_mode;		// the BVH builder to use
		U32 spp;					// samples per pixel to use
		bool output_images;			// might be useful to compare images with the example
		bool use_textures;			// whether or not textures are used
		bool use_arealights;		// whether or not area light sampling is used
		bool enable_reflections;	// whether to compute reflections in whitted integrator
	} m_settings;
	
	struct {
		std::string state_name;     // filenames of the state and scene files
		std::string scene_name;
		U32 ray_count;
		U32 build_time, trace_time;
	} m_results;

    App(const App&) = delete;            // forbidden
    App& operator=(const App&) = delete; // forbidden

	void process_args(std::vector<std::string>& args);
    void waitKey();
    void renderFrame(GLContext* gl);
    void renderScene(GLContext* gl, const Mat4f& world_to_camera, const Mat4f& projection);
    void loadMesh(const String& fileName);
    void saveMesh(const String& fileName);

    static void downscaleTextures(MeshBase* mesh);
    static void chopBehindPlane(MeshBase* mesh, const Vec4f& pleq);
    static bool fileExists(const String& file_name);

	void constructTracer();
};

} // namespace BDPT
} // namespace FW
