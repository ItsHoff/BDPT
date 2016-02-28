#define _CRT_SECURE_NO_WARNINGS

#include "App.hpp"

#include <algorithm>
#include <chrono>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <map>
#include <stdio.h>
#include <string>

#include "3d/Mesh.hpp"
#include "base/Main.hpp"
#include "base/Random.hpp"
#include "gpu/GLContext.hpp"
#include "io/File.hpp"
#include "io/StateDump.hpp"

#include "RayTracer.hpp"


namespace FW 
{

// Called to start the program
void init(std::vector<std::string>& args) {
    new BDPT::App(args);
}

namespace BDPT
{

bool fileExists(std::string fileName) {
    return std::ifstream(fileName).good();
}

App::App(std::vector<std::string>& cmd_args)
    : m_common_ctrl(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
    m_camera_ctrl(&m_common_ctrl, CameraControls::Feature_Default | CameraControls::Feature_StereoControls),
    m_action(Action_None),
    m_cull_mode(CullMode_None),
    m_num_bounces(1),
    m_light_size(0.25f),
    m_rt_mode(false),
    m_use_russian_roulette(false),
    m_normal_mapped(false),
    m_img(Vec2i(10, 10), ImageFormat::RGBA_Vec4f) // will get resized immediately
{
    // General settings
    m_common_ctrl.showFPS(true);
    m_common_ctrl.addStateObject(this);
    m_camera_ctrl.setKeepAligned(true);

    // Create buttons
    m_common_ctrl.addButton((S32*)&m_action, Action_LoadMesh, FW_KEY_M, "Load mesh or state... (M)");
    m_common_ctrl.addButton((S32*)&m_action, Action_ReloadMesh, FW_KEY_F5, "Reload mesh (F5)");
    m_common_ctrl.addButton((S32*)&m_action, Action_SaveMesh, FW_KEY_O, "Save mesh... (O)");
    m_common_ctrl.addSeparator();

    m_common_ctrl.addButton((S32*)&m_action, Action_ResetCamera, FW_KEY_NONE, "Reset camera");
    m_common_ctrl.addButton((S32*)&m_action, Action_EncodeCameraSignature, FW_KEY_NONE, "Encode camera signature");
    m_common_ctrl.addButton((S32*)&m_action, Action_DecodeCameraSignature, FW_KEY_NONE, "Decode camera signature...");
    m_window.addListener(&m_camera_ctrl);
    m_common_ctrl.addSeparator();

    m_common_ctrl.addButton((S32*)&m_action, Action_NormalizeScale, FW_KEY_NONE, "Normalize scale");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_FlipXY,                  FW_KEY_NONE,    "Flip X/Y");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_FlipYZ,                  FW_KEY_NONE,    "Flip Y/Z");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_FlipZ,                   FW_KEY_NONE,    "Flip Z");
    m_common_ctrl.addSeparator();

    m_common_ctrl.addButton((S32*)&m_action, Action_NormalizeNormals, FW_KEY_NONE, "Normalize normals");
    m_common_ctrl.addButton((S32*)&m_action, Action_FlipNormals, FW_KEY_NONE, "Flip normals");
    m_common_ctrl.addButton((S32*)&m_action, Action_RecomputeNormals, FW_KEY_NONE, "Recompute normals");
    m_common_ctrl.addSeparator();

    m_common_ctrl.addToggle((S32*)&m_cull_mode, CullMode_None, FW_KEY_NONE, "Disable backface culling");
    m_common_ctrl.addToggle((S32*)&m_cull_mode, CullMode_CW, FW_KEY_NONE, "Cull clockwise faces");
    m_common_ctrl.addToggle((S32*)&m_cull_mode, CullMode_CCW, FW_KEY_NONE, "Cull counter-clockwise faces");
    m_common_ctrl.addButton((S32*)&m_action, Action_FlipTriangles, FW_KEY_NONE, "Flip triangles");
    m_common_ctrl.addSeparator();

    //    m_common_ctrl.addButton((S32*)&m_action, Action_CleanMesh,               FW_KEY_NONE,    "Remove unused materials, denegerate triangles, and unreferenced vertices");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_CollapseVertices,        FW_KEY_NONE,    "Collapse duplicate vertices");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_DupVertsPerSubmesh,      FW_KEY_NONE,    "Duplicate vertices shared between multiple materials");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_FixMaterialColors,       FW_KEY_NONE,    "Override material colors with average over texels");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_DownscaleTextures,       FW_KEY_NONE,    "Downscale textures by 2x");
    //    m_common_ctrl.addButton((S32*)&m_action, Action_ChopBehindNear,          FW_KEY_NONE,    "Chop triangles behind near plane");
    //    m_common_ctrl.addSeparator();

    m_common_ctrl.addButton((S32*)&m_action, Action_PathTraceMode, FW_KEY_INSERT, "Path trace mode (INSERT)");
    m_common_ctrl.addButton((S32*)&m_action, Action_PlaceLightSourceAtCamera, FW_KEY_SPACE, "Place light at camera (SPACE)");
    m_common_ctrl.addToggle(&m_use_russian_roulette, FW_KEY_NONE, "Use Russian Roulette");
    m_common_ctrl.addToggle(&m_normal_mapped, FW_KEY_NONE, "Use normal mapping");

    // Add sliders
    m_common_ctrl.beginSliderStack();
    m_common_ctrl.addSlider(&m_num_bounces, 0, 8, false, FW_KEY_NONE, FW_KEY_NONE, "Number of indirect bounces= %d");
    m_common_ctrl.addSlider(&m_light_size, 0.01f, 200.0f, false, FW_KEY_NONE, FW_KEY_NONE, "Light source area= %f");
    m_common_ctrl.endSliderStack();

    m_window.addListener(this);
    m_window.addListener(&m_common_ctrl);

    m_window.setTitle("BDPT");
    m_common_ctrl.setStateFilePrefix("state_BDPT_");

    m_window.setSize(Vec2i(800, 600));
    m_pathtrace_renderer.reset(new PathTraceRenderer);
    m_area_light.reset(new AreaLight);

    process_args(cmd_args);

    // Load default state
    m_common_ctrl.loadState(m_common_ctrl.getStateFileName(1));
}

// returns the index of the needle in the haystack or -1 if not found
S32 find_argument(std::string needle, std::vector<std::string> haystack) {
    for (unsigned j = 0; j < haystack.size( ); ++j) {
        if (!haystack[j].compare(needle)) {
            return j;
        }
    }
    return -1;
}

void App::process_args(std::vector<std::string>& args) {
    // all of the possible cmd arguments and the corresponding enums (enum value is the index of the string in the vector)
    const std::vector<std::string> argument_names = {"-builder", "-spp", "-output_images", "-use_textures", "-bat_render", "-aa", "-ao", "-ao_length"};
    enum argument {arg_not_found, builder, spp, output_images, use_textures, bat_render};

    // similarly a list of the implemented BVH builder types
    const std::vector<std::string> builder_names = {"none", "sah", "object_median", "spatial_median", "linear"};
    enum builder_type {builder_not_found, builder_None, builder_SAH, builder_ObjectMedian, builder_SpatialMedian};

    m_settings.batch_render = false;
    m_settings.output_images = false;
    m_settings.use_textures = true;
    m_settings.spp = 1;
    m_settings.split_mode = SplitMode_Sah;

    for (unsigned i = 0; i < args.size( ); ++i) {
        // try to recognize the argument
        argument cmd = argument(find_argument(args[i], argument_names));
        switch (cmd) {
            case bat_render:
                m_settings.batch_render = true;
                break;
            case output_images:
                m_settings.output_images = true;
                break;
            case use_textures:
                m_settings.use_textures = true;
                break;
            case spp:
                ++i;
                m_settings.spp = std::stoi(args[i]);
                break;
            case builder: {
                ++i;
                builder_type type = builder_type(find_argument(args[i], builder_names));
                if (type == builder_not_found) {
                    type = builder_SAH;
                    std::cout << "BVH builder not recognized, using Surface Area Heuristic" << std::endl;
                    break;
                }
                switch (type) {
                    case builder_None:
                        m_settings.split_mode = SplitMode_None;
                        break;
                    case builder_SAH:
                        m_settings.split_mode = SplitMode_Sah;
                        break;
                    case builder_ObjectMedian:
                        m_settings.split_mode = SplitMode_ObjectMedian;
                        break;
                    case builder_SpatialMedian:
                        m_settings.split_mode = SplitMode_SpatialMedian;
                        break;
                }
                break;
            }
            default:
                if (args[i][0] == '-') {
                    std::cout << "argument \"" << args[i] << "\" not found!" << std::endl;
                }
        }
    }
    if (m_settings.batch_render) {
        m_settings.use_textures = false;
    }
}

App::~App( ) {
}

bool App::handleEvent(const Window::Event& ev) {
    // Close the program
    if (ev.type == Window::EventType_Close) {
        m_window.showModalMessage("Exiting...");
        delete this;
        return true;
    }

    Action action = m_action;
    m_action = Action_None;
    String name;
    Mat4f mat;

    // Handle all the possible actions
    switch (action) {
        case Action_None:
            break;
        case Action_LoadMesh:
            name = m_window.showFileLoadDialog("Load mesh or state", getMeshImportFilter( )+",dat:State file");
            if (name.getLength( ))
                if (name.endsWith(".dat"))
                    m_common_ctrl.loadState(name);
                else
                    loadMesh(name);
            break;
        case Action_ReloadMesh:
            if (m_mesh_file_name.getLength( ))
                loadMesh(m_mesh_file_name);
            break;
        case Action_SaveMesh:
            name = m_window.showFileSaveDialog("Save mesh", getMeshExportFilter( ));
            if (name.getLength( ))
                saveMesh(name);
            break;
        case Action_ResetCamera:
            if (m_mesh) {
                m_camera_ctrl.initForMesh(m_mesh.get( ));
                m_common_ctrl.message("Camera reset");
            }
            break;
        case Action_EncodeCameraSignature:
            m_window.setVisible(false);
            printf("\nCamera signature:\n");
            printf("%s\n", m_camera_ctrl.encodeSignature( ).getPtr( ));
            waitKey( );
            break;
        case Action_DecodeCameraSignature:
        {
            m_window.setVisible(false);
            printf("\nEnter camera signature:\n");

            char buf[1024];
            if (scanf_s("%s", buf, FW_ARRAY_SIZE(buf)))
                m_camera_ctrl.decodeSignature(buf);
            else
                setError("Signature too long!");

            if (!hasError( ))
                printf("Done.\n\n");
            else {
                printf("Error: %s\n", getError( ).getPtr( ));
                clearError( );
                waitKey( );
            }
        }
            break;
        case Action_NormalizeScale:
            if (m_mesh) {
                Vec3f lo, hi;
                m_mesh->getBBox(lo, hi);
                m_mesh->xform(Mat4f::scale(Vec3f(2.0f / (hi - lo).max( ))) * Mat4f::translate((lo + hi) * -0.5f));
            }
            break;
        case Action_FlipXY:
            std::swap(mat.col(0), mat.col(1));
            if (m_mesh) {
                m_mesh->xform(mat);
                m_mesh->flipTriangles( );
            }
            break;
        case Action_FlipYZ:
            std::swap(mat.col(1), mat.col(2));
            if (m_mesh) {
                m_mesh->xform(mat);
                m_mesh->flipTriangles( );
            }
            break;
        case Action_FlipZ:
            mat.col(2) = -mat.col(2);
            if (m_mesh) {
                m_mesh->xform(mat);
                m_mesh->flipTriangles( );
            }
            break;
        case Action_NormalizeNormals:
            if (m_mesh)
                m_mesh->xformNormals(mat.getXYZ( ), true);
            break;
        case Action_FlipNormals:
            mat = -mat;
            if (m_mesh)
                m_mesh->xformNormals(mat.getXYZ( ), false);
            break;
        case Action_RecomputeNormals:
            if (m_mesh)
                m_mesh->recomputeNormals( );
            break;
        case Action_FlipTriangles:
            if (m_mesh)
                m_mesh->flipTriangles( );
            break;
        case Action_CleanMesh:
            if (m_mesh)
                m_mesh->clean( );
            break;
        case Action_CollapseVertices:
            if (m_mesh)
                m_mesh->collapseVertices( );
            break;
        case Action_DupVertsPerSubmesh:
            if (m_mesh)
                m_mesh->dupVertsPerSubmesh( );
            break;
        case Action_FixMaterialColors:
            if (m_mesh)
                m_mesh->fixMaterialColors( );
            break;
        case Action_DownscaleTextures:
            if (m_mesh)
                downscaleTextures(m_mesh.get( ));
            break;
        case Action_ChopBehindNear:
            if (m_mesh) {
                Mat4f worldToClip = m_camera_ctrl.getCameraToClip( ) * m_camera_ctrl.getWorldToCamera( );
                Vec4f pleq = worldToClip.getRow(2) + worldToClip.getRow(3);
                chopBehindPlane(m_mesh.get( ), pleq);
            }
            break;
        case Action_PlaceLightSourceAtCamera:
            m_area_light->setOrientation(m_camera_ctrl.getCameraToWorld( ).getXYZ( ));
            m_area_light->setPosition(m_camera_ctrl.getPosition( ));
            m_common_ctrl.message("Placed light at camera");
            break;
        case Action_PathTraceMode:
            m_rt_mode = !m_rt_mode;
            if (m_rt_mode) {
                if (m_img.getSize() != m_window.getSize()) {
                    // Replace m_img with a new Image. TODO: Clean this up.
                    m_img.~Image();
                    new (&m_img) Image(m_window.getSize(), ImageFormat::RGBA_Vec4f);	// placement new, will get autodestructed
                }
                m_pathtrace_renderer->setNormalMapped(m_normal_mapped);
                m_pathtrace_renderer->setRussianRoulette(m_use_russian_roulette);
                m_pathtrace_renderer->startPathTracingProcess(m_mesh.get( ), m_area_light.get( ), m_rt.get( ), &m_img, m_num_bounces, m_camera_ctrl);
            } else {
                m_pathtrace_renderer->stop();
            }
            break;
        default:
            FW_ASSERT(false);
            break;
    }

    m_window.setVisible(true);

    if (ev.type == Window::EventType_Paint)
        renderFrame(m_window.getGL());
    m_window.repaint();
    return false;
}

void App::readState(StateDump& d) {
    String mesh_file_name;

    d.pushOwner("App");
    d.get(mesh_file_name, "m_mesh_file_name");
    d.get((S32&)m_cull_mode, "m_cull_mode");
    d.get((S32&)m_num_bounces, "m_num_bounces");
    d.get((bool&)m_use_russian_roulette, "m_use_russian_roulette");
    d.popOwner();

    m_area_light->readState(d);
    m_light_size = m_area_light->getSize().x;	// dirty; doesn't allow for rectangular lights, only square. TODO

    if (m_mesh_file_name != mesh_file_name && mesh_file_name.getLength()) {
        loadMesh(mesh_file_name);
    }
}

void App::writeState(StateDump& d) const {
    d.pushOwner("App");
    d.set(m_mesh_file_name, "m_mesh_file_name");
    d.set((S32)m_cull_mode, "m_cull_mode");
    d.set((S32&)m_num_bounces, "m_num_bounces");
    d.set((bool&)m_use_russian_roulette, "m_use_russian_roulette");
    d.popOwner();

    m_area_light->writeState(d);
}

void App::waitKey(void) {
    printf("Press any key to continue . . . ");
    _getch();
    printf("\n\n");
}

void App::renderFrame(GLContext* gl) {
    // Setup transformations.

    Mat4f world_to_camera = m_camera_ctrl.getWorldToCamera( );
    Mat4f projection = gl->xformFitToView(Vec2f(-1.0f, -1.0f), Vec2f(2.0f, 2.0f)) * m_camera_ctrl.getCameraToClip();

    glClearColor(0.2f, 0.4f, 0.8f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_rt_mode) {
        // if we are computing radiosity, refresh mesh colors every 0.5 seconds
        if (m_pathtrace_renderer->isRunning()) {
            m_pathtrace_renderer->updatePicture(&m_img);
            m_pathtrace_renderer->checkFinish();

            // restart cycle
            m_update_clock.start();
        }
        gl->drawImage(m_img, Vec2f(0));
        return;
    }

    // Initialize GL state.
    glEnable(GL_DEPTH_TEST);
    if (m_cull_mode == CullMode_None)
        glDisable(GL_CULL_FACE);
    else {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace((m_cull_mode == CullMode_CW) ? GL_CCW : GL_CW);
    }

    // No mesh => skip.
    if (!m_mesh) {
        gl->drawModalMessage("No mesh loaded!");
        return;
    }

    // Render.
    if (!gl->getConfig().isStereo)
        renderScene(gl, world_to_camera, projection);
    else {
        glDrawBuffer(GL_BACK_LEFT);
        renderScene(gl, m_camera_ctrl.getCameraToLeftEye() * world_to_camera, projection);
        glDrawBuffer(GL_BACK_RIGHT);
        glClear(GL_DEPTH_BUFFER_BIT);
        renderScene(gl, m_camera_ctrl.getCameraToRightEye() * world_to_camera, projection);
        glDrawBuffer(GL_BACK);
    }

    m_area_light->setSize(Vec2f(m_light_size));
    m_area_light->draw(world_to_camera, projection);

    // Display status line.
    m_common_ctrl.message(sprintf("Triangles = %d, vertices = %d, materials = %d",
        m_mesh->numTriangles(), m_mesh->numVertices(), m_mesh->numSubmeshes()), "meshStats");
}

void App::renderScene(GLContext* gl, const Mat4f& world_to_camera, const Mat4f& projection) {
    // Draw mesh.
    if (m_mesh)
        m_mesh->draw(gl, world_to_camera, projection);
}

const static F32 texAttrib[] = {
    0, 1,
    1, 1,
    0, 0,
    1, 0
};

void App::loadMesh(const String& file_name) {
    m_pathtrace_renderer->stop();
    m_rt_mode = false;

    // find the scene name; the file path without preceding folders and file extension
    std::string path = std::string(file_name.getPtr());
    size_t begin = path.find_last_of("/\\")+1,
        end = path.find_last_of(".");
    m_results.scene_name = path.substr(begin, end - begin);
    std::cout << "Scene name: " << m_results.scene_name << std::endl;

    m_window.showModalMessage(sprintf("Loading mesh from '%s'...", file_name.getPtr()));
    String old_error = clearError();
    std::unique_ptr<MeshBase> mesh((MeshBase*)importMesh(file_name));

    String new_error = getError();

    if (restoreError(old_error)) {
        m_common_ctrl.message(sprintf("Error while loading '%s': %s", file_name.getPtr(), new_error.getPtr()));
        return;
    }

    m_mesh_file_name = file_name;

    // convert input mesh to colored format
    m_mesh.reset(new MeshWithColors(*mesh));
    // fix input colors to white so we see something
    for (S32 i = 0; i < m_mesh->numVertices( ); ++i)
        m_mesh->mutableVertex(i).c = Vec3f(1, 1, 1);
    m_common_ctrl.message(sprintf("Loaded mesh from '%s'", file_name.getPtr()));

    // build the BVH!
    constructTracer();
}

void App::saveMesh(const String& fileName) {
    if (!m_mesh) {
        m_common_ctrl.message("No mesh to save!");
        return;
    }

    m_window.showModalMessage(sprintf("Saving mesh to '%s'...", fileName.getPtr()));
    String old_error = clearError();
    exportMesh(fileName, m_mesh.get());
    String new_error = getError();

    if (restoreError(old_error)) {
        m_common_ctrl.message(sprintf("Error while saving '%s': %s", fileName.getPtr(), new_error.getPtr()));
        return;
    }

    m_mesh_file_name = fileName;
    m_common_ctrl.message(sprintf("Saved mesh to '%s'", fileName.getPtr( )));
}

// This function iterates over all the "sub-meshes" (parts of the object with different materials),
// heaps all the vertices and triangles together, and calls the BVH constructor.
// It is the responsibility of the tree to free the data when deleted.
// This functionality is _not_ part of the RayTracer class in order to keep it separate
// from the specifics of the Mesh class.
void App::constructTracer() {
    // fetch vertex and triangle data ----->
    m_rt_triangles.clear();
    m_rt_triangles.reserve(m_mesh->numTriangles( ));

    for (U32 i = 0; i < m_mesh->numSubmeshes(); i++) {
        const Array<Vec3i>& idx = m_mesh->indices(i);
        for (U32 j = 0; j < idx.getSize(); j++) {
            const VertexPNTC &v0 = m_mesh->vertex(idx[j][0]);
            const VertexPNTC &v1 = m_mesh->vertex(idx[j][1]);
            const VertexPNTC &v2 = m_mesh->vertex(idx[j][2]);

            RTTriangle t = RTTriangle(v0, v1, v2);
            t.m_data.vertex_indices = idx[j];
            t.m_material = &(m_mesh->material(i));
            m_rt_triangles.push_back(t);
        }
    }

    // compute checksum
    m_rt_vertex_positions.clear();
    m_rt_vertex_positions.reserve(m_mesh->numVertices());
    for (int i = 0; i < m_mesh->numVertices(); ++i) {
        m_rt_vertex_positions.push_back(m_mesh->vertex(i).p);
    }

    String md5 = RayTracer::computeMD5(m_rt_vertex_positions);
    FW::printf("Mesh MD5: %s\n", md5.getPtr());

    // construct a new ray tracer (deletes the old one if there was one)
    m_rt.reset(new RayTracer());

    bool try_load_hierarchy = true;
    // always construct when measuring performance
    if (m_settings.batch_render) {
        try_load_hierarchy = false;
    }
    if (try_load_hierarchy) {
        // check if saved hierarchy exists
#ifdef _WIN64
        String hierarchyCacheFile = String("Hierarchy-") + md5 + String("-x64.bin");
#else
        String hierarchyCacheFile = String("Hierarchy-") + md5 + String("-x86.bin");
#endif
        if (fileExists(hierarchyCacheFile.getPtr())) {
            // yes, load!
            m_rt->loadHierarchy(hierarchyCacheFile.getPtr(), m_rt_triangles);
            ::printf("Loaded hierarchy from %s\n", hierarchyCacheFile.getPtr());
        } else {
            // no, construct...
            m_rt->constructHierarchy(m_rt_triangles, m_settings.split_mode);
            // .. and save!
            m_rt->saveHierarchy(hierarchyCacheFile.getPtr(), m_rt_triangles);
            ::printf("Saved hierarchy to %s\n", hierarchyCacheFile.getPtr());
        }
    } else {
        // nope, bite the bullet and construct it
        auto start = std::chrono::steady_clock::now();
        m_rt->constructHierarchy(m_rt_triangles, m_settings.split_mode);
        auto end = std::chrono::steady_clock::now();
        m_results.build_time = (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }
}

void App::downscaleTextures(MeshBase* mesh) {
    FW_ASSERT(mesh);
    Hash<const Image*, Texture> hash;
    for (U32 submesh_idx = 0; submesh_idx < mesh->numSubmeshes(); submesh_idx++)
        for (U32 texture_idx = 0; texture_idx < MeshBase::TextureType_Max; texture_idx++) {
            Texture& tex = mesh->material(submesh_idx).textures[texture_idx];
            if (tex.exists()) {
                const Image* orig = tex.getImage();
                if (!hash.contains(orig)) {
                    Image* scaled = orig->downscale2x();
                    hash.add(orig, (scaled) ? Texture(scaled, tex.getID()) : tex);
                }
                tex = hash.get(orig);
            }
        }
}

void App::chopBehindPlane(MeshBase* mesh, const Vec4f& pleq) {
    FW_ASSERT(mesh);
    S32 pos_attrib = mesh->findAttrib(MeshBase::AttribType_Position);
    if (pos_attrib == -1) {
        return;
    }

    for (U32 submesh_idx = 0; submesh_idx < mesh->numSubmeshes(); submesh_idx++) {
        Array<Vec3i>& inds = mesh->mutableIndices(submesh_idx);
        U32 tri_out = 0;
        for (U32 tri_in = 0; tri_in < inds.getSize(); tri_in++) {
            if (dot(mesh->getVertexAttrib(inds[tri_in].x, pos_attrib), pleq) >= 0.0f ||
                dot(mesh->getVertexAttrib(inds[tri_in].y, pos_attrib), pleq) >= 0.0f ||
                dot(mesh->getVertexAttrib(inds[tri_in].z, pos_attrib), pleq) >= 0.0f) {
                inds[tri_out++] = inds[tri_in];
            }
        }
        inds.resize(tri_out);
    }
    mesh->clean();
}

bool App::fileExists(const String& fn) {
    FILE* p_f = fopen(fn.getPtr(), "rb");
    if (p_f != 0) {
        fclose(p_f);
        return true;
    } else {
        return false;
    }
}

} // namespace BDPT
} // namespace FW