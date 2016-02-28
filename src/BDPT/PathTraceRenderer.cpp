#include "PathTraceRenderer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>

#include "gui/Image.hpp"
#include "io/File.hpp"
#include "io/ImageLodePngIO.hpp"

#include "AreaLight.hpp"
#include "RayTracer.hpp"


namespace FW 
{ namespace BDPT 
{

bool PathTraceRenderer::m_normal_mapped = true;
bool PathTraceRenderer::m_russian_roulette = true;

Vec3f cosineSampleDirection(const Vec3f& n, Random& R) {
	Mat3f basis = formBasis(n);
	F32 x = 1, y = 1;
    // Rejection sampling of a unit circle
	while (sqr(x) + sqr(y) > 1){
		x = R.getF32(-1, 1);
		y = R.getF32(-1, 1);
	}
    // Lift the point on the unit circle to unit ball
	F32 z = sqrt(1 - sqr(x) - sqr(y));
	return (basis * Vec3f(x, y, z)).normalized();
}

void PathTraceRenderer::getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular) {
    MeshBase::Material* mat = hit.tri->m_material;
    // Read value from albedo texture into diffuse.
    // If textured, use the texture; if not, use Material.diffuse.
    Vec2f tex_uv = hit.u * hit.tri->m_vertices[1].t + hit.v * hit.tri->m_vertices[2].t + (1 - hit.u - hit.v)*hit.tri->m_vertices[0].t;
    if (mat->textures[MeshBase::TextureType_Diffuse].exists()) {
        const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
        const Image& teximg = *tex.getImage();
        Vec2i texelCoords = getTexelCoords(tex_uv, teximg.getSize());
        diffuse = teximg.getVec4f(texelCoords).getXYZ();
        diffuse.x = pow(diffuse.x, 2.2f);
        diffuse.y = pow(diffuse.y, 2.2f);
        diffuse.z = pow(diffuse.z, 2.2f);
    }
    else {
        // no texture, use constant albedo from material structure.
        diffuse = mat->diffuse.getXYZ();
    }
    // Interpolate normal
    n = (hit.u * hit.tri->m_vertices[1].n + hit.v * hit.tri->m_vertices[2].n + (1 - hit.u - hit.v)*hit.tri->m_vertices[0].n).normalized();
    // Calculate normal mapping
    if (m_normal_mapped && mat->textures[MeshBase::TextureType_Normal].exists()){
        // Get the tangent space normal
        const Texture& n_tex = mat->textures[MeshBase::TextureType_Normal];
        const Image& n_teximg = *n_tex.getImage();
        Vec2i n_texelCoords = getTexelCoords(tex_uv, n_teximg.getSize());
        Vec3f nm = (2.0f * n_teximg.getVec4f(n_texelCoords).getXYZ() - 1.0f).normalized();

        // Calculate the transformation
        Vec3f d_pos1 = hit.tri->m_vertices[1].p - hit.tri->m_vertices[0].p;
        Vec3f d_pos2 = hit.tri->m_vertices[2].p - hit.tri->m_vertices[0].p;
        Vec2f d_uv1 = hit.tri->m_vertices[1].t - hit.tri->m_vertices[0].t;
        Vec2f d_uv2 = hit.tri->m_vertices[2].t - hit.tri->m_vertices[0].t;

        F32 r = clamp(1.0f / (d_uv1.x * d_uv2.y - d_uv1.y * d_uv2.x), -10000.0f, 10000.0f);
        Vec3f T = ((d_pos1 * d_uv2.y - d_pos2 * d_uv1.y) * r).normalized();
        Vec3f B = -((d_pos2 * d_uv1.x - d_pos1 * d_uv2.x) * r).normalized();
        Mat3f TBN;
        TBN.setCol(0, T);
        TBN.setCol(1, B);
        TBN.setCol(2, n);
        n = (TBN * nm).normalized();
    }
}

// This function is responsible for asynchronously rendering paths for a given block.
void PathTraceRenderer::pathTraceBlock(MulticoreLauncher::Task& task) {
    PathTracerContext& ctx = *(PathTracerContext*)task.data;
    RayTracer* rt = ctx.rt;
    Image* image = ctx.image.get();
    const CameraControls& cameraCtrl = *ctx.camera;

    // get camera orientation and projection
    Mat4f world_to_camera = cameraCtrl.getWorldToCamera();
    Mat4f camera_to_clip = Mat4f::fitToView(Vec2f(-1,-1), Vec2f(2,2), image->getSize()) * cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f clip_to_world = (camera_to_clip * world_to_camera).inverted();

    // get the block which we are rendering
    PathTracerBlock& block = ctx.blocks[task.idx];

    // Initialize rng
    static std::atomic<U32> seed = 0;
    U32 current_seed = seed.fetch_add(1);
    Random R(task.idx + current_seed);	// this is bogus, just to make the random numbers change each iteration

#ifdef ISPC
    // Pre-allocate traversal stack
    std::vector<ISPCCheckNode> b_nodes;
    b_nodes.reserve(2*log2f(rt->m_bvh.m_ispc_hierarchy.size()));
#else
    // Pre-allocate traversal stack
    std::vector<CheckNode> b_nodes;
    b_nodes.reserve(2*log2f(rt->m_bvh.m_hierarchy.size()));
#endif // ISPC

    for (U32 i = 0; i < block.width * block.height; i++) {
        // End if we're stopping
        if(ctx.force_exit) {
            return;
        }

        Vec3f Ei(0);
        // Position of the current pixel
        U32 pixel_x = block.x + (i % block.width);
        U32 pixel_y = block.y + (i / block.width);
		U32 dir_samples = 2;
		U32 samples_per_pixel = sqr(dir_samples);
		F32 sample_width = 1.0f / dir_samples;
        // Stratified sampling of the pixel
		for (U32 i = 0; i < samples_per_pixel; i++){
			// Generate ray through pixel
			float x = (pixel_x + (i % dir_samples) * sample_width + R.getF32(0, sample_width)) / image->getSize().x *  2.0f - 1.0f;
			float y = (pixel_y + (i / dir_samples) * sample_width + R.getF32(0, sample_width)) / image->getSize().y * -2.0f + 1.0f;
			// point on back plane in homogeneous coordinates
			Vec4f p1(x, y, 1.0f, 1.0f);
			// apply inverse projection, divide by w to get object-space points
			Vec4f Rdh = (clip_to_world * p1);
			Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();
			Vec3f Ro = cameraCtrl.getPosition();
			// Subtract front plane point from back plane point,
			// yields ray direction.
			// NOTE that it's not normalized; the direction Rd is defined
			// so that the segment to be traced is [Ro, Ro+Rd], i.e.,
			// intersections that come _after_ the point Ro+Rd are to be discarded.
			Rd = Rd - Ro; 

            // Trace!
			Ei += traceRay(ctx, R, Ro, Rd, 0, b_nodes) / samples_per_pixel / FW_PI;
		}

        // Put pixel.
        Vec4f prev = image->getVec4f(Vec2i(pixel_x, pixel_y));
        prev += Vec4f(Ei, 1.0f);
        image->setVec4f(Vec2i(pixel_x, pixel_y), prev);
    }
}


#ifdef ISPC
Vec3f PathTraceRenderer::traceRay(const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, U32 current_bounce, std::vector<ISPCCheckNode>& b_nodes){
#else
Vec3f PathTraceRenderer::traceRay(const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, U32 current_bounce, std::vector<CheckNode>& b_nodes){
#endif

	RayTracer& rt = *ctx.rt;
	AreaLight& light = *ctx.light;
	Vec3f result(0);

#ifdef ISPC
	RaycastResult hit = rt.ispcRaycast(orig + 100 * FLT_EPSILON * dir, dir * ctx.camera->getFar(), b_nodes);
#else
	RaycastResult hit = rt.raycast(orig + 100 * FLT_EPSILON * dir, dir * ctx.camera->getFar(), b_nodes);
#endif
    // Did we hit anything?
	if (hit.tri != nullptr){
		// Get the material properties
		Vec3f diffuse, mapped_n, specular;
		getTextureParameters(hit, diffuse, mapped_n, specular);
        // Flip normal if it points the wrong way
        if (dot(dir, mapped_n) > 0) {
            mapped_n *= -1;
        }

		// Sample the light
		F32 lpdf;
		Vec3f light_sample;
		light.sample(lpdf, light_sample, R);
		Vec3f shadow_dir = light_sample - hit.point;
#ifdef ISPC
		auto shadow = rt.ispcRaycast(hit.point + 0.01f * shadow_dir, shadow_dir, b_nodes);
#else
		auto shadow = rt.raycast(hit.point + 0.01f * shadow_dir, shadow_dir, b_nodes);
#endif
        // Add the direct illumination if the triangle is not in shadow
		if (shadow.tri == nullptr){
			F32 cos_l = clamp(dot(light.getNormal(), -shadow_dir.normalized()), 0.0f, 1.0f);
			F32 cos_t = clamp(dot(mapped_n, shadow_dir.normalized()), 0.0f, 1.0f);
			result += diffuse * light.getEmission() * cos_l * cos_t / (shadow_dir.lenSqr() * lpdf);
		}

		// Check for termination
		F32 pdf = 1.0f;
		bool terminate;
		if (m_russian_roulette && current_bounce >= ctx.bounces){
            // Terminate ray pased on russian roulette
            F32 rr_prob = 0.2f;
            pdf *= (1 - rr_prob);
			terminate = R.getF32(0, 1) < rr_prob;
		} else if (!m_russian_roulette && current_bounce >= ctx.bounces){
			terminate = true;
        } else {
            terminate = false;
        }
		// Continue tracing if not terminated
		if (!terminate){
			Vec3f new_dir = cosineSampleDirection(mapped_n, R);
			result += diffuse * traceRay(ctx, R, hit.point, new_dir, current_bounce + 1, b_nodes) / pdf;
		}
	}
	return result;
}

void PathTraceRenderer::startPathTracingProcess(const MeshWithColors* scene, AreaLight* light, RayTracer* rt, Image* dest, U32 bounces, const CameraControls& camera) {
    FW_ASSERT(!m_context.force_exit);

    // Initialize the context
    m_context.force_exit = false;
    m_context.camera = &camera;
    m_context.rt = rt;
    m_context.scene = scene;
    m_context.light = light;
    m_context.pass = 0;
    m_context.bounces = bounces;
    m_context.image.reset(new Image(dest->getSize(), ImageFormat::RGBA_Vec4f));
    m_context.dest_image = dest;
    m_context.image->clear();

    // Add rendering blocks.
    m_context.blocks.clear();
    U32 block_size = 32;
    U32 image_width = dest->getSize().x;
    U32 image_height = dest->getSize().y;
    U32 block_count_x = (image_width + block_size - 1) / block_size;
    U32 block_count_y = (image_height + block_size - 1) / block_size;

    for (U32 y = 0; y < block_count_y; ++y) {
        U32 block_start_y = y * block_size;
        U32 block_end_y = FW::min(block_start_y + block_size, image_height);
        U32 block_height = block_end_y - block_start_y;

        for (U32 x = 0; x < block_count_x; ++x) {
            U32 block_start_x = x * block_size;
            U32 block_end_x = FW::min(block_start_x + block_size, image_width);
            U32 block_width = block_end_x - block_start_x;

            PathTracerBlock block;
            block.x = block_size * x;
            block.y = block_size * y;
            block.width = block_width;
            block.height = block_height;
            m_context.blocks.push_back(block);
        }
    }
    
    dest->clear();

    // If you change this, change the one in checkFinish too.
	m_launcher.setNumThreads(m_launcher.getNumCores());

    // Send the task to the launcher
    m_launcher.popAll();
    m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.blocks.size());
}

void PathTraceRenderer::updatePicture(Image* dest) {
    FW_ASSERT(m_context.image != 0);
    FW_ASSERT(m_context.image->getSize() == dest->getSize());

    // Loop all the pixels and set dest to match current image
    for (int i = 0; i < dest->getSize().y; ++i) {
        for (int j = 0; j < dest->getSize().x; ++j) {
            Vec4f d = m_context.image->getVec4f(Vec2i(j,i));
	        // normalize by 1/w
            if (d.w != 0.0f) {
                d /= d.w;
            }
            // Gamma correction.
            Vec4f color = Vec4f(
                FW::pow(d.x, 1.0f / 2.2f),
                FW::pow(d.y, 1.0f / 2.2f),
                FW::pow(d.z, 1.0f / 2.2f),
                d.w
            );
            dest->setVec4f(Vec2i(j,i), color);
        }
    }
}

void PathTraceRenderer::checkFinish() {
    // have all the vertices from current bounce finished computing?
    if (m_launcher.getNumTasks() == m_launcher.getNumFinished()) {
        // yes, remove from task list
        m_launcher.popAll();
        ++m_context.pass;

        // uncomment this to write out a sequence of PNG images
        // after the completion of each full round through the image.
        // String file_name = sprintf("pt-%03dppp.png", m_context.pass);
        // File outfile(file_name, File::Create);
        // exportLodePngImage(outfile, m_context.dest_image);

        if (!m_context.force_exit) {
            // keep going
            // If you change this, change the one in startPathTracingProcess too.
            m_launcher.setNumThreads(m_launcher.getNumCores());

            m_launcher.popAll();
            m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.blocks.size());
            printf("Pass %d completed.\n", m_context.pass);
        } else {
            printf("Stopped.\n");
        }
    }
}

void PathTraceRenderer::stop() {
    if (isRunning()) {
        m_context.force_exit = true;
        while(m_launcher.getNumTasks() > m_launcher.getNumFinished()) {
            Sleep(1);
        }
        m_launcher.popAll();
    }
    m_context.force_exit = false;
}

} // namespace BDPT
} // namespace FW
