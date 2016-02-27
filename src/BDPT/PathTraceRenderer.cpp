#include "PathTraceRenderer.hpp"
#include "RayTracer.hpp"
#include "AreaLight.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>



namespace FW {
	bool DEBUG = false;

	bool PathTraceRenderer::m_normalMapped = true;
	bool PathTraceRenderer::m_russian_roulette = true;

	void PathTraceRenderer::getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular)
	{
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
		n = (hit.u * hit.tri->m_vertices[1].n + hit.v * hit.tri->m_vertices[2].n + (1 - hit.u - hit.v)*hit.tri->m_vertices[0].n).normalized();
		specular = n;		        // Dirty, but not using specular for anything else TODO: Clean this!
		if (m_normalMapped && mat->textures[MeshBase::TextureType_Normal].exists()){
			const Texture& n_tex = mat->textures[MeshBase::TextureType_Normal];
			const Image& n_teximg = *n_tex.getImage();

			Vec2i n_texelCoords = getTexelCoords(tex_uv, n_teximg.getSize());
			Vec3f nm = (2.0f * n_teximg.getVec4f(n_texelCoords).getXYZ() - 1.0f).normalized();

			Vec3f d_pos1 = hit.tri->m_vertices[1].p - hit.tri->m_vertices[0].p;
			Vec3f d_pos2 = hit.tri->m_vertices[2].p - hit.tri->m_vertices[0].p;
			Vec2f d_uv1 = hit.tri->m_vertices[1].t - hit.tri->m_vertices[0].t;
			Vec2f d_uv2 = hit.tri->m_vertices[2].t - hit.tri->m_vertices[0].t;

			float r = clamp(1.0f / (d_uv1.x * d_uv2.y - d_uv1.y * d_uv2.x), -10000.0f, 10000.0f);
			Vec3f T = ((d_pos1 * d_uv2.y - d_pos2 * d_uv1.y) * r).normalized();
			Vec3f B = -((d_pos2 * d_uv1.x - d_pos1 * d_uv2.x) * r).normalized();
			Mat3f TBN;
			TBN.setCol(0, T);
			TBN.setCol(1, B);
			TBN.setCol(2, n);
			n = (TBN * nm).normalized();
		}
	}


PathTracerContext::PathTracerContext()
    : m_bForceExit(false),
      m_bResidual(false),
      m_scene(nullptr),
      m_rt(nullptr),
      m_light(nullptr),
      m_pass(0),
      m_bounces(0),
      m_destImage(0),
      m_camera(nullptr)
{
}

PathTracerContext::~PathTracerContext()
{
}


PathTraceRenderer::PathTraceRenderer()
{
    m_raysPerSecond = 0.0f;
}

PathTraceRenderer::~PathTraceRenderer()
{
    stop();
}

// This function is responsible for asynchronously rendering paths for a given block.
void PathTraceRenderer::pathTraceBlock( MulticoreLauncher::Task& t )
{
    PathTracerContext& ctx = *(PathTracerContext*)t.data;

    const MeshWithColors* scene			= ctx.m_scene;
    RayTracer* rt						= ctx.m_rt;
    Image* image						= ctx.m_image.get();
    const CameraControls& cameraCtrl	= *ctx.m_camera;
    AreaLight* light					= ctx.m_light;

    // make sure we're on CPU
    image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1,-1), Vec2f(2,2), image->getSize())*cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // get the block which we are rendering
    PathTracerBlock& block = ctx.m_blocks[t.idx];

    static std::atomic<uint32_t> seed = 0;
    uint32_t current_seed = seed.fetch_add(1);
    Random R( t.idx + current_seed );	// this is bogus, just to make the random numbers change each iteration

#ifdef ISPC
    // Pre-allocate buffers
    std::vector<ISPCCheckNode> b_nodes;
    b_nodes.reserve(2*log2f(rt->bvh.ispc_hierarchy_.size()));
#else
    // Pre-allocate buffers
    std::vector<CheckNode> b_nodes;
    b_nodes.reserve(2*log2f(rt->bvh.hierarchy_.size()));
#endif // ISPC

    for ( int i = 0; i < block.m_width * block.m_height; ++i )
    {
        if( ctx.m_bForceExit ) {
            return;
        }

        Vec3f Ei;

        int pixel_x = block.m_x + (i % block.m_width);
        int pixel_y = block.m_y + (i / block.m_width);

		int dir_samples = 2;
		int samples_per_pixel = sqr(dir_samples);
		for (int i = 0; i < samples_per_pixel; i++){
			// generate ray through pixel
			float sample_width = 1.0f / dir_samples;
			float x = (pixel_x + (i % dir_samples) * sample_width + R.getF32(0, sample_width)) / image->getSize().x *  2.0f - 1.0f;
			float y = (pixel_y + (i / dir_samples) * sample_width + R.getF32(0, sample_width)) / image->getSize().y * -2.0f + 1.0f;
			// point on back plane in homogeneous coordinates
			Vec4f P1(x, y, 1.0f, 1.0f);

			// apply inverse projection, divide by w to get object-space points
			Vec4f Rdh = (invP * P1);
			Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();
			Vec3f Ro = cameraCtrl.getPosition();

			// Subtract front plane point from back plane point,
			// yields ray direction.
			// NOTE that it's not normalized; the direction Rd is defined
			// so that the segment to be traced is [Ro, Ro+Rd], i.e.,
			// intersections that come _after_ the point Ro+Rd are to be discarded.
			Rd = Rd - Ro; 

			Ei += traceRay(ctx, R, Ro, Rd, 0, b_nodes) / samples_per_pixel / FW_PI;
		}

        // Put pixel.
        Vec4f prev = image->getVec4f( Vec2i(pixel_x, pixel_y) );
        prev += Vec4f( Ei, 1.0f );
        image->setVec4f( Vec2i(pixel_x, pixel_y), prev );
    }
}

Vec3f PathTraceRenderer::traceRay(const PathTracerContext& ctx, Random& R, Vec3f& orig, Vec3f& dir, int current_bounce, std::vector<ISPCCheckNode>& b_nodes){
	RayTracer& rt = *ctx.m_rt;
	AreaLight& light = *ctx.m_light;
	Vec3f result;
	RaycastResult hit = rt.ispcRaycast(orig + 100 * FLT_EPSILON * dir, dir * ctx.m_camera->getFar(), b_nodes);
	if (hit.tri != nullptr){

		// Get the material properties
		Vec3f diffuse, mapped_n, interp_n;
		getTextureParameters(hit, diffuse, mapped_n, interp_n);
        if (dot(dir, mapped_n) > 0) {
            mapped_n *= -1;
        }

		// Sample the light
		float lpdf;
		Vec3f light_sample;
		light.sample(lpdf, light_sample, 0, R);
		Vec3f shadow_dir = light_sample - hit.point;
		auto shadow = rt.ispcRaycast(hit.point + 0.01f * shadow_dir, shadow_dir, b_nodes);
		if (shadow.tri == nullptr){
			float cos_l = clamp(dot(light.getNormal(), -shadow_dir.normalized()), 0.0f, 1.0f);
			float cos_t = clamp(dot(mapped_n, shadow_dir.normalized()), 0.0f, 1.0f);
			result += diffuse * light.getEmission() * cos_l * cos_t / (shadow_dir.lenSqr() * lpdf);
		}

		// Check for termination
		float rr_prob = 0.0f;
		bool terminate;
		if (m_russian_roulette && current_bounce >= ctx.m_bounces){
            rr_prob = 0.2f;
			terminate = R.getF32(0, 1) < rr_prob;
		} else if (!m_russian_roulette && current_bounce >= ctx.m_bounces){
			terminate = true;
        } else {
            terminate = false;
        }

		// Continue tracing if not terminated
		if (!terminate){
			float pdf;
			Vec3f new_dir;
			cosineSampleDirection(mapped_n, new_dir, pdf, R);
			if (dot(new_dir, interp_n) > 0){
				result += diffuse * traceRay(ctx, R, hit.point, new_dir, current_bounce + 1, b_nodes) / (1 - rr_prob);
			}
		}
	}
	return result;
}

void cosineSampleDirection(const Vec3f& n, Vec3f& new_dir, float& pdf, Random& R){
	Mat3f basis = formBasis(n);
	float x = 1, y = 1;
	while (sqr(x) + sqr(y) > 1){
		x = R.getF32(-1, 1);
		y = R.getF32(-1, 1);
	}
	float z = sqrt(1 - sqr(x) - sqr(y));
	new_dir = (basis * Vec3f(x, y, z)).normalized();
	pdf = clamp(dot(n, new_dir), 0.0f, 1.0f);
}

void PathTraceRenderer::startPathTracingProcess( const MeshWithColors* scene, AreaLight* light, RayTracer* rt, Image* dest, int bounces, const CameraControls& camera )
{
    FW_ASSERT( !m_context.m_bForceExit );

    m_context.m_bForceExit = false;
    m_context.m_bResidual = false;
    m_context.m_camera = &camera;
    m_context.m_rt = rt;
    m_context.m_scene = scene;
    m_context.m_light = light;
    m_context.m_pass = 0;
    m_context.m_bounces = bounces;
    m_context.m_image.reset(new Image( dest->getSize(), ImageFormat::RGBA_Vec4f));

    m_context.m_destImage = dest;
    m_context.m_image->clear();

    // Add rendering blocks.
    m_context.m_blocks.clear();
    {
        int block_size = 32;
        int image_width = dest->getSize().x;
        int image_height = dest->getSize().y;
        int block_count_x = (image_width + block_size - 1) / block_size;
        int block_count_y = (image_height + block_size - 1) / block_size;

        for(int y = 0; y < block_count_y; ++y) {
            int block_start_y = y * block_size;
            int block_end_y = FW::min(block_start_y + block_size, image_height);
            int block_height = block_end_y - block_start_y;

            for(int x = 0; x < block_count_x; ++x) {
                int block_start_x = x * block_size;
                int block_end_x = FW::min(block_start_x + block_size, image_width);
                int block_width = block_end_x - block_start_x;

                PathTracerBlock block;
                block.m_x = block_size * x;
                block.m_y = block_size * y;
                block.m_width = block_width;
                block.m_height = block_height;

                m_context.m_blocks.push_back(block);
            }
        }
    }

    dest->clear();

    // Fire away!

    // If you change this, change the one in checkFinish too.
	if (DEBUG)
		m_launcher.setNumThreads(1);
	else
		m_launcher.setNumThreads(m_launcher.getNumCores());

    m_launcher.popAll();
    m_launcher.push( pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size() );
}

void PathTraceRenderer::updatePicture( Image* dest )
{
    FW_ASSERT( m_context.m_image != 0 );
    FW_ASSERT( m_context.m_image->getSize() == dest->getSize() );

    for ( int i = 0; i < dest->getSize().y; ++i )
    {
        for ( int j = 0; j < dest->getSize().x; ++j )
        {
            Vec4f D = m_context.m_image->getVec4f(Vec2i(j,i));
            if ( D.w != 0.0f )
                D = D*(1.0f/D.w);

            // Gamma correction.
            Vec4f color = Vec4f(
                FW::pow(D.x, 1.0f / 2.2f),
                FW::pow(D.y, 1.0f / 2.2f),
                FW::pow(D.z, 1.0f / 2.2f),
                D.w
            );

            dest->setVec4f( Vec2i(j,i), color );
        }
    }
}

void PathTraceRenderer::checkFinish()
{
    // have all the vertices from current bounce finished computing?
    if ( m_launcher.getNumTasks() == m_launcher.getNumFinished() )
    {
        // yes, remove from task list
        m_launcher.popAll();

        ++m_context.m_pass;

        // uncomment this to write out a sequence of PNG images
        // after the completion of each full round through the image.
        //String fn = sprintf( "pt-%03dppp.png", m_context.m_pass );
        //File outfile( fn, File::Create );
        //exportLodePngImage( outfile, m_context.m_destImage );

        if ( !m_context.m_bForceExit )
        {
            // keep going

            // If you change this, change the one in startPathTracingProcess too.
			if (DEBUG)
				m_launcher.setNumThreads(1);
			else
				m_launcher.setNumThreads(m_launcher.getNumCores());

            m_launcher.popAll();
            m_launcher.push( pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size() );
            //::printf( "Next pass!" );
        }
        else ::printf( "Stopped." );
    }
}

void PathTraceRenderer::stop() {
    m_context.m_bForceExit = true;
    
    if ( isRunning() )
    {
        m_context.m_bForceExit = true;
        while( m_launcher.getNumTasks() > m_launcher.getNumFinished() )
        {
            Sleep( 1 );
        }
        m_launcher.popAll();
    }

    m_context.m_bForceExit = false;
}



} // namespace FW
