#pragma once

#include "3d/CameraControls.hpp"
#include "3d/Mesh.hpp"
#include "base/Random.hpp"

#include <vector>

struct timingResult {
	int duration;
	int rayCount;
};

namespace FW
{

class RayTracer;
struct RaycastResult;
struct RTTriangle;
class Image;

// This class contains functionality to render pictures using a ray tracer.
class Renderer
{
public:
	Renderer();
	~Renderer();

	enum ShadingMode
	{
		ShadingMode_Headlight = 0,
		ShadingMode_AmbientOcclusion,
		ShadingMode_Whitted
	};

	// draw a picture of the mesh from the viewpoint specified by the matrix.
	// shading mode determined by the enum.
	timingResult		rayTracePicture(RayTracer*, Image*, const CameraControls& camera, ShadingMode mode);

	// add all the emissive triangles in the mesh into m_lightTriangles
	void				gatherLightTriangles(RayTracer* rt);

	// controls how long the ambient occlusion rays are.
	// what looks good depends on the scene.
	void				setAORayLength(float len)		{ m_aoRayLength = len; }

	// how many ambient occlusion rays to shoot per primary ray.
	void				setAONumRays(int i)			{ m_aoNumRays = i; }

	// how many primary rays to shoot per pixel.
	void				setAANumRays(int i)			{ m_aaNumRays = i; }

	// the maximum number of bounces for the whitted integrator (extra).
	void				setWhittedBounces(int i)			{ m_whittedBounces = i; }

	void				setUseReflections(bool b)	{ m_useReflections = b; }
	void				setUseToneMapping(bool b)	{ m_useToneMapping = b; }
	void				setUseTextures(bool b)		{ m_useTextures = b; }
	void				setNormalMapping(bool b)	{ m_normalMapped = b; }
	void				setUseAreaLights(bool b)	{ m_useAreaLights = b; }
	void				setPointLightPos(Vec3f v)	{ m_pointLightPos = v; }
	void				setTextureFiltering(bool b) { m_filterTextures = b; }



    float				getRaysPerSecond					( void )			{ return m_raysPerSecond; }


protected:

    // simple headlight shading, ray direction dot surface normal
    Vec4f				computeShadingHeadlight				(const RaycastResult& hit, const CameraControls& cameraCtrl);

    // implement ambient occlusion as per the instructions
	Vec4f				computeShadingAmbientOcclusion		(RayTracer* rt, const RaycastResult& hit, const CameraControls& cameraCtrl, Random& rnd);

	// EXTRA: implement the whitted integrator extra
	Vec4f				computeShadingWhitted				(RayTracer* rt, const RaycastResult& hit, const CameraControls& cameraCtrl, Random& rnd, int num_bounces);

	// EXTRA: gets all parameters from the material's textures.
	void				getTextureParameters				(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular);

    float						m_aoRayLength;
	int							m_aoNumRays;
	int							m_aaNumRays;
	int							m_whittedBounces;
	bool						m_useToneMapping;
	bool						m_useReflections;
	bool						m_useTextures;
	bool						m_useAreaLights;
	bool						m_normalMapped;
	bool						m_filterTextures;
	float						m_combinedLightArea;
	Vec3f						m_pointLightPos; // Position for the simple pointlight you can use for the whitted integrator.
	std::vector<RTTriangle*>	m_lightTriangles;// Contains all triangles with an emission of over 0 in the scene. 
												 // Can be used for the area lights extra in order to sample random light-emitting triangles in the scene.

    __int64						m_s64TotalRays;
    float						m_raysPerSecond;
};

}	// namespace FW
