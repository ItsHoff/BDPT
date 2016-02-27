#pragma once

#include <base/Math.hpp>
#include <base/Random.hpp>
#include <io/StateDump.hpp>


namespace FW
{

class GLContext;

//------------------------------------------------------------------------
// a simple square-shaped area light source.

class AreaLight
{
public:
    AreaLight() : m_E(100,100,100), m_size(0.25f,0.25f) { }

    // this function draws samples on the light source for computing direct illumination
    // the "base" input can be used for driving QMC samplers; unless you do something to it yourself, has no effect.
    void			sample( float& pdf, Vec3f& p, int base, Random& rnd );

    Vec3f			getPosition(void) const			{ return Vec4f(m_xform.getCol(3)).getXYZ(); }
    void			setPosition(const Vec3f& p)		{ m_xform.setCol(3, Vec4f(p, 1.0f)); }

    Mat3f			getOrientation(void) const		{ return m_xform.getXYZ(); }
    void			setOrientation(const Mat3f& R)	{ m_xform.setCol(0,Vec4f(R.getCol(0),0.0f)); m_xform.setCol(1,Vec4f(R.getCol(1),0.0f)); m_xform.setCol(2,Vec4f(R.getCol(2),0.0f)); }

    Vec3f			getNormal(void) const			{ return -Vec4f(m_xform.getCol(2)).getXYZ(); }

    Vec2f			getSize(void) const				{ return m_size; }
    void			setSize(const Vec2f& s)			{ m_size = s; }

    Vec3f			getEmission(void) const			{ return m_E; }
    void			setEmission(const Vec3f& E)		{ m_E = E; }

    void			draw( const Mat4f& worldToCamera, const Mat4f& projection  );

    void			readState( StateDump& d )			{ d.pushOwner("areaLight"); d.get(m_xform,"xform"); d.get(m_size,"size"); d.get(m_E,"E"); d.popOwner(); }
    void			writeState( StateDump& d ) const	{ d.pushOwner("areaLight"); d.set(m_xform,"xform"); d.set(m_size,"size"); d.set(m_E,"E"); d.popOwner(); }

protected:
    Mat4f	m_xform;	// Encodes position and orientation in world space.
    Vec2f	m_size;		// Physical size of the emitter from the center of the light. I.e. half of the total width/height.
    Vec3f	m_E;		// Diffuse emission (W/m^2).
};

} // namespace FW
