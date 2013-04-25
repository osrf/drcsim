
#ifndef __AtlasVectorTypes_H
#define __AtlasVectorTypes_H

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

#ifndef NOT_IN_DOXYGEN
namespace Atlas {
#endif

/****************************************************************************/
//!
//!  \class AtlasVec4f AtlasVectorTypes.h
//!
//!  \brief The AtlasVec4f class is a light-weight 4 vector.
//!
class AtlasVec4f
{

public:

	//!
	//! \brief  Default constructor.  Vector values are uninitialized.
	//!
	AtlasVec4f() {}

	//!
	//! \brief  Constructor.  All vector values initialized to passed value.
	//!
	//! \_Parameters
	//!
	//!    \_in   initial_value - initial value of all vector values
	//!
	AtlasVec4f(float initial_value)
	{
		n[0] = n[1] = n[2] = n[3] = initial_value;
	}

	//!
	//! \brief  Constructor.  Vector values initialized to passed values.
	//!
	//! \_Parameters
	//!
	//!    \_in   x - 1st vector value
	//!    \_in   y - 2nd vector value
	//!    \_in   z - 3rd vector value
	//!    \_in   w - 4th vector value
	//!
	AtlasVec4f(float x, float y, float z, float w)
	{
		n[0] = x;
		n[1] = y;
		n[2] = z;
		n[3] = w;
	}

	float n[4];   //!<  Current vector values.  Can be accessed directly or through accessor functions.

	//
	//  Syntactic sugar for interpreting n[] as positions, rotations, etc.
	//
	inline float x() {return n[0];}   //!<  x value if vector is interpreted as position
	inline float y() {return n[1];}   //!<  y value of position
	inline float z() {return n[2];}   //!<  z value of position
	inline float w() {return n[3];}   //!<  w value of position

	inline void set_x(float f) {n[0] = f;}   //!<  set x value of position
	inline void set_y(float f) {n[1] = f;}   //!<  set y value of position
	inline void set_z(float f) {n[2] = f;}   //!<  set z value of position
	inline void set_w(float f) {n[3] = f;}   //!<  set w value of position

	inline float rz() {return n[0];}  //!<  z axis rotation if vector is interpreted as euler angle set
	inline float rx() {return n[1];}  //!<  x axis rotation of euler angle set
	inline float ry() {return n[2];}  //!<  y axis rotation of euler angle set

	inline void set_rz(float f) {n[0] = f;}   //!<  set z axis rotation of euler angle
	inline void set_rx(float f) {n[1] = f;}   //!<  set x axis rotation of euler angle
	inline void set_ry(float f) {n[2] = f;}   //!<  set y axis rotation of euler angle

	inline float yaw()   {return n[0];}  //!<  z axis rotation if vector is interpreted as euler angle set
	inline float roll()  {return n[1];}  //!<  x axis rotation of euler angle set
	inline float pitch() {return n[2];}  //!<  y axis rotation of euler angle set

	inline void set_yaw(float f)   {n[0] = f;}   //!<  set z axis rotation of euler angle
	inline void set_roll(float f)  {n[1] = f;}   //!<  set x axis rotation of euler angle
	inline void set_pitch(float f) {n[2] = f;}   //!<  set y axis rotation of euler angle
};


/****************************************************************************/
//!
//!  \class AtlasVec3f AtlasVectorTypes.h
//!
//!  \brief The AtlasVec3f class is a light-weight 3 vector.
//!
class AtlasVec3f
{

public:

	//!
	//! \_Description
	//!
	//!    Default constructor.  Vector values are uninitialized.
	//!
	AtlasVec3f() {}

	//!
	//! \brief  Constructor.  All vector values initialized to passed value.
	//!
	//! \_Parameters
	//!
	//!    \_in   initial_value - initial value of all vector values
	//!
	AtlasVec3f(float initial_value)
	{
		n[0] = n[1] = n[2] = initial_value;
	}

	//!
	//! \brief  Constructor.  Vector values initialized to passed values.
	//!
	//! \_Parameters
	//!
	//!    \_in   x - 1st vector value
	//!    \_in   y - 2nd vector value
	//!    \_in   z - 3rd vector value
	//!
	AtlasVec3f(float x, float y, float z)
	{
		n[0] = x;
		n[1] = y;
		n[2] = z;
	}


	float n[3];   //!<  Current vector values.  Can be accessed directly or through accessor functions.

	//
	//  Syntactic sugar for interpreting n[] as positions, rotations, etc.
	//
	inline float x() const {return n[0];}   //!<  x value if vector is interpreted as position
	inline float y() const {return n[1];}   //!<  y value of position
	inline float z() const {return n[2];}   //!<  z value of position

	inline void set_x(float f) {n[0] = f;}   //!<  set x value of position
	inline void set_y(float f) {n[1] = f;}   //!<  set y value of position
	inline void set_z(float f) {n[2] = f;}   //!<  set z value of position

	inline float rz() {return n[0];}  //!<  z axis rotation if vector is interpreted as euler angle set
	inline float rx() {return n[1];}  //!<  x axis rotation of euler angle set
	inline float ry() {return n[2];}  //!<  y axis rotation of euler angle set

	inline void set_rz(float f) {n[0] = f;}   //!<  set z axis rotation of euler angle
	inline void set_rx(float f) {n[1] = f;}   //!<  set x axis rotation of euler angle
	inline void set_ry(float f) {n[2] = f;}   //!<  set y axis rotation of euler angle

	inline float yaw()   {return n[0];}  //!<  z axis rotation if vector is interpreted as euler angle set
	inline float roll()  {return n[1];}  //!<  x axis rotation of euler angle set
	inline float pitch() {return n[2];}  //!<  y axis rotation of euler angle set

	inline void set_yaw(float f)   {n[0] = f;}   //!<  set z axis rotation of euler angle
	inline void set_roll(float f)  {n[1] = f;}   //!<  set x axis rotation of euler angle
	inline void set_pitch(float f) {n[2] = f;}   //!<  set y axis rotation of euler angle

};


/****************************************************************************/
//!
//!  \class AtlasQuaternion AtlasVectorTypes.h
//!
//!  \brief The AtlasQuaternion class is a light-weight quaternion.
//!
class AtlasQuaternion
{

public:

	//!
	//! \brief  Default constructor.  Default values represent identity rotation.
	//!
	AtlasQuaternion()
	{
		m_qw = 1.0f;
		m_qx = 0.0f;
		m_qy = 0.0f;
		m_qz = 0.0f;
	}

	//!
	//! \brief  Constructor.  Values initialized to passed values.
	//!
	//! \_Parameters
	//!
	//!    \_in   w   - w quaterion value
	//!    \_in   xyz - xyz quaterion values
	//!
	AtlasQuaternion(float w, AtlasVec3f xyz)
	{
		m_qw = w;
		m_qx = xyz.x();
		m_qy = xyz.y();
		m_qz = xyz.z();
	}

	float m_qw;
	float m_qx;
	float m_qy;
	float m_qz;

	inline float qw() const {return m_qw;}
	inline float qx() const {return m_qx;}
	inline float qy() const {return m_qy;}
	inline float qz() const {return m_qz;}

	inline void set_qw(float f) {m_qw = f;}
	inline void set_qx(float f) {m_qx = f;}
	inline void set_qy(float f) {m_qy = f;}
	inline void set_qz(float f) {m_qz = f;}
};

#ifndef NOT_IN_DOXYGEN
} // end namespace Atlas
#endif

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif

#endif  // __AtlasVectorTypes_H

