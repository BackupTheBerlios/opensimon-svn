/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "GraphicsRIB.h"
using namespace std;

GraphicsRIB::GraphicsRIB ()
{
  mFileName = string ("visualSimon");
  mFileNamePass1 = string ("visualSimonPass1");
  mFileNamePass2 = string ("visualSimonPass2");
  mFileNamePass3 = string ("visualSimonPass3");
  mPath = string ("renderings/");
  mFrameNumber = 0;
  mSphereNumber = 0;
  mPlaneNumber = 0;
  mBoxNumber = 0;
  mCapsuleNumber = 0;
}

void GraphicsRIB::beginFrame (float camX, float camY, float camZ,
                              float coiX, float coiY, float coiZ,
                              float upX,  float upY,  float upZ)
{
  float cam00, cam01, cam02;
  float cam10, cam11, cam12;
  float cam20, cam21, cam22;

  Vec3 cam (camX, camY, -camZ); // position of the camera
  Vec3 coi (coiX, coiY, -coiZ); // center of interest (target)
  Vec3 up  (upX,  upY,  -upZ);  // up-vector (roll)

  Vec3 z = coi - cam;  // direction of the camera, coi may not be equal cam
  z.normalize ();

  //float projectionOnXZ = (Vec3 (z [0], 0.0f, z [2])).length ();
  //float rotationY;
  //if (projectionOnXZ == 0.0f)
  //  if (z [1] < 0.0f)
  //    rotationY = 180;
  //  else
  //    rotationY = 0;
  //else
  //  rotationY = 180 * acos (z [2] / projectionOnXZ) / M_PI;

  //float projectionOnYZ = (Vec3 (0.0f, z [1], projectionOnXZ)).length ();
  //float rotationX = 180 * acos (projectionOnXZ / projectionOnYZ) / M_PI;
  //
  //if (z [1] < 0.0f)
  //  rotationX = -rotationX;

  //if (z [0] > 0.0f)
  //  rotationY = -rotationY;
  
  Vec3 x = cross (up, z);
  x.normalize ();
  Vec3 y = cross (z, x);
  y.normalize ();

  // Rotation der Kamera
  cam00 = x [0];
  cam01 = x [1];
  cam02 = x [2];

  cam10 = y [0];
  cam11 = y [1];
  cam12 = y [2];

  cam20 = z [0];
  cam21 = z [1];
  cam22 = z [2];

  float fov = 1.33 * 60; // sollte noch uebergeben werden...
  
  mFile << "FrameBegin " << mFrameNumber << endl;
  writeDisplayOptions (mFile);
  writeCameraProject (fov, mFile);
  //writeCameraTransform (cam, rotationX, rotationY, mFile);
  writeCameraTransform (cam00, cam01, cam02, cam10, cam11, cam12,
                        cam20, cam21, cam22, cam, mFile); 
  mFile << "  WorldBegin\n";
  mFile << "    LightSource \"ambientlight\" 1 \"intensity\" 0.5\n";
  mFile << "    LightSource \"distantlight\" 2 \"from\" [ 0 400 0 ]" <<
                                              " \"to\" [ 0 0 0 ]\n";

  // -----------------
  mPass1 << "FrameBegin " << mFrameNumber << endl;
  mPass1 << "  Hider \"photon\" \"emit\" 300000\n";
  //writeCameraTransform (cam, rotationX, rotationY, mPass1);
  writeCameraTransform (cam00, cam01, cam02, cam10, cam11, cam12,
                        cam20, cam21, cam22, cam, mPass1);
  mPass1 << "  WorldBegin\n";
  writeLightSources (40000000, mPass1);
  mPass1 << "    Attribute \"photon\" \"globalmap\" \"" << mPath <<
    "visualSimon" << setw (4) << setfill ('0') << mFrameNumber << ".gpm\"\n";
  mPass1 << "    Attribute \"photon\" \"estimator\" 100\n";
  mPass1 << "    Attribute \"trace\" \"maxspeculardepth\" 5\n";
  mPass1 << "    Attribute \"trace\" \"maxdiffusedepth\" 5\n";
  mPass1 << "    Attribute \"trace\" \"bias\" [0.001]\n";

  // ------------------
  mPass2 << "FrameBegin " << mFrameNumber << endl;
  writeDisplayOptions (mPass2);
  writeCameraProject (fov, mPass2);
  //writeCameraTransform (cam, rotationX, rotationY, mPass2);
  writeCameraTransform (cam00, cam01, cam02, cam10, cam11, cam12,
                        cam20, cam21, cam22, cam, mPass2);
  mPass2 << "  Attribute \"trace\" \"bias\" [0.001]\n";
  mPass2 << "  WorldBegin\n";
  mPass2 << "    Attribute \"dice\" \"rasterorient\" 0\n";
  mPass2 << "    Attribute \"visibility\" \"trace\" 1\n";
  writeLightSources (40000000, mPass2);
  mPass2 << "    Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "    Attribute \"visibility\" \"trace\" 1\n";
  mPass2 << "    Attribute \"photon\" \"globalmap\" \"" << mPath <<
    "visualSimon" << setw (4) << setfill ('0') << mFrameNumber << ".gpm\"\n";
  mPass2 << "    Attribute \"photon\" \"estimator\" [500]\n";
  mPass2 << "    Attribute \"irradiance\" \"maxerror\" 0.2\n";

  // ------------------
  mPass3 << "FrameBegin " << mFrameNumber << endl;
  writeDisplayOptions (mPass3);
  writeCameraProject (fov, mPass3);
  //writeCameraTransform (cam, rotationX, rotationY, mPass3);
  writeCameraTransform (cam00, cam01, cam02, cam10, cam11, cam12,
                        cam20, cam21, cam22, cam, mPass3);
  mPass3 << "  Attribute \"trace\" \"bias\" [0.001]\n";
  mPass3 << "  WorldBegin\n";
  mPass3 << "    Attribute \"visibility\" \"trace\" 1\n";
  writeLightSources (50000, mPass3);
}

void GraphicsRIB::writeDisplayOptions (ostringstream& stream)
{
  stream << "  Format 400 300 1\n";
  stream << "  ShadingInterpolation \"smooth\"\n";
  stream << "  PixelSamples 4 4\n";
  stream << "  Display \"" << mFileName << ".tif\" \"framebuffer\" \"rgb\"\n";
  stream << "  Display \"+" << mPath << mFileName << setw (4) << setfill ('0')
    << mFrameNumber << ".tif\" \"file\" \"rgb\"\n";
}

void GraphicsRIB::writeCameraProject (float fov, ostringstream& stream)
{
  stream << "  Projection \"perspective\" \"fov\" " << fov << endl;
}

void GraphicsRIB::writeCameraTransform (
    float cam00, float cam01, float cam02,
    float cam10, float cam11, float cam12,
    float cam20, float cam21, float cam22,
    const Vec3& position,
    //float rotationX,
    //float rotationY,
    ostringstream& stream)
{
  // inverse rotation
  stream << "  ConcatTransform [" <<
    cam00 << " " << cam10 << " " << cam20 << " 0 " <<
    cam01 << " " << cam11 << " " << cam21 << " 0 " <<
    cam02 << " " << cam12 << " " << cam22 << " 0 0 0 0 1]\n";
  // inverse translation
  stream << "  Translate " <<
    -position [0] << " " << -position [1] << " " << -position [2] << endl;

  //stream << "  Rotate " << rotationX << " 1.0 0.0 0.0\n";
  //stream << "  Rotate " << rotationY << " 0.0 1.0 0.0\n";
  //stream << "  Translate " <<
  //  -position [0] << " " << -position [1] << " " << -position [2] << endl;
}

void GraphicsRIB::writeLightSources (int intensity, ostringstream& stream)
{
  stream << "    LightSource \"cosinelight_rts\" \"light1\" \"from\" [0 " <<
    "400" << " 0] \"intensity\" " << intensity << endl;
}

void GraphicsRIB::finishFrame ()
{
  mFile << "  WorldEnd\n";
  mFile << "FrameEnd\n";
  mPass1 << "  WorldEnd\n";
  mPass1 << "FrameEnd\n";
  mPass2 << "  WorldEnd\n";
  mPass2 << "FrameEnd\n";
  mPass3 << "  WorldEnd\n";
  mPass3 << "FrameEnd\n";
  ofstream outPut1 ((mFileNamePass1 + ".rib").c_str ());
  ofstream outPut2 ((mFileNamePass2 + ".rib").c_str ());
  ofstream outPut3 ((mFileNamePass3 + ".rib").c_str ());
  ofstream outPut ((mFileName + ".rib").c_str ());
  outPut1 << mPass1.str ();
  outPut2 << mPass2.str ();
  outPut3 << mPass3.str ();
  outPut << mFile.str ();
  mPass1.str ().erase ();
  mPass2.str ().erase ();
  mPass3.str ().erase ();
  mFile.str  ().erase ();
  ++mFrameNumber;
  //mSphereNumber = 0;
  //mBoxNumber = 0;
  //mCapsuleNumber = 0;
  //mPlaneNumber = 0;
}

/** @brief Writes RIB code for a plane
 *
 * Writes RIB code of a ground plane for local and global illumonation models.
 * For global illumination we assume you will use the Pixie Renderer and have
 * the additional shaders from the Pixie WIKI (see global illumination)
 * installed on your platform. There are three passes for global illumination.
 * To not loose too many Photons in global illumination we draw a whole box
 * instead of a plane, also for standard RIB output to assure comparability.
 * The function therefore is not intended to be called more than once per
 * frame, i.e. it is not supposed that there exist two or more planes in the
 * world.
 *
 * @parameter position Where to position the ground plane and box
 * @parameter orientation How to rotate the ground plane and box
 * @parameter material The plane's and box's material, maybe including color
 * @parameter size Total dimension of the ground plane and box
 */
void GraphicsRIB::drawPlane (
    const Vec3& position,
    const Quaternion& orientation,
    const Material material,
    float size)
{
  Vec3 axis;
  float angle;
  orientation.getAxisAngle (axis, angle);

  // ----------------
  mFile << "    AttributeBegin\n";
  mFile << "      Surface " << "\"" << getSurface (material) << "\"" << endl;
  drawPlaneGeometry (position, axis, angle, size, material, mFile);
  mFile << "    AttributeEnd\n";

  // -----------------
  mPass1 << "    AttributeBegin\n";
  mPass1 << "      Attribute \"photon\" \"shadingmodel\" \"matte\"\n";
  drawPlaneGeometry (position, axis, angle, size, material, mPass1);
  mPass1 << "    AttributeEnd\n";

  // ----------------
  mPass2 << "    AttributeBegin\n";
  mPass2 << "      Surface \"indirectsurf1sided\" \"samples\" 1024\n";
  mPass2 << "      Attribute \"irradiance\" \"filemode\" \"w\" \"handle\" \""
   << mPath << "plane" << mPlaneNumber << ".icf\"\n";
  drawPlaneGeometry (position, axis, angle, size, material, mPass2);
  mPass2 << "    AttributeEnd\n";

  // ----------------
  mPass3 << "    AttributeBegin\n";
  mPass3 << "      Surface \"indirectsurf2\" \"Kd\" 0.8\n";
  mPass3 << "      Attribute \"irradiance\" \"filemode\" \"r\" \"handle\" \""
    << mPath << "plane" << mPlaneNumber << ".icf\"\n";
  drawPlaneGeometry (position, axis, angle, size, material, mPass3);
  mPass3 << "    AttributeEnd\n";

  ++mPlaneNumber;
}

/**
 * @brief Outputs a ground plane's (pure) geometry RIB code to a given stream
 *
 * Outputs the plane's (and kind of environmental box's) geometry in RIB format
 * to a given stream. The function is to be used by drawPlane only. Information
 * to be written includes color, any transform, and the primitives themselves.
 *
 * @parameter stream The stream to write the output to
 * @parameter position The plane's position
 * @parameter axis The plane's rotational axis for orientation
 * @parameter angle Angle (deg) to rotate the plane around a specified axis
 * @parameter size The plane's total dimension
 * @parameter material The plane's material
 */
void GraphicsRIB::drawPlaneGeometry (
    const Vec3& position,
    const Vec3& axis,
    float angle,
    float size,
    const Material material,
    ostringstream& stream)
{
  stream << "      AttributeBegin\n";
  stream << "        Sides 1\n";
  stream << "        Color " << "[" << getColor (material) << "]" << endl;
  stream << "        Translate " << position [X] << " " <<
                                    position [Y] << " " <<
                                   -position [Z] << endl;
  stream << "        Rotate " << RADTODEG (angle) << " " <<
                                 axis [X] << " " <<
                                 axis [Y] << " " <<
                                -axis [Z] << endl;
  stream << "        Translate 0 " << size/2 << " 0\n";
  stream << "        Scale " << size/2 << " " << size/2 << " " << size/2 <<
    endl;

  stream << "        Polygon \"P\" [-1 1 -1 -1 1 1 -1 -1 1 -1 -1 -1]\n";
  stream << "        Polygon \"P\" [1 -1 -1 1 -1 1 1 1 1 1 1 -1]\n";
  stream << "        Polygon \"P\" [-1 1 -1 1 1 -1 1 1 1 -1 1 1]\n";
  stream << "        Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  stream << "        Polygon \"P\" [-1 -1 1  1 -1 1  1 -1 -1  -1 -1 -1]\n";
  stream << "        Rotate 180 0 1 0\n";
  stream << "        Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";

  stream << "      AttributeEnd\n";
}

/** @brief Writes RIB code for a sphere
 *
 * Writes RIB code of a sphere for local and global illumonation models.  For
 * global illumination we assume you will use the Pixie Renderer and have the
 * additional shaders from the Pixie WIKI (see global illumination) installed
 * on your platform. There are three passes for global illumination.
 *
 * @parameter position Where to position the sphere
 * @parameter orientation How to rotate the sphere
 * @parameter material The sphere's material, maybe including color
 * @parameter radius Radius of the sphere
 */
void GraphicsRIB::drawSphere (
	const Vec3& position, 
	float radius, 
	const Quaternion& orientation,
	const Material material)
{

  Vec3 axis;
  float angle;
  orientation.getAxisAngle (axis, angle);

  mFile << "    AttributeBegin\n";
  mFile << "      Surface " << "\"" << getSurface (material) << "\"" << endl;
  drawSphereGeometry (position, axis, angle, radius, material, mFile);
  mFile << "    AttributeEnd\n";

  // ---------------------
  mPass1 << "    AttributeBegin\n";
  mPass1 << "      Attribute \"photon\" \"shadingmodel\" \"chrome\"\n";
  drawSphereGeometry (position, axis, angle, radius, material, mPass1);
  mPass1 << "    AttributeEnd\n";

  // ----------------------
  mPass2 << "    AttributeBegin\n";
  mPass2 << "      Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "      Surface \"indirectsurf1sided\" \"samples\" 1024\n";
  mPass2 << "      Attribute \"irradiance\" \"filemode\" \"w\" \"handle\" " <<
    " \"" << mPath << "sphere" << mSphereNumber << ".icf\"\n";
  mPass2 << "      Attribute \"irradiance\" \"maxerror\" 0.1\n";
  drawSphereGeometry (position, axis, angle, radius, material, mPass2);
  mPass2 << "    AttributeEnd\n";

  // ----------------------
  mPass3 << "    AttributeBegin\n";
  mPass3 << "      Attribute \"trace\"  \"bias\" [0.01]\n";
  mPass3 << "      Surface \"mirror\" \"samples\" 4 \"blur\" 0.001\n";
  mPass3 << "      Attribute \"irradiance\" \"filemode\" \"r\" \"handle\" " <<
    " \"" << mPath << "sphere" << mSphereNumber << ".icf\"\n";
  drawSphereGeometry (position, axis, angle, radius, material, mPass3);
  mPass3 << "    AttributeEnd\n";

  ++mSphereNumber;
}

/** @brief Outputs a sphere's (pure) geometry RIB code to a given stream
 *
 * Outputs the sphere's geometry in RIB format to a given stream. The function
 * is to be used by drawSphere only. Information to be written includes color,
 * any transform, and the primitives themselves.
 *
 * @parameter stream The stream to write the output to
 * @parameter position The sphere's position
 * @parameter axis The sphere's rotational axis for orientation
 * @parameter angle Angle (deg) to rotate the sphere around a specified axis
 * @parameter radius The sphere's radius
 * @parameter material The sphere's material
 */

void GraphicsRIB::drawSphereGeometry (
	const Vec3& position, 
	const Vec3& axis,
    float angle,
    float radius,
	const Material material,
    ostringstream& stream)

{
  stream << "      AttributeBegin\n";
  stream << "        Color " << "[" << getColor (material) << "]" << endl;
  stream << "        Translate " << position [X] << " " <<
                                    position [Y] << " " <<
                                   -position [Z] << endl;
  stream << "        Rotate " << RADTODEG (angle) << " " <<
                                 axis [X] << " " <<
                                 axis [Y] << " " <<
                                -axis [Z] << endl;
  
  stream << "        Sphere " << radius << " " <<
                                -radius << " " <<
                                 radius << " 360\n";
  stream << "      AttributeEnd\n";
}

void GraphicsRIB::drawBox (
    const Vec3& position,
    const Vec3& scale,
    const Quaternion& orientation,
    const Material material)
{

  Vec3 axis;
  float angle;
  orientation.getAxisAngle (axis, angle);

  mFile << "    AttributeBegin\n";
  mFile << "      Surface " << "\"" << getSurface (material) << "\"" << endl;
  drawBoxGeometry (position, scale, axis, angle, material, mFile);
  mFile << "    AttributeEnd\n";

  // ---------------------
  mPass1 << "    AttributeBegin\n";
  mPass1 << "      Attribute \"photon\" \"shadingmodel\" \"matte\"\n";
  drawBoxGeometry (position, scale, axis, angle, material, mPass1);
  mPass1 << "    AttributeEnd\n";

  // ----------------------
  mPass2 << "    AttributeBegin\n";
  mPass2 << "      Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "      Surface \"indirectsurf1sided\" \"samples\" 1024\n";
  mPass2 << "      Attribute \"irradiance\" \"filemode\" \"w\" \"handle\" " <<
    " \"" << mPath << "box" << mBoxNumber << ".icf\"\n";
  mPass2 << "      Attribute \"irradiance\" \"maxerror\" 0.1\n";
  drawBoxGeometry (position, scale, axis, angle, material, mPass2);
  mPass2 << "    AttributeEnd\n";

  // ----------------------
  mPass3 << "    AttributeBegin\n";
  mPass3 << "      Attribute \"trace\"  \"bias\" [0.01]\n";
  //mPass3 << "      Surface \"mirror\" \"samples\" 4 \"blur\" 0.001\n";
  mPass3 << "      Surface \"indirectsurf\"\n";
  mPass3 << "      Attribute \"irradiance\" \"filemode\" \"r\" \"handle\" " <<
    " \"" << mPath << "box" << mBoxNumber << ".icf\"\n";
  drawBoxGeometry (position, scale, axis, angle, material, mPass3);
  mPass3 << "    AttributeEnd\n";

  ++mBoxNumber;
}

void GraphicsRIB::drawBoxGeometry (
    const Vec3& position,
    const Vec3& scale,
    const Vec3& axis,
    float angle,
    const Material material,
    ostringstream& stream)
{

  stream << "      AttributeBegin\n";
  //stream << "        Color " << "[" << getColor (material) << "]" << endl;
  stream << "        Color [1 0 0]\n";
  stream << "        Translate " << position [X] << " " <<
                                    position [Y] << " " <<
                                   -position [Z] << endl;
  stream << "        Rotate " << RADTODEG (angle) << " " <<
                                 axis [X] << " " <<
                                 axis [Y] << " " <<
                                -axis [Z] << endl;
  stream << "        Scale " << scale [X] << " " <<
                                scale [Y] << " " <<
                                scale [Z] << " " << endl;

  stream << "        Polygon \"P\" [-1  1 -1  1  1 -1  1 -1 -1 -1 -1 -1]\n";
  stream << "        Polygon \"P\" [-1 -1  1  1 -1  1  1  1  1 -1  1  1]\n";
  stream << "        Rotate 90 0 1 0                                    \n";
  stream << "        Polygon \"P\" [-1  1 -1  1  1 -1  1 -1 -1 -1 -1 -1]\n";
  stream << "        Polygon \"P\" [-1 -1  1  1 -1  1  1  1  1 -1  1  1]\n";
  stream << "        Rotate 90 1 0 0                                    \n";
  stream << "        Polygon \"P\" [-1  1 -1  1  1 -1  1 -1 -1 -1 -1 -1]\n";
  stream << "        Polygon \"P\" [-1 -1  1  1 -1  1  1  1  1 -1  1  1]\n";
  stream << "      AttributeEnd\n";
}

void GraphicsRIB::drawCapsule (
    const Vec3& position, 
    float radius, 
    float height,
    const Quaternion& orientation,
    const Material material)
{
  Vec3 axis;
  float angle;
  orientation.getAxisAngle (axis, angle);

  mFile << "    AttributeBegin\n";
  mFile << "      Surface " << "\"" << getSurface (material) << "\"" << endl;
  drawCapsuleGeometry (position, axis, angle, radius, height, material, mFile);
  mFile << "    AttributeEnd\n";

  // ---------------------
  mPass1 << "    AttributeBegin\n";
  mPass1 << "      Attribute \"photon\" \"shadingmodel\" \"chrome\"\n";
  drawCapsuleGeometry (position, axis, angle, radius, height, material, mPass1);
  mPass1 << "    AttributeEnd\n";

  // ----------------------
  mPass2 << "    AttributeBegin\n";
  mPass2 << "      Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "      Surface \"indirectsurf1sided\" \"samples\" 1024\n";
  mPass2 << "      Attribute \"irradiance\" \"filemode\" \"w\" \"handle\" " <<
    " \"" << mPath << "capsule" << mCapsuleNumber << ".icf\"\n";
  mPass2 << "      Attribute \"irradiance\" \"maxerror\" 0.1\n";
  drawCapsuleGeometry (position, axis, angle, radius, height, material, mPass2);
  mPass2 << "    AttributeEnd\n";

  // ----------------------
  mPass3 << "    AttributeBegin\n";
  mPass3 << "      Attribute \"trace\"  \"bias\" [0.01]\n";
  mPass3 << "      Surface \"mirror\" \"samples\" 4 \"blur\" 0.001\n";
  mPass3 << "      Attribute \"irradiance\" \"filemode\" \"r\" \"handle\" " <<
    " \"" << mPath << "capsule" << mCapsuleNumber << ".icf\"\n";
  drawCapsuleGeometry (position, axis, angle, radius, height, material, mPass3);
  mPass3 << "    AttributeEnd\n";

  ++mCapsuleNumber;
}

void GraphicsRIB::drawCapsuleGeometry (
    const Vec3& position,
    const Vec3& axis,
    float angle,
    float radius,
    float height,
    const Material material,
    ostringstream& stream)
{
  stream << "      AttributeBegin\n";
  //stream << "        Color " << "[" << getColor (material) << "]" << endl;
  stream << "        Color 1.0 0.5 0.0\n";
  stream << "        Translate " << position [X] << " " <<
                                    position [Y] << " " <<
                                   -position [Z] << endl;
  stream << "        Rotate " << RADTODEG (angle) << " " <<
                                 axis [X] << " " <<
                                 axis [Y] << " " <<
                                -axis [Z] << endl;
  stream << "        Rotate 90 1 0 0\n";
  stream << "        AttributeBegin\n";
  stream << "          Translate 0 0 " << height/2 << endl;
  stream << "          Cylinder " << radius << " 0 " <<
                                    -height << " 360\n";
  stream << "          Sphere " << radius << " " <<
                                  -radius << " " <<
                                   radius << " 360\n";
  stream << "        AttributeEnd\n";
  stream << "        AttributeBegin\n";
  stream << "          Translate 0 0 " << -height/2 << endl;
  stream << "          Sphere " << radius << " " <<
                                  -radius << " " <<
                                   radius << " 360\n";
  stream << "        AttributeEnd\n";
  stream << "      AttributeEnd\n";
}

const string GraphicsRIB::getSurface (Material material)
{
  switch (material) {
    case materialIron:  return string ("metal");
                        break;
    case materialGlass: return string ("glass");
                        break;
    default:            return string ("plastic");
                        break;
  }
}

const string GraphicsRIB::getColor (Material material)
{
  return string ("1.0 1.0 1.0");
}
