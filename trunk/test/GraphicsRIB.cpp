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
  mPath = string ("rendering/");
  mFrameNumber = 0;
  mSphereNumber = 0;
  mPlaneNumber = 0;
}

void GraphicsRIB::beginFrame (float camX, float camY, float camZ,
                              float coiX, float coiY, float coiZ,
                              float upX,  float upY,  float upZ)
{
  mFile << "FrameBegin " << mFrameNumber << endl;
  mFile << "  Format 400 300 1\n";
  mFile << "  ShadingInterpolation \"smooth\"\n";
  mFile << "  PixelSamples 4 4\n";
  mFile << "  Display \"" << mFileName << ".tif\" \"framebuffer\" \"rgb\"\n";
  mFile << "  Display \"+" << mPath << mFileName << setw (4) << setfill ('0')
    << mFrameNumber << ".tif\" \"file\" \"rgb\"\n";
  mFile << "  Projection \"perspective\" \"fov\" 30\n";
  //mFile << "  Translate " << camX << " " << camY << " " << " " << camZ << endl;
  mFile << "  Translate 0 -50 500" << endl;
  //float coiAngle = asin (dot (Vec3 (0.0, 0.0, -1.0),
  //      (Vec3 (coiX, coiY, coiZ))));
  //Vec3  coiAxis  = cross (Vec3 (0.0, 0.0, -1.0), (Vec3 (coiX, coiY, coiZ)));
  mFile << "  Rotate 0 1 0 0\n";
  
  // up wird nicht angeschaut, immer 0 1 0
  mFile << "  WorldBegin\n";
  mFile << "    LightSource \"ambientlight\" 1 \"intensity\" 0.5\n";
  mFile << "    LightSource \"distantlight\" 2 \"from\" [ 0 400 0 ]" <<
                                              " \"to\" [ 0 0 0 ]\n";

  // -----------------
  mPass1 << "FrameBegin " << mFrameNumber << endl;
  mPass1 << "  Hider \"photon\" \"emit\" 300000\n";
  mPass1 << "  Translate 0 " << "-50" << " " << "500" << endl;
  mPass1 << "  Rotate 0 1 0 0\n";
  mPass1 << "  WorldBegin\n";
  mPass1 << "    LightSource \"cosinelight_rts\" 1 \"from\" [0 " << "400" <<
    " 0] \"intensity\" 40000000\n";
  mPass1 << "    Attribute \"photon\" \"globalmap\" \"visualSimon.gpm\"\n";
  mPass1 << "    Attribute \"photon\" \"estimator\" 100\n";
  mPass1 << "    Attribute \"trace\" \"maxspeculardepth\" 5\n";
  mPass1 << "    Attribute \"trace\" \"maxdiffusedepth\" 5\n";
  mPass1 << "    Attribute \"trace\" \"bias\" [0.001]\n";

  // ------------------
  mPass2 << "FrameBegin " << mFrameNumber << endl;
  mPass2 << "  Format 400 300 1\n";
  mPass2 << "  ShadingInterpolation \"smooth\"\n";
  mPass2 << "  PixelSamples 4 4\n";
  mPass2 << "  Display \"" << mFileName << ".tif\" \"framebuffer\" \"rgb\"\n";
  mPass2 << "  Display \"+" << mPath << mFileName << setw (4) << setfill ('0')
    << mFrameNumber << ".tif\" \"file\" \"rgb\"\n";
  mPass2 << "  Projection \"perspective\" \"fov\" 30\n";
  mPass2 << "  Translate 0 -50 500" << endl;
  mPass2 << "  Rotate 0 1 0 0\n";
  mPass2 << "  Attribute \"trace\" \"bias\" [0.001]\n";
  mPass2 << "  WorldBegin\n";
  mPass2 << "    Attribute \"dice\" \"rasterorient\" 0\n";
  mPass2 << "    Attribute \"visibility\" \"trace\" 1\n";
  mPass2 << "    LightSource \"cosinelight_rts\" \"light1\" \"from\" [0 " <<
    "400" << " 0] \"intensity\" 40000000\n";
  mPass2 << "    Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "    Attribute \"visibility\" \"trace\" 1\n";
  mPass2 << "    Attribute \"photon\" \"globalmap\" \"visualSimon.gpm\"\n";
  mPass2 << "    Attribute \"photon\" \"estimator\" [500]\n";
  mPass2 << "    Attribute \"irradiance\" \"maxerror\" 0.2\n";

  // ------------------
  mPass3 << "FrameBegin " << mFrameNumber << endl;
  mPass3 << "  Format 400 300 1\n";
  mPass3 << "  ShadingInterpolation \"smooth\"\n";
  mPass3 << "  PixelSamples 4 4\n";
  mPass3 << "  Display \"" << mFileName << ".tif\" \"framebuffer\" \"rgb\"\n";
  mPass3 << "  Display \"+" << mPath << mFileName << setw (4) << setfill ('0')
    << mFrameNumber << ".tif\" \"file\" \"rgb\"\n";
  mPass3 << "  Projection \"perspective\" \"fov\" 30\n";
  mPass3 << "  Translate 0 -50 500" << endl;
  mPass3 << "  Rotate 0 1 0 0\n";
  mPass3 << "  Attribute \"trace\" \"bias\" [0.001]\n";
  mPass3 << "  WorldBegin\n";
  mPass3 << "    Attribute \"visibility\" \"trace\" 1\n";
  mPass3 << "    LightSource \"cosinelight_rts\" \"light1\" \"from\" [0 " <<
    "400" << " 0] \"intensity\" 50000\n";
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
}

void GraphicsRIB::drawPlane (
    const Vec3& position,
    const Quaternion& orientation,
    const Material material,
    float size)
{
  mFile << "    AttributeBegin\n";
  mFile << "      Color " << "[" << getColor (material) << "]" << endl;
  mFile << "      Surface " << "\"" << getSurface (material) << "\"" << endl;
  mFile << "      Translate " << position [X] << " " <<
                                 position [Y] << " " <<
                                -position [Z] << endl;
  Vec3 axis;
  float angle;
  orientation.getAxisAngle (axis, angle);
  mFile << "      Rotate " << RADTODEG (angle) << " " <<
                              axis [X] << " " <<
                              axis [Y] << " " <<
                             -axis [Z] << endl;
  mFile << "      Translate 0 " << size/2 << " 0\n";
  mFile << "      Scale " << size/2 << " " << size/2 << " " << size/2 << endl;
  mFile << "      Polygon \"P\" [-1 1 -1 -1 1 1 -1 -1 1 -1 -1 -1]\n";
  mFile << "      Polygon \"P\" [1 -1 -1 1 -1 1 1 1 1 1 1 -1]\n";
  mFile << "      Polygon \"P\" [-1 1 -1 1 1 -1 1 1 1 -1 1 1]\n";
  mFile << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mFile << "      Polygon \"P\" [-1 -1 1  1 -1 1  1 -1 -1  -1 -1 -1]\n";
  mFile << "      Rotate 180 0 1 0\n";
  mFile << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mFile << "    AttributeEnd\n";

  // -----------------
  mPass1 << "    AttributeBegin\n";
  mPass1 << "      Attribute \"photon\" \"shadingmodel\" \"matte\"\n";
  mPass1 << "      Color " << "[" << getColor (material) << "]" << endl;
  mPass1 << "      Translate " << position [X] << " " <<
                                  position [Y] << " " <<
                                 -position [Z] << endl;
  mPass1 << "      Rotate " << RADTODEG (angle) << " " <<
                               axis [X] << " " <<
                               axis [Y] << " " <<
                              -axis [Z] << endl;
  mPass1 << "      Translate 0 " << size/2 << " 0\n";
  mPass1 << "      Scale " << size/2 << " " << size/2 << " " << size/2 << endl;
  mPass1 << "      Polygon \"P\" [-1 1 -1 -1 1 1 -1 -1 1 -1 -1 -1]\n";
  mPass1 << "      Polygon \"P\" [1 -1 -1 1 -1 1 1 1 1 1 1 -1]\n";
  mPass1 << "      Polygon \"P\" [-1 1 -1 1 1 -1 1 1 1 -1 1 1]\n";
  mPass1 << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mPass1 << "      Polygon \"P\" [-1 -1 1  1 -1 1  1 -1 -1  -1 -1 -1]\n";
  mPass1 << "      Rotate 180 0 1 0\n";
  mPass1 << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mPass1 << "    AttributeEnd\n";

  // ----------------
  mPass2 << "    AttributeBegin\n";
  //mPass2 << "      Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "      Surface \"indirectsurf1sided\" \"samples\" 1024\n";
  mPass2 << "      Attribute \"irradiance\" \"filemode\" \"w\" \"handle\" \""
   << mPath << "plane" << mPlaneNumber << ".icf\"\n";
  mPass2 << "      Color " << "[" << getColor (material) << "]" << endl;
  mPass2 << "      Translate " << position [X] << " " <<
                                  position [Y] << " " <<
                                 -position [Z] << endl;
  mPass2 << "      Rotate " << RADTODEG (angle) << " " <<
                               axis [X] << " " <<
                               axis [Y] << " " <<
                              -axis [Z] << endl;

  mPass2 << "      Translate 0 " << size/2 << " 0\n";
  mPass2 << "      Scale " << size/2 << " " << size/2 << " " << size/2 << endl;
  mPass2 << "      Polygon \"P\" [-1 1 -1 -1 1 1 -1 -1 1 -1 -1 -1]\n";
  mPass2 << "      Polygon \"P\" [1 -1 -1 1 -1 1 1 1 1 1 1 -1]\n";
  mPass2 << "      Polygon \"P\" [-1 1 -1 1 1 -1 1 1 1 -1 1 1]\n";
  mPass2 << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mPass2 << "      Polygon \"P\" [-1 -1 1  1 -1 1  1 -1 -1  -1 -1 -1]\n";
  mPass2 << "      Rotate 180 0 1 0\n";
  mPass2 << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";

  mPass2 << "    AttributeEnd\n";

  // ----------------
  mPass3 << "    AttributeBegin\n";
  mPass3 << "      Color " << "[" << getColor (material) << "]" << endl;
  mPass3 << "      Surface \"indirectsurf2\" \"Kd\" 0.8\n";
  mPass3 << "      Attribute \"irradiance\" \"filemode\" \"r\" \"handle\" \""
    << mPath << "plane" << mPlaneNumber << ".icf\"\n";
  mPass3 << "      Translate " << position [X] << " " <<
                                  position [Y] << " " <<
                                 -position [Z] << endl;
  mPass3 << "      Rotate " << RADTODEG (angle) << " " <<
                               axis [X] << " " <<
                               axis [Y] << " " <<
                              -axis [Z] << endl;
  mPass3 << "      Translate 0 " << size/2 << " 0\n";
  mPass3 << "      Scale " << size/2 << " " << size/2 << " " << size/2 << endl;
  mPass3 << "      Polygon \"P\" [-1 1 -1 -1 1 1 -1 -1 1 -1 -1 -1]\n";
  mPass3 << "      Polygon \"P\" [1 -1 -1 1 -1 1 1 1 1 1 1 -1]\n";
  mPass3 << "      Polygon \"P\" [-1 1 -1 1 1 -1 1 1 1 -1 1 1]\n";
  mPass3 << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mPass3 << "      Polygon \"P\" [-1 -1 1  1 -1 1  1 -1 -1  -1 -1 -1]\n";
  mPass3 << "      Rotate 180 0 1 0\n";
  mPass3 << "      Polygon \"P\" [-1 1 1 1 1 1 1 -1 1 -1 -1 1]\n";
  mPass3 << "    AttributeEnd\n";
  ++mPlaneNumber;
}

void GraphicsRIB::drawSphere (
	const Vec3& position, 
	float radius, 
	const Quaternion& orientation,
	const Material material)
{
  mFile << "    AttributeBegin\n";
  mFile << "      Color " << "[" << getColor (material) << "]" << endl;
  mFile << "      Surface " << "\"" << getSurface (material) << "\"" << endl;
  mFile << "      Translate " << position [X] << " " <<
                                 position [Y] << " " <<
                                -position [Z] << endl;
  Vec3 axis;
  float angle;
  orientation.getAxisAngle (axis, angle);
  mFile << "      Rotate " << RADTODEG (angle) << " " <<
                              axis [X] << " " << 
                              axis [Y] << " " <<
                             -axis [Z] << endl;
  mFile << "      Sphere " << radius << " " << 
                       "-" << radius << " " <<
                              radius << " 360\n";
  mFile << "    AttributeEnd\n";

  // ---------------------
  mPass1 << "    AttributeBegin\n";
  mPass1 << "      Attribute \"photon\" \"shadingmodel\" \"chrome\"\n";
  mPass1 << "      Color " << "[" << getColor (material) << "]" << endl;
  mPass1 << "      Translate " << position [X] << " " <<
                                  position [Y] << " " <<
                                 -position [Z] << endl;
  mPass1 << "      Rotate " << RADTODEG (angle) << " " <<
                               axis [X] << " " <<
                               axis [Y] << " " <<
                              -axis [Z] << endl;
  mPass1 << "      Sphere " << radius << " " <<
                        "-" << radius << " " <<
                               radius << " 360\n";
  mPass1 << "    AttributeEnd\n";

  // ----------------------
  mPass2 << "    AttributeBegin\n";
  mPass2 << "      Attribute \"visibility\" \"transmission\" \"opaque\"\n";
  mPass2 << "      Color " << "[" << getColor (material) << "]" << endl;
  mPass2 << "      Surface \"indirectsurf1sided\" \"samples\" 1024\n";
  mPass2 << "      Attribute \"irradiance\" \"filemode\" \"w\" \"handle\" " <<
    " \"" << mPath << "sphere" << mSphereNumber << ".icf\"\n";
  mPass2 << "      Attribute \"irradiance\" \"maxerror\" 0.1\n";
  mPass2 << "      Translate " << position [X] << " " <<
                                  position [Y] << " " <<
                                 -position [Z] << endl;
  mPass2 << "      Rotate " << RADTODEG (angle) << " " <<
                               axis [X] << " " <<
                               axis [Y] << " " <<
                              -axis [Z] << endl;
  mPass2 << "      Sphere " << radius << " " <<
                        "-" << radius << " " <<
                               radius << " 360\n";
  mPass2 << "    AttributeEnd\n";

  // ----------------------
  mPass3 << "    AttributeBegin\n";
  mPass3 << "      Attribute \"trace\"  \"bias\" [0.01]\n";
  mPass3 << "      Surface \"mirror\" \"samples\" 4 \"blur\" 0.001\n";
  mPass3 << "      Attribute \"irradiance\" \"filemode\" \"r\" \"handle\" " <<
    " \"" << mPath << "sphere" << mSphereNumber << ".icf\"\n";
  mPass3 << "      Translate " << position [X] << " " <<
                                  position [Y] << " " <<
                                  -position [Z] << endl;
  mPass3 << "      Rotate " << RADTODEG (angle) << " " <<
                               axis [X] << " " <<
                               axis [Y] << " " <<
                              -axis [Z] << endl;
  mPass3 << "      Sphere " << radius << " " <<
                        "-" << radius << " " <<
                               radius << " 360\n";
  mPass3 << "    AttributeEnd\n";
  ++mSphereNumber;
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
