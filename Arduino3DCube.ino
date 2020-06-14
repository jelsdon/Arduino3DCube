/*
 * Port of OneLoneCoder.com - 3D Graphics Part #1 - Triangles & Projections
 * to Arduino
 * 
 *  Original works located at:
 *  https://www.youtube.com/watch?v=ih20l3pJoeU
 *  https://www.github.com/onelonecoder
 *  https://www.onelonecoder.com
 *  https://www.youtube.com/javidx9
 *  GNU GPLv3
 *  https://github.com/OneLoneCoder/videos/blob/master/LICENSE
 * 
 *  Display is driven by U8g2: Library for monochrome displays, version 2
 *  https://github.com/olikraus/u8g2
 *  
 *  Hardware: Arduino UNO
 *            SSD1306 128k64 I2C
 *
 *
 *  Most of the OLC code is unchanged; modifications include
 *  - Addition of triangle function drawTri as u8g2's triangle functions are shaded
 *  - mesh (vector) replaced with cube (array of triangles) - vector libraries ate too much space
 *  - Store cube in PROGMEM to provide enough memory for u8g2 to run with full frame buffer (best speed)
 *  - Rotation speed static
 *  - Show FPS
 *  
 *  
 *  - jelsdon 2020
 */


#include <U8g2lib.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

struct vec3d
{
  float x, y, z;
};

struct triangle
{
  vec3d p[3];
};

struct mat4x4
{
  float m[4][4] = { 0 };
};



void MultiplyMatrixVector(vec3d &i, vec3d &o, mat4x4 &m)
{
  o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
  o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
  o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
  float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

  if (w != 0.0f)
  {
    o.x /= w; o.y /= w; o.z /= w;
  }
}

void drawTri(u8g2_uint_t x0, u8g2_uint_t y0, u8g2_uint_t x1, u8g2_uint_t y1, u8g2_uint_t x2, u8g2_uint_t y2)
{
  u8g2.drawLine(x0, y0, x1, y1);
  u8g2.drawLine(x1, y1, x2, y2);
  u8g2.drawLine(x2, y2, x0, y0);

}

const static triangle cube[12] PROGMEM = {
  // SOUTH
  { 0.0f, 0.0f, 0.0f,    0.0f, 1.0f, 0.0f,    1.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f,    1.0f, 1.0f, 0.0f,    1.0f, 0.0f, 0.0f },

  // EAST
  { 1.0f, 0.0f, 0.0f,    1.0f, 1.0f, 0.0f,    1.0f, 1.0f, 1.0f },
  { 1.0f, 0.0f, 0.0f,    1.0f, 1.0f, 1.0f,    1.0f, 0.0f, 1.0f },

  // NORTH
  { 1.0f, 0.0f, 1.0f,    1.0f, 1.0f, 1.0f,    0.0f, 1.0f, 1.0f },
  { 1.0f, 0.0f, 1.0f,    0.0f, 1.0f, 1.0f,    0.0f, 0.0f, 1.0f },

  // WEST
  { 0.0f, 0.0f, 1.0f,    0.0f, 1.0f, 1.0f,    0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f,    0.0f, 1.0f, 0.0f,    0.0f, 0.0f, 0.0f },

  // TOP
  { 0.0f, 1.0f, 0.0f,    0.0f, 1.0f, 1.0f,    1.0f, 1.0f, 1.0f },
  { 0.0f, 1.0f, 0.0f,    1.0f, 1.0f, 1.0f,    1.0f, 1.0f, 0.0f },

  // BOTTOM
  { 1.0f, 0.0f, 1.0f,    0.0f, 0.0f, 1.0f,    0.0f, 0.0f, 0.0f },
  { 1.0f, 0.0f, 1.0f,    0.0f, 0.0f, 0.0f,    1.0f, 0.0f, 0.0f },
};


triangle tri;
mat4x4 matProj, matRotZ, matRotX;
float fTheta;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds

void setup()   {
  u8g2.begin();

  // Projection Matrix
  float fNear = 0.1f;
  float fFar = 1000.0f;
  float fFov = 90.0f;
  float fAspectRatio = (float) SCREEN_HEIGHT / (float) SCREEN_WIDTH;
  float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);
  startMillis = millis();

  matProj.m[0][0] = fAspectRatio * fFovRad;
  matProj.m[1][1] = fFovRad;
  matProj.m[2][2] = fFar / (fFar - fNear);
  matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
  matProj.m[2][3] = 1.0f;
  matProj.m[3][3] = 0.0f;
}

float fElapsedTime = 0;

void loop() {
  u8g2.clearBuffer();

  currentMillis = millis(); 
  
//  fTheta += 1.0f * (currentMillis - startMillis);
 fTheta += 1.0f * 88;

u8g2.setFont(u8g2_font_6x10_tn);
u8g2.setCursor(0, 15);
u8g2.print(1000/(currentMillis - startMillis));
  startMillis = currentMillis;

  // Rotation Z
  matRotZ.m[0][0] = cosf(fTheta);
  matRotZ.m[0][1] = sinf(fTheta);
  matRotZ.m[1][0] = -sinf(fTheta);
  matRotZ.m[1][1] = cosf(fTheta);
  matRotZ.m[2][2] = 1;
  matRotZ.m[3][3] = 1;

  // Rotation X
  matRotX.m[0][0] = 1;
  matRotX.m[1][1] = cosf(fTheta * 0.5f);
  matRotX.m[1][2] = sinf(fTheta * 0.5f);
  matRotX.m[2][1] = -sinf(fTheta * 0.5f);
  matRotX.m[2][2] = cosf(fTheta * 0.5f);
  matRotX.m[3][3] = 1;

  for ( int i = 0; i < 12; i++)
  { 
    memcpy_P( &tri, &cube[i], sizeof( triangle));
    //triangle tri = pgm_read_word_near(cube + i);
    triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;

    // Rotate in Z-Axis
    MultiplyMatrixVector(tri.p[0], triRotatedZ.p[0], matRotZ);
    MultiplyMatrixVector(tri.p[1], triRotatedZ.p[1], matRotZ);
    MultiplyMatrixVector(tri.p[2], triRotatedZ.p[2], matRotZ);

    // Rotate in X-Axis
    MultiplyMatrixVector(triRotatedZ.p[0], triRotatedZX.p[0], matRotX);
    MultiplyMatrixVector(triRotatedZ.p[1], triRotatedZX.p[1], matRotX);
    MultiplyMatrixVector(triRotatedZ.p[2], triRotatedZX.p[2], matRotX);

    // Offset into the screen
    triTranslated = triRotatedZX;
    triTranslated.p[0].z = triRotatedZX.p[0].z + 3.0f;
    triTranslated.p[1].z = triRotatedZX.p[1].z + 3.0f;
    triTranslated.p[2].z = triRotatedZX.p[2].z + 3.0f;

    // Project triangles from 3D --> 2D
    MultiplyMatrixVector(triTranslated.p[0], triProjected.p[0], matProj);
    MultiplyMatrixVector(triTranslated.p[1], triProjected.p[1], matProj);
    MultiplyMatrixVector(triTranslated.p[2], triProjected.p[2], matProj);

    // Scale into view
    triProjected.p[0].x += 1.0f; triProjected.p[0].y += 1.0f;
    triProjected.p[1].x += 1.0f; triProjected.p[1].y += 1.0f;
    triProjected.p[2].x += 1.0f; triProjected.p[2].y += 1.0f;
    
    triProjected.p[0].x *= 0.5f * (float) SCREEN_WIDTH;
    triProjected.p[0].y *= 0.5f * (float) SCREEN_HEIGHT;
    triProjected.p[1].x *= 0.5f * (float) SCREEN_WIDTH;
    triProjected.p[1].y *= 0.5f * (float) SCREEN_HEIGHT;
    triProjected.p[2].x *= 0.5f * (float) SCREEN_WIDTH;
    triProjected.p[2].y *= 0.5f * (float) SCREEN_HEIGHT;

    drawTri(triProjected.p[0].x, triProjected.p[0].y,
            triProjected.p[1].x, triProjected.p[1].y,
            triProjected.p[2].x, triProjected.p[2].y);
  }

  u8g2.sendBuffer();
}
