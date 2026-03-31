// xmodelview.cpp - Standalone CoD1 XModel Viewer
//
// Build:  cmake -B build && cmake --build build
// Usage:
//   xmodelview [--basepath=<cod1_dir>] <modelname>    # interactive
//   xmodelview [--basepath=<cod1_dir>] --batch [--outdir=./shots]
//
// Controls (interactive): mouse-drag to orbit, scroll to zoom, S=screenshot, N=next, Q=quit

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cassert>
#include <setjmp.h>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <functional>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <unordered_map>

#include <dlfcn.h>
#ifdef __cplusplus
extern "C" {
#endif
#include <jpeglib.h>
#ifdef __cplusplus
}
#endif
#include "pk3.h"   // minimal pk3 reader

// ============================================================
// Basic math
// ============================================================

static inline float v3dot(const float *a, const float *b)
{ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
static inline void v3cross(const float *a, const float *b, float *o)
{ o[0]=a[1]*b[2]-a[2]*b[1]; o[1]=a[2]*b[0]-a[0]*b[2]; o[2]=a[0]*b[1]-a[1]*b[0]; }
static inline float v3len(const float *v) { return sqrtf(v3dot(v,v)); }
static inline void v3norm(float *v) { float l=v3len(v); if(l>1e-6f){v[0]/=l;v[1]/=l;v[2]/=l;} }
static inline void v3add(const float *a, const float *b, float *o)
{ o[0]=a[0]+b[0]; o[1]=a[1]+b[1]; o[2]=a[2]+b[2]; }
static inline void v3sub(const float *a, const float *b, float *o)
{ o[0]=a[0]-b[0]; o[1]=a[1]-b[1]; o[2]=a[2]-b[2]; }
static inline void v3scale(const float *a, float s, float *o)
{ o[0]=a[0]*s; o[1]=a[1]*s; o[2]=a[2]*s; }

// Column-major 4x4 matrix
typedef float mat4[16];
static void mat4_identity(mat4 m) {
    memset(m,0,64);
    m[0]=m[5]=m[10]=m[15]=1.f;
}
static void mat4_mul(const mat4 a, const mat4 b, mat4 out) {
    mat4 tmp;
    for(int j=0;j<4;j++) for(int i=0;i<4;i++) {
        float s=0; for(int k=0;k<4;k++) s+=a[i+k*4]*b[k+j*4];
        tmp[i+j*4]=s;
    }
    memcpy(out,tmp,64);
}
static void mat4_perspective(mat4 m, float fovy_rad, float aspect, float zn, float zf) {
    memset(m,0,64);
    float f=1.f/tanf(fovy_rad*.5f);
    m[0]=f/aspect; m[5]=f;
    m[10]=(zf+zn)/(zn-zf); m[11]=-1.f;
    m[14]=2.f*zf*zn/(zn-zf);
}
static void mat4_lookat(mat4 m, const float *eye, const float *center, const float *up) {
    float f[3],r[3],u2[3];
    v3sub(center,eye,f); v3norm(f);
    v3cross(f,up,r); v3norm(r);
    v3cross(r,f,u2);
    mat4_identity(m);
    m[0]=r[0]; m[4]=r[1]; m[8]=r[2];
    m[1]=u2[0];m[5]=u2[1];m[9]=u2[2];
    m[2]=-f[0];m[6]=-f[1];m[10]=-f[2];
    m[12]=-v3dot(r,eye);
    m[13]=-v3dot(u2,eye);
    m[14]=v3dot(f,eye);
}
static void mat4_rotz(mat4 m, float ang) {
    mat4_identity(m);
    float c=cosf(ang),s=sinf(ang);
    m[0]=c; m[4]=-s; m[1]=s; m[5]=c;
}
static void mat4_rotx(mat4 m, float ang) {
    mat4_identity(m);
    float c=cosf(ang),s=sinf(ang);
    m[5]=c; m[9]=-s; m[6]=s; m[10]=c;
}

// Quaternion [w,x,y,z] math
static void quat_mul(const float *a, const float *b, float *o) {
    o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
static void quat_rotvec(const float *q, const float *v, float *o) {
    float tx=2.f*(q[2]*v[2]-q[3]*v[1]);
    float ty=2.f*(q[3]*v[0]-q[1]*v[2]);
    float tz=2.f*(q[1]*v[1]-q[2]*v[0]);
    o[0]=v[0]+q[0]*tx+q[2]*tz-q[3]*ty;
    o[1]=v[1]+q[0]*ty+q[3]*tx-q[1]*tz;
    o[2]=v[2]+q[0]*tz+q[1]*ty-q[2]*tx;
}

// ============================================================
// DXT block decoder → RGBA8
// ============================================================

struct Color8 { uint8_t r,g,b,a; };
static Color8 rgb565(uint16_t c) {
    return { (uint8_t)((c>>11)<<3), (uint8_t)(((c>>5)&63)<<2), (uint8_t)((c&31)<<3), 255 };
}
static Color8 lerp2(Color8 a, Color8 b) {
    return {(uint8_t)((2*a.r+b.r)/3),(uint8_t)((2*a.g+b.g)/3),(uint8_t)((2*a.b+b.b)/3),255};
}
static Color8 lerp1(Color8 a, Color8 b) {
    return {(uint8_t)((a.r+b.r)/2),(uint8_t)((a.g+b.g)/2),(uint8_t)((a.b+b.b)/2),255};
}

// Decode one DXT1 8-byte block into 4x4 RGBA pixels
static void decode_dxt1_block(const uint8_t *src, Color8 out[16], bool punchAlpha=false)
{
    uint16_t c0 = src[0]|(src[1]<<8);
    uint16_t c1 = src[2]|(src[3]<<8);
    uint32_t lu = src[4]|(src[5]<<8)|(src[6]<<16)|((uint32_t)src[7]<<24);
    Color8 pal[4];
    pal[0]=rgb565(c0); pal[1]=rgb565(c1);
    if (c0>c1) {
        pal[2]=lerp2(pal[0],pal[1]); pal[3]=lerp2(pal[1],pal[0]);
    } else {
        pal[2]=lerp1(pal[0],pal[1]);
        pal[3]= punchAlpha ? Color8{0,0,0,0} : pal[2];
    }
    for(int i=0;i<16;i++) out[i]=pal[(lu>>(i*2))&3];
}

// Decode DXT3: explicit 4-bit alpha + DXT1 RGB
static void decode_dxt3_block(const uint8_t *src, Color8 out[16])
{
    Color8 rgb[16];
    decode_dxt1_block(src+8, rgb);
    for(int i=0;i<16;i++) {
        uint8_t a = (i&1) ? (src[i/2]>>4)&0xf : src[i/2]&0xf;
        out[i]={rgb[i].r,rgb[i].g,rgb[i].b,(uint8_t)(a*17)};
    }
}

// Decode DXT5: interpolated alpha + DXT1 RGB
static void decode_dxt5_block(const uint8_t *src, Color8 out[16])
{
    Color8 rgb[16];
    decode_dxt1_block(src+8, rgb);
    uint8_t a0=src[0], a1=src[1];
    uint8_t apal[8];
    apal[0]=a0; apal[1]=a1;
    if(a0>a1){ for(int i=2;i<8;i++) apal[i]=(uint8_t)(((8-i)*a0+(i-1)*a1)/7); }
    else { for(int i=2;i<6;i++) apal[i]=(uint8_t)(((6-i)*a0+(i-1)*a1)/5); apal[6]=0; apal[7]=255; }
    // 48-bit index table: src[2..7]
    uint64_t bits = 0;
    for(int i=0;i<6;i++) bits |= ((uint64_t)src[2+i])<<(i*8);
    for(int i=0;i<16;i++) {
        uint8_t idx=(bits>>(i*3))&7;
        out[i]={rgb[i].r,rgb[i].g,rgb[i].b,apal[idx]};
    }
}

// Full DDS decode → RGBA8 (returns pixels, sets w/h; empty on failure)
static std::vector<uint8_t> decode_dds(const std::vector<uint8_t> &data, int &w, int &h)
{
    if(data.size() < 128) return {};
    if(memcmp(data.data(),"DDS ",4)!=0) return {};
    const uint8_t *d=data.data()+4;
    auto r32=[&](int off){ uint32_t v; memcpy(&v,d+off,4); return v; };
    h=(int)r32(8); w=(int)r32(12);
    uint32_t pfFlags=r32(76), fourCC=r32(80);
    bool isCompressed=(pfFlags&4)!=0;
    bool isRGBA      =(pfFlags&0x41)!=0;

    size_t pixCount=(size_t)w*h;
    std::vector<uint8_t> rgba(pixCount*4, 255);
    const uint8_t *src=data.data()+128;

    if(isCompressed) {
        bool dxt1=(fourCC==0x31545844), dxt3=(fourCC==0x33545844), dxt5=(fourCC==0x35545844);
        if(!dxt1&&!dxt3&&!dxt5) return {};
        int bw=(w+3)/4, bh=(h+3)/4;
        size_t blockBytes=dxt1?8:16;
        for(int by=0;by<bh;by++) for(int bx=0;bx<bw;bx++) {
            Color8 block[16];
            if(dxt1)       decode_dxt1_block(src, block, true);
            else if(dxt3)  decode_dxt3_block(src, block);
            else            decode_dxt5_block(src, block);
            src+=blockBytes;
            for(int py=0;py<4;py++) for(int px=0;px<4;px++) {
                int ix=bx*4+px, iy=by*4+py;
                if(ix>=w||iy>=h) continue;
                auto &c=block[py*4+px];
                size_t idx=((size_t)iy*w+ix)*4;
                rgba[idx]=c.r; rgba[idx+1]=c.g; rgba[idx+2]=c.b; rgba[idx+3]=c.a;
            }
        }
    } else if(isRGBA) {
        uint32_t bpp=r32(88)/8;
        uint32_t rmask=r32(92),gmask=r32(96),bmask=r32(100),amask=r32(104);
        auto findShift=[](uint32_t mask)->int{ if(!mask)return 0; int s=0; while(!(mask&1)){mask>>=1;s++;} return s; };
        int rs=findShift(rmask),gs=findShift(gmask),bs=findShift(bmask),as=findShift(amask);
        for(size_t i=0;i<pixCount;i++) {
            uint32_t pix=0; memcpy(&pix,src+i*bpp,bpp<4?bpp:4);
            rgba[i*4+0]=(uint8_t)((pix&rmask)>>rs);
            rgba[i*4+1]=(uint8_t)((pix&gmask)>>gs);
            rgba[i*4+2]=(uint8_t)((pix&bmask)>>bs);
            rgba[i*4+3]=amask?((uint8_t)((pix&amask)>>as)):255;
        }
    } else return {};
    return rgba;
}

// ============================================================
// VFS - virtual filesystem over multiple pk3 files
// ============================================================

struct VFS {
    std::vector<std::string> paks; // sorted: highest priority last (we search last-first)

    void addPak(const std::string &path) { paks.push_back(path); }

    std::vector<uint8_t> read(const std::string &path) const {
        // Search from last added (highest pak number = highest priority)
        for(int i=(int)paks.size()-1;i>=0;i--) {
            auto data=pk3_read(paks[i].c_str(), path.c_str());
            if(!data.empty()) return data;
        }
        return {};
    }

    std::set<std::string> listPrefix(const std::string &prefix) const {
        std::set<std::string> result;
        for(auto &p:paks) {
            auto entries=pk3_list(p.c_str(), prefix.c_str());
            for(auto &e:entries) result.insert(e);
        }
        return result;
    }
};

static VFS gVFS;

// ============================================================
// xmodel binary reader (port of tr_xmodel.h xmR_t)
// ============================================================

struct XmR {
    const uint8_t *buf; int pos, size;
    XmR(const std::vector<uint8_t> &d): buf(d.data()), pos(0), size((int)d.size()) {}
    uint8_t  u8()  { return pos<size ? buf[pos++] : 0; }
    int8_t   s8()  { return (int8_t)u8(); }
    uint16_t u16() { if(pos+2>size){pos=size;return 0;} uint16_t v=buf[pos]|(buf[pos+1]<<8); pos+=2; return v; }
    int16_t  s16() { return (int16_t)u16(); }
    uint32_t u32() { if(pos+4>size){pos=size;return 0;} uint32_t v=buf[pos]|(buf[pos+1]<<8)|(buf[pos+2]<<16)|((uint32_t)buf[pos+3]<<24); pos+=4; return v; }
    float    f32() { union{uint32_t i;float f;} u; u.i=u32(); return u.f; }
    void     skip(int n){ pos+=n; if(pos>size)pos=size; }
    std::string str() { std::string s; uint8_t c; while((c=u8())!=0) s+=(char)c; return s; }
    void vec3(float *o){ o[0]=f32();o[1]=f32();o[2]=f32(); }
    void quat(float *q){ // compact: 3×s16 / 32768 → x,y,z; w=sqrt(1-...)
        float x=s16()/32768.f, y=s16()/32768.f, z=s16()/32768.f;
        float ww=1.f-x*x-y*y-z*z;
        q[0]=(ww>0.f)?sqrtf(ww):0.f; q[1]=x; q[2]=y; q[3]=z;
    }
};

// ============================================================
// xmodel structures
// ============================================================

struct Bone {
    int    parent;
    char   name[64];
    float  lTrans[3], lRot[4]; // local
    float  wTrans[3], wRot[4]; // world (computed)
};

static void computeWorldBones(Bone *bones, int n) {
    for(int i=0;i<n;i++) {
        if(bones[i].parent<0) {
            memcpy(bones[i].wTrans, bones[i].lTrans, 12);
            memcpy(bones[i].wRot,   bones[i].lRot,   16);
        } else {
            const Bone &p=bones[bones[i].parent];
            quat_rotvec(p.wRot, bones[i].lTrans, bones[i].wTrans);
            bones[i].wTrans[0]+=p.wTrans[0];
            bones[i].wTrans[1]+=p.wTrans[1];
            bones[i].wTrans[2]+=p.wTrans[2];
            quat_mul(p.wRot, bones[i].lRot, bones[i].wRot);
        }
    }
}

struct Vertex { float x,y,z, nx,ny,nz, u,v; };

struct Surface {
    std::vector<Vertex>   verts;
    std::vector<uint32_t> indices;
    std::string           matName;
};

struct XModel {
    std::vector<Surface> surfaces;
    float mins[3], maxs[3], center[3];
    float radius;
};

// ============================================================
// xmodel parsing (ported from tr_model_xmodel.c)
// ============================================================

#define COD1_XMODEL_VERSION 0x0E
#define COD1_RIGGED         65535
#define XMODEL_MAX_BONES    256
#define XMODEL_MAX_LODS     3
#define XMODEL_MAX_MATS     32

struct XmLod { std::string name; std::vector<std::string> matNames; };

static bool loadParts(const std::string &lodName, Bone *bones, int &numBones)
{
    auto data=gVFS.read("xmodelparts/"+lodName);
    if(data.empty()){ fprintf(stderr,"xmodelparts/%s: not found\n",lodName.c_str()); return false; }
    XmR r(data);
    if(r.u16()!=COD1_XMODEL_VERSION) return false;
    int boneCount=(int)r.u16(), rootCount=(int)r.u16();
    numBones=boneCount+rootCount;
    if(numBones<=0||numBones>XMODEL_MAX_BONES) return false;

    for(int i=0;i<numBones;i++) {
        bones[i].parent=-1;
        bones[i].lRot[0]=1.f; bones[i].lRot[1]=bones[i].lRot[2]=bones[i].lRot[3]=0.f;
        memset(bones[i].lTrans,0,12);
        memset(bones[i].wTrans,0,12);
        bones[i].wRot[0]=1.f; bones[i].wRot[1]=bones[i].wRot[2]=bones[i].wRot[3]=0.f;
    }
    // Non-root bones: [rootCount .. numBones-1]
    for(int i=0;i<boneCount;i++) {
        int idx=rootCount+i;
        bones[idx].parent=(int)r.s8();
        r.vec3(bones[idx].lTrans);
        r.quat(bones[idx].lRot);
    }
    // All bone names + 24-byte skip each
    for(int i=0;i<numBones;i++) {
        auto name=r.str(); r.skip(24);
        snprintf(bones[i].name,64,"%s",name.c_str());
    }
    computeWorldBones(bones, numBones);
    return true;
}

static int decodeFanStrip(XmR &r, std::vector<uint32_t> &idx, int maxTris)
{
    int written=0;
    uint8_t cnt=r.u8();
    uint32_t i1=r.u16(), i2=r.u16(), i3=r.u16();
    if(i1!=i2&&i1!=i3&&i2!=i3&&(int)idx.size()/3<maxTris){idx.push_back(i1);idx.push_back(i2);idx.push_back(i3);written++;}
    for(uint8_t k=3;k<cnt;) {
        uint32_t i4=i3, i5=r.u16(); k++;
        if(i4!=i2&&i4!=i5&&i2!=i5&&(int)idx.size()/3<maxTris){idx.push_back(i4);idx.push_back(i2);idx.push_back(i5);written++;}
        if(k>=cnt) break;
        i2=i5; i3=r.u16(); k++;
        if(i4!=i2&&i4!=i3&&i2!=i3&&(int)idx.size()/3<maxTris){idx.push_back(i4);idx.push_back(i2);idx.push_back(i3);written++;}
    }
    return written;
}

static bool loadSurfs(const std::string &lodName, const Bone *bones, int numBones,
                      const XmLod &lod, XModel &model)
{
    auto data=gVFS.read("xmodelsurfs/"+lodName);
    if(data.empty()){ fprintf(stderr,"xmodelsurfs/%s: not found\n",lodName.c_str()); return false; }
    XmR r(data);
    if(r.u16()!=COD1_XMODEL_VERSION) return false;
    int numSurfs=(int)r.u16();
    if(numSurfs<=0) return false;

    for(int i=0;i<numSurfs;i++) {
        r.skip(1);
        int vertCount=(int)r.u16(), triCount=(int)r.u16();
        r.skip(2);
        int ogBone=(int)r.u16();
        bool rigged=(ogBone==COD1_RIGGED);
        if(rigged) r.skip(4);

        Surface surf;
        int matIdx=(int)lod.matNames.size()>0 ? std::min(i,(int)lod.matNames.size()-1) : -1;
        if(matIdx>=0) surf.matName=lod.matNames[matIdx];

        // Decode triangles first (fan-encoded)
        int decoded=0;
        while(decoded<triCount) {
            int got=decodeFanStrip(r, surf.indices, triCount);
            if(got<=0) break;
            decoded+=got;
        }

        // Per-vertex bone data
        std::vector<int>   boneIdx(vertCount, ogBone);
        std::vector<int>   wCount(vertCount, 0);
        std::vector<float> localPos(vertCount*3), localNorm(vertCount*3);

        for(int j=0;j<vertCount;j++) {
            r.vec3(&localNorm[j*3]);
            float u=r.f32(), v=r.f32();
            int wi=0, bi=ogBone;
            if(rigged){ wi=(int)r.u16(); bi=(int)r.u16(); }
            float lp[3]; r.vec3(lp);
            if(wi) r.skip(4);
            wCount[j]=wi; boneIdx[j]=bi;
            localPos[j*3]=lp[0]; localPos[j*3+1]=lp[1]; localPos[j*3+2]=lp[2];

            // Transform to world space via bone
            float wp[3], wn[3];
            if(bi>=0&&bi<numBones) {
                quat_rotvec(bones[bi].wRot, lp, wp);
                wp[0]+=bones[bi].wTrans[0]; wp[1]+=bones[bi].wTrans[1]; wp[2]+=bones[bi].wTrans[2];
                quat_rotvec(bones[bi].wRot, &localNorm[j*3], wn);
            } else {
                memcpy(wp,lp,12); memcpy(wn,&localNorm[j*3],12);
            }
            float nl=v3len(wn); if(nl>1e-6f){wn[0]/=nl;wn[1]/=nl;wn[2]/=nl;}

            Vertex vert{wp[0],wp[1],wp[2], wn[0],wn[1],wn[2], u,v};
            surf.verts.push_back(vert);
        }
        // Skip bone weights
        for(int j=0;j<vertCount;j++)
            for(int k=0;k<wCount[j];k++) { r.u16(); r.skip(12); r.f32(); }

        model.surfaces.push_back(std::move(surf));
    }
    return true;
}

static bool loadXModel(const std::string &name, XModel &model)
{
    std::string path = (name.find("xmodel/")==0) ? name : "xmodel/"+name;
    auto data=gVFS.read(path);
    if(data.empty()){ fprintf(stderr,"xmodel not found: %s\n",path.c_str()); return false; }

    XmR r(data);
    if(r.u16()!=COD1_XMODEL_VERSION){ fprintf(stderr,"bad xmodel version\n"); return false; }

    // Bounds
    r.vec3(model.mins); r.vec3(model.maxs);

    // LODs
    XmLod lods[XMODEL_MAX_LODS]; int numLods=0;
    for(int i=0;i<XMODEL_MAX_LODS;i++) {
        r.f32(); // lod distance
        auto n=r.str();
        if(!n.empty()&&numLods<XMODEL_MAX_LODS) lods[numLods++].name=n;
    }
    if(numLods==0) return false;
    r.skip(4); // collision lod enum

    // Skip collision surfaces
    uint32_t ncs=r.u32();
    for(uint32_t i=0;i<ncs&&i<4096;i++) {
        uint32_t nt=r.u32();
        r.skip((int)nt*48+24); // tris + bounds(24)
        r.skip(12); // boneIdx+contents+surfFlags
    }

    // Material names per LOD
    for(int i=0;i<numLods;i++) {
        int mc=(int)r.u16();
        for(int j=0;j<mc;j++) {
            auto mn=r.str();
            if(j<XMODEL_MAX_MATS) lods[i].matNames.push_back(mn);
        }
    }

    // Load skeleton + surfaces for LOD 0
    Bone bones[XMODEL_MAX_BONES]; int numBones=0;
    if(!loadParts(lods[0].name, bones, numBones)) return false;
    if(!loadSurfs(lods[0].name, bones, numBones, lods[0], model)) return false;

    // Compute center + radius from all vertices
    float mn[3]={1e30f,1e30f,1e30f}, mx[3]={-1e30f,-1e30f,-1e30f};
    for(auto &s:model.surfaces)
        for(auto &v:s.verts) {
            mn[0]=std::min(mn[0],v.x); mn[1]=std::min(mn[1],v.y); mn[2]=std::min(mn[2],v.z);
            mx[0]=std::max(mx[0],v.x); mx[1]=std::max(mx[1],v.y); mx[2]=std::max(mx[2],v.z);
        }
    model.center[0]=(mn[0]+mx[0])*.5f;
    model.center[1]=(mn[1]+mx[1])*.5f;
    model.center[2]=(mn[2]+mx[2])*.5f;
    float d[3]; v3sub(mx,mn,d);
    model.radius=v3len(d)*.5f;
    if(model.radius<0.1f) model.radius=10.f;

    // Center vertices at origin
    for(auto &s:model.surfaces)
        for(auto &v:s.verts) {
            v.x-=model.center[0]; v.y-=model.center[1]; v.z-=model.center[2];
        }

    printf("Loaded %s: %d surfaces, radius=%.1f\n", path.c_str(), (int)model.surfaces.size(), model.radius);
    return true;
}

// ============================================================
// Shader material table
// Parse scripts/*.shader to map material name → first diffuse texture
// ============================================================

static std::unordered_map<std::string,std::string> gShaderMap;

static void toLower(std::string &s){ for(char &c:s) if(c>='A'&&c<='Z') c+=32; }

static void parseShaderFile(const std::vector<uint8_t> &data)
{
    // Tokenise: split by whitespace, track brace depth
    std::string text(data.begin(), data.end());
    std::string curShader;
    int depth=0;
    std::string mapped;
    size_t i=0, n=text.size();

    auto skipWS=[&](){
        for(;;) {
            while(i<n && std::isspace((unsigned char)text[i])) i++;
            if(i+1<n && text[i]=='/' && text[i+1]=='/') {
                i+=2;
                while(i<n && text[i]!='\n') i++;
                continue;
            }
            if(i+1<n && text[i]=='/' && text[i+1]=='*') {
                i+=2;
                while(i+1<n && !(text[i]=='*' && text[i+1]=='/')) i++;
                if(i+1<n) i+=2;
                continue;
            }
            break;
        }
    };
    auto readToken=[&]()->std::string{
        skipWS();
        if(i>=n) return {};
        if(text[i]=='{'||text[i]=='}') return std::string(1,text[i++]);
        size_t s=i;
        while(i<n && !std::isspace((unsigned char)text[i]) && text[i]!='{' && text[i]!='}') i++;
        return text.substr(s,i-s);
    };

    while(i<n) {
        auto tok=readToken();
        if(tok.empty()) break;
        if(tok=="{") {
            depth++;
        } else if(tok=="}") {
            depth--;
            if(depth==0){curShader.clear();mapped.clear();}
        } else if(depth==0) {
            curShader=tok; toLower(curShader);
            mapped.clear();
        } else if(depth>0) {
            // Look for map/clampMap/animMap directives
            std::string tl=tok; toLower(tl);
            if((tl=="map"||tl=="clampmap")&&mapped.empty()) {
                auto val=readToken();
                if(!val.empty()&&val!="$lightmap"&&val!="$whiteimage"&&val!="$nodraw") {
                    toLower(val);
                    mapped=val;
                    if(!curShader.empty()) gShaderMap[curShader]=mapped;
                }
            } else if(tl=="animmap"&&mapped.empty()) {
                readToken(); // animation rate
                auto val=readToken();
                if(!val.empty()&&val!="$lightmap"&&val!="$whiteimage"&&val!="$nodraw") {
                    toLower(val);
                    mapped=val;
                    if(!curShader.empty()) gShaderMap[curShader]=mapped;
                }
            }
        }
    }
}

static void loadAllShaders()
{
    auto shaderFiles=gVFS.listPrefix("scripts/");
    for(auto &sf:shaderFiles){
        if(sf.size()<7) continue;
        std::string ext=sf.substr(sf.size()-7);
        toLower(ext);
        if(ext==".shader"){
            auto d=gVFS.read(sf);
            if(!d.empty()) parseShaderFile(d);
        }
    }
    printf("Shader table: %d entries\n",(int)gShaderMap.size());
}

// ============================================================
// SDL2_image optional loader (dlopen so we don't need -dev pkg)
// ============================================================

typedef SDL_Surface* (*IMG_Load_RW_fn)(SDL_RWops*,int);
static IMG_Load_RW_fn g_IMG_Load_RW = nullptr;
static void tryLoadSDL2Image() {
    static bool tried=false; if(tried) return; tried=true;
    void *lib=dlopen("libSDL2_image-2.0.so.0",RTLD_LAZY);
    if(!lib){ fprintf(stderr,"SDL2_image not loaded (TGA/JPG skins will be white): %s\n",dlerror()); return; }
    g_IMG_Load_RW=(IMG_Load_RW_fn)dlsym(lib,"IMG_Load_RW");
    if(g_IMG_Load_RW) printf("SDL2_image loaded (TGA/JPG support enabled)\n");
}

// ============================================================
// Texture loading
// ============================================================

static GLuint uploadRGBA(const std::vector<uint8_t> &rgba, int w, int h)
{
    GLuint tex; glGenTextures(1,&tex);
    glBindTexture(GL_TEXTURE_2D,tex);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,rgba.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    return tex;
}

static GLuint makeCheckerTex()
{
    // 8×8 magenta/black checkerboard — instantly visible as "missing texture"
    uint8_t pix[8*8*4];
    for(int y=0;y<8;y++) for(int x=0;x<8;x++){
        bool on=((x^y)&1)!=0;
        uint8_t *p=pix+(y*8+x)*4;
        p[0]=on?255:0; p[1]=0; p[2]=on?255:0; p[3]=255;
    }
    GLuint tex; glGenTextures(1,&tex);
    glBindTexture(GL_TEXTURE_2D,tex);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,8,8,0,GL_RGBA,GL_UNSIGNED_BYTE,pix);
    return tex;
}

static bool decode_tga(const std::vector<uint8_t> &data, int &w, int &h, std::vector<uint8_t> &rgba)
{
    if(data.size() < 18) return false;
    const uint8_t *d=data.data();
    int idLen=d[0], cmapType=d[1], imgType=d[2];
    if(cmapType!=0) return false;
    if(imgType!=2 && imgType!=3 && imgType!=10 && imgType!=11) return false;

    w=(int)(d[12] | (d[13]<<8));
    h=(int)(d[14] | (d[15]<<8));
    int bpp=d[16];
    uint8_t desc=d[17];
    if(w<=0 || h<=0) return false;

    int bytesPerPixel=0;
    if(imgType==3 || imgType==11) {
        if(bpp!=8) return false;
        bytesPerPixel=1;
    } else {
        if(bpp==24) bytesPerPixel=3;
        else if(bpp==32) bytesPerPixel=4;
        else return false;
    }

    size_t off=(size_t)18 + (size_t)idLen;
    size_t pixelCount=(size_t)w*h;
    rgba.assign(pixelCount*4, 255);

    auto mapIndex=[&](size_t i)->size_t{
        size_t x=i % (size_t)w;
        size_t y=i / (size_t)w;
        if((desc & 0x10) != 0) x=(size_t)w-1-x;
        if((desc & 0x20) == 0) y=(size_t)h-1-y;
        return (y*(size_t)w + x)*4;
    };
    auto writePixel=[&](size_t i, const uint8_t *src){
        size_t dst=mapIndex(i);
        if(bytesPerPixel==1) {
            rgba[dst+0]=src[0];
            rgba[dst+1]=src[0];
            rgba[dst+2]=src[0];
            rgba[dst+3]=255;
        } else {
            rgba[dst+0]=src[2];
            rgba[dst+1]=src[1];
            rgba[dst+2]=src[0];
            rgba[dst+3]=(bytesPerPixel==4)?src[3]:255;
        }
    };

    if(imgType==2 || imgType==3) {
        size_t need=off + pixelCount*(size_t)bytesPerPixel;
        if(need > data.size()) return false;
        for(size_t i=0;i<pixelCount;i++, off+=(size_t)bytesPerPixel)
            writePixel(i, data.data()+off);
        return true;
    }

    size_t written=0;
    while(written<pixelCount && off<data.size()) {
        uint8_t packet=data[off++];
        size_t count=(size_t)(packet & 0x7f) + 1;
        if(packet & 0x80) {
            if(off + (size_t)bytesPerPixel > data.size()) return false;
            const uint8_t *src=data.data()+off;
            off += (size_t)bytesPerPixel;
            for(size_t i=0;i<count && written<pixelCount;i++,written++) writePixel(written, src);
        } else {
            size_t need=off + count*(size_t)bytesPerPixel;
            if(need > data.size()) return false;
            for(size_t i=0;i<count && written<pixelCount;i++,written++,off+=(size_t)bytesPerPixel)
                writePixel(written, data.data()+off);
        }
    }
    return written==pixelCount;
}

static bool decodeToRGBA(const std::vector<uint8_t> &data, std::vector<uint8_t> &rgba, int &w, int &h)
{
    if(data.size()>4&&memcmp(data.data(),"DDS ",4)==0){
        rgba=decode_dds(data,w,h);
        if(!rgba.empty()) return true;
    }
    if(decode_tga(data,w,h,rgba)) return true;
    if(g_IMG_Load_RW){
        SDL_RWops *rw=SDL_RWFromMem((void*)data.data(),(int)data.size());
        SDL_Surface *surf=g_IMG_Load_RW(rw,0); SDL_RWclose(rw);
        if(surf){
            SDL_Surface *c=SDL_ConvertSurfaceFormat(surf,SDL_PIXELFORMAT_RGBA32,0);
            SDL_FreeSurface(surf);
            if(c){
                w=c->w; h=c->h;
                rgba.resize((size_t)w*h*4);
                for(int y=0;y<c->h;y++)
                    memcpy(rgba.data()+(size_t)y*w*4, (uint8_t*)c->pixels+(size_t)y*c->pitch, (size_t)w*4);
                SDL_FreeSurface(c);
                return true;
            }
        }
    }
    return false;
}

static GLuint decodeAndUpload(const std::vector<uint8_t> &data)
{
    std::vector<uint8_t> rgba;
    int w=0,h=0;
    if(!decodeToRGBA(data,rgba,w,h)) return 0;
    return uploadRGBA(rgba,w,h);
}

static bool canDecodeTexturePath(const std::string &path)
{
    auto data=gVFS.read(path);
    if(data.empty()) return false;
    std::vector<uint8_t> rgba;
    int w=0,h=0;
    return decodeToRGBA(data,rgba,w,h);
}

static std::string resolvePath(const std::string &p)
{
    // Try exact, then swap/add extension
    static const char *exts[]={"",".dds",".jpg",".tga",".png",nullptr};
    std::string base=p;
    toLower(base);
    size_t dot=base.rfind('/');
    size_t dotPos=base.rfind('.');
    if(dotPos!=std::string::npos&&(dot==std::string::npos||dotPos>dot)) base=base.substr(0,dotPos);

    for(int i=0;exts[i];i++){
        std::string candidate=(i==0)?base:(base+exts[i]);
        if(!gVFS.read(candidate).empty()) return candidate;
    }
    return {};
}

static std::unordered_map<std::string,GLuint> gTexCache;
static std::unordered_map<std::string,std::string> gTexPathCache;

static void clearTextureCache()
{
    for(auto &it : gTexCache) {
        if(it.second != 0) glDeleteTextures(1, &it.second);
    }
    gTexCache.clear();
    gTexPathCache.clear();
}

static void addUniquePath(std::vector<std::string> &paths, const std::string &path)
{
    if(path.empty()) return;
    if(std::find(paths.begin(),paths.end(),path)==paths.end()) paths.push_back(path);
}

static void addTemplateSkinFallbacks(const std::string &matName, std::vector<std::string> &paths)
{
    size_t at=matName.find("@default");
    if(at==std::string::npos) return;

    std::string prefix=matName.substr(0,at);
    addUniquePath(paths,"skins/"+prefix+"@hand");
    addUniquePath(paths,"skins/"+prefix+"@characterhand");

    auto variants=gVFS.listPrefix("skins/"+prefix+"@");
    for(auto &v:variants) {
        if(v.empty()||v.back()=='/') continue;
        std::string lv=v;
        toLower(lv);
        if(lv.find("@default")!=std::string::npos) continue;
        addUniquePath(paths,lv);
    }
}

static std::string resolveTexturePath(const std::string &matName)
{
    std::string key=matName;
    toLower(key);
    auto it=gTexPathCache.find(key);
    if(it!=gTexPathCache.end()) return it->second;
    auto remember=[&](const std::string &path){ gTexPathCache[key]=path; return path; };

    // 1) Check shader table (material name might be a shader, not a direct texture)
    std::string lname=key;
    // strip path prefix for shader lookup: "skins/foo@default" → try "skins/foo@default"
    {
        auto it=gShaderMap.find(lname);
        if(it==gShaderMap.end()){
            // Also try without extension
            std::string base=lname;
            size_t dot=base.rfind('.'); if(dot!=std::string::npos) base=base.substr(0,dot);
            it=gShaderMap.find(base);
            if(it==gShaderMap.end()) it=gShaderMap.find("skins/"+base);
        }
        if(it!=gShaderMap.end()){
            auto path=resolvePath(it->second); if(!path.empty()) return remember(path);
        }
    }

    // 2) Direct path candidates
    std::string base=key;
    size_t dot=base.rfind('.');
    if(dot!=std::string::npos) base=base.substr(0,dot);

    std::vector<std::string> paths={
        "skins/"+key,
        "skins/"+base,
        "skins/"+base+".dds",
        "skins/"+base+".jpg",
        "skins/"+base+".tga",
        "skins/"+base+".png",
        "textures/"+base,
        "textures/"+base+".dds",
        "textures/"+base+".jpg",
        "textures/"+base+".tga",
        "textures/"+base+".png",
        "textures/"+key,
        key,
        base,
    };
    addTemplateSkinFallbacks(key, paths);
    for(auto &p:paths){
        auto path=resolvePath(p);
        if(!path.empty()) return remember(path);
    }
    return remember({});
}

static GLuint loadTexture(const std::string &matName)
{
    std::string key=matName;
    toLower(key);
    auto it=gTexCache.find(key);
    if(it!=gTexCache.end()) return it->second;
    auto remember=[&](GLuint tex){ gTexCache[key]=tex; return tex; };

    auto path=resolveTexturePath(matName);
    if(path.empty()) return remember(0);

    auto data=gVFS.read(path);
    if(data.empty()) return remember(0);
    return remember(decodeAndUpload(data));
}

enum {
    ALPHA_OPAQUE = 0,
    ALPHA_CUTOUT = 1,
    ALPHA_BLEND  = 2,
};

static int classifyAlphaMode(const std::string &matName)
{
    std::string s=matName;
    toLower(s);

    auto path=resolveTexturePath(matName);
    if(!path.empty()) {
        s += " ";
        s += path;
    }

    if(s.find("_masked@") != std::string::npos ||
       s.find("foliage_masked@") != std::string::npos ||
       s.find("fence") != std::string::npos ||
       s.find("wiremesh") != std::string::npos ||
       s.find("barbed") != std::string::npos ||
       s.find("/net") != std::string::npos)
        return ALPHA_CUTOUT;

    if(s.find("decal@") != std::string::npos ||
       s.find("transparents/") != std::string::npos ||
       s.find("glass") != std::string::npos)
        return ALPHA_BLEND;

    return ALPHA_OPAQUE;
}

// ============================================================
// OpenGL shader
// ============================================================

static const char *VERT_SRC = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNorm;
layout(location=2) in vec2 aUV;
uniform mat4 uMVP;
uniform mat4 uMod;
out vec3 vNorm;
out vec2 vUV;
void main(){
    gl_Position=uMVP*vec4(aPos,1.0);
    vNorm=mat3(uMod)*aNorm;
    vUV=aUV;
}
)";

static const char *FRAG_SRC = R"(
#version 330 core
in vec3 vNorm; in vec2 vUV;
uniform sampler2D uTex;
uniform int uAlphaMode;
out vec4 fColor;
void main(){
    vec3 n=normalize(vNorm);
    vec3 L1=normalize(vec3(1.0,0.8,1.2));
    vec3 L2=normalize(vec3(-0.5,-1.0,0.3));
    float d=max(dot(n,L1),0.0)+0.25*max(dot(n,L2),0.0);
    vec4 col=texture(uTex,vUV);
    if(uAlphaMode == 1){
        if(col.a > 0.5) discard;
        fColor=vec4(col.rgb*(0.35+0.65*d),1.0);
    } else if(uAlphaMode == 2){
        fColor=vec4(col.rgb*(0.35+0.65*d),col.a);
    } else {
        fColor=vec4(col.rgb*(0.35+0.65*d),1.0);
    }
}
)";

static GLuint compileShader(GLenum type, const char *src)
{
    GLuint s=glCreateShader(type);
    glShaderSource(s,1,&src,nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
    if(!ok){ char buf[512]; glGetShaderInfoLog(s,512,nullptr,buf); fprintf(stderr,"Shader: %s\n",buf); }
    return s;
}
static GLuint makeProgram() {
    GLuint vs=compileShader(GL_VERTEX_SHADER,VERT_SRC);
    GLuint fs=compileShader(GL_FRAGMENT_SHADER,FRAG_SRC);
    GLuint p=glCreateProgram();
    glAttachShader(p,vs); glAttachShader(p,fs);
    glLinkProgram(p);
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

// ============================================================
// GPU mesh per surface
// ============================================================

struct GPUSurf {
    GLuint vao=0, vbo=0, ibo=0;
    int    indexCount=0;
    GLuint tex=0;
    int    alphaMode=ALPHA_OPAQUE;
};

static GPUSurf uploadSurface(const Surface &s, GLuint whiteTex)
{
    GPUSurf g;
    if(s.verts.empty()||s.indices.empty()) return g;

    glGenVertexArrays(1,&g.vao);
    glBindVertexArray(g.vao);

    glGenBuffers(1,&g.vbo);
    glBindBuffer(GL_ARRAY_BUFFER,g.vbo);
    glBufferData(GL_ARRAY_BUFFER,s.verts.size()*sizeof(Vertex),s.verts.data(),GL_STATIC_DRAW);

    glGenBuffers(1,&g.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,g.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,s.indices.size()*sizeof(uint32_t),s.indices.data(),GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)12);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)24);

    glBindVertexArray(0);
    g.indexCount=(int)s.indices.size();

    if(!s.matName.empty()) {
        g.tex=loadTexture(s.matName);
        g.alphaMode=classifyAlphaMode(s.matName);
    }
    if(!g.tex) g.tex=whiteTex;
    return g;
}

static void freeSurface(GPUSurf &g) {
    if(g.vao){ glDeleteVertexArrays(1,&g.vao); g.vao=0; }
    if(g.vbo){ glDeleteBuffers(1,&g.vbo); g.vbo=0; }
    if(g.ibo){ glDeleteBuffers(1,&g.ibo); g.ibo=0; }
}

// ============================================================
// Screenshot + gallery output
// ============================================================

struct JpegErrorMgr {
    jpeg_error_mgr pub;
    jmp_buf        setjmpBuf;
    char           msg[JMSG_LENGTH_MAX];
};

struct GalleryItem {
    std::string modelName;
    std::string fileName;
};

static void jpegErrorExit(j_common_ptr cinfo)
{
    auto *err=(JpegErrorMgr*)cinfo->err;
    (*cinfo->err->format_message)(cinfo, err->msg);
    longjmp(err->setjmpBuf, 1);
}

static bool readRGBPixels(int w, int h, std::vector<uint8_t> &pix)
{
    pix.resize((size_t)w*h*3);
    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,pix.data());
    if(glGetError()!=GL_NO_ERROR) return false;

    for(int y=0;y<h/2;y++) {
        uint8_t *a=pix.data()+(size_t)y*w*3;
        uint8_t *b=pix.data()+(size_t)(h-1-y)*w*3;
        for(int x=0;x<w*3;x++) std::swap(a[x],b[x]);
    }
    return true;
}

static bool saveJPEG(const std::string &path, int w, int h, int quality)
{
    std::vector<uint8_t> pix;
    if(!readRGBPixels(w,h,pix)){
        fprintf(stderr,"glReadPixels failed for %s\n",path.c_str());
        return false;
    }

    FILE *f=fopen(path.c_str(),"wb");
    if(!f){ fprintf(stderr,"Cannot write %s\n",path.c_str()); return false; }

    jpeg_compress_struct cinfo{};
    JpegErrorMgr jerr{};
    cinfo.err=jpeg_std_error(&jerr.pub);
    cinfo.err->error_exit=jpegErrorExit;

    if(setjmp(jerr.setjmpBuf)) {
        fprintf(stderr,"JPEG write failed for %s: %s\n",path.c_str(),jerr.msg);
        jpeg_destroy_compress(&cinfo);
        fclose(f);
        return false;
    }

    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo,f);
    cinfo.image_width=w;
    cinfo.image_height=h;
    cinfo.input_components=3;
    cinfo.in_color_space=JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    if(quality>=85) {
        cinfo.comp_info[0].h_samp_factor=1;
        cinfo.comp_info[0].v_samp_factor=1;
    }
    jpeg_start_compress(&cinfo, TRUE);

    while(cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row=(JSAMPROW)(pix.data() + (size_t)cinfo.next_scanline*w*3);
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(f);
    printf("Screenshot: %s\n",path.c_str());
    return true;
}

static std::string htmlEscape(const std::string &s)
{
    std::string out;
    out.reserve(s.size()+16);
    for(char c:s) {
        switch(c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default: out += c; break;
        }
    }
    return out;
}

static void writeGallery(const std::string &outdir, const std::vector<GalleryItem> &items)
{
    std::ofstream f(outdir + "/index.html", std::ios::binary);
    if(!f) {
        fprintf(stderr,"Cannot write %s/index.html\n",outdir.c_str());
        return;
    }

    f << "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">"
      << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
      << "<title>xmodelview gallery</title><style>"
      << ":root{color-scheme:light;background:#ede7d8;color:#1b1a17;font-family:Georgia,serif}"
      << "body{margin:0;padding:24px;background:linear-gradient(180deg,#f7f2e8,#e7dfcf)}"
      << "header{position:sticky;top:0;backdrop-filter:blur(8px);background:rgba(247,242,232,.85);padding:16px 0 20px;z-index:2}"
      << "h1{margin:0 0 8px;font-size:28px}p{margin:0;color:#5a544a}"
      << ".bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin-top:14px}"
      << "input{flex:1;min-width:240px;padding:12px 14px;border:1px solid #bcae95;border-radius:999px;background:#fffaf0;font:inherit}"
      << "#count{font:600 14px/1.2 monospace;color:#6a614f}"
      << ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:18px;margin-top:22px}"
      << ".card{background:#fffaf0;border:1px solid #d4c7b1;border-radius:16px;overflow:hidden;box-shadow:0 10px 30px rgba(61,46,24,.08)}"
      << ".thumb{display:block;aspect-ratio:4/3;background:#d8d1c4}"
      << ".thumb img{display:block;width:100%;height:100%;object-fit:cover}"
      << ".meta{padding:12px 14px 14px}.name{font:600 15px/1.3 monospace;word-break:break-word}"
      << ".file{margin-top:6px;font:12px/1.3 monospace;color:#736957;word-break:break-all}"
      << ".hidden{display:none!important}"
      << "</style></head><body><header><h1>xmodelview gallery</h1><p>Batch screenshots with client-side search.</p>"
      << "<div class=\"bar\"><input id=\"q\" type=\"search\" placeholder=\"Search model names...\" autofocus>"
      << "<span id=\"count\"></span></div></header><main class=\"grid\" id=\"grid\">";

    for(const auto &item:items) {
        std::string search=item.modelName + " " + item.fileName;
        toLower(search);
        f << "<article class=\"card\" data-name=\"" << htmlEscape(search) << "\">"
          << "<a class=\"thumb\" href=\"" << htmlEscape(item.fileName) << "\">"
          << "<img loading=\"lazy\" src=\"" << htmlEscape(item.fileName) << "\" alt=\"" << htmlEscape(item.modelName) << "\"></a>"
          << "<div class=\"meta\"><div class=\"name\">" << htmlEscape(item.modelName) << "</div>"
          << "<div class=\"file\">" << htmlEscape(item.fileName) << "</div></div></article>";
    }

    f << "</main><script>"
      << "const q=document.getElementById('q');const cards=[...document.querySelectorAll('.card')];const count=document.getElementById('count');"
      << "function update(){const s=q.value.trim().toLowerCase();let n=0;for(const c of cards){const ok=!s||c.dataset.name.includes(s);c.classList.toggle('hidden',!ok);if(ok)n++;}"
      << "count.textContent=`${n} / ${cards.length} shown`;}"
      << "q.addEventListener('input',update);update();"
      << "</script></body></html>\n";
    printf("Gallery: %s/index.html\n",outdir.c_str());
}

// ============================================================
// Render one frame
// CoD1 space: X=right, Y=forward, Z=up
// Camera orbits using spherical coords (theta=azimuth, phi=elevation)
// ============================================================

static void renderModel(const std::vector<GPUSurf> &surfs, GLuint prog,
                        float theta, float phi, float dist,
                        int W, int H)
{
    glViewport(0,0,W,H);
    glClearColor(0.18f,0.18f,0.22f,1.f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);

    // Clamp elevation so we don't flip through the poles
    if(phi> 1.48f) phi= 1.48f;
    if(phi<-1.48f) phi=-1.48f;

    mat4 proj,view,mod,mvp,tmp;
    mat4_perspective(proj, 55.f*3.14159f/180.f, (float)W/H, dist*0.001f, dist*50.f);

    // Spherical camera position (Z-up)
    float ex = dist * cosf(phi) * cosf(theta);
    float ey = dist * cosf(phi) * sinf(theta);
    float ez = dist * sinf(phi);
    float eye[3]={ex,ey,ez}, center[3]={0,0,0}, up[3]={0,0,1};
    mat4_lookat(view,eye,center,up);
    mat4_identity(mod);
    mat4_mul(view,mod,tmp);
    mat4_mul(proj,tmp,mvp);

    glUseProgram(prog);
    glUniformMatrix4fv(glGetUniformLocation(prog,"uMVP"),1,GL_FALSE,mvp);
    glUniformMatrix4fv(glGetUniformLocation(prog,"uMod"),1,GL_FALSE,mod);
    GLint alphaModeLoc=glGetUniformLocation(prog,"uAlphaMode");

    for(auto &g:surfs) {
        if(!g.vao) continue;
        if(g.alphaMode==ALPHA_BLEND) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
        } else {
            glDisable(GL_BLEND);
            glDepthMask(GL_TRUE);
        }
        glUniform1i(alphaModeLoc,g.alphaMode);
        glBindTexture(GL_TEXTURE_2D,g.tex);
        glBindVertexArray(g.vao);
        glDrawElements(GL_TRIANGLES,g.indexCount,GL_UNSIGNED_INT,0);
    }
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

// ============================================================
// Utility: model name → safe filename
// ============================================================

static std::string safeName(const std::string &n) {
    std::string s=n;
    // Remove "xmodel/" prefix
    if(s.find("xmodel/")==0) s=s.substr(7);
    for(char &c:s) if(c=='/'||c=='\\') c='_';
    return s;
}

// ============================================================
// Main
// ============================================================

int main(int argc, char **argv)
{
    std::string basepath;
    std::string modelName;
    std::string outdir="shots";
    bool batch=false;
    bool checkTextures=false;
    int jpgQuality=90;
    int W=800, H=600;

    for(int i=1;i<argc;i++) {
        std::string a=argv[i];
        if(a.find("--basepath=")==0)  basepath=a.substr(11);
        else if(a.find("--outdir=")==0) outdir=a.substr(9);
        else if(a.find("--quality=")==0) jpgQuality=atoi(a.substr(10).c_str());
        else if(a.find("--jpg-quality=")==0) jpgQuality=atoi(a.substr(14).c_str());
        else if(a.find("--width=")==0)  W=atoi(a.substr(8).c_str());
        else if(a.find("--height=")==0) H=atoi(a.substr(9).c_str());
        else if(a=="--batch")           batch=true;
        else if(a=="--check-textures")  checkTextures=true;
        else                            modelName=a;
    }
    jpgQuality=std::clamp(jpgQuality, 1, 100);

    if(checkTextures && modelName.empty()) batch=true;

    // Auto-detect CoD1 steam path
    if(basepath.empty()) {
        const char *home=getenv("HOME");
        if(home) basepath=std::string(home)+"/.steam/steam/steamapps/common/Call of Duty";
    }

    // Validate args early
    if(!batch && !checkTextures && modelName.empty()) {
        fprintf(stderr,"Usage: xmodelview [--basepath=<cod1_dir>] <modelname>\n"
                       "       xmodelview [--basepath=<cod1_dir>] --batch [--outdir=./shots] [--quality=90]\n"
                       "       xmodelview [--basepath=<cod1_dir>] --check-textures [--batch|<modelname>]\n"
                       "Controls: mouse-drag=orbit, scroll=zoom, S=screenshot, N/arrow=next, Q=quit\n");
        return 1;
    }

    // Load all pak*.pk3 files
    {
        std::vector<std::string> paks;
        for(int i=0;i<=0xb;i++) {
            char fn[32]; snprintf(fn,sizeof(fn),"pak%x.pk3",i);
            std::string full=basepath+"/main/"+fn;
            if(FILE *tf=fopen(full.c_str(),"rb")){fclose(tf); paks.push_back(full);}
        }
        // Also localized
        for(int i=0;i<=5;i++) {
            char fn[64]; snprintf(fn,sizeof(fn),"localized_english_pak%d.pk3",i);
            std::string full=basepath+"/main/"+fn;
            if(FILE *tf=fopen(full.c_str(),"rb")){fclose(tf); paks.push_back(full);}
        }
        if(paks.empty()){ fprintf(stderr,"No pk3 found in %s/main/\n",basepath.c_str()); return 1; }
        for(auto &p:paks) gVFS.addPak(p);
        printf("Loaded %d pk3 files\n",(int)paks.size());
    }

    loadAllShaders();
    tryLoadSDL2Image();

    // Collect model list
    std::vector<std::string> modelList;
    if(batch) {
        auto entries=gVFS.listPrefix("xmodel/");
        for(auto &e:entries) {
            // Skip directory entries (end with '/') and short names
            if(e.empty()||e.back()=='/'||e.size()<=7) continue;
            modelList.push_back(e);
        }
        std::sort(modelList.begin(),modelList.end());
        // Deduplicate (same model in multiple paks)
        modelList.erase(std::unique(modelList.begin(),modelList.end()),modelList.end());
        printf("Batch mode: %d models → %s/\n",(int)modelList.size(),outdir.c_str());
    } else {
        if(modelName.empty()){ fprintf(stderr,"Usage: xmodelview [--batch] <modelname>\n"); return 1; }
        modelList.push_back(modelName);
    }

    {
        std::error_code ec;
        std::filesystem::create_directories(outdir, ec);
        if(ec) {
            fprintf(stderr,"Cannot create output directory %s: %s\n",outdir.c_str(),ec.message().c_str());
            return 1;
        }
    }

    if(checkTextures) {
        int missing=0, checked=0, loadFailed=0, decodeFailed=0;
        std::set<std::string> missingKeys;
        for(auto &name:modelList) {
            XModel model;
            if(!loadXModel(name,model)) { loadFailed++; continue; }
            std::set<std::string> seen;
            for(auto &s:model.surfaces) {
                if(s.matName.empty()) continue;
                std::string key=s.matName;
                toLower(key);
                if(!seen.insert(key).second) continue;
                checked++;
                auto path=resolveTexturePath(s.matName);
                if(path.empty()) {
                    missing++;
                    missingKeys.insert(key);
                    printf("Missing texture: %s -> %s\n", name.c_str(), s.matName.c_str());
                } else if(!canDecodeTexturePath(path)) {
                    decodeFailed++;
                    missingKeys.insert(key);
                    printf("Undecodable texture: %s -> %s (%s)\n", name.c_str(), s.matName.c_str(), path.c_str());
                }
            }
        }
        printf("Texture check: models=%d checked=%d missing=%d decode_failed=%d unique_issues=%d load_failed=%d\n",
               (int)modelList.size(), checked, missing, decodeFailed, (int)missingKeys.size(), loadFailed);
        return missingKeys.empty() ? 0 : 2;
    }

    // SDL + GL init
    if(SDL_Init(SDL_INIT_VIDEO)<0){ fprintf(stderr,"SDL_Init: %s\n",SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS,1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES,4);

    SDL_Window *win=SDL_CreateWindow("xmodelview",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,W,H,SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
    if(!win){ fprintf(stderr,"SDL_CreateWindow: %s\n",SDL_GetError()); return 1; }
    SDL_GLContext ctx=SDL_GL_CreateContext(win);
    SDL_GL_SetSwapInterval(1);

    glewExperimental=GL_TRUE;
    if(glewInit()!=GLEW_OK){ fprintf(stderr,"GLEW init failed\n"); return 1; }

    GLuint prog=makeProgram();
    GLuint whiteTex=makeCheckerTex();

    int modelIdx=0;
    std::vector<GPUSurf> gpuSurfs;
    std::vector<GalleryItem> galleryItems;
    XModel currentModel;
    // Spherical orbit camera (CoD1 Z-up):
    // theta=-π/2 → camera on -Y axis → looking at "front" of character
    float theta = -3.14159f*0.5f + 3.14159f*0.25f; // slight offset from pure front
    float phi   = 0.28f;  // ~16° elevation
    float dist=100.f;
    bool mouseDown=false;
    int lastMX=0,lastMY=0;
    bool running=true;
    bool needLoad=true;
    bool screenshotPending=batch;

    auto loadCurrent=[&](){
        for(auto &g:gpuSurfs) freeSurface(g);
        gpuSurfs.clear();
        if(batch) clearTextureCache();
        currentModel={};
        if(modelIdx>=(int)modelList.size()) return;
        SDL_SetWindowTitle(win,modelList[modelIdx].c_str());
        if(!loadXModel(modelList[modelIdx],currentModel)) return;
        for(auto &s:currentModel.surfaces) gpuSurfs.push_back(uploadSurface(s,whiteTex));
        dist=currentModel.radius*2.5f;
        theta=-3.14159f*0.25f; phi=0.28f;
    };

    while(running && modelIdx<(int)modelList.size()) {
        if(needLoad){ loadCurrent(); needLoad=false; }

        SDL_Event ev;
        while(SDL_PollEvent(&ev)) {
            if(ev.type==SDL_QUIT) running=false;
            if(ev.type==SDL_KEYDOWN) {
                if(ev.key.keysym.sym==SDLK_ESCAPE||ev.key.keysym.sym==SDLK_q) running=false;
                if(ev.key.keysym.sym==SDLK_s) {
                    SDL_GetWindowSize(win,&W,&H);
                    renderModel(gpuSurfs,prog,theta,phi,dist,W,H);
                    glFinish();
                    saveJPEG(outdir+"/"+safeName(modelList[modelIdx])+".jpg",W,H,jpgQuality);
                    SDL_GL_SwapWindow(win);
                }
                if(ev.key.keysym.sym==SDLK_n||ev.key.keysym.sym==SDLK_RIGHT) {
                    modelIdx++; needLoad=true;
                }
                if(ev.key.keysym.sym==SDLK_LEFT && modelIdx>0) {
                    modelIdx--; needLoad=true;
                }
            }
            if(ev.type==SDL_MOUSEBUTTONDOWN&&ev.button.button==SDL_BUTTON_LEFT)
                { mouseDown=true; lastMX=ev.button.x; lastMY=ev.button.y; }
            if(ev.type==SDL_MOUSEBUTTONUP&&ev.button.button==SDL_BUTTON_LEFT) mouseDown=false;
            if(ev.type==SDL_MOUSEMOTION&&mouseDown) {
                theta-=(ev.motion.x-lastMX)*0.01f;
                phi  -=(ev.motion.y-lastMY)*0.01f;
                lastMX=ev.motion.x; lastMY=ev.motion.y;
            }
            if(ev.type==SDL_MOUSEWHEEL) dist*=(ev.wheel.y>0?0.9f:1.1f);
            if(ev.type==SDL_WINDOWEVENT&&ev.window.event==SDL_WINDOWEVENT_RESIZED)
                { W=ev.window.data1; H=ev.window.data2; }
        }

        // Slow auto-rotate in batch mode
        if(batch) theta+=0.004f;

        SDL_GetWindowSize(win,&W,&H);
        renderModel(gpuSurfs,prog,theta,phi,dist,W,H);

        if(screenshotPending && !batch) { screenshotPending=false; }
        if(batch) {
            // One frame to render, screenshot immediately, advance
            static int frameCount=0;
            frameCount++;
            if(frameCount>=5) {
                frameCount=0;
                glFinish();
                std::string fileName=safeName(modelList[modelIdx])+".jpg";
                if(saveJPEG(outdir+"/"+fileName,W,H,jpgQuality))
                    galleryItems.push_back({modelList[modelIdx],fileName});
                modelIdx++;
                needLoad=true;
                if(modelIdx>=(int)modelList.size()) running=false;
            }
        }
        SDL_GL_SwapWindow(win);
    }

    if(batch) writeGallery(outdir, galleryItems);

    for(auto &g:gpuSurfs) freeSurface(g);
    glDeleteProgram(prog);
    glDeleteTextures(1,&whiteTex);
    SDL_GL_DeleteContext(ctx);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
