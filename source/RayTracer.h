#pragma once
#include <G3D/G3DAll.h>

class RayTracer {
protected:

    int m_indirectRaysPP;
    bool m_multiThreaded;
    bool m_fixedPrimitives;
    int m_maxRecursion;

    //called from traceImage
    Radiance3 traceOneRay(const Ray& ray, const shared_ptr<Array<shared_ptr<Light>>>& lights, const shared_ptr<TriTree>& tree, int recursionNumber=0) const;
    
    //called from traceOneRay
    //Iterates over whole scene and calls sphere and triangle intersect when neccessary; return nullptr if no intersect
    shared_ptr<Surfel> intersectScene(const Ray& ray, const shared_ptr<TriTree>& tree) const;

    //called from intersectScene
    bool sphereRayIntersect(const Ray& ray, const Point3& center, float radius) const;

    //called from intersectScene
    bool triangleRayIntersect(const Point3& P, const Vector3& w, const Array<Point3> V, float& dist, float bCoord[3]) const;

    //called from intersect functions
    Radiance3 shade(shared_ptr<Surfel> &surfel, const Ray& w_i, const shared_ptr<Array<shared_ptr<Light>>>& lights, const shared_ptr<TriTree>& tree) const;

    //Returns if a given point is visible to a given Ray
    bool isVisible(const Point3& surfacePoint, const Ray& ray, const shared_ptr<TriTree>& tree) const;

    //Returns indirect lighting value
    Radiance3 measureLight(shared_ptr<Surfel>& surfel, const shared_ptr<Array<shared_ptr<Light>>>& lights, const shared_ptr<TriTree>& tree, int recursionNumber=0) const;

public:

    //Constructor
    RayTracer(int indirectRaysPP, bool multiThreaded, bool fixedPrimitives);

     //calls traceOneRay for every pixel, returns render time as String
    String traceImage(const shared_ptr<Scene>& scene, const shared_ptr <Camera> &camera, const shared_ptr <Image> &image);
};
