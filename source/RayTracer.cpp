#include "RayTracer.h"

RayTracer::RayTracer(int indirectRaysPP, bool multiThreaded, bool fixedPrimitives) : m_indirectRaysPP(indirectRaysPP), m_multiThreaded(multiThreaded), m_fixedPrimitives(fixedPrimitives) {
    m_maxRecursion = 2;
}

Radiance3 RayTracer::traceOneRay(const Ray& ray, const shared_ptr<Array<shared_ptr<Light>>>& lights, const shared_ptr<TriTree>& tree, int recursionNumber) const {
        shared_ptr<Surfel> hit = intersectScene(ray, tree);
        Radiance3 emmitedRadiance = Radiance3::zero();
        if(notNull(hit) && m_indirectRaysPP == 0){
           emmitedRadiance = hit->reflectivity(Random::threadCommon()) * 0.05f;
        }
    return  emmitedRadiance + (shade(hit, ray , lights, tree)) + measureLight(hit, lights, tree, recursionNumber+1);
}

shared_ptr<Surfel> RayTracer::intersectScene(const Ray& ray, const shared_ptr<TriTree>& tree) const {
    shared_ptr<Surfel> closest = nullptr;
    float closestDist = -1.0;
    for(int i = 0; i < tree->size(); ++i) {
        Array<Point3> vertices(tree->operator[](i).position(tree->vertexArray(), 0), tree->operator[](i).position(tree->vertexArray(), 1), tree->operator[](i).position(tree->vertexArray(), 2));
        float dist;
        float bCoord[3];
        if(triangleRayIntersect(ray.origin(), ray.direction(), vertices, dist, bCoord)) {
            if(closestDist == -1.0 || closestDist > dist) {
                TriTreeBase::Hit hit = TriTreeBase::Hit();
                hit.distance = dist;
                hit.triIndex = i;
                hit.u = bCoord[0];
                hit.v = bCoord[1];
                hit.backface = tree->operator[](i).normal(tree->vertexArray()).dot(ray.direction()) >= 0;
                closestDist = dist;
                closest = tree->sample(hit);
            }
        }
    }
    return closest;
}

bool RayTracer::sphereRayIntersect(const Ray& ray, const Point3& center, float radius) const {
    //a = 1, so it is not included
    float b = 2*ray.direction().dot(ray.origin()-center);
    float c = (ray.origin()-center).dot(ray.origin()-center) - (radius*radius);

    float radicand = (b*b) - 4*c;
    if(radicand < 0) return false;
    
    float t0 = ((-1*b) + sqrt(radicand))/2;
    float t1 = ((-1*b) - sqrt(radicand))/2;

    if (t1 < 0 && t0 < 0) {
        return false;
    }

    return false;
}

bool RayTracer::triangleRayIntersect(const Point3& P, const Vector3& w, const Array<Point3> V, float& dist, float bCoord[3]) const {
    //MVP
    float eps = 0;

    // Edge vectors
	    const Vector3& edge1 = V[1] - V[0];
	    const Vector3& edge2 = V[2] - V[0];

    // Face normal
	    const Vector3& normal = edge1.cross(edge2).direction();

	    const Vector3& q = w.cross(edge2);
	    const float a = edge1.dot(q);

    // Backfacing / nearly parallel, or close to the limit of precision?
	    if ((normal.dot(w) >= 0) || (abs(a) <= eps)) return false;
	    const Vector3& s = (P - V[0]) / a;
	    const Vector3& r = s.cross(edge1);

	    bCoord[0] = s.dot(q);
	    bCoord[1] = r.dot(w);
	    bCoord[2] = 1.0f - bCoord[0] - bCoord[1];

    // Intersected outside triangle?
	    if ((bCoord[0] < 0.0f) || (bCoord[1] < 0.0f) || (bCoord[2] < 0.0f)) return false;

	    dist = edge2.dot(r);
	    return (dist >= 0.0f);
}


Radiance3 RayTracer::measureLight(shared_ptr<Surfel>& surfel, const shared_ptr<Array<shared_ptr<Light>>>& lights, const shared_ptr<TriTree>& tree, int recursionNumber) const {

    if(notNull(surfel) && recursionNumber <= m_maxRecursion){
        Vector3 normal = surfel->geometricNormal;
        Point3 pos = surfel->position;
        Radiance3 L = Radiance3::zero();
        for(int i = 0; i < m_indirectRaysPP; ++i) {

             L += (traceOneRay( Ray(pos, Vector3::cosHemiRandom(normal , Random::threadCommon())) ,lights, tree, recursionNumber+1))/(m_indirectRaysPP);
        }
        return L;
    }else{return Radiance3::zero();}
}

Radiance3 RayTracer::shade(shared_ptr<Surfel> &surfel, const Ray& worldRay, const shared_ptr<Array<shared_ptr<Light>>>& lights, const shared_ptr<TriTree>& tree) const{
    if( notNull(surfel) ){
        Radiance3 L= Radiance3::zero();
        for(int i = 0; i < lights->size(); ++i){
            shared_ptr<Light> curLight = lights->operator[](i);

            //Point on the surface
            Point3 pointX = surfel->position;
            Point3 pointY = lights->operator[](i)->position().xyz();

            Vector3 rayDirection(-1*worldRay.direction());

            Vector3 w_o(rayDirection);
            Vector3 w_i((pointY - pointX).direction());
            Vector3 normal = surfel->shadingNormal;

            Ray lightRay(pointY, (pointX - pointY).direction());
            float bumpDistance = 0.0001;

            if(isVisible(pointX + bumpDistance * surfel->geometricNormal * -sign(surfel->geometricNormal.dot(lightRay.direction())), lightRay, tree)){
                Biradiance3 biRad = curLight->biradiance(pointX);
                const Color3& fscattering = surfel->finiteScatteringDensity(w_i, w_o);
                L += biRad * fscattering * abs(w_i.dot(normal));
                L += surfel->reflectivity(Random::threadCommon()) * 0.05f;
            }
        }
        return L;
    }else{
        return Radiance3(0,0,0);
    }
}

bool RayTracer::isVisible(const Point3& surfacePoint, const Ray& ray, const shared_ptr<TriTree>& tree) const{
    float eps = 0.0001;
    Vector3 hitRay = surfacePoint - ray.origin();
    shared_ptr<Surfel> testSurfel = intersectScene(ray, tree);
    if(notNull(testSurfel)) {
        float intersectDist = (testSurfel->position - ray.origin()).length();
        return !(intersectDist <= (hitRay.length() - eps));
    }
    return false;
}


String RayTracer::traceImage(const shared_ptr<Scene>& scene, const shared_ptr <Camera> &camera, const shared_ptr <Image> &image) {
    Array<shared_ptr<Surface>> surfaceArray;
    scene->onPose(surfaceArray);
    shared_ptr<TriTree> tree = shared_ptr<TriTree>(new TriTree());
    tree->setContents(surfaceArray);

    shared_ptr<Array<shared_ptr<Light>>> lights;
    lights = std::make_shared<Array<shared_ptr<Light>>>(scene->lightingEnvironment().lightArray);

    const int width = image->width();
    const int height = image->height();
        
    StopWatch watch = StopWatch();

    if(m_multiThreaded) {
         watch.tick();
         Thread::runConcurrently(Point2int32(0,0), Point2int32(width, height), [this, image, camera, width, height, lights, tree] (Point2int32 point) -> void {image->set( point[0], point[1], traceOneRay( camera->worldRay( (float)point[0], (float)point[1], Rect2D( Vector2( (float)width, (float)height ) ) ), lights, tree));});
         watch.tock();
    } else {
        watch.tick();
        for(int y = 0; y < height; ++y ){
            for(int x = 0; x < width; ++x){
                image->set( x, y, traceOneRay( camera->worldRay( (float)x, (float)y, Rect2D( Vector2( (float)width, (float)height ) ) ), lights, tree) );
            }
        }
        watch.tock();
    }
    debugPrintf("%s seconds\n", std::to_string(watch.smoothElapsedTime()).c_str());
    return String(std::to_string(watch.smoothElapsedTime()));
}